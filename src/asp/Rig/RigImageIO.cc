/* Copyright (c) 2021-2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License. */

#include <asp/Rig/RigImageIO.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/CameraImage.h>
#include <asp/Rig/SystemUtils.h>
#include <asp/Rig/BasicAlgs.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/OpenCVUtils.h>

#include <vw/Core/Log.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

namespace rig {

// The images from the bag may need to be resized to be the same
// size as in the calibration file. Sometimes the full-res images
// can be so blurry that interest point matching fails, hence the
// resizing.
// Similar logic to deal with differences between image size and calibrated size
// is used further down this code.
void adjustImageSize(rig::CameraParameters const& cam_params, cv::Mat & image) {
  int64_t raw_image_cols = image.cols;
  int64_t raw_image_rows = image.rows;
  int64_t calib_image_cols = cam_params.GetDistortedSize()[0];
  int64_t calib_image_rows = cam_params.GetDistortedSize()[1];
  int64_t factor = raw_image_cols / calib_image_cols;

  // If the raw image has size 0, skip, but give a warning. This happens
  // when the image is not found. If prior interest point matches exist,
  // the workflow can still continue.
  if (raw_image_cols == 0 || raw_image_rows == 0) {
    LOG(WARNING) << "Image has size 0, skipping.";
    return;
  }
  
  if ((raw_image_cols != calib_image_cols * factor) ||
      (raw_image_rows != calib_image_rows * factor)) {
    LOG(FATAL) << "Image width and height are: "
               << raw_image_cols << ' ' << raw_image_rows << "\n"
               << "Calibrated image width and height are: "
               << calib_image_cols << ' ' << calib_image_rows << "\n"
               << "These must be equal up to an integer factor.\n";
  }

  if (factor != 1) {
    // TODO(oalexan1): This kind of resizing may be creating aliased images.
    cv::Mat local_image;
    cv::resize(image, local_image, cv::Size(), 1.0/factor, 1.0/factor, cv::INTER_AREA);
    local_image.copyTo(image);
  }

  // Check
  if (image.cols != calib_image_cols || image.rows != calib_image_rows)
    LOG(FATAL) << "The images have the wrong size.";
}

// Read an image with 3 floats per pixel. OpenCV's imread() cannot do that.
void readXyzImage(std::string const& filename, cv::Mat & img) {
  std::ifstream f;
  f.open(filename.c_str(), std::ios::binary | std::ios::in);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for reading: " << filename << "\n";

  int rows, cols, channels;
  // TODO(oalexan1): Replace below with int32_t and check that it is same thing.
  f.read((char*)(&rows), sizeof(rows));         // NOLINT
  f.read((char*)(&cols), sizeof(cols));         // NOLINT
  f.read((char*)(&channels), sizeof(channels)); // NOLINT

  img = cv::Mat::zeros(rows, cols, CV_32FC3);

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      cv::Vec3f P;
      // TODO(oalexan1): See if using reinterpret_cast<char*> does the same
      // thing.
      for (int c = 0; c < channels; c++)
        f.read((char*)(&P[c]), sizeof(P[c])); // NOLINT
      img.at<cv::Vec3f>(row, col) = P;
    }
  }

  return;
}

void readImageEntry(// Inputs
                    std::string const& image_file,
                    Eigen::Affine3d const& world_to_cam,
                    std::vector<std::string> const& cam_names,
                    int cam_type,
                    double timestamp,
                    int num_overlaps,
                    // Outputs
                    std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                    std::vector<std::map<double, rig::ImageMessage>> & depth_maps) {
  
  // Aliases
  std::map<double, ImageMessage> & image_map = image_maps[cam_type];
  std::map<double, ImageMessage> & depth_map = depth_maps[cam_type];

  if (image_map.find(timestamp) != image_map.end())
    vw::vw_out(vw::WarningMessage) 
      << "Duplicate timestamp " << std::setprecision(17) << timestamp
      << " for sensor id " << cam_type << "\n";
  
  // Read the image as grayscale, in order for feature matching to work.
  // Non-byte images need to normalized to [0, 255] and converted to byte type.
  // If num_overlaps is 0, no feature matching will happen, so no need to read
  // the images.
  if (num_overlaps > 0) {
    bool hasByte = asp::hasByteChannels(image_file);
    if (hasByte) {
      image_map[timestamp].image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    } else {
      vw::vw_out() << "Non-byte image detected. Will normalize.\n";
      vw::ImageViewRef<vw::PixelMask<float>> masked_image;
      asp::normalizeImage(image_file, masked_image);
      asp::maskedToScaledByteCvImage(masked_image, image_map[timestamp].image);
    }
  }
  
  // Populate the other fields 
  image_map[timestamp].name         = image_file;
  image_map[timestamp].timestamp    = timestamp;
  image_map[timestamp].world_to_cam = world_to_cam;

  // Sanity check
  if (depth_map.find(timestamp) != depth_map.end())
     vw::vw_out(vw::WarningMessage) << "Duplicate timestamp " << std::setprecision(17) 
       << timestamp << " for sensor id " << cam_type << "\n";

  // Read the depth data, if present
  std::string depth_file = fs::path(image_file).replace_extension(".pc").string();
  if (fs::exists(depth_file)) {
    //vw::vw_out() << "Reading: " << depth_file << std::endl;
    rig::readXyzImage(depth_file, depth_map[timestamp].image);
    depth_map[timestamp].name      = depth_file;
    depth_map[timestamp].timestamp = timestamp;
  }
}

// Write an image with 3 floats per pixel. OpenCV's imwrite() cannot do that.
void saveXyzImage(std::string const& filename, cv::Mat const& img) {
  if (img.depth() != CV_32F)
    LOG(FATAL) << "Expecting an image with float values.\n";
  if (img.channels() != 3) LOG(FATAL) << "Expecting 3 channels.\n";

  std::ofstream f;
  f.open(filename.c_str(), std::ios::binary | std::ios::out);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for writing: " << filename << "\n";

  // Assign these to explicit variables so we know their type and size in bytes
  // TODO(oalexan1): Replace below with int32_t and check that it is same thing.
  int rows = img.rows, cols = img.cols, channels = img.channels();

  // TODO(oalexan1): Avoid C-style cast. Test if
  // reinterpret_cast<char*> does the same thing.
  f.write((char*)(&rows), sizeof(rows));         // NOLINT
  f.write((char*)(&cols), sizeof(cols));         // NOLINT
  f.write((char*)(&channels), sizeof(channels)); // NOLINT

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      cv::Vec3f const& P = img.at<cv::Vec3f>(row, col);  // alias
      // TODO(oalexan1): See if using reinterpret_cast<char*> does the same
      // thing.
      for (int c = 0; c < channels; c++)
        f.write((char*)(&P[c]), sizeof(P[c])); // NOLINT
    }
  }

  return;
}

// Save images and depth clouds to disk
void saveImagesAndDepthClouds(std::vector<rig::cameraImage> const& cams) {
  for (size_t it = 0; it < cams.size(); it++) {

    std::cout << "Writing: " << cams[it].image_name << std::endl;
    cv::imwrite(cams[it].image_name, cams[it].image);

    if (cams[it].depth_cloud.cols > 0 && cams[it].depth_cloud.rows > 0) {
      std::cout << "Writing: " << cams[it].depth_name << std::endl;
      rig::saveXyzImage(cams[it].depth_name, cams[it].depth_cloud);
    }
  }

  return;
}

// Save the list of images, for use with bundle_adjust.
void saveImageList(std::vector<rig::cameraImage> const& cams,
                   std::string const& image_list) {

  // Create the directory having image_list
  std::string dir = fs::path(image_list).parent_path().string();
  rig::createDir(dir);
  
  std::cout << "Writing: " << image_list << std::endl;
  std::ofstream ofs(image_list.c_str());
  for (size_t it = 0; it < cams.size(); it++)
    ofs << cams[it].image_name << std::endl;
  ofs.close();

  return;
}

} // namespace rig
