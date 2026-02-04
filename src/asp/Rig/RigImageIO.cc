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
#include <asp/Rig/camera_image.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/basic_algs.h>

#include <asp/Core/ImageUtils.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Core/Exception.h>

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

/// Checks if the given image file has an 8-bit channel type.
/// Return true if the image is 8-bit, false otherwise or if an error occurs.
bool isByteType(const std::string& image_path) {

  boost::shared_ptr<vw::DiskImageResource> rsrc = vw::DiskImageResourcePtr(image_path);

  auto pixel_format = rsrc->pixel_format();
  auto channel_type = rsrc->channel_type();

  vw::vw_out() << "Inspecting Image: " << image_path << std::endl;
  vw::vw_out() << "  Pixel Format: " << pixel_format << std::endl;
  vw::vw_out() << "  Channel Type: " << channel_type << std::endl;

  return (channel_type == vw::VW_CHANNEL_UINT8) && 
         (pixel_format == vw::VW_PIXEL_GRAY   ||
          pixel_format == vw::VW_PIXEL_RGB    ||
          pixel_format == vw::VW_PIXEL_RGBA   ||
          pixel_format == vw::VW_PIXEL_GRAYA);
}

void readImageEntry(// Inputs
                    std::string const& image_file,
                    Eigen::Affine3d const& world_to_cam,
                    std::vector<std::string> const& cam_names,
                    int cam_type,
                    double timestamp,
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
  
   bool isByte = isByteType(image_file);
   std::cout << "---is byte is " << isByte << std::endl;
    
///ppppp
// User-requested test code to check VW image reading and stats
  vw::vw_out() << "---Reading with vw::DiskImageView: " << image_file << std::endl;
  
  vw::ImageViewRef<float> img_view = vw::DiskImageView<float>(image_file);
  
  // This is just an example. A real implementation should fetch the nodata
  // value from the image resource if it exists.
  float nodata_val = -32768.0; 
  boost::shared_ptr<vw::DiskImageResource> rsrc = vw::DiskImageResourcePtr(image_file);
  if (rsrc->has_nodata_read()) {
    nodata_val = rsrc->nodata_read();
    std::cout << "--read nodata value: " << nodata_val << std::endl;
  } else {
    std::cout << "--no nodata value found, using default: " << nodata_val << std::endl;
  }
  std::cout << "---now call read nodata value function: " << std::endl;
  vw::read_nodata_val(image_file, nodata_val);
  std::cout << "--after reading nodata value: " << nodata_val << std::endl;

  vw::ImageViewRef<vw::PixelMask<float>> 
    masked_image = vw::create_mask(img_view, nodata_val);
  
  std::string tag = image_file;
  std::string prefix = ""; 
  bool reuse_cache = false;
  vw::Vector6f stats = asp::gather_stats(masked_image, tag, prefix, image_file, reuse_cache);
  
  std::cout << "--min is " << stats[0] << ", max is " << stats[1] << std::endl;
  float mean = stats[2];
  float stddev = stats[3];
  std::cout << "--mean is " << mean << ", stddev is " << stddev << std::endl;
  
  double min_val = std::max(stats[0], mean - 2*stddev);
  std::cout << "--computed min val is " << min_val << std::endl;
  double max_val = std::min(stats[1], mean + 2*stddev);
  std::cout << "--computed max val is " << max_val << std::endl;
  
  masked_image = normalize(masked_image, min_val, max_val, 0.0, 255.0);
  //qqqq
  vw::vw_out() << "VW stats for " << image_file << ": " << stats << std::endl;
  
  // Create an empty cv::Mat with the same dimensions as the VW image.
  cv::Mat cv_image(masked_image.rows(), masked_image.cols(), CV_8UC1);
  
  // Copy the pixel data from the vw::ImageViewRef to the cv::Mat.
  for (int r = 0; r < cv_image.rows; r++) {
    for (int c = 0; c < cv_image.cols; c++) {
      auto pix = masked_image(c, r);
      double value = 0.0;
      if (is_valid(pix))
        value = pix.child();
      // round
      value = std::round(value);
      // make non-negative
      if (value < 0.0)
        value = 0.0;
      // clamp to 255
      if (value > 255.0)
        value = 255.0;
      
      // cast to uchar and assign to cv::Mat
      cv_image.at<uchar>(r, c) = static_cast<uchar>(value);
    }
  }
  
  std::string out = "tmp.tif";
  std::cout << "---Writing normalized image to " << out << std::endl;
  cv::imwrite(out, cv_image);

  exit(0);

  // Read the image as grayscale, in order for feature matching to work
  // For texturing, texrecon should use the original color images.
  //vw::vw_out() << "Reading: " << image_file << std::endl;
  image_map[timestamp].image        = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  image_map[timestamp].name         = image_file;
  image_map[timestamp].timestamp    = timestamp;
  image_map[timestamp].world_to_cam = world_to_cam;

  // Sanity check
  if (depth_map.find(timestamp) != depth_map.end())
    LOG(WARNING) << "Duplicate timestamp " << std::setprecision(17) << timestamp
                 << " for sensor id " << cam_type << "\n";

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
    LOG(FATAL) << "Expecting an image with float values\n";
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
