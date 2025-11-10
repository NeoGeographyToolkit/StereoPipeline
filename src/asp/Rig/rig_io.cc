/* Copyright (c) 2021, United States Government, as represented by the
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
 * under the License.
 */

#include <asp/Rig/rig_io.h>
#include <asp/Rig/detector.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/basic_algs.h>

#include <vw/InterestPoint/MatcherIO.h>

#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <set>

namespace fs = boost::filesystem;

namespace rig {
  
// TODO(oalexan1): Use vw logic.
// Form the match file name. Assume the input images are of the form
// cam_name/image.jpg. Use the ASP convention of the match file being
// run/run-image1__image2.match. This assumes all input images are unique.
std::string matchFileName(std::string const& match_dir,
                          std::string const& left_image, 
                          std::string const& right_image,
                          std::string const& suffix) {
  std::string left_cam_name
    = boost::filesystem::path(left_image).parent_path().stem().string();
  std::string right_cam_name
    = boost::filesystem::path(right_image).parent_path().stem().string();

  if (left_cam_name == "" || right_cam_name == "")
    LOG(FATAL) << "The image name must have the form cam_name/image. Got: "
               << left_image << " and " << right_image << ".\n";

  std::string left_stem = boost::filesystem::path(left_image).stem().string();
  std::string right_stem = boost::filesystem::path(right_image).stem().string();
  std::string match_file = match_dir + "/run-" + left_stem + "__"
    + right_stem + suffix + ".match";

  return match_file;
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

// Given all the merged and filtered tracks in pid_cid_fid, for each
// image pair cid1 and cid2 with cid1 < cid2 < cid1 + num_overlaps + 1,
// save the matches of this pair which occur in the set of tracks.
void saveInlierMatchPairs(// Inputs
                           std::vector<rig::cameraImage> const& cams,
                           int num_overlaps,
                           std::vector<std::map<int, int>> const& pid_to_cid_fid,
                           std::vector<std::vector<std::pair<float, float>>>
                           const& keypoint_vec,
                           std::vector<std::map<int, std::map<int, int>>>
                           const& pid_cid_fid_inlier,
                           std::string const& out_dir) {

  MATCH_MAP matches;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // When num_overlaps == 0, we save only matches read from nvm rather
        // ones made wen this tool was run.
        bool is_good = (cid1 < cid2 && (num_overlaps == 0 || cid2 < cid1 + num_overlaps + 1));
        if (!is_good)
          continue;

        // Consider inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1) ||
            !rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2))
          continue;

        auto cid_pair = std::make_pair(cid1, cid2);

        vw::ip::InterestPoint ip1(keypoint_vec[cid1][fid1].first, keypoint_vec[cid1][fid1].second);
        vw::ip::InterestPoint ip2(keypoint_vec[cid2][fid2].first, keypoint_vec[cid2][fid2].second);

        matches[cid_pair].first.push_back(ip1);
        matches[cid_pair].second.push_back(ip2);
      }
    }
  }  // End iterations over pid

  for (auto it = matches.begin(); it != matches.end(); it++) {
    auto & cid_pair = it->first;
    rig::MATCH_PAIR const& match_pair = it->second;

    int left_cid = cid_pair.first;
    int right_cid = cid_pair.second;

    std::string match_dir = out_dir + "/matches";
    rig::createDir(match_dir);

    std::string suffix = "";
    std::string match_file = rig::matchFileName(match_dir,
                                                      cams[left_cid].image_name,
                                                      cams[right_cid].image_name,
                                                      suffix);

    std::cout << "Writing: " << match_file << std::endl;
    vw::ip::write_binary_match_file(match_file, match_pair.first, match_pair.second);
  }
  
  // The image names (without directory) must be unique, or else bundle
  // adjustment will fail.
  std::set<std::string> base_names;
  for (size_t it = 0; it < cams.size(); it++) {
    std::string base_name = fs::path(cams[it].image_name).filename().string(); 
    if (base_names.find(base_name) != base_names.end())
      LOG(FATAL) << "Non-unique image name (without directory): " << base_name 
                 << ". It will not be possible to use these matches with bundle_adjust.";
    base_names.insert(base_name);
  }
  
} // end function saveInlierMatchPairs

// Find convergence angles between every pair of images and save to disk their
// percentiles assumed that the cameras in world_to_cam are up-to-date given the
// current state of optimization, and that the residuals (including the
// reprojection errors) have also been updated beforehand.
void savePairwiseConvergenceAngles(// Inputs
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
  std::vector<rig::cameraImage> const& cams,
  std::vector<Eigen::Affine3d> const& world_to_cam,
  std::vector<Eigen::Vector3d> const& xyz_vec,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_inlier,
  std::string const& conv_angles_file) {

  std::map<std::pair<int, int>, std::vector<double>> conv_angles;
  
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) continue;

      Eigen::Vector3d cam_ctr1 = (world_to_cam[cid1].inverse()) * Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d ray1 = xyz_vec[pid] - cam_ctr1;
      ray1.normalize();

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // Look at each cid and next cids
        if (cid2 <= cid1)
          continue;

        // Deal with inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2)) continue;

        Eigen::Vector3d cam_ctr2 = (world_to_cam[cid2].inverse()) * Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d ray2 = xyz_vec[pid] - cam_ctr2;
        ray2.normalize();

        // Calculate the convergence angle
        double conv_angle = (180.0 / M_PI) * acos(ray1.dot(ray2));
        if (std::isnan(conv_angle) || std::isinf(conv_angle)) continue;

        // Add to the image pair
        std::pair<int, int> pair(cid1, cid2);
        conv_angles[pair].push_back(conv_angle);
      }
    }
  }

  // Sort the convergence angles per pair
  std::cout << "Writing: " << conv_angles_file << std::endl;
  std::ofstream ofs(conv_angles_file.c_str());
  ofs << "# Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << "# left_image right_image 25% 50% 75% num_matches\n";
  ofs.precision(17);
  for (auto it = conv_angles.begin(); it != conv_angles.end(); it++) {

    // Sort the values first
    std::vector<double> & vals = it->second; // alias
    std::sort(vals.begin(), vals.end());
    int len = vals.size();
    
    int cid1 = (it->first).first;
    int cid2 = (it->first).second;
    ofs << cams[cid1].image_name << ' ' << cams[cid2].image_name << ' '
        << vals[0.25 * len] << ' ' << vals[0.5 * len] << ' ' << vals[0.75*len] << ' '
        << len << std::endl;
  }
  ofs.close();

  return;
} // end function savePairwiseConvergenceAngles
  
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
  
}  // namespace rig

