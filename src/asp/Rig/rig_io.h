// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#ifndef __ASP_RIG_RIG_IO_H__
#define __ASP_RIG_RIG_IO_H__

#include <asp/Rig/RigTypeDefs.h>
#include <string>
#include <vector>
#include <map>

#include <opencv2/core/mat.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Forward declarations of types from other headers
namespace rig {
  struct cameraImage;
}

namespace rig {

// Form the match file name using the ASP convention.
//
// Assumes the input images are of the form cam_name/image.jpg.
// The match file will be named run/run-image1__image2.match.
std::string matchFileName(std::string const& match_dir,
                          std::string const& left_image,
                          std::string const& right_image,
                          std::string const& suffix);

// Write an image with 3 floats per pixel.
//
// This is used for saving 3D point cloud data (XYZ) as an image.
// OpenCV's imwrite() cannot do this.
void saveXyzImage(std::string const& filename, cv::Mat const& img);

// Save images and their corresponding depth clouds to disk.
void saveImagesAndDepthClouds(std::vector<rig::cameraImage> const& cams);

// Save the inlier matches for each image pair to disk.
//
// Given all the merged and filtered tracks, this function iterates through
// them and extracts the pairwise matches that belong to inlier tracks,
// saving them to .match files.
void saveInlierMatchPairs(// Inputs
                          std::vector<rig::cameraImage> const& cams,
                          int num_overlaps,
                          std::vector<std::map<int, int>> const& pid_to_cid_fid,
                          rig::KeypointVec
                          const& keypoint_vec,
                          PidCidFid const& pid_cid_fid_inlier,
                          std::string const& out_dir);

// Find convergence angles between every pair of images and save
// their percentiles to disk.
void savePairwiseConvergenceAngles(// Inputs
                                   std::vector<std::map<int, int>> const& pid_to_cid_fid,
                                   rig::KeypointVec
                                   const& keypoint_vec,
                                   std::vector<rig::cameraImage> const& cams,
                                   std::vector<Eigen::Affine3d> const& world_to_cam,
                                   std::vector<Eigen::Vector3d> const& xyz_vec,
                                   PidCidFid const& pid_cid_fid_inlier,
                                   std::string const& conv_angles_file);

// Save the list of images, for use with bundle_adjust.
void saveImageList(std::vector<rig::cameraImage> const& cams,
                   std::string const& image_list);

}  // end namespace rig

#endif  // __ASP_RIG_RIG_IO_H__
