// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#ifndef ASP_RIG_RIG_IO_H
#define ASP_RIG_RIG_IO_H

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
  class RigSet;
  struct ImageMessage;
}

namespace asp {
  class nvmData;
}

namespace rig {

// For each image, find its sensor name and timestamp. The info can be in a list or
// from the file or directory structure. If flexible_strategy is true, then 
// can try from list first, and if that fails, then from file/directory structure.
void readImageSensorTimestamp(std::string const& image_sensor_list, 
                              std::vector<std::string> const& image_files,
                              std::vector<std::string> const& cam_names,
                              bool flexible_strategy,
                              // Outputs
                              std::vector<int> & cam_types,
                              std::vector<double> & timestamps);

void readCameraPoses(// Inputs
                     std::string const& camera_poses_file,
                     // Outputs
                     asp::nvmData & nvm);

// Read camera information and images from a list or from an NVM file.
// Can interpolate/extrapolate poses for data from an extra list.
// Only later we will consider if the features are shifted or not in the nvm.
void readListOrNvm(// Inputs
                   std::string const& camera_poses_list,
                   std::string const& nvm_file,
                   std::string const& image_sensor_list,
                   std::string const& extra_list,
                   bool use_initial_rig_transforms,
                   double bracket_len, bool nearest_neighbor_interp,
                   bool read_nvm_no_shift,
                   rig::RigSet const& R,
                   // Outputs
                   asp::nvmData & nvm,
                   std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                   std::vector<std::map<double, rig::ImageMessage>> & depth_maps);

// Form the match file name using the ASP convention.
//
// Assumes the input images are of the form cam_name/image.jpg.
// The match file will be named run/run-image1__image2.match.
std::string matchFileName(std::string const& match_dir,
                          std::string const& left_image,
                          std::string const& right_image,
                          std::string const& suffix);

// Save the inlier matches for each image pair to disk.
//
// Given all the merged and filtered tracks, this function iterates through
// them and extracts the pairwise matches that belong to inlier tracks,
// saving them to .match files.
void saveInlierMatchPairs(// Inputs
                          std::vector<rig::cameraImage> const& cams,
                          int num_overlaps,
                          rig::PidCidFid                const& pid_to_cid_fid,
                          rig::KeypointVec              const& keypoint_vec,
                          PidCidFidMap                  const& pid_cid_fid_inlier,
                          std::string                   const& out_dir);

// Find convergence angles between every pair of images and save
// their percentiles to disk.
void savePairwiseConvergenceAngles(// Inputs
                                   rig::PidCidFid                const& pid_to_cid_fid,
                                   rig::KeypointVec              const& keypoint_vec,
                                   std::vector<rig::cameraImage> const& cams,
                                   std::vector<Eigen::Affine3d>  const& world_to_cam,
                                   std::vector<Eigen::Vector3d>  const& xyz_vec,
                                   PidCidFidMap                  const& pid_cid_fid_inlier,
                                   std::string                   const& conv_angles_file);

}  // end namespace rig

#endif  // ASP_RIG_RIG_IO_H
