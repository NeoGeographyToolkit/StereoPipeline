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

#ifndef INTEREST_POINT_H_
#define INTEREST_POINT_H_

#include <asp/Rig/RigTypeDefs.h>
#include <asp/Rig/detector.h>

#include <vw/InterestPoint/InterestData.h>

#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <utility>

namespace rig {
  class CameraParameters;
}

namespace aspOpenMVG {
  namespace matching {
    class PairWiseMatches;
  }
}

namespace asp {
  class nvmData;
}

namespace rig {

class cameraImage;
class RigSet;
class ImageMessage;

// Copy IP information from an OpenCV KeyPoint object.
void setFromCvKeypoint(Eigen::Vector2d const& key, cv::Mat const& cv_descriptor,
                       vw::ip::InterestPoint& ip);

void detectFeatures(const cv::Mat& image, bool verbose,
                    // Outputs
                    cv::Mat* descriptors, Eigen::Matrix2Xd* keypoints);

// This really likes haz cam first and nav cam second
void matchFeatures(std::mutex* match_mutex, int left_image_index, int right_image_index,
                   cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                   Eigen::Matrix2Xd const& left_keypoints,
                   Eigen::Matrix2Xd const& right_keypoints, bool verbose,
                   // Output
                   MATCH_PAIR* matches);

void detectMatchAppendFeatures(// Inputs
                         std::vector<rig::cameraImage>      const& cams,
                         std::vector<rig::CameraParameters> const& cam_params,
                         std::string                        const& out_dir, 
                         bool save_matches,
                         bool filter_matches_using_cams,
                         std::vector<Eigen::Affine3d>       const& world_to_cam,
                         int num_overlaps,
                         std::vector<std::pair<int, int>>   const& input_image_pairs, 
                         int initial_max_reprojection_error, int num_match_threads,
                         bool read_nvm_no_shift, bool no_nvm_matches, bool verbose,
                         // Outputs
                         rig::KeypointVec                        & keypoint_vec,
                         rig::PidCidFid                          & pid_to_cid_fid,
                         std::vector<Eigen::Vector3d>            & xyz_vec,
                         asp::nvmData                            & nvm);

// For nvm data that has the keypoints shifted relative to the optical
// center, undo this shift when 'undo_shift' is true. So, add the optical center.
// When 'undo_shift' is false, subtract the optical center.
void shiftKeypoints(bool undo_shift, rig::RigSet const& R,
                    asp::nvmData & nvm); // output
  
// Read camera information and images from a list or from an NVM file.
// Can interpolate/extrapolate poses for data from an extra list.  
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
  
// Break up each track of keypoints of length N into N pairs, (T0,
// T1), (T1, T2), ,,. (T(N-1), T0). Find their indices in the merged
// set of keypoints. Repeat this for each input map to merge and
// accumulate the pairs. Later these will be combined into new tracks
// and any repeated data will be fused. This is very tied to the
// addKeypoints() function.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
void addMatchPairs(// Append from these
                   rig::PidCidFid                        const& pid_to_cid_fid,
                   rig::CidToKeypointMatVec              const& cid_to_keypoint_map,
                   std::map<int, int>                    const& cid2cid,
                   std::vector<Eigen::Vector2d>          const& keypoint_offsets,
                   KeyPointMap                           const& merged_keypoint_map, 
                   int cid_shift, size_t num_out_cams,
                   aspOpenMVG::matching::PairWiseMatches      & match_map); // append here

// Given some tracks read from nvm from disk, append the ones from
// nvm. Some remapping is needed.  given that 'fid' values already
// exist for the given tracks and that the nvm read from disk
// may have the images in different order. New keypoints are recorded
// with the help of fid_count and merged_keypoint_map.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
void transformAppendNvm(// Append from these
                        rig::PidCidFid               const& nvm_pid_to_cid_fid,
                        rig::CidToKeypointMatVec     const& nvm_cid_to_keypoint_map,
                        std::map<int, int>           const& cid2cid,
                        std::vector<Eigen::Vector2d> const& keypoint_offsets,
                        int cid_shift,
                        size_t num_out_cams,
                        // Outputs, append to these 
                        std::vector<int>                  & fid_count,
                        KeyPointMap                       & merged_keypoint_map,
                        rig::PidCidFid                    & pid_to_cid_fid);
  
// Add keypoints from a map, appending to existing keypoints. Take into
// account how this map's cid gets transformed to the new map cid.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
void addKeypoints(// Append from these
                  rig::PidCidFid               const& pid_to_cid_fid,
                  rig::CidToKeypointMatVec     const& cid_to_keypoint_map,
                  std::map<int, int>           const& cid2cid,
                  std::vector<Eigen::Vector2d> const& keypoint_offsets,
                  int cid_shift,
                  size_t num_out_cams,
                  // Outputs, append to these 
                  std::vector<int>                  & keypoint_count,
                  KeyPointMap                       & merged_keypoint_map);

void flagOutlierByExclusionDist(// Inputs
                                std::vector<rig::CameraParameters> const& cam_params,
                                std::vector<rig::cameraImage>      const& cams,
                                rig::PidCidFid                     const& pid_to_cid_fid,
                                rig::KeypointVec                   const& keypoint_vec,
                                // Outputs
                                PidCidFidMap                            & pid_cid_fid_inlier);

void flagOutliersByTriAngleAndReprojErr
(// Inputs
 double min_triangulation_angle, double max_reprojection_error,
 rig::PidCidFid               const& pid_to_cid_fid,
 rig::KeypointVec             const& keypoint_vec,
 std::vector<Eigen::Affine3d> const& world_to_cam, 
 std::vector<Eigen::Vector3d> const& xyz_vec,
 PidCidFidMap                 const& pid_cid_fid_to_residual_index,
 std::vector<double>          const& residuals,
 // Outputs
 PidCidFidMap                      & pid_cid_fid_inlier);

void savePairwiseConvergenceAngles(// Inputs
  rig::PidCidFid                const& pid_to_cid_fid,
  rig::KeypointVec              const& keypoint_vec,
  std::vector<rig::cameraImage> const& cams,
  std::vector<Eigen::Affine3d>  const& world_to_cam,
  std::vector<Eigen::Vector3d>  const& xyz_vec,
  PidCidFidMap                  const& pid_cid_fid_inlier,
  std::string                   const& conv_angles_file);

}  // namespace rig

#endif  // INTEREST_POINT_H_
