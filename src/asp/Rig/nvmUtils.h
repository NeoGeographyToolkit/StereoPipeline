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

#pragma once

#include <asp/Core/nvm.h>
#include <asp/Rig/camera_image.h> // For rig::cameraImage
#include <asp/Rig/rig_config.h>   // For rig::RigSet
#include <asp/Rig/RigTypeDefs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <vector>
#include <string>

#include <OpenMVG/tracks.hpp>

namespace rig {

// Helper function to find a keypoint for given iterator and update cid in the merged
// map with repetitions removed (via cid2cid).
bool updateCidFindKeypoint(std::map<int, int>::const_iterator map_it,
                           std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                           std::map<int, int>            const& cid2cid,
                           std::vector<Eigen::Vector2d>  const& keypoint_offsets,
                           int cid_shift,
                           // outputs
                           int                                & cid, 
                           std::pair<float, float>            & K);

// The nvm file produced by Theia can have files in arbitrary order. Find the map
// which will help bring the cid in the correct order.
void findCidReorderMap(asp::nvmData const& nvm,
                       std::vector<rig::cameraImage> const& cams,
                       // output
                       std::map<int, int> & nvm_cid_to_cams_cid);

// For nvm data that has the keypoints shifted relative to the optical
// center, undo this shift when 'undo_shift' is true. So, add the optical center.
// When 'undo_shift' is false, subtract the optical center.
void shiftKeypoints(bool undo_shift, rig::RigSet const& R,
                    asp::nvmData & nvm); // output

// Transform nvm matches. Account for the fact that the nvm file will
// likely have the images in different order than in the 'cams'
// vector, and may have more such images, as later we may have used
// bracketing to thin them out. Also many need to add a keypoint
// offset.
// TODO(oalexan1): Integrate this with transformAppendNvm().
void transformNvm(// Inputs
                  std::vector<rig::cameraImage>   const& cams,
                  std::vector<Eigen::Vector2d>          const& keypoint_offsets,
                  asp::nvmData                          const& nvm,
                  // Outputs
                  std::vector<std::map<int, int>> & pid_to_cid_fid,
                  rig::KeypointVec & keypoint_vec,
                  std::vector<Eigen::Vector3d> & xyz_vec);

// Given some tracks read from nvm from disk, append the ones from
// nvm. Some remapping is needed.  given that 'fid' values already
// exist for the given tracks and that the nvm read from disk
// may have the images in different order. New keypoints are recorded
// with the help of fid_count and merged_keypoint_map.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
// TODO(oalexan1): cid_shift and keypoint_offsets should be applied outside
// this function as they make it hard to understand.
void transformAppendNvm(// Append from these
                        std::vector<std::map<int, int>>  const& nvm_pid_to_cid_fid,
                        std::vector<Eigen::Matrix2Xd>    const& nvm_cid_to_keypoint_map,
                        std::map<int, int>               const& cid2cid,
                        std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                        int cid_shift,
                        size_t num_out_cams,
                        // Outputs, append to these
                        std::vector<int> & fid_count,
                        std::vector<std::map<std::pair<float, float>, int>>
                        & merged_keypoint_map,
                        std::vector<std::map<int, int>> & pid_to_cid_fid);

} // end namespace rig
