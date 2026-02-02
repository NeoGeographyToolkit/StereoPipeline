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

#ifndef __ASP_RIG_RIG_DATA_H__
#define __ASP_RIG_RIG_DATA_H__

#include <asp/Rig/rig_config.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

// Structured to hold rig data for i/o and for optimization

namespace rig {

// Structured extrinsics data (human-readable, used for I/O and logic)
struct Extrinsics {
  // Camera extrinsics as Affine3d transforms
  std::vector<Eigen::Affine3d> world_to_ref;
  std::vector<Eigen::Affine3d> world_to_cam;
};

// Optimization state (flattened data for Ceres parameter blocks). Must be populated
// before optimization, and data from it must be exported after optimization.
struct OptState {
  // Camera extrinsics (flattened to vector<double> for Ceres)
  std::vector<double> world_to_cam_vec;
  std::vector<double> world_to_ref_vec;
  std::vector<double> ref_to_cam_vec;
  std::vector<double> ref_identity_vec;
  std::vector<double> right_identity_vec;
  
  // Camera intrinsics
  std::vector<double> focal_lengths;
  std::vector<Eigen::Vector2d> optical_centers;
  std::vector<Eigen::VectorXd> distortions;
  
  // Depth-to-image transforms (flattened)
  std::vector<double> depth_to_image_vec;
  std::vector<double> depth_to_image_scales;
};

// Convert from structured extrinsics to optimization state
void toOptState(const Extrinsics& extrinsics, const RigSet& R, OptState& state, 
                bool no_rig, bool affine_depth_to_image, int num_depth_params);

// Convert from optimization state back to structured extrinsics
void fromOptState(const OptState& state, Extrinsics& extrinsics, RigSet& R,
                  bool no_rig, bool affine_depth_to_image, int num_depth_params);

} // namespace rig

#endif // __ASP_RIG_RIG_DATA_H__
