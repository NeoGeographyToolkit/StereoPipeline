// Structured to hold rig data for i/o and for optimization

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

#include <asp/Rig/RigData.h>
#include <asp/Rig/TransformUtils.h>

namespace rig {

// Must call this before running the optimization to populate the OptState structure
void toOptState(const Extrinsics& extrinsics, const RigSet& R, OptState& state,
                bool no_rig, bool affine_depth_to_image, int num_depth_params) {
  // Convert world_to_ref transforms
  state.world_to_ref_vec.resize(extrinsics.world_to_ref.size() * rig::NUM_RIGID_PARAMS);
  for (size_t cid = 0; cid < extrinsics.world_to_ref.size(); cid++)
    rig::rigid_transform_to_array(extrinsics.world_to_ref[cid],
                                  &state.world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);
  
  // Convert world_to_cam transforms (only if no_rig)
  if (no_rig) {
    state.world_to_cam_vec.resize(extrinsics.world_to_cam.size() * rig::NUM_RIGID_PARAMS);
    for (size_t cid = 0; cid < extrinsics.world_to_cam.size(); cid++)
      rig::rigid_transform_to_array(extrinsics.world_to_cam[cid],
                                    &state.world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid]);
  }
  
  // Convert ref_to_cam transforms from R
  state.ref_to_cam_vec.resize(R.ref_to_cam_trans.size() * rig::NUM_RIGID_PARAMS);
  for (size_t cam_type = 0; cam_type < R.ref_to_cam_trans.size(); cam_type++)
    rig::rigid_transform_to_array(R.ref_to_cam_trans[cam_type],
                                  &state.ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type]);
  
  // Convert depth_to_image transforms from R (affine or rigid)
  state.depth_to_image_vec.resize(R.depth_to_image.size() * num_depth_params);
  for (size_t cam_type = 0; cam_type < R.depth_to_image.size(); cam_type++) {
    if (affine_depth_to_image)
      rig::affine_transform_to_array(R.depth_to_image[cam_type],
                                     &state.depth_to_image_vec[num_depth_params * cam_type]);
    else
      rig::rigid_transform_to_array(R.depth_to_image[cam_type],
                                    &state.depth_to_image_vec[num_depth_params * cam_type]);
  }
  
  // Copy intrinsics from R to state
  int num_cam_types = R.cam_names.size();
  state.focal_lengths.resize(num_cam_types);
  state.optical_centers.resize(num_cam_types);
  state.distortions.resize(num_cam_types);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    state.focal_lengths[cam_type] = R.cam_params[cam_type].GetFocalLength();
    state.optical_centers[cam_type] = R.cam_params[cam_type].GetOpticalOffset();
    state.distortions[cam_type] = R.cam_params[cam_type].GetDistortion();
  }
  
  // Setup identity transforms for ref cam and right bracketing cam placeholders.
  // These need to have different pointers because CERES wants it that way.
  Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  state.ref_identity_vec.resize(rig::NUM_RIGID_PARAMS);
  state.right_identity_vec.resize(rig::NUM_RIGID_PARAMS);
  rig::rigid_transform_to_array(identity, &state.ref_identity_vec[0]);
  rig::rigid_transform_to_array(identity, &state.right_identity_vec[0]);
}

// Must all this after optimization to update the Extrinsics structure
void fromOptState(const OptState& state, Extrinsics& extrinsics, RigSet& R,
                  bool no_rig, bool affine_depth_to_image, int num_depth_params) {
  // Convert world_to_ref transforms
  size_t num_world_to_ref = state.world_to_ref_vec.size() / rig::NUM_RIGID_PARAMS;
  extrinsics.world_to_ref.resize(num_world_to_ref);
  for (size_t cid = 0; cid < num_world_to_ref; cid++)
    rig::array_to_rigid_transform(extrinsics.world_to_ref[cid],
                                  &state.world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);
  
  // Convert world_to_cam transforms (only if no_rig)
  if (no_rig) {
    size_t num_world_to_cam = state.world_to_cam_vec.size() / rig::NUM_RIGID_PARAMS;
    extrinsics.world_to_cam.resize(num_world_to_cam);
    for (size_t cid = 0; cid < num_world_to_cam; cid++)
      rig::array_to_rigid_transform(extrinsics.world_to_cam[cid],
                                    &state.world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid]);
  }
  
  // Convert ref_to_cam transforms back to R
  size_t num_ref_to_cam = state.ref_to_cam_vec.size() / rig::NUM_RIGID_PARAMS;
  R.ref_to_cam_trans.resize(num_ref_to_cam);
  for (size_t cam_type = 0; cam_type < num_ref_to_cam; cam_type++)
    rig::array_to_rigid_transform(R.ref_to_cam_trans[cam_type],
                                  &state.ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type]);
  
  // Convert depth_to_image transforms back to R (affine or rigid)
  size_t num_depth_to_image = state.depth_to_image_vec.size() / num_depth_params;
  R.depth_to_image.resize(num_depth_to_image);
  for (size_t cam_type = 0; cam_type < num_depth_to_image; cam_type++) {
    if (affine_depth_to_image)
      rig::array_to_affine_transform(R.depth_to_image[cam_type],
                                     &state.depth_to_image_vec[num_depth_params * cam_type]);
    else
      rig::array_to_rigid_transform(R.depth_to_image[cam_type],
                                    &state.depth_to_image_vec[num_depth_params * cam_type]);
  }
  
  // Copy optimized intrinsics from state back to R
  int num_cam_types = R.cam_names.size();
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    R.cam_params[cam_type].SetFocalLength(Eigen::Vector2d(state.focal_lengths[cam_type],
                                                           state.focal_lengths[cam_type]));
    R.cam_params[cam_type].SetOpticalOffset(state.optical_centers[cam_type]);
    R.cam_params[cam_type].SetDistortion(state.distortions[cam_type]);
  }
}

} // namespace rig
