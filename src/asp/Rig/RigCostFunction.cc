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

#include <Rig/camera_image.h>
#include <Rig/rig_config.h>
#include <Rig/cost_function.h>
#include <Rig/transform_utils.h>
#include <iostream>

namespace rig {

// If applicable, set up the parameters block to fix the rig translations and/or rotations
void setUpFixRigOptions(bool no_rig, bool fix_rig_translations, bool fix_rig_rotations,
                        ceres::SubsetManifold*& constant_transform_manifold) {
  
  constant_transform_manifold = NULL;
  
  int beg = 0, end = 0;
  if (!no_rig && fix_rig_translations) {
    beg = 0; 
    end = 3;
  }
  
  if (!no_rig && fix_rig_rotations) {
    if (!fix_rig_translations)
      beg = 3; // only fix rotation
    end = rig::NUM_RIGID_PARAMS;
  }
  
  // Make a vector that goes from beg to end with increment 1
  std::vector<int> fixed_indices;
  for (int it = beg; it < end; it++)
    fixed_indices.push_back(it);
  
  if (!fixed_indices.empty())
    constant_transform_manifold = new ceres::SubsetManifold(rig::NUM_RIGID_PARAMS,
                                                            fixed_indices);
}

// Find pointers to the camera and reference images that bracket the
// camera image. Great care is needed here. Two cases are considered,
// if there is a rig or not. If no_rig is true, then the reference images are
// the same as the camera images. 
// TODO(oalexan1): This needs to go somewhere else.
// TODO(oalexan1): Move the rig cost functions here.
void calcBracketing(// Inputs
                  bool no_rig, int cid, int cam_type,
                  std::vector<rig::cameraImage> const& cams,
                  std::vector<double> const& ref_timestamps,
                  rig::RigSet   const& R,
                  // Will not be changed but need access
                  std::vector<double> & world_to_cam_vec,
                  std::vector<double> & world_to_ref_vec,
                  std::vector<double> & ref_to_cam_vec,
                  std::vector<double> & ref_identity_vec,
                  std::vector<double> & right_identity_vec,
                  // Outputs
                  double* & beg_cam_ptr, 
                  double* & end_cam_ptr, 
                  double* & ref_to_cam_ptr,
                  double  & beg_ref_timestamp, 
                  double  & end_ref_timestamp,
                  double  & cam_timestamp) {

  if (!no_rig) {
    // Model the rig, use timestamps
    int beg_ref_index = cams[cid].beg_ref_index;
    int end_ref_index = cams[cid].end_ref_index;

    // Left bracketing ref cam for a given cam. For a ref cam, this is itself.
    beg_cam_ptr = &world_to_ref_vec[rig::NUM_RIGID_PARAMS * beg_ref_index];

    // Right bracketing camera. When the cam is the ref type,
    // or when this cam is the last one and has exactly
    // same timestamp as the ref cam, this is not used.
    if (R.isRefSensor(R.cam_names[cam_type]) || beg_ref_index == end_ref_index)
      end_cam_ptr = &right_identity_vec[0];
    else
      end_cam_ptr = &world_to_ref_vec[rig::NUM_RIGID_PARAMS * end_ref_index];

    // The beg and end timestamps will be the same only for the
    // ref cam or for last non-ref cam whose timestamp is same
    // as ref cam timestamp which is also last.
    beg_ref_timestamp = ref_timestamps[beg_ref_index];
    end_ref_timestamp = ref_timestamps[end_ref_index];
    cam_timestamp = cams[cid].timestamp;  // uses current camera's clock

  } else {
    // No rig. Then, beg_cam_ptr is just current camera, not the
    // ref bracketing cam, end_cam_ptr is the identity. The timestamps
    // will be the same, so the camera brackets itself.
    cam_timestamp     = cams[cid].timestamp;
    beg_ref_timestamp = cam_timestamp;
    end_ref_timestamp = cam_timestamp;

    // Note how we use world_to_cam_vec and not world_to_ref_vec for 
    // the beg cam. The end cam is unused.
    beg_cam_ptr = &world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid];
    end_cam_ptr = &right_identity_vec[0];
  }

  // Transform from reference camera to given camera. Won't be used when
  // FLAGS_no_rig is true or when the cam is of ref type.
  if (no_rig || R.isRefSensor(R.cam_names[cam_type]))
    ref_to_cam_ptr = &ref_identity_vec[0];
  else
    ref_to_cam_ptr = &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type];

  return;
}

}  // end namespace rig
