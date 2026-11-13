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

/// \file BundleAdjustTransform.cc
///
/// Produce updated cameras by applying optimized parameters or a rigid
/// transform, for bundle_adjust.

#include <asp/Camera/BundleAdjustTransform.h>
#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Camera/CsmModel.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Math/Matrix.h>

#include <vector>

using namespace vw;

namespace asp {

// Given an input pinhole camera and param changes, apply those, returning
// the new camera. Note that all intrinsic parameters are stored as multipliers
// in asp::BaState.
vw::PinholeModel transformedPinholeCamera(int camera_index,
                                          asp::BaState const& ba_state,
                                          vw::PinholeModel const& in_cam) {

  // Start by making a copy of the camera. Note that this does not make a copy of the
  // distortion params, as that's a pointer. So will have to make a copy of it further down.
  vw::PinholeModel out_cam = in_cam;

  double const* pos_pose_ptr   = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr     = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose(pos_pose_info.pose());

  // Update the lens distortion parameters. The arrays hold the values
  // directly (additive). Note how we make a new copy of the distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    lens[i] = distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  out_cam.set_point_offset(Vector2(center_ptr[0], center_ptr[1]),
                           false); // do not update the internals yet
  double new_focus = focus_ptr[0];
  // At the last step, recompute the internals given the new values
  out_cam.set_focal_length(Vector2(new_focus,new_focus), true);

  return out_cam;
}

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::OpticalBarModel
transformedOpticalBarCamera(int camera_index,
                            asp::BaState const& ba_state,
                            vw::OpticalBarModel const& in_cam) {

  // Start by making a copy of the camera
  vw::OpticalBarModel out_cam = in_cam;

  double const* pos_pose_ptr  = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr    = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // Intrinsic parameters are stored as values (additive).

  // Update the other intrinsic parameters.
  out_cam.set_speed              (intrinsic_ptr[0]);
  out_cam.set_motion_compensation(intrinsic_ptr[1]);
  out_cam.set_scan_time          (intrinsic_ptr[2]);

  if (out_cam.get_have_velocity_vec()) {
    vw::Vector3 vel(intrinsic_ptr[3], intrinsic_ptr[4], intrinsic_ptr[5]);
    vw::Vector3 final_pose(intrinsic_ptr[6], intrinsic_ptr[7], intrinsic_ptr[8]);
    out_cam.set_velocity(vel);
    out_cam.set_final_pose(final_pose);
  }

  // Update the center and focus
  out_cam.set_optical_center(Vector2(center_ptr[0], center_ptr[1]));
  double new_focus = focus_ptr[0];
  out_cam.set_focal_length(new_focus);

  return out_cam;
}

// Given an input CSM camera, intrinsic and extrinsic param changes, apply
// those, returning the new camera.
boost::shared_ptr<asp::CsmModel> transformedCsmCamera(int camera_index,
                                                      asp::BaState const& ba_state,
                                                      asp::CsmModel const& in_cam) {
  // Get the latest version of the camera parameters
  double const* pos_pose_ptr  = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr    = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* dist_ptr      = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Read the position and pose
  CameraAdjustment correction(pos_pose_ptr);

  // All intrinsic parameters are stored as values (additive).
  vw::Vector2 optical_center = in_cam.optical_center();
  double focal_length        = in_cam.focal_length();
  optical_center[0] = center_ptr[0];
  optical_center[1] = center_ptr[1];
  focal_length      = focus_ptr [0];

  // Update the lens distortion parameters. The arrays hold the values
  // directly, so a zero-seeded coefficient can be optimized.
  std::vector<double> distortion = in_cam.distortion();
  for (size_t i = 0; i < distortion.size(); i++)
    distortion[i] = dist_ptr[i];

  // Duplicate the input camera model
  boost::shared_ptr<asp::CsmModel> copy;
  in_cam.deep_copy(copy);

  // Update the intrinsics of the copied model
  copy->set_optical_center(optical_center);
  copy->set_focal_length(focal_length);
  copy->set_distortion(distortion);

  // Form the adjusted camera having the updated position and pose
  AdjustedCameraModel adj_cam(copy, correction.position(), correction.pose());

  // Apply the adjustment to the camera 
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  copy->applyTransform(ecef_transform);

  // TODO(oalexan1): Test if adj cam and cam with applied adjustment return same
  // results

  return copy;
}

/// Given a transform with origin at the planet center, like output by pc_align,
/// read the adjustments from param storage, apply this transform on top of
/// them, and write the adjustments back to the param storage. Cameras
/// do not change.
void apply_transform_to_params(vw::Matrix4x4 const& M, asp::BaState &ba_state,
                                std::vector<vw::CamPtr> const& cam_ptrs) {

  for (unsigned i = 0; i < ba_state.num_cameras(); i++) {

    // Load the current position/pose of this camera.
    double* cam_ptr = ba_state.get_camera_ptr(i);
    CameraAdjustment cam_adjust(cam_ptr);

    // Create the adjusted camera model
    vw::AdjustedCameraModel cam(cam_ptrs[i], cam_adjust.position(),
                                cam_adjust.pose());
    // Apply the transform
    cam.apply_transform(M);

    // Copy back the adjustments to the camera array.
    cam_adjust.copy_from_adjusted_camera(cam);
    cam_adjust.pack_to_array(cam_ptr);
  }
} // end function apply_transform_to_cameras

// This function takes advantage of the fact that when it is called the cam_ptrs
//  have the same information as is in ba_state!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BaState & ba_state,
                                        std::vector<vw::CamPtr>
                                        const& cam_ptrs) {

  for (unsigned i = 0; i < ba_state.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<vw::PinholeModel> pin_ptr =
      boost::dynamic_pointer_cast<vw::PinholeModel>(cam_ptrs[i]);
    pin_ptr->apply_transform(M);

    // Write out to ba_state
    pack_pinhole_to_arrays(*pin_ptr, i, ba_state);
  }

} // end function apply_transform_to_cameras_pinhole

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
// information as is in ba_state.
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BaState & ba_state,
                                            std::vector<vw::CamPtr>
                                            const& cam_ptrs) {

  // Convert the transform format
  vw::Matrix3x3 R = submatrix(M, 0, 0, 3, 3);
  vw::Vector3   T;
  for (int r = 0; r < 3; r++)
    T[r] = M(r, 3);

  double scale = pow(det(R), 1.0/3.0);
  for (size_t r = 0; r < R.rows(); r++)
    for (size_t c = 0; c < R.cols(); c++)
      R(r, c) /= scale;

  for (unsigned i = 0; i < ba_state.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<vw::OpticalBarModel> bar_ptr =
      boost::dynamic_pointer_cast<vw::OpticalBarModel>(cam_ptrs[i]);
    bar_ptr->apply_transform(R, T, scale);

    // Write out to ba_state
    pack_optical_bar_to_arrays(*bar_ptr, i, ba_state);
  }

} // end function apply_transform_to_cameras_optical_bar

// This function takes advantage of the fact that when it is called the cam_ptrs
//  have the same information as is in ba_state.
// This applies the transform to the camera inline, but does not copy
// the camera position and orientation to the arrays.
void apply_transform_to_cameras_csm(vw::Matrix4x4 const& M,
                                    asp::BaState & ba_state,
                                    std::vector<vw::CamPtr>
                                    const& cam_ptrs) {
  for (unsigned i = 0; i < ba_state.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<asp::CsmModel> csm_ptr =
      boost::dynamic_pointer_cast<asp::CsmModel>(cam_ptrs[i]);
    if (csm_ptr == NULL)
        vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");
    csm_ptr->applyTransform(M);
    // Write out to ba_state. This does not copy camera position 
    // and orientation. The adjustment stays as 0 pose and identity rotation.
    // That is why the transform M does not get applied twice.
    pack_csm_to_arrays(*csm_ptr, i, ba_state);
  }

} // end function apply_transform_to_cameras_csm

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<vw::CamPtr> &camera_models,
                           boost::shared_ptr<vw::ControlNetwork> const& cnet) {

  // Apply the transform to the cameras
  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    vw::PinholeModel * pincam
      = dynamic_cast<vw::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

    pincam->apply_transform(rotation, translation, scale);
  } // End loop through cameras

  // Apply the transform to all of the world points in the ControlNetwork
  ControlNetwork::iterator iter;
  for (iter = cnet->begin(); iter != cnet->end(); iter++) {
    if (iter->type() == ControlPoint::GroundControlPoint)
      continue; // Don't convert the ground control points!

    Vector3 position     = iter->position();
    Vector3 new_position = scale*rotation*position + translation;
    iter->set_position(new_position);
  }
} // End function ApplyRigidTransform

} // end namespace asp
