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

/// \file BundleAdjustParamMap.cc
/// Camera parameter packing, unpacking, initialization, and mapping for bundle adjustment.

#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/FileUtils.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

using namespace vw;

namespace asp {

CameraAdjustment::CameraAdjustment() {}
CameraAdjustment::CameraAdjustment(double const* array) {
  this->read_from_array(array);
}

// Data access
vw::Vector3 CameraAdjustment::position() const { return m_position_data; }
vw::Quat    CameraAdjustment::pose    () const { return m_pose_data;     }

/// Populate from a six element array.
void CameraAdjustment::read_from_array(double const* array) {
  m_position_data = vw::Vector3(array[0], array[1], array[2]);
  m_pose_data     = vw::axis_angle_to_quaternion(vw::Vector3(array[3], array[4], array[5]));
}

/// Populate from an AdjustedCameraModel
void CameraAdjustment::copy_from_adjusted_camera
  (vw::AdjustedCameraModel const& cam) {
  m_position_data = cam.translation();
  m_pose_data     = cam.rotation();
}

/// Populate from a PinholeModel
void CameraAdjustment::copy_from_pinhole(vw::PinholeModel const& cam) {
  m_position_data = cam.camera_center();
  m_pose_data     = cam.camera_pose();
}

/// Populate from OpticalBarModel
void CameraAdjustment::copy_from_optical_bar(vw::OpticalBarModel const& cam) {
  m_position_data = cam.camera_center();
  m_pose_data     = cam.camera_pose();
}

/// Populate from CSM. Since with CSM we apply adjustments to existing
/// cameras, these start as 0.
void CameraAdjustment::copy_from_csm(asp::CsmModel const& cam) {
  // Zero position and identity rotation
  m_position_data = vw::Vector3(0, 0, 0);
  vw::Matrix3x3 I;
  I.set_identity();
  m_pose_data = vw::Quat(I);
}

/// Populate from an adjustment file on disk.
void CameraAdjustment::read_from_adjust_file(std::string const& filename) {

  // Not used, just for the api
  vw::Vector2 pixel_offset = vw::Vector2();
  double scale = 1.0;

  vw::vw_out() << "Reading camera adjustment: " << filename << std::endl;
  asp::read_adjustments(filename, m_position_data, m_pose_data,
                        pixel_offset, scale);
}

/// Pack the data to a six element array.
void CameraAdjustment::pack_to_array(double* array) const {
  vw::Vector3 pose_vec = m_pose_data.axis_angle();
  const int VEC_SIZE = 3;
  for (size_t i = 0; i < VEC_SIZE; i++) {
    array[i           ] = m_position_data[i];
    array[i + VEC_SIZE] = pose_vec[i];
  }
}

void pack_pinhole_to_arrays(vw::PinholeModel const& camera,
                            int camera_index,
                            asp::BaState & ba_state) {

  double* pos_pose_ptr   = ba_state.get_camera_ptr              (camera_index);
  double* center_ptr     = ba_state.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double* distortion_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_pinhole(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We solve for the intrinsic values directly (additive). Seed each at the
  // camera's own value, so a zero coefficient can still be optimized, as the
  // gradient does not depend on the seed.
  vw::Vector2 center = camera.point_offset();
  vw::Vector2 focus  = camera.focal_length();
  center_ptr[0] = center[0];
  center_ptr[1] = center[1];
  focus_ptr [0] = focus[0];
  // Lens distortion
  vw::Vector<double> lens = camera.lens_distortion()->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    distortion_ptr[i] = lens[i];
}

void pack_optical_bar_to_arrays(vw::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BaState & ba_state) {

  double* pos_pose_ptr   = ba_state.get_camera_ptr              (camera_index);
  double* center_ptr     = ba_state.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double* intrinsics_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_optical_bar(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We solve for the intrinsic values directly (additive). Seed each at the
  // camera's own value.
  vw::Vector2 center = camera.get_optical_center();
  center_ptr[0] = center[0];
  center_ptr[1] = center[1];
  focus_ptr [0] = camera.get_focal_length();
  // Motion state, stored in the distortion slot:
  // [speed, MCF, scan_time, velocity(3), final_pose(3)].
  intrinsics_ptr[0] = camera.get_speed();
  intrinsics_ptr[1] = camera.get_motion_compensation();
  intrinsics_ptr[2] = camera.get_scan_time();
  vw::Vector3 vel(0, 0, 0), final_pose(0, 0, 0);
  if (camera.get_have_velocity_vec()) {
    vel        = camera.get_velocity();
    final_pose = camera.get_final_pose();
  }
  intrinsics_ptr[3] = vel[0];
  intrinsics_ptr[4] = vel[1];
  intrinsics_ptr[5] = vel[2];
  intrinsics_ptr[6] = final_pose[0];
  intrinsics_ptr[7] = final_pose[1];
  intrinsics_ptr[8] = final_pose[2];
}

// This does not copy the camera position and orientation
void pack_csm_to_arrays(asp::CsmModel const& camera,
                        int camera_index,
                        asp::BaState & ba_state) {

  double* pos_pose_ptr   = ba_state.get_camera_ptr              (camera_index);
  double* center_ptr     = ba_state.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double* distortion_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose. We start with 0 pose and identity rotation. Nothing
  // gets copied from camera position and orientation.
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_csm(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We solve for the intrinsic values directly (additive). Seed each at the
  // camera's own value, so a zero coefficient can still be optimized.
  vw::Vector2 center = camera.optical_center();
  center_ptr[0] = center[0];
  center_ptr[1] = center[1];
  focus_ptr [0] = camera.focal_length();
  // Distortion
  std::vector<double> dist = camera.distortion();
  for (size_t i = 0; i < dist.size(); i++)
    distortion_ptr[i] = dist[i];
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

/// Warn if the GCP are far from the tie points, to catch the common lat/lon
/// swap in a GCP file. Compare the bounding box of the GCP with the bounding
/// box of the tie points. Warn if these are more than 100 km apart. The cnet
/// must already be triangulated, which is the case at the call site.

void put_adjustments_in_params(std::string const& input_prefix,
                               std::vector<std::string> const& image_files,
                               std::vector<std::string> const& camera_files,
                               // Output
                               asp::BaState & ba_state) {

  const size_t num_cameras = ba_state.num_cameras();

  for (size_t icam = 0; icam < num_cameras; icam++) {
    std::string adjust_file
      = asp::bundle_adjust_file_name(input_prefix, image_files[icam], camera_files[icam]);

    double * cam_ptr = ba_state.get_camera_ptr(icam);
    CameraAdjustment adjustment;
    adjustment.read_from_adjust_file(adjust_file);
    adjustment.pack_to_array(cam_ptr);
  }
}

// Take input cameras and corrections in ba_state, and create new cameras
// incorporating the corrections. 
void create_corrected_cameras(std::vector<vw::CamPtr> const& input_cameras,
                              asp::BaState const& ba_state,
                              std::vector<vw::CamPtr> & out_cameras) {
  const size_t num_cameras = ba_state.num_cameras();
  out_cameras.resize(num_cameras);

  for (size_t icam = 0; icam < num_cameras; icam++) {
    CameraAdjustment correction(ba_state.get_camera_ptr(icam));
    // The pointer is managed by vw::CamPtr
    out_cameras[icam] = vw::CamPtr(new vw::AdjustedCameraModel(input_cameras[icam],
                                   correction.position(), correction.pose()));
  }
}

/// Create the param storage. Collect in it any input adjustments and initial transform.
/// Return a copy of the cameras having these adjustments applied to them.
bool init_cams(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
       std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
       std::vector<vw::CamPtr> & new_cam_models) {

  bool cameras_changed = false;

  // Initialize all of the camera adjustments to zero.
  ba_state.init_cams_as_zero();
  const size_t num_cameras = ba_state.num_cameras();

  // Sanity check, must have same number of cameras
  if (num_cameras != opt.camera_models.size())
      vw_throw(ArgumentErr() << "Expecting " << num_cameras << " cameras, got "
                           << opt.camera_models.size() << ".\n");

  // Read the adjustments from a previous run, if present. Put them in params.
  if (opt.input_prefix != "") {
    put_adjustments_in_params(opt.input_prefix, opt.image_files, opt.camera_files,
                              ba_state); // output
    cameras_changed = true;
  }

  // Apply any initial transform to params
  if (initial_transform_file != "") {
    if (opt.stereo_session == "csm") {
      double scale = pow(vw::det(initial_transform), 1.0/3.0);
      if (std::abs(scale - 1.0) > 1e-6) {
        // TODO(oalexan1): This gives wrong results for now so needs to be sorted out.
        // Likely the only way to apply a scale to a linescan camera is to multiply
        // all camera centers by the scale. Using a rotation and translation center
        // like for AdjustedCameraModel is not enough.
        vw_throw(ArgumentErr()
                 << "CSM camera models do not support applying a transform with a scale.\n");
      }
    }

    // Update ba_state with the alignment. This may be on top of any initial adjustment.
    // from the previous code, already contained in ba_state. Cameras
    // do not change.
    apply_transform_to_params(initial_transform, ba_state, opt.camera_models);
    cameras_changed = true;
  }

  // Make a copy of the cameras with given corrections in ba_state, incorporating
  // any adjustments and initial transform.
  create_corrected_cameras(opt.camera_models, ba_state, new_cam_models);

  return cameras_changed;
}

/// Specialization for pinhole cameras
bool init_cams_pinhole(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
     std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
     std::vector<vw::CamPtr> & new_cam_models) {

  bool cameras_changed = false;

  // Copy the camera parameters from the models to ba_state
  const size_t num_cameras = ba_state.num_cameras();

  for (int icam = 0; icam < num_cameras; icam++) {
    PinholeModel* pin_ptr = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());

    // Make a deep copy of the camera, including of the lens distortion
    PinholeModel pin_cam = *pin_ptr;

    // Read the adjustments from a previous run, if present
    if (opt.input_prefix != "") {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << "\n";
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);

      // Strictly speaking, it is not necessary to call unadjusted_model(), as
      // in bundle_adjust the input cameras are loaded unadjusted, unlike in stereo.
      AdjustedCameraModel adj_cam(vw::unadjusted_model(opt.camera_models[icam]),
                                  adjustment.position(), adjustment.pose());
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      pin_cam.apply_transform(ecef_transform);

      cameras_changed = true;
    }

    // Apply any initial transform to the pinhole cameras. This may be on top
    // of any initial adjustment. 
    if (initial_transform_file != "") {
      pin_cam.apply_transform(initial_transform);
      cameras_changed = true;
    }

    pack_pinhole_to_arrays(pin_cam, icam, ba_state);
  } // End loop through cameras

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++) {

    PinholeModel* in_cam  = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());

    // Start with a deep copy of the input camera. Then overwrite its parameters.
    PinholeModel* out_cam = new PinholeModel(*in_cam); // deep copy, memory managed below
    *out_cam = transformedPinholeCamera(icam, ba_state, *in_cam);
    new_cam_models[icam] = vw::CamPtr(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for optical bar cameras.
bool init_cams_optical_bar(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
                    std::string const& initial_transform_file,
                    vw::Matrix<double> const& initial_transform,
                    std::vector<vw::CamPtr> &new_cam_models) {

  if (opt.input_prefix != "")
    vw::vw_throw(vw::ArgumentErr()
                 << "Applying initial adjustments to optical bar cameras "
                 << "and --inline-adjustments is not implemented. "
                 << "Remove this option.\n");

  bool cameras_changed = false;

  // Copy the camera parameters from the models to ba_state
  const size_t num_cameras = ba_state.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    vw::OpticalBarModel* bar_ptr
      = dynamic_cast<vw::OpticalBarModel*>(opt.camera_models[icam].get());
    pack_optical_bar_to_arrays(*bar_ptr, icam, ba_state);
  } // End loop through cameras

  // Apply any initial transform to the pinhole cameras
  if (initial_transform_file != "") {
    apply_transform_to_cameras_optical_bar(initial_transform, ba_state,
                                           opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++) {

    vw::OpticalBarModel* in_cam
      = dynamic_cast<vw::OpticalBarModel*>(opt.camera_models[icam].get());

    // Start with a copy of the input camera, then overwrite its content.
    // The pointer is managed below.
    vw::OpticalBarModel* out_cam = new vw::OpticalBarModel(*in_cam);
    *out_cam = transformedOpticalBarCamera(icam, ba_state, *in_cam);
    new_cam_models[icam] = boost::shared_ptr<vw::CameraModel>(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for CSM
bool init_cams_csm(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
                   std::string const& initial_transform_file,
                   vw::Matrix<double> const& initial_transform,
                   std::vector<vw::CamPtr> &new_cam_models) {

  bool cameras_changed = false;

  // Apply any adjustments inline. Copy the camera parameters from the models to
  // ba_state. Do not copy the adjustments, as they are already applied
  // to the camera proper.
  const size_t num_cameras = ba_state.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    asp::CsmModel* csm_ptr
        = dynamic_cast<asp::CsmModel*>(opt.camera_models[icam].get());

    // Read the adjustments from a previous run, if present. Apply them
    // inline to the camera model.
    if (opt.input_prefix != "") {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << "\n";
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);

      // Strictly speaking, it is not necessary to call unadjusted_model(), as
      // in bundle_adjust the input cameras are loaded unadjusted, unlike in stereo.
      AdjustedCameraModel adj_cam(vw::unadjusted_model(opt.camera_models[icam]),
                                  adjustment.position(), adjustment.pose());
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      csm_ptr->applyTransform(ecef_transform);

      cameras_changed = true;
    }

    // This does not copy the camera position and orientation, only the intrinsics    
    pack_csm_to_arrays(*csm_ptr, icam, ba_state);
  } // End loop through cameras

  // Apply any initial transform to the CSM cameras
  if (initial_transform_file != "") {
    // Apply the transform to the cameras inline. This modifies opt.camera_models.
    // The transform does not get copied to ba_state. Only intrinsics get copied.
    apply_transform_to_cameras_csm(initial_transform, ba_state, opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++) {

    asp::CsmModel* in_cam
      = dynamic_cast<asp::CsmModel*>(opt.camera_models[icam].get());
    if (in_cam == NULL)
      vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");

    auto out_cam = transformedCsmCamera(icam, ba_state, *in_cam);
    new_cam_models[icam] = boost::shared_ptr<vw::CameraModel>(out_cam);
  }

  return cameras_changed;
}

// When distortion params are shared, their number must agree

void calcOptimizedCameras(asp::BaBaseOptions const& opt,
                          asp::BaState const& ba_state,
                          std::vector<vw::CamPtr> & optimized_cams) {

  optimized_cams.clear();

  int num_cameras = opt.image_files.size();
  for (int icam = 0; icam < num_cameras; icam++) {

    // TODO(oalexan1): The logic below may need to be a function and should be called
    // in a couple other places.
    switch (opt.camera_type) {
    case BaCameraType_Pinhole:
      {
        vw::PinholeModel const* in_cam
          = dynamic_cast<vw::PinholeModel const*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
        vw::PinholeModel * out_cam = new PinholeModel(); // pointer managed below
        *out_cam = transformedPinholeCamera(icam, ba_state, *in_cam);
        optimized_cams.push_back(vw::CamPtr(out_cam));
      }
      break;
    case BaCameraType_OpticalBar:
      {
        vw::OpticalBarModel const* in_cam
          = dynamic_cast<vw::OpticalBarModel const*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting an optical bar camera.\n");
        vw::OpticalBarModel * out_cam = new OpticalBarModel(); // pointer managed below
        *out_cam = transformedOpticalBarCamera(icam, ba_state, *in_cam);
        optimized_cams.push_back(vw::CamPtr(out_cam)); // will manage the memory
      }
      break;
    case  BaCameraType_CSM:
      {
        asp::CsmModel const* in_cam
          = dynamic_cast<asp::CsmModel const*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");
        auto out_cam = transformedCsmCamera(icam, ba_state, *in_cam);
        optimized_cams.push_back(out_cam);
      }
      break;
    case BaCameraType_Other:
      {
        CameraAdjustment cam_adjust(ba_state.get_camera_ptr(icam));
        // The pointer is managed by the smart pointer vw::CamPtr
        vw::CamPtr out_cam
          (new AdjustedCameraModel(vw::unadjusted_model(opt.camera_models[icam]),
                                          cam_adjust.position(), cam_adjust.pose()));
        optimized_cams.push_back(out_cam);
      }
      break;
    default:
      vw_throw(ArgumentErr() << "Unknown camera type.\n");
    }
  }
}

// Find the average for the gsd for all pixels whose rays intersect at the given
// triangulated point.
// TODO(oalexan1): Export points out of ba_state and crn, then use the 
// other function further down instead.

void get_optical_center(vw::CameraModel const* cam, vw::Vector2 & center) {

  // Cast to pinhole model
  vw::PinholeModel const* pin_ptr
    = dynamic_cast<vw::PinholeModel const*>(cam);
  if (pin_ptr != NULL) {
    center = pin_ptr->point_offset();
    return;
  }

  // Cast to optical bar model
  vw::OpticalBarModel const* bar_ptr
    = dynamic_cast<vw::OpticalBarModel const*>(cam);
  if (bar_ptr != NULL) {
    center = bar_ptr->get_optical_center();
    return;
  }

  // Cast to CSM model
  asp::CsmModel const* csm_ptr = dynamic_cast<asp::CsmModel const*>(cam);
  if (csm_ptr != NULL) {
    center = csm_ptr->optical_center();
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in get_optical_center().\n");
}

void set_optical_center(vw::CameraModel* cam, vw::Vector2 const& center) {

  // Cast to pinhole model
  vw::PinholeModel* pin_ptr
    = dynamic_cast<vw::PinholeModel*>(cam);
  if (pin_ptr != NULL) {
    pin_ptr->set_point_offset(center);
    return;
  }

  // Cast to optical bar model
  vw::OpticalBarModel* bar_ptr
    = dynamic_cast<vw::OpticalBarModel*>(cam);
  if (bar_ptr != NULL) {
    bar_ptr->set_optical_center(center);
    return;
  }

  // Cast to CSM model
  asp::CsmModel* csm_ptr = dynamic_cast<asp::CsmModel*>(cam);
  if (csm_ptr != NULL) {
    csm_ptr->set_optical_center(center);
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in set_optical_center().\n");
}

// Get the focal length
void get_focal_length(vw::CameraModel const* cam, double & focal) {

  // Cast to pinhole model
  vw::PinholeModel const* pin_ptr
    = dynamic_cast<vw::PinholeModel const*>(cam);
  if (pin_ptr != NULL) {
    focal = pin_ptr->focal_length()[0];
    return;
  }

  // Cast to optical bar model
  vw::OpticalBarModel const* bar_ptr
    = dynamic_cast<vw::OpticalBarModel const*>(cam);
  if (bar_ptr != NULL) {
    focal = bar_ptr->get_focal_length();
    return;
  }

  // Cast to CSM model
  asp::CsmModel const* csm_ptr = dynamic_cast<asp::CsmModel const*>(cam);
  if (csm_ptr != NULL) {
    focal = csm_ptr->focal_length();
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in get_focal_length().\n");
}

// Set the focal length
void set_focal_length(vw::CameraModel* cam, double const& focal) {

  // Cast to pinhole model
  vw::PinholeModel* pin_ptr
    = dynamic_cast<vw::PinholeModel*>(cam);
  if (pin_ptr != NULL) {
    pin_ptr->set_focal_length(vw::Vector2(focal, focal));
    return;
  }

  // Cast to optical bar model
  vw::OpticalBarModel* bar_ptr
    = dynamic_cast<vw::OpticalBarModel*>(cam);
  if (bar_ptr != NULL) {
    bar_ptr->set_focal_length(focal);
    return;
  }

  // Cast to CSM model
  asp::CsmModel* csm_ptr = dynamic_cast<asp::CsmModel*>(cam);
  if (csm_ptr != NULL) {
    csm_ptr->set_focal_length(focal);
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in set_focal_length().\n");
}

void get_distortion(vw::CameraModel const* cam, vw::Vector<double> &dist) {

  // Cast to pinhole model
  vw::PinholeModel const* pin_ptr
    = dynamic_cast<vw::PinholeModel const*>(cam);
  if (pin_ptr != NULL) {
    dist = pin_ptr->lens_distortion()->distortion_parameters();
    return;
  }

  // Cast to optical bar model
  // TODO(oalexan1): This must be member function and called in a couple of other places.
  vw::OpticalBarModel const* bar_ptr
    = dynamic_cast<vw::OpticalBarModel const*>(cam);
  if (bar_ptr != NULL) {
    dist.set_size(asp::NUM_OPTICAL_BAR_EXTRA_PARAMS);
    dist[0] = bar_ptr->get_speed();
    dist[1] = bar_ptr->get_motion_compensation();
    dist[2] = bar_ptr->get_scan_time();

    if (bar_ptr->get_have_velocity_vec()) {
      vw::Vector3 vel = bar_ptr->get_velocity();
      vw::Vector3 final_pose = bar_ptr->get_final_pose();
      dist[3] = vel[0];
      dist[4] = vel[1];
      dist[5] = vel[2];
      dist[6] = final_pose[0];
      dist[7] = final_pose[1];
      dist[8] = final_pose[2];
    } else {
      // Ensure initialization
      dist[3] = 0.0;
      dist[4] = 0.0;
      dist[5] = 0.0;
      dist[6] = 0.0;
      dist[7] = 0.0;
      dist[8] = 0.0;
    }

    return;
  }

  // Cast to CSM model
  asp::CsmModel const* csm_ptr = dynamic_cast<asp::CsmModel const*>(cam);
  if (csm_ptr != NULL) {
    std::vector<double> csm_dist = csm_ptr->distortion();
    dist.set_size(csm_dist.size());
    for (size_t i = 0; i < csm_dist.size(); i++)
      dist[i] = csm_dist[i];
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in get_distortion().\n");
}

// Set the distortion. The underlying model must already have distortion of this size.
void set_distortion(vw::CameraModel* cam, vw::Vector<double> const& dist) {

  // Cast to pinhole model
  vw::PinholeModel* pin_ptr
    = dynamic_cast<vw::PinholeModel*>(cam);
  if (pin_ptr != NULL) {
    boost::shared_ptr<LensDistortion> distortion = pin_ptr->lens_distortion()->copy();
    if (dist.size() != distortion->distortion_parameters().size())
      vw::vw_throw(vw::ArgumentErr() << "Expecting "
                   << distortion->distortion_parameters().size()
                   << " distortion parameters for a pinhole model.\n");
    distortion->set_distortion_parameters(dist);
    pin_ptr->set_lens_distortion(distortion.get());
    return;
  }

  // Cast to optical bar model
  // TODO(oalexan1): This must be a member function and called in a couple of other places.
  vw::OpticalBarModel* bar_ptr
    = dynamic_cast<vw::OpticalBarModel*>(cam);
  if (bar_ptr != NULL) {
    if (dist.size() != asp::NUM_OPTICAL_BAR_EXTRA_PARAMS)
      vw::vw_throw(vw::ArgumentErr() << "Expecting " << asp::NUM_OPTICAL_BAR_EXTRA_PARAMS
                   << " distortion parameters for an optical bar model.\n");
    bar_ptr->set_speed(dist[0]);
    bar_ptr->set_motion_compensation(dist[1]);
    bar_ptr->set_scan_time(dist[2]);

    if (bar_ptr->get_have_velocity_vec()) {
      vw::Vector3 vel(dist[3], dist[4], dist[5]);
      vw::Vector3 final_pose(dist[6], dist[7], dist[8]);
      bar_ptr->set_velocity(vel);
      bar_ptr->set_final_pose(final_pose);
    }

    return;
  }

  // Cast to CSM model
  asp::CsmModel* csm_ptr = dynamic_cast<asp::CsmModel*>(cam);
  if (csm_ptr != NULL) {
    std::vector<double> csm_dist = csm_ptr->distortion();
    if (dist.size() != csm_dist.size())
      vw::vw_throw(vw::ArgumentErr() << "Expecting " << csm_dist.size()
                   << " distortion parameters for a CSM model.\n");

    csm_dist.clear();
    for (size_t i = 0; i < dist.size(); i++)
      csm_dist.push_back(dist[i]);
    csm_ptr->set_distortion(csm_dist);
    return;
  }

  // If we get here, the camera type is not recognized
  vw::vw_throw(vw::ArgumentErr() << "Unknown camera type in set_distortion().\n");
}

// If some cameras share an intrinsic parameter, that parameter must start with
// the same value for all cameras sharing it. This is a bugfix.
// Return true if the cameras were changed.
bool syncUpInitialSharedParams(BACameraType camera_type,
                               asp::BaState const& ba_state,
                               std::vector<vw::CamPtr>& camera_models) {

  bool cameras_changed = false;

  if (camera_type != BaCameraType_Pinhole && camera_type != BaCameraType_OpticalBar &&
      camera_type != BaCameraType_CSM)
    return cameras_changed; // Not applicable as not optimizing intrinsics

  // Create groups of cameras that share focal length, optical center, and distortion.
  std::map<int, std::set<int>> focal_groups, center_groups, distortion_groups;
  for (size_t icam = 0; icam < ba_state.num_cameras(); icam++) {
    int focal_group      = ba_state.get_focus_offset(icam);
    int center_group     = ba_state.get_center_offset(icam);
    int distortion_group = ba_state.get_distortion_offset(icam);
    focal_groups     [focal_group].insert(icam);
    center_groups    [center_group].insert(icam);
    distortion_groups[distortion_group].insert(icam);
  }

  // Iterate over focus groups. Get the focus. Apply it to all cameras in the group.
  // Print a warning if the focus is not the same for all cameras in the group.
  bool printed_focal_warning = false;
  for (auto const& group : focal_groups) {
    int focus_group = group.first;
    std::set<int> const& cams = group.second;
    int first_index = *cams.begin();
    double focal_length = -1.0;
    get_focal_length(camera_models[first_index].get(), focal_length);
    for (int icam: cams) {
      double curr_focal_length = -1.0;
      get_focal_length(camera_models[icam].get(), curr_focal_length);
      if (curr_focal_length != focal_length) {
        if (!printed_focal_warning) {
          vw::vw_out(vw::WarningMessage)
            << "Found cameras sharing a focal length, but different focal "
            "lengths. Copying the value from the first camera in each group.\n";
          printed_focal_warning = true;
          cameras_changed = true;
        }
        set_focal_length(camera_models[icam].get(), focal_length);
      }
    }
  }
  // Same for optical center
  bool printed_center_warning = false;
  for (auto const& group: center_groups) {
    int center_group = group.first;
    std::set<int> const& cams = group.second;
    int first_index = *cams.begin();
    vw::Vector2 optical_center = Vector2(-1, -1);
    get_optical_center(camera_models[first_index].get(), optical_center);
    for (int icam: cams) {
      Vector2 curr_optical_center = Vector2(-1, -1);
      get_optical_center(camera_models[icam].get(), curr_optical_center);
      if (curr_optical_center != optical_center) {
        if (!printed_center_warning) {
          vw::vw_out(vw::WarningMessage)
            << "Found cameras sharing an optical center, but different centers. "
            "Copying the value from the first camera in each group.\n";
          printed_center_warning = true;
          cameras_changed = true;
        }
        set_optical_center(camera_models[icam].get(), optical_center);
      }
    }
  }

  // Same for distortion
  bool printed_distortion_warning = false;
  for (auto const& group: distortion_groups) {
    int distortion_group = group.first;
    std::set<int> const& cams = group.second;
    int first_index = *cams.begin();
    vw::Vector<double> distortion_params;
    get_distortion(camera_models[first_index].get(), distortion_params);
    for (int icam: cams) {
      vw::Vector<double> curr_distortion_params;
      get_distortion(camera_models[icam].get(), curr_distortion_params);
      if (curr_distortion_params != distortion_params) {
        if (!printed_distortion_warning) {
          vw::vw_out(vw::WarningMessage)
            << "Found cameras sharing a distortion model, but different distortion "
            "parameters. Copying the value from the first camera in each group.\n";
          printed_distortion_warning = true;
          cameras_changed = true;
        }
        set_distortion(camera_models[icam].get(), distortion_params);
      }
    }
  }

  return cameras_changed;
}

// This is needed to allocate enough storage for the distortion parameters.
int calcMaxNumDistParams(std::vector<vw::CamPtr> const& camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<double> const& intrinsics_limits) {

  int num_cameras = camera_models.size();
  std::vector<int> num_dist_params(num_cameras, 0);
  for (size_t cam_it  = 0; cam_it < camera_models.size(); cam_it++) {
    if (camera_type == BaCameraType_Pinhole) {
      auto pin_ptr = boost::dynamic_pointer_cast<vw::PinholeModel>
                      (camera_models[cam_it]);
      if (!pin_ptr)
        vw_throw(ArgumentErr() << "Expecting a Pinhole camera.\n");
      num_dist_params[cam_it]
        = pin_ptr->lens_distortion()->distortion_parameters().size();
    } else if (camera_type == BaCameraType_OpticalBar) {
      num_dist_params[cam_it] = asp::NUM_OPTICAL_BAR_EXTRA_PARAMS;
    } else if (camera_type == BaCameraType_CSM) {
      auto csm_ptr = boost::dynamic_pointer_cast<asp::CsmModel>(camera_models[cam_it]);
      if (!csm_ptr)
        vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");
      num_dist_params[cam_it] = csm_ptr->distortion().size();
    } else if (camera_type == BaCameraType_Other) {
      num_dist_params[cam_it] = 0; // distortion does not get handled
    } else {
      vw_throw(ArgumentErr() << "Unknown camera type.\n");
    }

    // For the case where the camera has zero distortion parameters, use one
    // dummy parameter just so we don't have to change the parameter block logic
    // later on.
    // TODO(oalexan1): Test with a mix of cameras with and without distortion.
    if (camera_type != BaCameraType_Other && num_dist_params[cam_it] < 1)
      num_dist_params[cam_it] = 1;
  }

  // It is simpler to allocate the same number of distortion params per camera
  // even if some cameras have fewer. The extra ones won't be used. 
  int max_num_dist_params =
    *std::max_element(num_dist_params.begin(), num_dist_params.end());

  asp::distortion_sanity_check(num_dist_params, intrinsics_opts,
                               intrinsics_limits);

  return max_num_dist_params;
}

// This is needed to ensure distortion coefficients are not so small
// that they don't get optimized. This modifies the camera models in place.
// This does not change indices given by --fixed-distortion-indices.
void ensureMinDistortion(std::vector<vw::CamPtr> & camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<int> const& fixed_distortion_indices,
                         int max_num_dist_params,
                         double min_distortion) {

  int num_cameras = camera_models.size();
  bool message_printed = false;

  // Check that all distortion indices are between 0 and max_num_dist_params-1.
  for (size_t i = 0; i < fixed_distortion_indices.size(); i++) {
    if (fixed_distortion_indices[i] < 0 ||
        fixed_distortion_indices[i] >= max_num_dist_params)
      vw_throw(ArgumentErr() << "The values in --fixed-distortion-indices "
               << "are out of range given the number of distortion coefficients.\n");
  }

  // Put fixed_distortion_indices in a set for faster searching
  std::set<int> fixed_dist_set;
  for (size_t i = 0; i < fixed_distortion_indices.size(); i++)
    fixed_dist_set.insert(fixed_distortion_indices[i]);

  for (size_t cam_it  = 0; cam_it < camera_models.size(); cam_it++) {

    // See if this logic is needed
    if (camera_type == BaCameraType_Pinhole) {

      auto pin_ptr = boost::dynamic_pointer_cast<vw::PinholeModel>
                      (camera_models[cam_it]);
      if (!pin_ptr)
        vw_throw(ArgumentErr() << "Expecting a Pinhole camera.\n");

    } else if (camera_type == BaCameraType_CSM) {

      auto csm_ptr = boost::dynamic_pointer_cast<asp::CsmModel>(camera_models[cam_it]);
      if (!csm_ptr)
        vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");

      // Non radtan cameras have distortion that is not normalized by focal
      // length. Better not mess with it.  
      DistortionType dist_type = csm_ptr->distortion_type();
      if (dist_type != DistortionType::RADTAN)
        continue;

    } else if (camera_type == BaCameraType_OpticalBar) {
      continue; // likely not needed for optical bar
    } else if (camera_type == BaCameraType_Other) {
      continue; // distortion does not get handled
    } else {
      vw_throw(ArgumentErr() << "Unknown camera type.\n");
    }

    if (!intrinsics_opts.float_distortion_params(cam_it))
      continue; // distortion is not being optimized for this camera

    // Adjust the distortion parameters, unless they are fixed, via
    // --fixed-distortion-indices.
    vw::Vector<double> dist_params;
    asp::get_distortion(camera_models[cam_it].get(), dist_params);
    for (size_t i = 0; i < dist_params.size(); i++) {
      if (std::abs(dist_params[i]) < std::abs(min_distortion) &&
          !fixed_dist_set.count(i)) {
        dist_params[i] = min_distortion;
        if (!message_printed) {
          vw::vw_out(vw::WarningMessage)
            << "Setting distortion parameters to at least " << min_distortion
            << " (option --min-distortion) to ensure they are optimized.\n";
            message_printed = true;
        }
      }
    }
    asp::set_distortion(camera_models[cam_it].get(), dist_params);

  } // end loop through cameras

  return;
}

// Sanity check. This does not prevent the user from setting the wrong datum,
// but it can catch unreasonable height values for GCP.

} // end namespace asp
