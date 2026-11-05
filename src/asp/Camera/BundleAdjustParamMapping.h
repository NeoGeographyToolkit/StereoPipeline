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

/// \file BundleAdjustParamMapping.h
/// Camera parameter packing, unpacking, initialization, and mapping for bundle adjustment.

#ifndef __BUNDLE_ADJUST_PARAM_MAPPING_H__
#define __BUNDLE_ADJUST_PARAM_MAPPING_H__

#include <asp/Core/BaBaseOptions.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/BaState.h>

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>

#include <vector>
#include <string>

namespace vw {
  class PinholeModel;
  class OpticalBarModel;
  class AdjustedCameraModel;
}

namespace asp {

// Class to manage camera adjustment parameters
class CameraAdjustment {
public:
  CameraAdjustment();
  CameraAdjustment(double const* array);

  vw::Vector3 position() const;
  vw::Quat    pose    () const;

  void read_from_array(double const* array);
  void copy_from_adjusted_camera(vw::AdjustedCameraModel const& cam);
  void copy_from_pinhole(vw::PinholeModel const& cam);
  void copy_from_optical_bar(vw::OpticalBarModel const& cam);
  void copy_from_csm(asp::CsmModel const& cam);
  void read_from_adjust_file(std::string const& filename);
  void pack_to_array(double* array) const;

private:
  vw::Vector3 m_position_data;
  vw::Quat    m_pose_data;
};

// Packs info from various camera models into the provided arrays.
void pack_pinhole_to_arrays(vw::PinholeModel const& camera,
                            int camera_index,
                            asp::BaState & ba_state);

void pack_optical_bar_to_arrays(vw::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BaState & ba_state);

void pack_csm_to_arrays(asp::CsmModel const& camera,
                        int camera_index,
                        asp::BaState & ba_state);

// Apply a transform to the camera parameters
void apply_transform_to_params(vw::Matrix4x4 const& M, asp::BaState &ba_state,
                               std::vector<vw::CamPtr> const& camera_models,
                               asp::IntrinsicOptions const& intrinsics_opts,
                               std::vector<int> const& num_dist_params);

void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BaState &ba_state,
                                        std::vector<vw::CamPtr> const& camera_models,
                                        asp::IntrinsicOptions const& intrinsics_opts,
                                        std::vector<int> const& num_dist_params);

void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BaState &ba_state,
                                            std::vector<vw::CamPtr> const& camera_models);

void apply_transform_to_cameras_csm(vw::Matrix4x4 const& M,
                                    asp::BaState &ba_state,
                                    std::vector<vw::CamPtr> const& camera_models);

void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double scale,
                           std::vector<vw::CamPtr> const& camera_models,
                           asp::BaState & ba_state);

// Read adjustments from a previous run and put them into the parameters
void put_adjustments_in_params(std::string const& input_prefix,
                               std::vector<std::string> const& image_files,
                               std::vector<std::string> const& camera_files,
                               asp::BaState & ba_state);

// Create camera models with latest adjustments applied to them
void create_corrected_cameras(std::vector<vw::CamPtr> const& input_cameras,
                              asp::BaState const& ba_state,
                              std::vector<vw::CamPtr> & out_cameras);

// Initialize camera parameter arrays
bool init_cams(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
               std::string const& initial_transform_file,
               vw::Matrix<double> const& initial_transform,
               std::vector<vw::CamPtr> & new_cam_models);

bool init_cams_pinhole(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
                       std::string const& initial_transform_file,
                       vw::Matrix<double> const& initial_transform,
                       std::vector<vw::CamPtr> & new_cam_models);

bool init_cams_optical_bar(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
                           std::string const& initial_transform_file,
                           vw::Matrix<double> const& initial_transform,
                           std::vector<vw::CamPtr> & new_cam_models);

bool init_cams_csm(asp::BaBaseOptions const& opt, asp::BaState & ba_state,
                   std::string const& initial_transform_file,
                   vw::Matrix<double> const& initial_transform,
                   std::vector<vw::CamPtr> & new_cam_models);

// Find the cameras with the latest adjustments
void calcOptimizedCameras(asp::BaBaseOptions const& opt,
                          asp::BaState const& ba_state,
                          std::vector<vw::CamPtr> & optimized_cams);

// Interface for setting/getting intrinsics for all supported camera models
void get_optical_center(vw::CameraModel const* cam, vw::Vector2 & center);
void set_optical_center(vw::CameraModel* cam, vw::Vector2 const& center);
void get_focal_length(vw::CameraModel const* cam, double & focal);
void set_focal_length(vw::CameraModel* cam, double const& focal);
void get_distortion(vw::CameraModel const* cam, vw::Vector<double> &dist);
void set_distortion(vw::CameraModel* cam, vw::Vector<double> const& dist);

// Synchronize initial shared intrinsic parameters across cameras
bool syncUpInitialSharedParams(BACameraType camera_type,
                               asp::BaState const& ba_state,
                               std::vector<vw::CamPtr>& camera_models);

// Allocate storage for distortion parameters
int calcMaxNumDistParams(std::vector<vw::CamPtr> const& camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<double> const& intrinsics_limits);

// Ensure distortion coefficients are not too small
void ensureMinDistortion(std::vector<vw::CamPtr> & camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<int> const& fixed_distortion_indices,
                         int max_num_dist_params,
                         double min_distortion);

} // namespace asp

#endif // __BUNDLE_ADJUST_PARAM_MAPPING_H__
