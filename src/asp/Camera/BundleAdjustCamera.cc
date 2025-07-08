// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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

/// \file BundleAdjustCamera.cc
///

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>

#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/KML.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Functors.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/LensDistortion.h>

#include <boost/random/uniform_int_distribution.hpp>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

namespace asp {

// Control per each group of cameras or for all cameras which intrinsics
// should be floated.
bool asp::IntrinsicOptions::float_optical_center(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_center[sensor_id];
}

bool asp::IntrinsicOptions::float_focal_length(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_focus[sensor_id];
}

bool asp::IntrinsicOptions::float_distortion_params(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_distortion[sensor_id];
}

// Constructor
asp::BAParams::BAParams(int num_points, int num_cameras,
          // Parameters below here only apply to pinhole models.
          bool using_intrinsics,
          int max_num_dist_params,
          IntrinsicOptions intrinsics_opts): 
    m_num_points        (num_points),
    m_num_cameras       (num_cameras),
    m_params_per_point  (PARAMS_PER_POINT),
    m_num_pose_params   (NUM_CAMERA_PARAMS),
    m_num_shared_intrinsics    (0),
    m_num_intrinsics_per_camera(0),
    m_max_num_dist_params(max_num_dist_params),
    m_focus_offset(0),
    m_distortion_offset(0),
    m_intrinsics_opts    (intrinsics_opts),
    m_points_vec        (num_points * PARAMS_PER_POINT,  0),
    m_cameras_vec       (num_cameras * NUM_CAMERA_PARAMS, 0),
    m_intrinsics_vec    (0),
    m_outlier_points_vec(num_points, false),
    m_rand_gen(boost::random::mt19937()) {

    if (!using_intrinsics)
      return; // If we are not using intrinsics, nothing else to do.

    // Calculate how many values are stored per-camera, and
    //  what the offset is to a particular intrinsic value.
    // - The start of the array is always an entry for each intrinsic in case
    //   it is shared.
    // The logic is very different when intrinsics are shared per sensor,
    // rather than not shared at all or all shared.
    // TODO(oalexan1): Integrate this logic.
    if (!intrinsics_opts.share_intrinsics_per_sensor) {
      if (!intrinsics_opts.center_shared)
        m_num_intrinsics_per_camera += NUM_CENTER_PARAMS;
      if (intrinsics_opts.focus_shared)
        m_focus_offset = NUM_CENTER_PARAMS;
      else {
        m_num_intrinsics_per_camera += NUM_FOCUS_PARAMS;
        if (!intrinsics_opts.center_shared)
          m_focus_offset = NUM_CENTER_PARAMS;
      }
      if (intrinsics_opts.distortion_shared)
        m_distortion_offset = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS;
      else {
        m_num_intrinsics_per_camera += max_num_dist_params;
        if (!intrinsics_opts.center_shared)
          m_distortion_offset += NUM_CENTER_PARAMS;
        if (!intrinsics_opts.focus_shared)
          m_distortion_offset += NUM_FOCUS_PARAMS;
      }

      // For simplicity, we always set this to the same size and allocate
      // the same storage even if none of the parameters are shared.
      m_num_shared_intrinsics = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS
                                + max_num_dist_params;
      m_intrinsics_vec.resize(m_num_shared_intrinsics +
                              num_cameras*m_num_intrinsics_per_camera,
                              0); // Initialize to 0
    } else {
      m_num_intrinsics_per_camera = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS
                                    + max_num_dist_params;
      m_intrinsics_vec.resize(intrinsics_opts.num_sensors * m_num_intrinsics_per_camera,
                              0); // Initialize to 0
    }
  }

// Copy constructor
asp::BAParams::BAParams(asp::BAParams const& other):
      m_num_points        (other.m_num_points),
      m_num_cameras       (other.m_num_cameras),
      m_params_per_point  (other.m_params_per_point),
      m_num_pose_params   (other.m_num_pose_params),
      m_num_shared_intrinsics    (other.m_num_shared_intrinsics),
      m_num_intrinsics_per_camera(other.m_num_intrinsics_per_camera),
      m_max_num_dist_params(other.m_max_num_dist_params),
      m_focus_offset      (other.m_focus_offset),
      m_distortion_offset (other.m_distortion_offset),
      m_intrinsics_opts    (other.m_intrinsics_opts),
      m_points_vec        (other.m_points_vec.size()        ),
      m_cameras_vec       (other.m_cameras_vec.size()       ),
      m_intrinsics_vec    (other.m_intrinsics_vec.size()    ),
      m_outlier_points_vec(other.m_outlier_points_vec.size()),
      m_rand_gen(boost::random::mt19937()) {
    copy_points    (other);
    copy_cameras   (other);
    copy_intrinsics(other);
    copy_outliers  (other);
  }

// Set all camera position and pose values to zero.
void asp::BAParams::init_cams_as_zero() {
  for (int i=0; i < m_cameras_vec.size(); i++)
    m_cameras_vec[i] = 0.0;
}

// When using the copy functions, the sizes must match!
/// Copy one set of values from another instance.
void asp::BAParams::copy_points(asp::BAParams const& other) {
  for (size_t i = 0; i < m_points_vec.size(); i++)
    m_points_vec[i] = other.m_points_vec[i];
}
void asp::BAParams::copy_cameras(asp::BAParams const& other) {
  for (size_t i = 0; i < m_cameras_vec.size(); i++)
    m_cameras_vec[i] = other.m_cameras_vec[i];
}
void asp::BAParams::copy_intrinsics(asp::BAParams const& other) {
  for (size_t i = 0; i < m_intrinsics_vec.size(); i++)
    m_intrinsics_vec[i] = other.m_intrinsics_vec[i];
}
void asp::BAParams::copy_outliers(asp::BAParams const& other) {
  for (size_t i = 0; i < m_outlier_points_vec.size(); i++)
    m_outlier_points_vec[i] = other.m_outlier_points_vec[i];
}

// Compute the offset in m_intrinsics_vec to the requested data
size_t asp::BAParams::get_center_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.center_shared)
      return 0;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera;
  }

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera;
}

size_t asp::BAParams::get_focus_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.focus_shared)
      return m_focus_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + m_focus_offset;
  } 

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera + NUM_CENTER_PARAMS;
}

size_t asp::BAParams::get_distortion_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.distortion_shared)
      return m_distortion_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + 
              m_distortion_offset;
  }

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera + NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS;
}

/// Apply a random offset to each camera position.
void asp::BAParams::randomize_cameras() {
  // These are stored as x,y,z, axis_angle.
  // - We move the position +/- 5 meters.
  // - Currently we don't adjust the angle.
  boost::random::uniform_int_distribution<> xyz_dist(0, 10);
  const size_t NUM_CAMERA_PARAMS = 6;
  VW_ASSERT((m_cameras_vec.size() % NUM_CAMERA_PARAMS) == 0,
            vw::LogicErr() << "Camera parameter length is not a multiple of 6!");
  const size_t num_cameras = m_cameras_vec.size() / NUM_CAMERA_PARAMS;
  for (size_t c = 0; c < num_cameras; c++) {
    double* ptr = get_camera_ptr(c);
    for (size_t i = 0; i < 3; i++) {
      int diff = xyz_dist(m_rand_gen) - 5;
      ptr[i] += diff;
    }
  }
}

/// Randomly scale each intrinsic value.
void asp::BAParams::randomize_intrinsics(std::vector<double> const& intrinsic_limits) {
  // Intrinsic values are stored as multipliers, here we 
  //  multiply from 0.5 to 1.5, being careful about shared and constant values.
  // - If intrinsic limits are specified, use that range instead.
  boost::random::uniform_int_distribution<> dist(0, 100);
  const double DENOM = 100.0;
  const double DEFAULT_MIN   = 0.5;
  const double DEFAULT_MAX   = 1.5;
  const double DEFAULT_RANGE = DEFAULT_MAX - DEFAULT_MIN;

  const size_t num_intrinsics = intrinsic_limits.size() / 2;
  float percent, scale, range = 0;
  // Iterate over cameras
  for (size_t c = 0; c < num_cameras(); c++) {
    size_t intrinsics_index = 0;
    if (m_intrinsics_opts.float_focal_length(c) &&
        !(m_intrinsics_opts.focus_shared && (c>0))) {
      double* ptr = get_intrinsic_focus_ptr(c);
      for (int i=0; i<NUM_FOCUS_PARAMS; i++) {
        percent = static_cast<double>(dist(m_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        ++intrinsics_index;
      }
    } // End focus case
    intrinsics_index = NUM_FOCUS_PARAMS; // In case we did not go through the loop
    if (m_intrinsics_opts.float_optical_center(c) && 
        !(m_intrinsics_opts.center_shared && (c>0))) {
      double* ptr = get_intrinsic_center_ptr(c);
      for (int i = 0; i < NUM_CENTER_PARAMS; i++) {
        percent = static_cast<double>(dist(m_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] 
            - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        ++intrinsics_index;
      }
    } // End center case
    intrinsics_index = NUM_FOCUS_PARAMS+NUM_CENTER_PARAMS; // In case we did not go through the loops
    if (m_intrinsics_opts.float_distortion_params(c) && 
        !(m_intrinsics_opts.distortion_shared && (c > 0))) {
      double* ptr = get_intrinsic_distortion_ptr(c);
      for (int i = 0; i < m_max_num_dist_params; i++) {
        percent = static_cast<double>(dist(m_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        intrinsics_index++;
      }
    } // End distortion case
  } // End camera loop
}

/// Print stats for optimized ground control points.
void asp::BAParams::print_gcp_stats(std::string const& out_prefix, 
                       vw::ba::ControlNetwork const& cnet,
                       vw::cartography::Datum const& d) const {

  std::string gcp_report = out_prefix + "-gcp-report.txt";
  vw::vw_out() << "Writing: " << gcp_report << std::endl;
  
  std::ofstream gfs(gcp_report.c_str());
  gfs.precision(17); 

  gfs << "# Ground control point report\n";
  // TODO(oalexan1): Print this to report file! Compare with existing report!
  gfs << "# input_gcp optimized_gcp diff\n";
  
  int gcp_count = 0;
  for (int ipt = 0; ipt < num_points(); ipt++) {
  
    if (cnet[ipt].type() != vw::ba::ControlPoint::GroundControlPoint)
      continue;
  
    if (get_point_outlier(ipt))
      continue; // skip outliers

    vw::Vector3 input_gcp = cnet[ipt].position();
    vw::Vector3 opt_gcp   = get_point(ipt);
  
    gfs << "GCP count: " << gcp_count << std::endl;
    gfs << "ECEF: " << input_gcp << ' ' << opt_gcp << ' '
        << input_gcp - opt_gcp << std::endl;

    // Now convert to llh
    input_gcp = d.cartesian_to_geodetic(input_gcp);
    opt_gcp   = d.cartesian_to_geodetic(opt_gcp);

    gfs << "Lon-lat-height: " << input_gcp << ' ' << opt_gcp << ' '
       << input_gcp - opt_gcp << std::endl;
    gfs << "\n";
    gcp_count++;
  }
  gfs.close();
}

void asp::BAParams::record_points_to_kml(const std::string &kml_path,
                            const vw::cartography::Datum& datum,
                            size_t skip, const std::string name,
                            const std::string icon) {
  if (datum.name() == asp::UNSPECIFIED_DATUM) {
    vw::vw_out(vw::WarningMessage) << "No datum specified. Cannot write file: "
                                   << kml_path << std::endl;
    return;
  }
  
  // Open the file
  vw::vw_out() << "Writing: " << kml_path << std::endl;
  vw::KMLFile kml(kml_path, name);
  
    // Set up a simple point icon with no labels
  const bool hide_labels = true;
  kml.append_style( "point", "", 1.0, icon, hide_labels);
  kml.append_style( "point_highlight", "", 1.1, icon, hide_labels);
  kml.append_stylemap( "point_placemark", "point",
                       "point_highlight");
  
  // Loop through the points
  const bool extrude = true;
  for (size_t i=0; i<num_points(); i+=skip) {
    
    if (get_point_outlier(i))
      continue; // skip outliers
    
    // Convert the point to GDC coords
    vw::Vector3 xyz         = get_point(i);
    vw::Vector3 lon_lat_alt = datum.cartesian_to_geodetic(xyz);

    // Add this to the output file
    kml.append_placemark(lon_lat_alt.x(), lon_lat_alt.y(), "", "", "point_placemark",
                         lon_lat_alt[2], extrude);
  }
  kml.close_kml();
}

/// Simple class to manage position/rotation information.
/// - This is the data type stored in pc_align output files,
///   bundle adjustment files, and the position of pinhole cameras.

// Constructors
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
  m_pose_data     = vw::math::axis_angle_to_quaternion(vw::Vector3(array[3], array[4], array[5]));
}

/// Populate from an AdjustedCameraModel
void CameraAdjustment::copy_from_adjusted_camera
  (vw::camera::AdjustedCameraModel const& cam) {
  m_position_data = cam.translation();
  m_pose_data     = cam.rotation();
}

/// Populate from a PinholeModel
void CameraAdjustment::copy_from_pinhole(vw::camera::PinholeModel const& cam) {
  m_position_data = cam.camera_center();
  m_pose_data     = cam.camera_pose();
}

/// Populate from OpticalBarModel
void CameraAdjustment::copy_from_optical_bar(vw::camera::OpticalBarModel const& cam) {
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

void pack_pinhole_to_arrays(vw::camera::PinholeModel const& camera,
                            int camera_index,
                            asp::BAParams & param_storage) {

  double* pos_pose_ptr   = param_storage.get_camera_ptr              (camera_index);
  double* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_pinhole(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We are solving for multipliers to the intrinsic values, so they all start at 1.0.
  // Center point and focal length
  center_ptr[0] = 1.0;
  center_ptr[1] = 1.0;
  focus_ptr [0] = 1.0;
  // Lens distortion
  vw::Vector<double> lens = camera.lens_distortion()->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    distortion_ptr[i] = 1.0;
}

void pack_optical_bar_to_arrays(vw::camera::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BAParams & param_storage) {

  double* pos_pose_ptr   = param_storage.get_camera_ptr              (camera_index);
  double* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double* intrinsics_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_optical_bar(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We are solving for multipliers to the intrinsic values, so they all start at 1.0.
  // Center point and focal length
  center_ptr[0] = 1.0;
  center_ptr[1] = 1.0;
  focus_ptr [0] = 1.0;
  // Pack the speed, MCF, and scan time into the distortion pointer.
  for (size_t i = 0; i < asp::NUM_OPTICAL_BAR_EXTRA_PARAMS; i++)
    intrinsics_ptr[i] = 1.0;
}

// This does not copy the camera position and orientation
void pack_csm_to_arrays(asp::CsmModel const& camera,
                        int camera_index,
                        asp::BAParams & param_storage) {

  double* pos_pose_ptr   = param_storage.get_camera_ptr              (camera_index);
  double* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose. We start with 0 pose and identity rotation. Nothing
  // gets copied from camera position and orientation.
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_csm(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We are solving for multipliers to the intrinsic values, so they all start at 1.0.
  // Center point and focal length
  center_ptr[0] = 1.0;
  center_ptr[1] = 1.0;
  focus_ptr [0] = 1.0;
  // Distortion
  for (size_t i = 0; i < camera.distortion().size(); i++)
    distortion_ptr[i] = 1.0;
}
  
/// Given a transform with origin at the planet center, like output by pc_align,
/// read the adjustments from param storage, apply this transform on top of
/// them, and write the adjustments back to the param storage. Cameras
/// do not change.
void apply_transform_to_params(vw::Matrix4x4 const& M, asp::BAParams &param_storage,
                                std::vector<vw::CamPtr> const& cam_ptrs) {

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {

    // Load the current position/pose of this camera.
    double* cam_ptr = param_storage.get_camera_ptr(i);
    CameraAdjustment cam_adjust(cam_ptr);

    // Create the adjusted camera model
    vw::camera::AdjustedCameraModel cam(cam_ptrs[i], cam_adjust.position(), 
                                        cam_adjust.pose());
    // Apply the transform
    cam.apply_transform(M);

    // Copy back the adjustments to the camera array.
    cam_adjust.copy_from_adjusted_camera(cam);
    cam_adjust.pack_to_array(cam_ptr);
  }
} // end function apply_transform_to_cameras

// This function takes advantage of the fact that when it is called the cam_ptrs
//  have the same information as is in param_storage!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BAParams & param_storage,
                                        std::vector<vw::CamPtr>
                                        const& cam_ptrs){

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<camera::PinholeModel> pin_ptr = 
      boost::dynamic_pointer_cast<vw::camera::PinholeModel>(cam_ptrs[i]);
    pin_ptr->apply_transform(M);

    // Write out to param_storage
    pack_pinhole_to_arrays(*pin_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_pinhole

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
// information as is in param_storage.
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BAParams & param_storage,
                                            std::vector<vw::CamPtr>
                                            const& cam_ptrs){

  // Convert the transform format
  vw::Matrix3x3 R = submatrix(M, 0, 0, 3, 3);
  vw::Vector3   T;
  for (int r = 0; r < 3; r++) 
    T[r] = M(r, 3);
  
  double scale = pow(det(R), 1.0/3.0);
  for (size_t r = 0; r < R.rows(); r++)
    for (size_t c = 0; c < R.cols(); c++)
      R(r, c) /= scale;

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<vw::camera::OpticalBarModel> bar_ptr = 
      boost::dynamic_pointer_cast<vw::camera::OpticalBarModel>(cam_ptrs[i]);
    bar_ptr->apply_transform(R, T, scale);

    // Write out to param_storage
    pack_optical_bar_to_arrays(*bar_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_optical_bar

// This function takes advantage of the fact that when it is called the cam_ptrs
//  have the same information as is in param_storage.
// This applies the transform to the camera inline, but does not copy
// the camera position and orientation to the arrays.
void apply_transform_to_cameras_csm(vw::Matrix4x4 const& M,
                                    asp::BAParams & param_storage,
                                    std::vector<vw::CamPtr>
                                    const& cam_ptrs) {
  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<asp::CsmModel> csm_ptr = 
      boost::dynamic_pointer_cast<asp::CsmModel>(cam_ptrs[i]);
    if (csm_ptr == NULL)
        vw_throw( ArgumentErr() << "Expecting a CSM camera.\n" );
    csm_ptr->applyTransform(M);
    // Write out to param_storage. This does not copy camera position 
    // and orientation. The adjustment stays as 0 pose and identity rotation.
    // That is why the transform M does not get applied twice.
    pack_csm_to_arrays(*csm_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_csm

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<vw::CamPtr> &camera_models,
                           boost::shared_ptr<vw::ba::ControlNetwork> const& cnet) {

  // Apply the transform to the cameras
  for (size_t icam = 0; icam < camera_models.size(); icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

    pincam->apply_transform(rotation, translation, scale);
  } // End loop through cameras

  // Apply the transform to all of the world points in the ControlNetwork
  ControlNetwork::iterator iter;
  for (iter=cnet->begin(); iter!=cnet->end(); ++iter) {
    if (iter->type() == ControlPoint::GroundControlPoint)
      continue; // Don't convert the ground control points!

    Vector3 position     = iter->position();
    Vector3 new_position = scale*rotation*position + translation;
    iter->set_position(new_position);
  }
} // End function ApplyRigidTransform


/// Generate a warning if the GCP's are really far from the IP points
/// - This is intended to help catch the common lat/lon swap in GCP files.
void check_gcp_dists(std::vector<vw::CamPtr> const& camera_models,
                     boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
                     double forced_triangulation_distance) {

  // Count the points and triangulate
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  const int num_cnet_points = static_cast<int>(cnet.size());
  double gcp_count = 0, ip_count = 0;
  Vector3 mean_gcp(0, 0, 0);
  Vector3 mean_ip (0, 0, 0);
  for (int ipt = 0; ipt < num_cnet_points; ipt++) {

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;
    
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {
      gcp_count += 1.0;
      mean_gcp += cnet[ipt].position();
    } else {
      // Use triangulation to estimate the position of this control point using
      // the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double ans = vw::ba::triangulate_control_point(cp_new, camera_models, minimum_angle,
						     forced_triangulation_distance);
      if (ans < 0 || cp_new.position() == Vector3())
        continue; // Skip points that fail to triangulate

      ip_count += 1.0;
      mean_ip += cp_new.position();
    }
  } // End loop through control network points

  if (ip_count == 0 || gcp_count == 0)
    return; // Can't do this check if we don't have both point types.

  // Average the points
  mean_gcp = mean_gcp / gcp_count;
  mean_ip = mean_ip / ip_count;

  double dist = norm_2(mean_ip - mean_gcp);
  if (dist > 100000)
    vw_out() << "WARNING: GCPs are over 100 km from the other points. Are your lat/lon GCP coordinates swapped?\n";
}

/// Load all of the reference disparities specified in the input text file
/// and store them in the vectors.  Return the number loaded.
int load_reference_disparities(std::string const& disp_list_filename,
                               std::vector<vw::ImageView<vw::PixelMask<vw::Vector2f>>> &
                                  disp_vec,
                               std::vector<vw::ImageViewRef<vw::PixelMask<vw::Vector2f>>> &
                                  interp_disp) {
  // TODO: Disparities can be large, but if small it is better to
  // read them in memory.
  std::istringstream is(disp_list_filename);
  std::string disp_file;
  while (is >> disp_file) {
    if (disp_file != "none") {
      vw::vw_out() << "Reading: " << disp_file << std::endl;
      disp_vec.push_back(copy(vw::DiskImageView<vw::PixelMask<vw::Vector2f>>(disp_file)));
    }else{
      // Read in an empty disparity
      disp_vec.push_back(vw::ImageView<vw::PixelMask<vw::Vector2f>>());
    }
    interp_disp.push_back(interpolate(disp_vec.back(),
                                      vw::BilinearInterpolation(), 
                                      vw::ConstantEdgeExtension()));
  }
  return static_cast<int>(disp_vec.size());
}

// Initialize the position and orientation of each pinhole camera model using
// a least squares error transform to match the provided camera positions.
// This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet, 
 std::vector<vw::CamPtr> & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<vw::Vector3> const & estimated_camera_gcc) {

  vw_out() << "Initializing camera positions from input file." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = image_files.size();
  if (int(estimated_camera_gcc.size()) != num_cameras)
    vw_throw( ArgumentErr() << "No camera matches provided to init function!\n" );

  vw_out() << "Num cameras: " << num_cameras << std::endl;

  int num_matches_found = 0;
  for (int i=0; i<num_cameras; i++)
    if (estimated_camera_gcc[i] != Vector3(0,0,0))
      ++num_matches_found;

  vw_out() << "Number of matches found: " << num_matches_found << std::endl;

  const int MIN_NUM_MATCHES = 3;
  if (num_matches_found < MIN_NUM_MATCHES)
    vw_throw(ArgumentErr() << "At least " << MIN_NUM_MATCHES 
              << " camera position matches are required to initialize cameras "
              << "based on camera positions only.\n" );

  // Populate matrices containing the current and known camera positions.
  vw::Matrix<double> points_in(3, num_matches_found), points_out(3, num_matches_found);
  typedef vw::math::MatrixCol<vw::Matrix<double>> ColView;
  int index = 0;
  for (int i=0; i<num_cameras; i++) {
    // Skip cameras with no matching record
    if (estimated_camera_gcc[i] == Vector3(0,0,0))
      continue;

    // Get the two GCC positions
    Vector3 gcc_in  = camera_models[i]->camera_center(Vector2(0,0));
    Vector3 gcc_out = estimated_camera_gcc[i];

    // Store in matrices
    ColView colIn (points_in,  index); 
    ColView colOut(points_out, index);
    colIn  = gcc_in;
    colOut = gcc_out;
    ++index;

  } // End matrix populating loop

  // Call function to compute a 3D affine transform between the two point sets
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  vw::math::find_3D_transform(points_in, points_out, rotation, translation, scale);

  // Update the camera and point information with the new transform
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet);
  return true;
}

// Given at least two images, each having at least 3 GCP that are not seen in other
// images, find and apply a transform to the camera system based on them.
void transform_cameras_with_indiv_image_gcp
(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
 std::vector<vw::CamPtr> & camera_models) {
  
  vw_out() << "Applying transform to cameras given several GCP not shared "
           << "among the images.\n";

  int num_cams = camera_models.size();

  // Create pinhole cameras
  std::vector<PinholeModel> pinhole_cams;
  for (int icam = 0; icam < num_cams; icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL,
	      vw::ArgumentErr() << "A pinhole camera expected.\n");
    pinhole_cams.push_back(*pincam);
  }
  
  // Extract from the control network each pixel for each camera together
  // with its xyz.
  std::vector<std::vector<Vector3>> xyz;
  std::vector<std::vector<Vector2>> pix;
  xyz.resize(num_cams);
  pix.resize(num_cams);
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias

  for (int ipt = 0; ipt < cnet.size(); ipt++) {
    
    // Keep only gcp
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
            
    for (auto measure = cnet[ipt].begin(); measure != cnet[ipt].end(); measure++) {
      int cam_it = measure->image_id();
      if (cam_it < 0 || cam_it >= num_cams)
        vw_throw(ArgumentErr() << "Error: cnet index out of range.\n");

      Vector2 pixel(measure->position()[0],  measure->position()[1]);
      pix[cam_it].push_back(pixel);
      xyz[cam_it].push_back(cnet[ipt].position());
    }
  }  

  Matrix3x3 rotation;
  Vector3   translation;
  double    scale;
  vw::camera::align_cameras_to_ground(xyz, pix, pinhole_cams, rotation, translation, scale);

  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);
}

/// Initialize the position and orientation of each pinhole camera model using
/// a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void transform_cameras_with_shared_gcp(
            boost::shared_ptr<ControlNetwork> const& cnet_ptr,
            std::vector<vw::CamPtr> & camera_models) {
  
  vw_out() << "Applying transform to cameras given several GCP shared among the images.\n";

  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  
  // Verify that all cameras are pinhole
  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL,
	      vw::ArgumentErr() << "A pinhole camera expected.\n");
  }
  
  // Put the good ground control points in a vector.
  int num_cnet_points = cnet.size();
  std::vector<vw::Vector3> in_xyz, out_xyz; 
  int num_gcp      = 0;
  int num_good_gcp = 0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++) {
    
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;

    num_gcp++;
    
    // Use triangulation to estimate the position of this control point using
    //   the current set of camera models.
    ControlPoint untrans_cp = cnet[ipt];
    double minimum_angle = 1.0e-3; // Likely this is too small for a good GCP, but better than 0.
    double forced_triangulation_distance = -1.0;
    double err = vw::ba::triangulate_control_point(untrans_cp, camera_models,
						   minimum_angle, forced_triangulation_distance);
    
    if (untrans_cp.position() != Vector3() && cnet[ipt].position()  != Vector3() && 
        err >= 0) {
      // Store the computed and correct position of this point
      in_xyz.push_back(untrans_cp.position());
      out_xyz.push_back(cnet[ipt].position());
      num_good_gcp++; // Only count points that triangulate
    } else {
      vw_out() << "Discarding GCP that could not be triangulated: " << cnet[ipt] << ".\n";
    }
  } // End good GCP counting

  // Sanity check
  const int MIN_GCP_COUNT = 3;
  if (num_good_gcp < MIN_GCP_COUNT) {
    vw_out() << "Num GCP       = " << num_gcp      << std::endl;
    vw_out() << "Num valid GCP = " << num_good_gcp << std::endl;
    vw_throw( ArgumentErr()
	      << "Not enough valid GCPs to apply a transform to the cameras. "
	      << "You may need to use --transform-cameras-using-gcp.\n" );
  }

  // Copy these points to a matrix as required by the API about to be used. 
  vw::Matrix<double> points_in(3, num_good_gcp), points_out(3, num_good_gcp);
  typedef vw::math::MatrixCol<vw::Matrix<double>> ColView;
  for (size_t ipt = 0; ipt < in_xyz.size(); ipt++) {
    ColView colIn (points_in,  ipt); 
    ColView colOut(points_out, ipt);
    colIn  = in_xyz[ipt];
    colOut = out_xyz[ipt];
  } // End loop through control network points
  
  // Call function to compute a 3D affine transform between the two point sets
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  vw::math::find_3D_transform(points_in, points_out, rotation, translation, scale);
  
  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  vw_out() << "This transform can be disabled with --disable-pinhole-gcp-init.\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);

  return;
} // End function transform_cameras_with_shared_gcp

/// Initialize the position and orientation of a pinhole camera model using
/// GCP. It invokes OpenCV's PnP functionality.
void init_camera_using_gcp(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
                           std::vector<vw::CamPtr> & camera_models) {
  
  // Sanity check
  if (camera_models.size() != 1) 
    vw::vw_throw(vw::ArgumentErr() 
                 << "Cannot initialize more than a camera at a time using GCP. "
                 << "Consider using --transform-cameras-with-shared-gcp or "
                 << "--transform-cameras-using-gcp.\n");
  
  vw_out() << "Initializing a Pinhole camera using GCP.\n";

  int icam = 0;
  vw::camera::PinholeModel * pincam
    = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
  VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");
  
  std::vector<vw::Vector2> pixel_observations;
  std::vector<vw::Vector3> ground_points;
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  for (int ipt = 0; ipt < cnet.size(); ipt++){

    // Loop through all the ground control points only
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
    ground_points.push_back(cnet[ipt].position());
    
    int num_meas = 0;
    for (ControlPoint::const_iterator measure = cnet[ipt].begin();
         measure != cnet[ipt].end(); measure++) {
      
      int cam_it = measure->image_id();
      if (cam_it != 0) 
        vw_throw(ArgumentErr() << "Error: Expecting GCP for a single camera.\n");
      
      Vector2 pixel(measure->position()[0], measure->position()[1]);
      num_meas++;
      if (num_meas > 1)
        vw::vw_throw(vw::ArgumentErr() << "Expecting a single camera pixel per gcp.\n");
      
      pixel_observations.push_back(pixel);
    }
  }

  // Update the camera pose with given observations and intrinsics
  asp::findCameraPose(ground_points, pixel_observations, *pincam);

  return;
  
} // End function init_camera_using_gcp

// Given an input pinhole camera and param changes, apply those, returning
// the new camera. Note that all intrinsic parameters are stored as multipliers
// in asp::BAParams.
vw::camera::PinholeModel transformedPinholeCamera(int camera_index,
                                                  asp::BAParams const& param_storage,
                                                  vw::camera::PinholeModel const& in_cam) {

  // Start by making a copy of the camera. Note that this does not make a copy of the
  // distortion params, as that's a pointer. So will have to make a copy of it further down.
  vw::camera::PinholeModel out_cam = in_cam;

  double const* pos_pose_ptr   = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose(pos_pose_info.pose());

  // Update the lens distortion parameters. Note how we make a new copy of the
  // distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    lens[i] *= distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  Vector2 old_center = out_cam.point_offset();
  Vector2 old_focus  = out_cam.focal_length();
  out_cam.set_point_offset(Vector2(center_ptr[0]*old_center[0],
                                  center_ptr[1]*old_center[1]), 
                           false); // do not update the internals yet
  double new_focus = old_focus[0]*focus_ptr[0];
  // At the last step, recompute the internals given the new values
  out_cam.set_focal_length(Vector2(new_focus,new_focus), true);
  
  return out_cam;
}

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::camera::OpticalBarModel 
transformedOpticalBarCamera(int camera_index,
                            asp::BAParams const& param_storage,
                            vw::camera::OpticalBarModel const& in_cam) {

  // Start by making a copy of the camera
  vw::camera::OpticalBarModel out_cam = in_cam;

  double const* pos_pose_ptr  = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr    = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // All intrinsic parameters are stored as multipliers!

  // Update the other intrinsic parameters.
  out_cam.set_speed              (out_cam.get_speed()*intrinsic_ptr[0]);
  out_cam.set_motion_compensation(out_cam.get_motion_compensation()*intrinsic_ptr[1]);
  out_cam.set_scan_time          (out_cam.get_scan_time()*intrinsic_ptr[2]);

  if (out_cam.get_have_velocity_vec()) {
    vw::Vector3 vel = out_cam.get_velocity();
    vel[0] *= intrinsic_ptr[3];
    vel[1] *= intrinsic_ptr[4];
    vel[2] *= intrinsic_ptr[5];
    out_cam.set_velocity(vel);
  }
  
  // Update the center and focus
  Vector2 old_center = out_cam.get_optical_center();
  float   old_focus  = out_cam.get_focal_length();
  out_cam.set_optical_center(Vector2(center_ptr[0]*old_center[0],
                                    center_ptr[1]*old_center[1]));
  double new_focus = old_focus*focus_ptr[0];
  out_cam.set_focal_length(new_focus);

  return out_cam;
}

// Given an input CSM camera, intrinsic and extrinsic param changes, apply
// those, returning the new camera.
boost::shared_ptr<asp::CsmModel> transformedCsmCamera(int camera_index,
                                                      asp::BAParams const& param_storage,
                                                      asp::CsmModel const& in_cam) {
  // Get the latest version of the camera parameters
  double const* pos_pose_ptr  = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr    = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* dist_ptr      = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Read the position and pose
  CameraAdjustment correction(pos_pose_ptr);

  // All intrinsic parameters are stored as multipliers
  vw::Vector2 optical_center = in_cam.optical_center();
  double focal_length        = in_cam.focal_length();
  optical_center[0] = center_ptr[0] * optical_center[0];
  optical_center[1] = center_ptr[1] * optical_center[1];
  focal_length      = focus_ptr [0] * focal_length;

  // Update the lens distortion parameters in the new camera.
  // - These values are also optimized as scale factors.
  std::vector<double> distortion = in_cam.distortion();
  for (size_t i = 0; i < distortion.size(); i++) {

    // Ensure this approach does not fail when the input distortion is 0.
    // TODO(oalexan1): This is not enough, however. The user must choose
    // to manually enter some small fake distortion, maybe on the order
    // of 1e-8. It is hard to tell what numbers to put below. They should
    // not be as small as 1e-16 but sometimes 1e-8 (for higher order polynomial
    // coefficients) may be too much.
    if (distortion[i] == 0.0)
      distortion[i] = 1e-16;

    distortion[i] = dist_ptr[i] * distortion[i];
  }

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

// Save convergence angle percentiles for each image pair having matches
void saveConvergenceAngles(std::string const& conv_angles_file,
                           std::vector<asp::MatchPairStats> const& convAngles,
                           std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << conv_angles_file << "\n";
  std::ofstream ofs (conv_angles_file.c_str());
  ofs.precision(8);
  ofs << "# Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << "# left_image right_image 25% 50% 75% num_matches\n";
  for (size_t conv_it = 0; conv_it < convAngles.size(); conv_it++) {
    auto const & c = convAngles[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' ' << c.val75  << ' ' << c.num_vals << "\n";
  }
  ofs.close();

  return;
}

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in meters. It is assumed that dem_georef
// was created by bilinear interpolation. The cameras must be with
// the latest adjustments applied to them.
void calcPairMapprojOffsets(int left_cam_index, int right_cam_index,
                            std::vector<vw::CamPtr>            const& optimized_cams,
                            std::vector<vw::ip::InterestPoint> const& left_ip,
                            std::vector<vw::ip::InterestPoint> const& right_ip,
                            vw::cartography::GeoReference      const& dem_georef,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            // Will append below
                            std::vector<vw::Vector<float, 4>>       & mapprojPoints,
                            std::vector<float>                      & mapprojOffsets) {
  
  // Wipe mapprojOffsets
  mapprojOffsets.clear();
  
  // Will append to mapprojPoints, so don't wipe it
  
  for (size_t ip_it = 0; ip_it < left_ip.size(); ip_it++) {
    
    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double height_error_tol = 0.001; // 1 mm should be enough
    double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
    double max_rel_tol      = 1e-14;
    int num_max_iter        = 50;   // Using many iterations can be very slow
    Vector3 xyz_guess;
    
    Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    Vector3 left_dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
      (optimized_cams[left_cam_index]->camera_center(left_pix),
       optimized_cams[left_cam_index]->pixel_to_vector(left_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef, 
       treat_nodata_as_zero, has_intersection,
       height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);
    if (!has_intersection) 
      continue;
    
    // Do the same for right. Use left pixel as initial guess
    xyz_guess = left_dem_xyz;
    Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);
    Vector3 right_dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
      (optimized_cams[right_cam_index]->camera_center(right_pix),
       optimized_cams[right_cam_index]->pixel_to_vector(right_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef, 
       treat_nodata_as_zero, has_intersection,
       height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);
    if (!has_intersection) 
      continue;

    Vector3 mid_pt = (left_dem_xyz + right_dem_xyz)/2.0;
    double dist = norm_2(left_dem_xyz - right_dem_xyz);

    // Keep in the same structure both the midpoint between these two
    // mapprojected ip, as lon-lat,height, and their distance, as
    // later the bookkeeping of mapprojOffsets will be different.
    // Float precision is enough, and will save on memory.
    Vector<float, 4> point;
    subvector(point, 0, 3) = dem_georef.datum().cartesian_to_geodetic(mid_pt);
    point[3] = dist;
    
    mapprojPoints.push_back(point);
    mapprojOffsets.push_back(dist);
  }
  
  return;
}

// Save mapprojected matches offsets for each image pair having matches
void saveMapprojOffsets(
     std::string                       const& out_prefix,
     vw::cartography::GeoReference     const& mapproj_dem_georef,
     std::vector<vw::Vector<float, 4>> const& mapprojPoints,
     std::vector<asp::MatchPairStats>  const& mapprojOffsets,
     std::vector<std::vector<float>>        & mapprojOffsetsPerCam,
     std::vector<std::string>          const& imageFiles) {
  
  std::string mapproj_offsets_stats_file 
    = out_prefix + "-mapproj_match_offset_stats.txt";
  vw_out() << "Writing: " << mapproj_offsets_stats_file << "\n";
  std::ofstream ofs (mapproj_offsets_stats_file.c_str());
  ofs.precision(8); // 8 digits of precision for errors is enough

  ofs << "# Percentiles of distances between mapprojected matching pixels in an "
      << "image and the others.\n";
  ofs << "# image_name 25% 50% 75% 85% 95% count\n";
  for (size_t image_it = 0; image_it < imageFiles.size(); image_it++) {
    auto & vals = mapprojOffsetsPerCam[image_it]; // alias
    int len = vals.size();
    float val25 = -1.0, val50 = -1.0, val75 = -1.0, val85 = -1.0, val95 = -1.0, count = 0;
    if (!vals.empty()) {
      std::sort(vals.begin(), vals.end());
      val25 = vals[0.25 * len];
      val50 = vals[0.50 * len];
      val75 = vals[0.75 * len];
      val85 = vals[0.85 * len];
      val95 = vals[0.95 * len];
      count = len;
    }

    ofs << imageFiles[image_it] << ' '
        << val25 << ' ' << val50 << ' ' << val75 << ' '
        << val85 << ' ' << val95 << ' ' << count << "\n";
  }
  ofs.close();

  std::string mapproj_offsets_pair_stats_file 
    = out_prefix + "-mapproj_match_offset_pair_stats.txt";
  vw::vw_out() << "Writing: " << mapproj_offsets_pair_stats_file << "\n";
  ofs = std::ofstream(mapproj_offsets_pair_stats_file.c_str());

  ofs << "# Percentiles of distances between matching pixels after mapprojecting onto DEM.\n"
      << "# Per image pair and measured in DEM pixel units.\n";
  ofs << "# left_image right_image 25% 50% 75% 85% 95% num_matches_per_pair\n";
  ofs.precision(8); // 8 digits of precision for errors is enough
  for (size_t conv_it = 0; conv_it < mapprojOffsets.size(); conv_it++) {
    auto const & c = mapprojOffsets[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' ' << c.val75 << ' '
        << c.val85 << ' ' << c.val95 << ' ' << c.num_vals << "\n";
  }
  ofs.close();

  std::string mapproj_offsets_file = out_prefix + "-mapproj_match_offsets.txt";
  vw_out() << "Writing: " << mapproj_offsets_file << "\n";
  ofs = std::ofstream(mapproj_offsets_file.c_str());
  // 12 digits of precision for errors is enough. 
  // That is 9 digits after decimal period for lon and lat.
  ofs.precision(12); 
  ofs << "# lon, lat, height_above_datum, mapproj_ip_dist_meters\n";
  ofs << "# " << mapproj_dem_georef.datum() << std::endl;

  // Write all the points to the file
  for (size_t it = 0; it < mapprojPoints.size(); it++) {
    Vector3 llh = subvector(mapprojPoints[it], 0, 3);
    ofs << llh[0] << ", " << llh[1] <<", " << llh[2] << ", "
         << mapprojPoints[it][3] << std::endl;
  }
  
  ofs.close();
  
  return;
}

// This is called either with original or inlier ip
void processMatchPair(size_t left_index, size_t right_index,
                      std::vector<vw::ip::InterestPoint> const& left_ip,
                      std::vector<vw::ip::InterestPoint> const& right_ip,
                      std::vector<vw::CamPtr> const& optimized_cams,
                      vw::cartography::GeoReference const& mapproj_dem_georef,
                      vw::ImageViewRef<vw::PixelMask<double>> const& interp_mapproj_dem,
                      vw::cartography::Datum const& datum,
                      bool save_mapproj_match_points_offsets,
                      bool propagate_errors,
                      vw::Vector<double> const& horizontal_stddev_vec,
                      // Outputs
                      // Will append to entities below
                      std::vector<asp::MatchPairStats>  & convAngles,
                      std::vector<vw::Vector<float, 4>> & mapprojPoints,
                      std::vector<asp::MatchPairStats> & mapprojOffsets,
                      std::vector<std::vector<float>>  & mapprojOffsetsPerCam,
                      std::vector<asp::HorizVertErrorStats> & horizVertErrors) {

  convAngles.push_back(asp::MatchPairStats()); // add an element, then populate it
  std::vector<double> sorted_angles;
  asp::convergence_angles(optimized_cams[left_index].get(), optimized_cams[right_index].get(),
                          left_ip, right_ip, sorted_angles);
  convAngles.back().populate(left_index, right_index, sorted_angles);

  if (save_mapproj_match_points_offsets) {
    std::vector<float> localMapprojOffsets;
    asp::calcPairMapprojOffsets(left_index, right_index,
                                optimized_cams,
                                left_ip, right_ip,
                                mapproj_dem_georef, interp_mapproj_dem,  
                                mapprojPoints, // will append here
                                localMapprojOffsets);
    mapprojOffsets.push_back(asp::MatchPairStats()); // add an elem, then populate it
    mapprojOffsets.back().populate(left_index, right_index, localMapprojOffsets);
    for (size_t map_it = 0; map_it < localMapprojOffsets.size(); map_it++) {
      mapprojOffsetsPerCam[left_index].push_back(localMapprojOffsets[map_it]);
      mapprojOffsetsPerCam[right_index].push_back(localMapprojOffsets[map_it]);
    }
  }

  if (propagate_errors) {
    // Ensure her that proper values are passed for the input std devs
    horizVertErrors.push_back(asp::HorizVertErrorStats()); // add an elem, then populate it
    asp::propagatedErrorStats(left_index, right_index,
                              optimized_cams[left_index].get(), 
                              optimized_cams[right_index].get(),
                              left_ip, right_ip, 
                              horizontal_stddev_vec[left_index],
                              horizontal_stddev_vec[right_index],
                              datum, 
                              horizVertErrors.back());      
  }

  return;
}

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.
void matchFilesProcessing(vw::ba::ControlNetwork       const& cnet,
                          asp::BaBaseOptions           const& opt,
                          std::vector<vw::CamPtr>      const& optimized_cams,
                          bool                                remove_outliers,
                          std::set<int>                const& outliers,
                          std::string                  const& mapproj_dem,
                          bool                                propagate_errors, 
                          vw::Vector<double>           const& horizontal_stddev_vec,
                          bool                                save_clean_matches,
                          std::map<std::pair<int, int>, std::string> const& match_files) {

  vw_out() << "Creating reports.\n";
  
  std::vector<vw::Vector<float, 4>> mapprojPoints; // all points, not just stats
  std::vector<asp::MatchPairStats> convAngles, mapprojOffsets;
  std::vector<std::vector<float>> mapprojOffsetsPerCam;
  std::vector<asp::HorizVertErrorStats> horizVertErrors;
  
  // Wipe the outputs
  mapprojPoints.clear();
  convAngles.clear();
  mapprojOffsets.clear();
  mapprojOffsetsPerCam.clear();
  horizVertErrors.clear();

  bool save_mapproj_match_points_offsets = (!mapproj_dem.empty());
  vw::cartography::GeoReference mapproj_dem_georef;
  ImageViewRef<PixelMask<double>> interp_mapproj_dem;
  if (save_mapproj_match_points_offsets)
    asp::create_interp_dem(mapproj_dem, mapproj_dem_georef, interp_mapproj_dem);

  int num_cameras = opt.image_files.size();
  mapprojOffsetsPerCam.resize(num_cameras);

  // Iterate over the control network, and, for each inlier pair of matches,
  // remember what pair it is from. Needed only if there is outlier filtering
  // or matches were read from an isis cnet.
  // TODO(oalexan1): This uses a lot of memory. Need to keep just indices,
  // somehow, not quadruplets of floats.
  // TODO(oalexan1): Make this into a function.
  typedef std::tuple<float, float, float, float> Quadruplet;
  std::map<std::pair<int, int>, std::set<Quadruplet>> match_map;
  if (remove_outliers || !opt.isis_cnet.empty() || !opt.nvm.empty()) {
    for (int ipt = 0; ipt < cnet.size(); ipt++) {
      // Skip outliers
      if (outliers.find(ipt) != outliers.end())
        continue;
      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;
      
      for (auto m1 = cnet[ipt].begin(); m1 != cnet[ipt].end(); m1++) {
        for (auto m2 = cnet[ipt].begin(); m2 != cnet[ipt].end(); m2++) {
          int left_index = m1->image_id();
          int right_index = m2->image_id();
          // Can have left_index > right_index
          if (left_index == right_index) 
            continue;
          match_map[std::make_pair(left_index, right_index)].insert
            (Quadruplet(m1->position()[0], m1->position()[1],
                        m2->position()[0], m2->position()[1]));
        }
      }
    }
  } 
  
  // If we read the matches from an ISIS cnet or nvm, there are no match files.
  // Then create them. 
  std::map<std::pair<int, int>, std::string> local_match_files = match_files;
  if (opt.isis_cnet != "" || opt.nvm != "") {
    // iterate over match pairs
    local_match_files.clear();
    for (auto const& match_pair: match_map) {
      int left_index  = match_pair.first.first;
      int right_index = match_pair.first.second;
      // When creating match files from scratch, let the first index
      // be less than the second.
      if (left_index > right_index) 
        continue;
      std::string match_file 
        = vw::ip::match_filename(opt.out_prefix,
                                 opt.image_files[left_index],
                                 opt.image_files[right_index]);
      local_match_files[std::make_pair(left_index, right_index)] = match_file;
    }
  }
  
  // Work on individual image pairs
  for (const auto& match_it: local_match_files) {
   
    std::pair<int, int> cam_pair   = match_it.first;
    std::string         match_file = match_it.second;
    size_t left_index  = cam_pair.first;
    size_t right_index = cam_pair.second;
    if (left_index == right_index) 
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Cannot have interest point "
                   << "matches between an image and itself.\n");
                   
    std::vector<ip::InterestPoint> orig_left_ip, orig_right_ip;
    if (opt.isis_cnet != "" || opt.nvm != "") {
      // Must create the matches from the cnet.
      auto & match_pair = match_map[std::make_pair(left_index, right_index)]; // alias
      // Iterate over this set of quadruplets, and build matches
      for (auto const& q: match_pair) {
        double s = 1.0; // scale
        orig_left_ip.push_back(vw::ip::InterestPoint(std::get<0>(q), std::get<1>(q), s));
        orig_right_ip.push_back(vw::ip::InterestPoint(std::get<2>(q), std::get<3>(q), s));
      }
      
      // Write the matches formed from the cnet to disk
      if (opt.output_cnet_type == "match-files") {
        vw::vw_out() << "Writing: " << match_file << std::endl;
        vw::ip::write_binary_match_file(match_file, orig_left_ip, orig_right_ip);
      }
      
    } else {
      // Read existing matches. Skip over match files that don't exist.
      if (!boost::filesystem::exists(match_file)) {
        vw_out() << "Skipping non-existent match file: " << match_file << std::endl;
        continue;
      }
      // Read the original IP, to ensure later we write to disk only
      // the subset of the IP from the control network which
      // are part of these original ones. 
      vw::ip::read_binary_match_file(match_file, orig_left_ip, orig_right_ip);
    }

    // Create a new convergence angle storage struct
    asp::MatchPairStats & convAngle = convAngles.back(); // alias
    if (!remove_outliers) {
      // Do some processing with orig ip. Otherwise this will be done below
      // with the inlier ip.
      processMatchPair(left_index, right_index,
                       orig_left_ip, orig_right_ip,
                       optimized_cams,
                       mapproj_dem_georef, interp_mapproj_dem, opt.datum,
                       save_mapproj_match_points_offsets,
                       propagate_errors, horizontal_stddev_vec,
                       // Will append to entities below
                       convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                       horizVertErrors);
      // Since no outliers are removed, nothing else to do
      continue;
    }
    // Keep only inliers and non-gcp. GCP are used in optimization but are not
    // part of the originally found interest point matches.
    std::vector<vw::ip::InterestPoint> left_ip, right_ip;
    for (size_t ip_iter = 0; ip_iter < orig_left_ip.size(); ip_iter++) {
      Quadruplet q(orig_left_ip[ip_iter].x, orig_left_ip[ip_iter].y,
                   orig_right_ip[ip_iter].x, orig_right_ip[ip_iter].y);
      auto & match_pair = match_map[std::make_pair(left_index, right_index)]; // alias
      if (match_pair.find(q) == match_pair.end()) 
        continue;

      // We do not copy descriptors, those take storage
      left_ip.push_back(ip::InterestPoint(orig_left_ip[ip_iter].x, 
                                          orig_left_ip[ip_iter].y,
                                          orig_left_ip[ip_iter].scale));
      right_ip.push_back(ip::InterestPoint(orig_right_ip[ip_iter].x, 
                                           orig_right_ip[ip_iter].y,
                                           orig_right_ip[ip_iter].scale));
    }
    
    // Filter by disparity
    // TODO(oalexan1): Note that this does not update the outliers set. Likely this
    // processing needs to move where other outlier filtering logic is.
    bool quiet = true; // Otherwise too many messages are printed
    if (opt.remove_outliers_params[0] > 0 && opt.remove_outliers_params[1] > 0.0) {
      // The typical value of 75 for opt.remove_outliers_params[1] may be too low.
      // Adjust it. pct = 75 becomes pct = 90. pct = 100 becomes pct = 100. So,
      // if starting under 100, it gets closer to 100 but stays under it.
      double pct = opt.remove_outliers_params[0];
      pct = 100.0 * (pct + 150.0) / 250.0;
      asp::filter_ip_by_disparity(pct, opt.remove_outliers_params[1],
                                  quiet, left_ip, right_ip);
    }
    
    if (num_cameras == 2) {
      // Compute the coverage fraction
      Vector2i right_image_size = file_image_size(opt.image_files[1]);
      int right_ip_width = right_image_size[0]*
        static_cast<double>(100.0 - std::max(opt.ip_edge_buffer_percent, 0))/100.0;
      Vector2i ip_size(right_ip_width, right_image_size[1]);
      double ip_coverage = asp::calc_ip_coverage_fraction(right_ip, ip_size);
      // Careful with the line below, it gets used in process_icebridge_batch.py.
      vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";
    }

    // Process the inlier ip
    processMatchPair(left_index, right_index, left_ip, right_ip,
                     optimized_cams, mapproj_dem_georef, interp_mapproj_dem,
                     opt.datum,
                     save_mapproj_match_points_offsets, 
                     propagate_errors, horizontal_stddev_vec,
                     // Will append to entities below
                     convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                     horizVertErrors);

    if (opt.output_cnet_type != "match-files" || !save_clean_matches)
      continue; // Do not write match files

    // Make a clean copy of the file
    std::string clean_match_file = ip::clean_match_filename(match_file);
    if (opt.clean_match_files_prefix != "") {
      // Ensure "clean" does not show up twice
      clean_match_file = match_file;
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.clean_match_files_prefix.size(), opt.out_prefix);
    }
    else if (opt.match_files_prefix != "") {
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.match_files_prefix.size(), opt.out_prefix);
    }
    
    vw_out() << "Saving " << left_ip.size() << " filtered interest points.\n";
    vw_out() << "Writing: " << clean_match_file << std::endl;
    vw::ip::write_binary_match_file(clean_match_file, left_ip, right_ip);

  } // End loop through the match files

  // Save the produced files  
  
  std::string conv_angles_file = opt.out_prefix + "-convergence_angles.txt";
  asp::saveConvergenceAngles(conv_angles_file, convAngles, opt.image_files);

  if (!opt.mapproj_dem.empty())
    asp::saveMapprojOffsets(opt.out_prefix,
                            mapproj_dem_georef,
                            mapprojPoints,
                            mapprojOffsets, 
                            mapprojOffsetsPerCam, // will change
                            opt.image_files);

  if (opt.propagate_errors) {
    std::string horiz_vert_errors_file = opt.out_prefix + "-triangulation_uncertainty.txt";
    asp::saveHorizVertErrors(horiz_vert_errors_file, horizVertErrors, opt.image_files);
  }
  
  return;
}

// Find stats of propagated errors
void propagatedErrorStats(size_t left_cam_index, size_t right_cam_index,
                          vw::camera::CameraModel const * left_cam,
                          vw::camera::CameraModel const * right_cam,
                          std::vector<vw::ip::InterestPoint> const& left_ip,
                          std::vector<vw::ip::InterestPoint> const& right_ip,
                          double stddev1, double stddev2,
                          vw::cartography::Datum const& datum,
                          // Output
                          asp::HorizVertErrorStats & stats) {

  // Create a stereo model, to be used for triangulation
  double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos
                        (asp::stereo_settings().min_triangulation_angle*M_PI/180);
  vw::stereo::StereoModel stereo_model(left_cam, right_cam, 
                                       asp::stereo_settings().use_least_squares,
                                       angle_tol);

  // Create space for horiz and vert vectors of size num_ip
  int num_ip = left_ip.size();
  std::vector<double> horiz_errors, vert_errors;

  // Find the triangulated point and propagate the errors
  for (int ip_it = 0; ip_it < num_ip; ip_it++) {
    // Compute the error in the horizontal and vertical directions
    vw::Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    vw::Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);
    
    Vector3 triVec(0, 0, 0), errorVec(0, 0, 0);
    vw::Vector2 outStdev;
    try {
      triVec = stereo_model(left_pix, right_pix, errorVec);
      outStdev = asp::propagateCovariance(triVec, datum, 
                                          stddev1, stddev2, 
                                          left_cam, right_cam, 
                                          left_pix, right_pix);
    } catch (std::exception const& e) {
      errorVec = Vector3(0, 0, 0);
    }
    
    if (errorVec == Vector3(0, 0, 0))
      continue; // this can happen either because triangulation failed or an exception
    
    horiz_errors.push_back(outStdev[0]);
    vert_errors.push_back(outStdev[1]);
  }
  
  // Initialize the output
  stats = asp::HorizVertErrorStats();
  stats.left_cam_index  = left_cam_index;
  stats.right_cam_index = right_cam_index;

  if (!horiz_errors.empty()) {
    stats.num_errors = horiz_errors.size();
    stats.horiz_error_mean = vw::math::mean(horiz_errors);
    stats.vert_error_mean = vw::math::mean(vert_errors);
    
    if (horiz_errors.size() > 1) {
      // This divides by num - 1
      stats.horiz_error_stddev 
        = vw::math::standard_deviation(horiz_errors, stats.horiz_error_mean);
      stats.vert_error_stddev
        = vw::math::standard_deviation(vert_errors, stats.vert_error_mean);
    }
    
    // Leave this for the last
    stats.horiz_error_median = vw::math::destructive_median(horiz_errors);
    stats.vert_error_median = vw::math::destructive_median(vert_errors);
  }

  return;                 
} // End function propagatedErrorStats

// Save pinhole camera positions and orientations in a single file.
// Only works with Pinhole cameras.
void saveCameraReport(asp::BaBaseOptions const& opt, asp::BAParams const& param_storage,
                      vw::cartography::Datum const& datum, 
                      std::string const& prefix) {

  std::string output_path = opt.out_prefix + "-" + prefix + "-cameras.csv";
  vw_out() << "Writing: " << output_path << std::endl;
  std::ofstream fh(output_path.c_str());
  fh.precision(17);
  fh << "# input_cam_file, cam_ctr_x, cam_ctr_y, cam_ctr_z (ecef meters), "
     << "cam2ned rotation rows\n";
  
  int num_cameras = opt.image_files.size();

  // TODO(oalexan1): Create here a report file. Write camera name,
  // camera center, ecef position, ecef quaternion, and ned roll-pitch-yaw.
  // Use same Euler angles as in numpy. Likely eigen can do it.
  for (int icam = 0; icam < num_cameras; icam++) {

    vw::Vector3 cam_ctr;
    vw::Matrix3x3 cam2ecef;
    switch(opt.camera_type) {
      case BaCameraType_Pinhole: {
        // Get the camera model from the original one with parameters in
        // param_storage applied to it (which could be original ones or optimized). 
        // Note that we do not modify the original camera.
        vw::camera::PinholeModel const* in_cam
          = dynamic_cast<vw::camera::PinholeModel const*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
        // Apply current intrinsics and extrinsics to the camera
        vw::camera::PinholeModel out_cam 
          = transformedPinholeCamera(icam, param_storage, *in_cam);
        cam_ctr = out_cam.camera_center(vw::Vector2());
        cam2ecef = out_cam.get_rotation_matrix();
        break;
      }
      case BaCameraType_OpticalBar:
        vw::vw_throw(vw::ArgumentErr() << "Saving a camera report is not implemented "
                    << "for optical bar cameras.\n");
        break;
      case BaCameraType_CSM:
        vw::vw_throw(vw::ArgumentErr() << "Saving a camera report is not implemented "
                      << "for CSM cameras.\n");
        break;
      default: {
        // Apply extrinsics adjustments to a pinhole camera
        // TODO(oalexan1): Make this into a function called adjustedPinholeCamera().
        // Use it where needed.
        CameraAdjustment adjustment(param_storage.get_camera_ptr(icam));
        PinholeModel* in_cam = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
        
        // Make a copy of the camera, and apply the adjustments to the copy. Need to go back
        // to the original camera to get the adjustments needed to apply.
        // TODO(oalexan1): This is a little awkward.
        PinholeModel out_cam = *in_cam;
        AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                                    adjustment.position(), adjustment.pose());
        vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
        out_cam.apply_transform(ecef_transform);
        cam_ctr = out_cam.camera_center(vw::Vector2());
        cam2ecef = out_cam.get_rotation_matrix();
      }
    }

    fh << opt.camera_files[icam] << ", "
       << cam_ctr[0] << ", " << cam_ctr[1] << ", " << cam_ctr[2];

    // Find the matrix for converting NED to ECEF
    vw::Vector3 loc_llh = datum.cartesian_to_geodetic(cam_ctr);
    vw::Matrix3x3 ned2ecef = datum.lonlat_to_ned_matrix(loc_llh);

    // How a camera moves relative to the world is given by the camera-to-world
    // matrix. That is a little counter-intuitive.
    vw::Matrix3x3 cam2ned = inverse(ned2ecef) * cam2ecef;
    for (int row = 0; row < cam2ned.rows(); row++) {
      for (int col = 0; col < cam2ned.cols(); col++) {
        fh << ", " << cam2ned(row, col);
      } 
    }
    fh << "\n";
  }

  fh.close();
  return;
}

// Save stats of horizontal and vertical errors propagated from cameras
// to triangulation
void saveHorizVertErrors(std::string const& horiz_vert_errors_file,
                         std::vector<asp::HorizVertErrorStats> const& horizVertErrors,
                         std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << horiz_vert_errors_file << "\n";
  std::ofstream ofs (horiz_vert_errors_file.c_str());
  ofs.precision(8);
  ofs << "# Horizontal and vertical propagated triangulation uncertainties (in meters) for each image pair having matches.\n";
  ofs << "# left_image right_image horiz_error_median vert_error_median horiz_error_mean vert_error_mean horiz_error_stddev vert_error_stddev num_meas\n";
  for (size_t conv_it = 0; conv_it < horizVertErrors.size(); conv_it++) {
     auto const & c = horizVertErrors[conv_it]; // alias
     ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
         << c.horiz_error_median << ' ' << c.vert_error_median << ' '
         << c.horiz_error_mean   << ' ' << c.vert_error_mean   << ' '
         << c.horiz_error_stddev << ' ' << c.vert_error_stddev << ' '
         << c.num_errors << "\n";
  }
  ofs.close();

  return;
} 

} // end namespace asp
