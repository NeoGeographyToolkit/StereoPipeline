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


/// \file BundleAdjustCamera.h
///

/// Camera logic used in bundle_adjust. It is kept here as it may be
/// expected to make use of all cameras supported by ASP.

#ifndef __BUNDLE_ADJUST_CAMERA_H__
#define __BUNDLE_ADJUST_CAMERA_H__

#include <string>

// TODO(oalexan1): Move most of these headers to the .cc file
#include <vw/Camera/CameraUtilities.h>
#include <vw/BundleAdjustment/AdjustRef.h>
#include <vw/FileIO/KML.h>
#include <vw/BundleAdjustment/ModelBase.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Math/Geometry.h>

#include <asp/Core/FileUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/BundleAdjustUtils.h>

// TODO(oalexan1): This header file is slow to compile. See if it can be moved
// to .cc.
#include <asp/Core/EigenUtils.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

// TODO(oalexan1): Move all this code to the .cc file and to the asp namespace.
// TODO(oalexan1): Remove these directives from the header file

namespace asp {
  std::string UNSPECIFIED_DATUM = "unspecified_datum";
}

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

/// Structure to fully describe how the intrinsics are being handled.
/// - Currently only pinhole cameras support intrinsics in bundle_adjust.
struct IntrinsicOptions {
  bool center_constant;
  bool center_shared;
  bool focus_constant;
  bool focus_shared;
  bool distortion_constant;
  bool distortion_shared;

  IntrinsicOptions() : center_constant    (true), center_shared    (true),
                       focus_constant     (true), focus_shared     (true),
                       distortion_constant(true), distortion_shared(true)
  {}
};

/// Class to store parameters as they are being bundle adjusted.
/// - Currently only supports either one camera or all unique cameras.
class BAParamStorage {

public:

  static const int PARAMS_PER_POINT  = 3;
  static const int NUM_CAMERA_PARAMS = 6; // Position and pose.
  // These two are only for pinhole cameras.
  static const int NUM_CENTER_PARAMS = 2; // TODO: Share info with other classes!
  static const int NUM_FOCUS_PARAMS  = 1;

  
  boost::random::mt19937 m_rand_gen;
  
  BAParamStorage(int num_points, int num_cameras,
                 // Parameters below here only apply to pinhole models.
                 bool using_intrinsics=false,
                 int num_distortion_params=0,
                 IntrinsicOptions  intrin_opts=IntrinsicOptions()
                )
    : m_num_points        (num_points),
      m_num_cameras       (num_cameras),
      m_params_per_point  (PARAMS_PER_POINT),
      m_num_pose_params   (NUM_CAMERA_PARAMS),
      m_num_shared_intrinsics    (0),
      m_num_intrinsics_per_camera(0),
      m_num_distortion_params(num_distortion_params),
      m_focus_offset(0),
      m_distortion_offset(0),
      m_intrin_options    (intrin_opts),
      m_points_vec        (num_points *PARAMS_PER_POINT,  0),
      m_cameras_vec       (num_cameras*NUM_CAMERA_PARAMS, 0),
      m_intrinsics_vec    (0),
      m_outlier_points_vec(num_points, false),
      m_rand_gen(std::time(0)) {

        if (!using_intrinsics)
          return; // If we are not using intrinsics, nothing else to do.

        // Calculate how many values are stored per-camera, and
        //  what the offset is to a particular intrinsic value.
        // - The start of the array is always an entry for each intrinsic in case
        //   it is shared.
        if (!intrin_opts.center_shared)
          m_num_intrinsics_per_camera += NUM_CENTER_PARAMS;
        if (intrin_opts.focus_shared)
          m_focus_offset = NUM_CENTER_PARAMS;
        else {
          m_num_intrinsics_per_camera += NUM_FOCUS_PARAMS;
          if (!intrin_opts.center_shared)
            m_focus_offset = NUM_CENTER_PARAMS;
        }
        if (intrin_opts.distortion_shared)
          m_distortion_offset = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS;
        else {
          m_num_intrinsics_per_camera += num_distortion_params;
          if (!intrin_opts.center_shared)
            m_distortion_offset += NUM_CENTER_PARAMS;
          if (!intrin_opts.focus_shared)
            m_distortion_offset += NUM_FOCUS_PARAMS;
        }

        // For simplicity, we always set this to the same size even
        //  if none of the parameters are shared.
        m_num_shared_intrinsics = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS
                                  + num_distortion_params;

        m_intrinsics_vec.resize(m_num_shared_intrinsics +
                                num_cameras*m_num_intrinsics_per_camera);
      }

  // Copy constructor
  BAParamStorage(BAParamStorage const& other)
    : m_num_points        (other.m_num_points),
      m_num_cameras       (other.m_num_cameras),
      m_params_per_point  (other.m_params_per_point),
      m_num_pose_params   (other.m_num_pose_params),
      m_num_shared_intrinsics    (other.m_num_shared_intrinsics),
      m_num_intrinsics_per_camera(other.m_num_intrinsics_per_camera),
      m_num_distortion_params(other.m_num_distortion_params),
      m_focus_offset      (other.m_focus_offset),
      m_distortion_offset (other.m_distortion_offset),
      m_intrin_options    (other.m_intrin_options),
      m_points_vec        (other.m_points_vec.size()        ),
      m_cameras_vec       (other.m_cameras_vec.size()       ),
      m_intrinsics_vec    (other.m_intrinsics_vec.size()    ),
      m_outlier_points_vec(other.m_outlier_points_vec.size()),
      m_rand_gen(std::time(0)) {
    copy_points    (other);
    copy_cameras   (other);
    copy_intrinsics(other);
    copy_outliers  (other);
  }

  // Set all camera position and pose values to zero.
  void clear_cameras() {
    for (int i=0; i < m_cameras_vec.size(); ++i)
      m_cameras_vec[i] = 0.0;
  }

  // When using the copy functions, the sizes must match!

  /// Copy one set of values from another instance.
  void copy_points(BAParamStorage const& other) {
    for (size_t i=0; i<m_points_vec.size(); ++i)
      m_points_vec[i] = other.m_points_vec[i];
  }
  void copy_cameras(BAParamStorage const& other) {
    for (size_t i=0; i<m_cameras_vec.size(); ++i)
      m_cameras_vec[i] = other.m_cameras_vec[i];
  }
  void copy_intrinsics(BAParamStorage const& other) {
    for (size_t i=0; i<m_intrinsics_vec.size(); ++i)
      m_intrinsics_vec[i] = other.m_intrinsics_vec[i];
  }
  void copy_outliers(BAParamStorage const& other) {
    for (size_t i=0; i<m_outlier_points_vec.size(); ++i)
      m_outlier_points_vec[i] = other.m_outlier_points_vec[i];
  }

  /// Apply a random offset to each camera position.
  void randomize_cameras() {
    // These are stored as x,y,z, axis_angle.
    // - We move the position +/- 5 meters.
    // - Currently we don't adjust the angle.
    boost::random::uniform_int_distribution<> xyz_dist(0, 10);
    const size_t NUM_CAMERA_PARAMS = 6;
    VW_ASSERT((m_cameras_vec.size() % NUM_CAMERA_PARAMS) == 0,
                LogicErr() << "Camera parameter length is not a multiple of 6!");
    const size_t num_cameras = m_cameras_vec.size() / NUM_CAMERA_PARAMS;
    for (size_t c=0; c<num_cameras; ++c) {
      double* ptr = get_camera_ptr(c);
      for (size_t i=0; i<3; ++i) {
        int o = xyz_dist(m_rand_gen) - 5;
        ptr[i] += o;
      }
      //for (size_t i=3; i<PARAMS_PER_CAM; ++i) {
      //}
    }
  }

  /// Randomly scale each intrinsic value.
  void randomize_intrinsics(std::vector<double> const& intrinsic_limits) {
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
    for (size_t c=0; c<num_cameras(); ++c) { // For each camera...
      size_t intrin_index = 0;
      if (!m_intrin_options.focus_constant && !(m_intrin_options.focus_shared && (c>0))) {
        double* ptr = get_intrinsic_focus_ptr(c);
        for (int i=0; i<NUM_FOCUS_PARAMS; ++i) {
          percent = static_cast<double>(dist(m_rand_gen))/DENOM;
          if (intrin_index < num_intrinsics) {
            range = intrinsic_limits[2*intrin_index+1] - intrinsic_limits[2*intrin_index];
            scale = percent*range + intrinsic_limits[2*intrin_index];
          } else
            scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
          ptr[i] *= scale;
          ++intrin_index;
        }
      } // End focus case
      intrin_index = NUM_FOCUS_PARAMS; // In case we did not go through the loop
      if (!m_intrin_options.center_constant && !(m_intrin_options.center_shared && (c>0))) {
        double* ptr = get_intrinsic_center_ptr(c);
        for (int i=0; i<NUM_CENTER_PARAMS; ++i) {
          percent = static_cast<double>(dist(m_rand_gen))/DENOM;
          if (intrin_index < num_intrinsics) {
            range = intrinsic_limits[2*intrin_index+1] - intrinsic_limits[2*intrin_index];
            scale = percent*range + intrinsic_limits[2*intrin_index];
          } else
            scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
          ptr[i] *= scale;
          ++intrin_index;
        }
      } // End center case
      intrin_index = NUM_FOCUS_PARAMS+NUM_CENTER_PARAMS; // In case we did not go through the loops
      if (!m_intrin_options.distortion_constant && !(m_intrin_options.distortion_shared && (c>0))) {
        double* ptr = get_intrinsic_distortion_ptr(c);
        for (int i=0; i<m_num_distortion_params; ++i) {
          percent = static_cast<double>(dist(m_rand_gen))/DENOM;
          if (intrin_index < num_intrinsics) {
            range = intrinsic_limits[2*intrin_index+1] - intrinsic_limits[2*intrin_index];
            scale = percent*range + intrinsic_limits[2*intrin_index];
          } else
            scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
          ptr[i] *= scale;
          ++intrin_index;
        }
      } // End distortion case
    } // End camera loop
  }
  
  int num_points       () const {return m_num_points; }
  int num_cameras      () const {return m_num_cameras;}
  int num_intrinsics   () const {return m_num_intrinsics_per_camera;} // Per camera
  int params_per_point () const {return m_params_per_point;}
  int params_per_camera() const {return m_num_pose_params;}

  std::vector<double> & get_point_vector() {
    return m_points_vec;
  }
  std::vector<double> & get_camera_vector() {
    return m_cameras_vec;
  }
  std::vector<double> & get_intrinsics_vector() {
    return m_intrinsics_vec;
  }
  
  double* get_point_ptr(int point_index) {
    return &(m_points_vec[point_index*m_params_per_point]);
  }
  double const* get_point_ptr(int point_index) const {
    return &(m_points_vec[point_index*m_params_per_point]);
  }
  
  double* get_camera_ptr(int cam_index) {
    return &(m_cameras_vec[cam_index*m_num_pose_params]);
  }
  double const* get_camera_ptr(int cam_index) const {
    return &(m_cameras_vec[cam_index*m_num_pose_params]);
  }

  // ------- These functions are only needed for pinhole cameras ------
  double* get_intrinsic_center_ptr(int cam_index) {
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_center_offset(cam_index)]);
  }
  double const* get_intrinsic_center_ptr(int cam_index) const {
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_center_offset(cam_index)]);
  }
  
  double* get_intrinsic_focus_ptr(int cam_index) {
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_focus_offset(cam_index)]);
  }
  double const* get_intrinsic_focus_ptr(int cam_index) const{
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_focus_offset(cam_index)]);
  }

  double* get_intrinsic_distortion_ptr(int cam_index) {
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_distortion_offset(cam_index)]);
  }
  double const* get_intrinsic_distortion_ptr(int cam_index) const{
    if (m_intrinsics_vec.empty()) return 0;
    return &(m_intrinsics_vec[get_distortion_offset(cam_index)]);
  }
  // ------- End pinhole camera functions ------
  
  void set_point_outlier(int point_index, bool status) {
    m_outlier_points_vec[point_index] = status;
  }
  bool get_point_outlier(int point_index) const {
    return m_outlier_points_vec[point_index];
  }

  /// Return the number of points flagged as outliers.
  int get_num_outliers() const {
    int count = 0;
    for (size_t i=0; i<m_num_points; ++i) {
      if (m_outlier_points_vec[i])
        ++count;
    }
    return count;
  }
  
  /// Get the values for a point
  vw::Vector3 get_point(int point_index)  const{
    double const* ptr = get_point_ptr(point_index);
    vw::Vector3 pt;
    for (int i=0; i<3; ++i)
      pt[i] = ptr[i];
    return pt;
  }
  
  /// Update the values for a point
  void set_point(int point_index, vw::Vector3 const& pt) {
    double* ptr = get_point_ptr(point_index);
    for (int i=0; i<3; ++i)
      ptr[i] = pt[i];
  }

  /// Print stats for optimized ground control points.
  void print_gcp_stats(vw::ba::ControlNetwork const& cnet,
                       vw::cartography::Datum const& d) {
    vw::vw_out() << "Ground control point results:\n";
    vw::vw_out() << "input_gcp optimized_gcp diff\n";
    for (int ipt = 0; ipt < num_points(); ipt++){
      if (cnet[ipt].type() != vw::ba::ControlPoint::GroundControlPoint)
        continue;
      if (get_point_outlier(ipt))
        continue; // skip outliers

      vw::Vector3 input_gcp = cnet[ipt].position();
      vw::Vector3 opt_gcp   = get_point(ipt);

      vw::vw_out() << "xyz: " << input_gcp << ' ' << opt_gcp << ' '
                   << input_gcp - opt_gcp << std::endl;

      // Now convert to llh
      input_gcp = d.cartesian_to_geodetic(input_gcp);
      opt_gcp   = d.cartesian_to_geodetic(opt_gcp  );

      vw::vw_out() << "llh: " << input_gcp << ' ' << opt_gcp << ' '
                   << input_gcp - opt_gcp << std::endl;
    }
  }

  /// Create a KML file containing the positions of the given points.
  /// - Points are stored as x,y,z in the points vector up to num_points.
  /// - Only every skip'th point is recorded to the file.
  void record_points_to_kml(const std::string &kml_path,
                            const vw::cartography::Datum& datum,
                            size_t skip=100, const std::string name="points",
                            const std::string icon="http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png") {

    if (datum.name() == asp::UNSPECIFIED_DATUM) {
      vw::vw_out(vw::WarningMessage) << "No datum specified, can't write file: " << kml_path << std::endl;
      return;
    }

    // Open the file
    vw_out() << "Writing: " << kml_path << std::endl;
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
      kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                            "", "", "point_placemark",
                            lon_lat_alt[2], extrude );
    }
    kml.close_kml();
  }


private: // Variables

  int m_num_points, m_num_cameras, m_params_per_point, m_num_pose_params;
  
  // m_intrinsics_vec starts out with m_num_shared_intrinsics values which are
  //  shared between all cameras, followed by the per-camera intrinsics for each camera.
  int m_num_shared_intrinsics, m_num_intrinsics_per_camera, m_num_distortion_params;
  
  // These store the offset to the focus or distortion data from the start of
  //  either the shared parameters at the start of m_intrinsics_vec or from
  //  the block of intrinsics data for the specified camera.
  int m_focus_offset, m_distortion_offset;
  
  IntrinsicOptions m_intrin_options;
  
  // Raw data storage
  std::vector<double> m_points_vec, m_cameras_vec, m_intrinsics_vec;
  std::vector<bool> m_outlier_points_vec;

private: // Functions

  /// Compute the offset in m_intrinsics_vec to the requested data.
  size_t get_center_offset(int cam_index) const {
    if (m_intrin_options.center_shared)
      return 0;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera;
  }
  size_t get_focus_offset(int cam_index) const {
    if (m_intrin_options.focus_shared)
      return m_focus_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + m_focus_offset;
  }
  size_t get_distortion_offset(int cam_index) const {
    if (m_intrin_options.distortion_shared)
      return m_distortion_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + m_distortion_offset;
  }
}; // End class BAParamStorage


/// Simple class to manage position/rotation information.
/// - This is the data type stored in pc_align output files,
///   bundle adjustment files, and the position of pinhole cameras.
class CameraAdjustment {
public:

  // Constructors
  CameraAdjustment() {}
  CameraAdjustment(double const* array) {read_from_array(array); }

  // Data access
  vw::Vector3 position() const { return m_position_data; }
  vw::Quat    pose    () const { return m_pose_data;     }
  
  /// Populate from a six element array.
  void read_from_array(double const* array) {
    m_position_data = vw::Vector3(array[0], array[1], array[2]);
    m_pose_data     = vw::math::axis_angle_to_quaternion(Vector3(array[3], array[4], array[5]));
  }

  /// Populate from an AdjustedCameraModel
  void copy_from_adjusted_camera(vw::camera::AdjustedCameraModel const& cam) {
    m_position_data = cam.translation();
    m_pose_data     = cam.rotation();
  }

  /// Populate from a PinholeModel
  void copy_from_pinhole(vw::camera::PinholeModel const& cam) {
    m_position_data = cam.camera_center();
    m_pose_data     = cam.camera_pose();
  }

  /// Populate from OpticalBarModel
  void copy_from_optical_bar(vw::camera::OpticalBarModel const& cam) {
    m_position_data = cam.camera_center();
    m_pose_data     = cam.camera_pose();
  }

  /// Populate from an adjustment file on disk.
  void read_from_adjust_file(std::string const& filename) {
    
    // Vectors, but we only use the first position.
    std::vector<vw::Vector3> position_correction;
    std::vector<vw::Quat>    pose_correction;

    // Not used, just for the api
    bool piecewise_adjustments = false;
    vw::Vector2 adjustment_bounds;
    Vector2 pixel_offset = Vector2();
    double scale = 1.0;
    std::string session;
    
    vw::vw_out() << "Reading adjusted camera model: " << filename << std::endl;
    asp::read_adjustments(filename, piecewise_adjustments,
                          adjustment_bounds, position_correction, pose_correction,
                          pixel_offset, scale, session);
    m_position_data = position_correction[0];
    m_pose_data     = pose_correction    [0];
  }
  
  /// Pack the data to a six element array.
  void pack_to_array(double* array) const {
    vw::Vector3 pose_vec = m_pose_data.axis_angle();
    const int VEC_SIZE = 3;
    for (size_t i = 0; i < VEC_SIZE; i++) {
      array[i           ] = m_position_data[i];
      array[i + VEC_SIZE] = pose_vec[i];
    }
  }
  
private:
  vw::Vector3 m_position_data;
  vw::Quat    m_pose_data;

}; // End class CameraAdjustment

//==================================================================================

// TODO: Move to a class?
/// Packs info from a pinhole camera into the provided arrays.
/// - It is up to the caller to make sure the arrays are properly sized.
void pack_pinhole_to_arrays(vw::camera::PinholeModel const& camera,
                            int camera_index,
                            BAParamStorage & param_storage) {

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
  center_ptr[0] = 1.0; //camera.point_offset()[0];
  center_ptr[1] = 1.0; //camera.point_offset()[1];
  focus_ptr [0] = 1.0; //camera.focal_length()[0];

  // Pack the lens distortion parameters.
  vw::Vector<double> lens = camera.lens_distortion()->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    distortion_ptr[i] = 1.0;
}


void pack_optical_bar_to_arrays(vw::camera::OpticalBarModel const& camera,
                                int camera_index,
                                BAParamStorage & param_storage) {

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
  center_ptr[0] = 1.0; //camera.point_offset()[0];
  center_ptr[1] = 1.0; //camera.point_offset()[1];
  focus_ptr [0] = 1.0; //camera.focal_length()[0];

  // Pack the speed, MCF, and scan time into the distortion pointer.
  intrinsics_ptr[0] = 1.0;
  intrinsics_ptr[1] = 1.0;
  intrinsics_ptr[2] = 1.0;
}


/// Modify an existing pinhole model in-place using the stored parameters.
/// - Since the stored parameters are multipliers, be careful calling this more than once!
// TODO(oalexan1): This is very fragile code!
void populate_pinhole_from_arrays(int camera_index,
                                  BAParamStorage const& param_storage,
                                  vw::camera::PinholeModel & camera) {

  double const* pos_pose_ptr   = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  camera.set_camera_center(pos_pose_info.position());
  camera.set_camera_pose  (pos_pose_info.pose    ());

  // All intrinsic parameters are stored as multipliers!

  // Update the lens distortion parameters.
  boost::shared_ptr<LensDistortion> distortion = camera.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    lens[i] *= distortion_ptr[i];

  distortion->set_distortion_parameters(lens);
  camera.set_lens_distortion(distortion.get());

  // Update the center and focus
  Vector2 old_center = camera.point_offset();
  Vector2 old_focus  = camera.focal_length();
  camera.set_point_offset(Vector2(center_ptr[0]*old_center[0],
                                  center_ptr[1]*old_center[1]), false);
  double new_focus = old_focus[0]*focus_ptr[0];
  camera.set_focal_length(Vector2(new_focus,new_focus), true); // Recompute internals.
}


/// Modify an existing optical bar model in-place using the stored parameters.
/// - Since the stored parameters are multipliers, be careful calling this more than once!
void populate_optical_bar_from_arrays(int camera_index,
                                      BAParamStorage const& param_storage,
                                      vw::camera::OpticalBarModel & camera) {

  double const* pos_pose_ptr  = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr    = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  camera.set_camera_center(pos_pose_info.position());
  camera.set_camera_pose  (pos_pose_info.pose    ());

  // All intrinsic parameters are stored as multipliers!

  // Update the other intrinsic parameters.
  camera.set_speed              (camera.get_speed()*intrinsic_ptr[0]);
  camera.set_motion_compensation(camera.get_motion_compensation()*intrinsic_ptr[1]);
  camera.set_scan_time          (camera.get_scan_time()*intrinsic_ptr[2]);


  // Update the center and focus
  Vector2 old_center = camera.get_optical_center();
  float   old_focus  = camera.get_focal_length();
  camera.set_optical_center(Vector2(center_ptr[0]*old_center[0],
                                    center_ptr[1]*old_center[1]));
  double new_focus = old_focus*focus_ptr[0];
  camera.set_focal_length(new_focus);
}


// TODO: Class function?
/// Given a transform with origin at the planet center, like output
/// by pc_align, read the adjustments from cameras_vec, apply this
/// transform on top of them, and write the adjustments back to the vector.
/// - Works for pinhole and non-pinhole case.
void apply_transform_to_cameras(vw::Matrix4x4 const& M, BAParamStorage &param_storage,
                                std::vector<boost::shared_ptr<camera::CameraModel> > const& cam_ptrs){

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {

    // Load the current position/pose of this camera.
    double* cam_ptr = param_storage.get_camera_ptr(i);
    CameraAdjustment cam_adjust(cam_ptr);

    // Create the adjusted camera model
    vw::camera::AdjustedCameraModel cam(cam_ptrs[i], cam_adjust.position(), cam_adjust.pose());
    // Apply the transform
    cam.apply_transform(M);

    // Copy back the adjustments to the camera array.
    cam_adjust.copy_from_adjusted_camera(cam);
    cam_adjust.pack_to_array(cam_ptr);
  }
} // end function apply_transform_to_cameras


// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M, BAParamStorage &param_storage,
                                        std::vector<boost::shared_ptr<camera::CameraModel> > const& cam_ptrs){

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
    boost::shared_ptr<camera::PinholeModel> pin_ptr = 
      boost::dynamic_pointer_cast<vw::camera::PinholeModel>(cam_ptrs[i]);
    pin_ptr->apply_transform(R, T, scale);

    // Write out to param_storage
    pack_pinhole_to_arrays(*pin_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_pinhole

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M, BAParamStorage &param_storage,
                                        std::vector<boost::shared_ptr<camera::CameraModel> > const& cam_ptrs){

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

} // end function apply_transform_to_cameras_pinhole


//=================================================================

/// Load all of the reference disparities specified in the input text file
/// and store them in the vectors.  Return the number loaded.
template <class DispPixelT>
int load_reference_disparities(std::string const& disp_list_filename,
                               std::vector< ImageView   <DispPixelT> > &disp_vec,
                               std::vector< ImageViewRef<DispPixelT> > &interp_disp) {
  // TODO: Disparities can be large, but if small it is better to
  // read them in memory.
  std::istringstream is(disp_list_filename);
  std::string disp_file;
  while (is >> disp_file) {
    if (disp_file != "none") {
      vw_out() << "Reading: " << disp_file << std::endl;
      disp_vec.push_back(copy(DiskImageView<DispPixelT>(disp_file)));
    }else{
      // Read in an empty disparity
      disp_vec.push_back(ImageView<DispPixelT>());
    }
    interp_disp.push_back(interpolate(disp_vec.back(),
                                      BilinearInterpolation(), ConstantEdgeExtension()));
  }
  return static_cast<int>(disp_vec.size());
}

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<boost::shared_ptr<CameraModel> > &camera_models,
                           boost::shared_ptr<ControlNetwork> const& cnet) {

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
void check_gcp_dists(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                     boost::shared_ptr<ControlNetwork> const& cnet_ptr,
                     double forced_triangulation_distance) {

  // Make one iteration just to count the points.
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  const int num_cnet_points = static_cast<int>(cnet.size());
  double gcp_count=0, ip_count=0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++){

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;
    
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      gcp_count += 1.0;
    else {
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double ans = vw::ba::triangulate_control_point(cp_new, camera_models, minimum_angle,
						     forced_triangulation_distance);
      if (ans < 0 || cp_new.position() == Vector3())
        continue; // Skip points that fail to triangulate

      ip_count += 1.0;
    }
  } // End loop through control network points
  if (ip_count == 0 || gcp_count == 0)
    return; // Can't do this check if we don't have both point types.

  // Make another iteration to compute the mean.
  Vector3 mean_gcp(0,0,0);
  Vector3 mean_ip (0,0,0);
  for (int ipt = 0; ipt < num_cnet_points; ipt++){

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;
    
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {

      mean_gcp += (cnet[ipt].position() / gcp_count);
      ++gcp_count;
    }else {
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double ans = vw::ba::triangulate_control_point(cp_new, camera_models, minimum_angle,
						     forced_triangulation_distance);
      // Skip points for which triangulation failed
      if (cp_new.position() == Vector3() || ans < 0)
	continue;

      mean_ip += (cp_new.position() / ip_count);
    }
  } // End loop through control network points

  double dist = norm_2(mean_ip - mean_gcp);
  if (dist > 100000)
    vw_out() << "WARNING: GCPs are over 100 km from the other points. Are your lat/lon GCP coordinates swapped?\n";
}

//============================================================================

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided camera positions.
/// - This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions
(boost::shared_ptr<ControlNetwork> const& cnet, 
 std::vector<boost::shared_ptr<CameraModel> > & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<Vector3> const & estimated_camera_gcc) {

  vw_out() << "Initializing camera positions from input file..." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = image_files.size();
  if (int(estimated_camera_gcc.size()) != num_cameras)
    vw_throw( ArgumentErr() << "No camera matches provided to init function!\n" );

  vw_out() << "Num cameras: " << num_cameras << std::endl;

  int num_matches_found = 0;
  for (int i=0; i<num_cameras; ++i)
    if (estimated_camera_gcc[i] != Vector3(0,0,0))
      ++num_matches_found;

  vw_out() << "Number of matches found: " << num_matches_found << std::endl;

  const int MIN_NUM_MATCHES = 3;
  if (num_matches_found < MIN_NUM_MATCHES)
    vw_throw( ArgumentErr() << "At least " << MIN_NUM_MATCHES 
              << " camera position matches are required to initialize sensor models!\n" );

  // Populate matrices containing the current and known camera positions.
  vw::Matrix<double> points_in(3, num_matches_found), points_out(3, num_matches_found);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  int index = 0;
  for (int i=0; i<num_cameras; ++i) {
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

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void init_pinhole_model_with_multi_gcp(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
				       std::vector<boost::shared_ptr<CameraModel> > & camera_models) {
  
  vw_out() << "Initializing camera positions from ground control points.\n";
  vw_out()<< "Assume at least three GCP are seen in at least two images.\n";
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  
  // DEBUG: Print out all pinhole cameras and verify they are pinhole cameras.
  for (size_t icam = 0; icam < camera_models.size(); icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL,
	      vw::ArgumentErr() << "A pinhole camera expected.\n");
  }
  
  // Count up the number of good ground control points
  // - Maybe this should be a function of the ControlNet class?
  const int num_cnet_points = static_cast<int>(cnet.size());
  int num_gcp      = 0;
  int num_good_gcp = 0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++){
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
    ++num_gcp;
    
    // Use triangulation to estimate the position of this control point using
    //   the current set of camera models.
    ControlPoint cp_new = cnet[ipt];
    // Making minimum_angle below big may throw away valid points at this stage // really???
    double minimum_angle = 0;
    double forced_triangulation_distance = -1;
    double err = vw::ba::triangulate_control_point(cp_new, camera_models,
						   minimum_angle, forced_triangulation_distance);
    if ((cp_new.position() != Vector3()) && (cnet[ipt].position() != Vector3()) &&
	// Note that if there is only one camera, we allow
	// for triangulation to return a half-baked answer,
	// as then there is nothing we can do. We need this
	// answer, as imperfect as it is, to create initial
	// camera models from GCP.
	(err > 0 || camera_models.size() == 1)
	)
      ++num_good_gcp; // Only count points that triangulate
    else {
      vw_out() << "Discarding GCP: " << cnet[ipt]; // Built in endl
    }
  } // End good GCP counting
  
    // Update the number of GCP that we are using
  const int MIN_NUM_GOOD_GCP = 3;
  if (num_good_gcp < MIN_NUM_GOOD_GCP) {
    vw_out() << "Num GCP       = " << num_gcp         << std::endl;
    vw_out() << "Num valid GCP = " << num_good_gcp    << std::endl;
    vw_throw( ArgumentErr()
	      << "Not enough valid GCPs for affine transform pinhole initialization. "
	      << "You may need to use --disable-pinhole-gcp-init or "
	      << "--transform-cameras-using-gcp.\n" );
  }
  
  vw::Matrix<double> points_in(3, num_good_gcp), points_out(3, num_good_gcp);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  int index = 0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++){
    // Loop through all the ground control points only
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
    
    // Use triangulation to estimate the position of this control point using
    //   the current set of camera models.
    ControlPoint cp_new = cnet[ipt];
    // Making minimum_angle below big may throw away valid points at this stage // really???
    double minimum_angle = 0;
    double forced_triangulation_distance = -1;
    vw::ba::triangulate_control_point(cp_new, camera_models,
				      minimum_angle, forced_triangulation_distance);
    
    // Store the computed and correct position of this point in Eigen matrices
    Vector3 inp  = cp_new.position();
    Vector3 outp = cnet[ipt].position();
    if (inp == Vector3() || outp == Vector3())
      continue; // Skip points that fail to triangulate
    
    // Store in matrices
    ColView colIn (points_in,  index); 
    ColView colOut(points_out, index);
    colIn  = inp;
    colOut = outp;
    
    ++index;
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
  vw_out() << "This transform can be disabled with --disable-pinhole-gcp-init\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);

} // End function init_pinhole_model_with_gcp

// Given original cams in sfm_cams and individually scaled cameras in
// aux_cams, get the median scale change from the first set to the second one.
// It is important to do the median, since scaling the cameras individually
// is a bit of a shaky business.
double find_median_scale_change(std::vector<PinholeModel> const & sfm_cams,
				std::vector<PinholeModel> const & aux_cams,
				std::vector< std::vector<Vector3> > const& xyz){
  
  int num_cams = sfm_cams.size();

  std::vector<double> scales;
  
  for (int it1 = 0; it1 < num_cams; it1++) {

    bool is_good = (xyz[it1].size() >= 3);
    if (!is_good)
      continue;
    
    for (int it2 = it1 + 1; it2 < num_cams; it2++) {
      
      bool is_good = (xyz[it2].size() >= 3);
      if (!is_good)
	continue;
    
      double len1 = norm_2(sfm_cams[it1].camera_center()
			   - sfm_cams[it2].camera_center());
      double len2 = norm_2(aux_cams[it1].camera_center()
			   - aux_cams[it2].camera_center());
      
      double scale = len2/len1;
      scales.push_back(scale);
    }
  }

  if (scales.empty())
    vw_throw( LogicErr() << "Could not find two images with at least 3 GCP each.\n");
    
  double median_scale = vw::math::destructive_median(scales);

  return median_scale;
}


// Given some GCP so that at least two images have at at least three GCP each,
// but each GCP is allowed to show in one image only, use the GCP
// to transform cameras to ground coordinates.
void align_cameras_to_ground(std::vector< std::vector<Vector3> > const& xyz,
			     std::vector< std::vector<Vector2> > const& pix,
			     std::vector<PinholeModel> & sfm_cams,
			     Matrix3x3 & rotation, 
			     Vector3 & translation,
			     double & scale){
  
  std::string camera_type = "pinhole";
  bool refine_camera = true;
  bool verbose = false; 

  // Cameras individually aligned to ground using GCP. They may not be
  // self-consistent, and are only used to give an idea of the
  // transform to apply to the unaligned cameras.
  std::vector<PinholeModel> aux_cams;

  int num_cams = sfm_cams.size();
  for (int it = 0; it < num_cams; it++) {
    // Export to the format used by the API
    std::vector<double> pixel_values;
    for (size_t c = 0; c < pix[it].size(); c++) {
      pixel_values.push_back(pix[it][c][0]);
      pixel_values.push_back(pix[it][c][1]);
    }

    boost::shared_ptr<CameraModel> out_cam(new PinholeModel(sfm_cams[it]));

    bool is_good = (xyz[it].size() >= 3);
    if (is_good) 
      fit_camera_to_xyz(camera_type, refine_camera,  
			xyz[it], pixel_values, verbose, out_cam);
    
    aux_cams.push_back(*((PinholeModel*)out_cam.get()));
  }

  double world_scale = find_median_scale_change(sfm_cams, aux_cams, xyz);
  vw_out() << "Initial guess scale to apply when converting to world coordinates using GCP: "
	   << world_scale << ".\n";

  // So far we aligned both cameras individually to GCP and we got an
  // idea of scale.  Yet we would like to align them without changing
  // the relationship between them, so using a single transform for
  // all not an individual transform for each.  This way we will
  // transform the SfM-computed cameras to the new coordinate system.

  // Start by estimating a such a transform.
  int num_pts = 0;
  for (int it = 0; it < num_cams; it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) 
      num_pts += pix[it].size();
  }
  
  vw::Matrix<double> in_pts, out_pts;
  in_pts.set_size(3, num_pts);
  out_pts.set_size(3, num_pts);
  
  int col = 0;
  for (int it = 0; it < num_cams; it++) {
    
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) {
      // For each camera, find xyz values in the input cameras
      // that map to GCP. Use the scale for that.
      for (int c = 0; c < xyz[it].size(); c++) {
	
	// Distance from camera center to xyz for the individually aligned cameras
	double len = norm_2(aux_cams[it].camera_center() - xyz[it][c]);
	len = len / world_scale;
	Vector3 trans_xyz = sfm_cams[it].camera_center()
	  + len * sfm_cams[it].pixel_to_vector(pix[it][c]);
	for (int row = 0; row < in_pts.rows(); row++) {
	  in_pts(row, col)  = trans_xyz[row];
	  out_pts(row, col) = xyz[it][c][row];
	}
	
	col++;
      }
    }
  }
  
  if (col != num_pts) 
    vw_throw( LogicErr() << "Book-keeping failure in aligning cameras to ground.\n");

  // The initial transform to world coordinates
  Vector<double> C;
  vw::math::find_3D_transform(in_pts, out_pts, rotation, translation, scale);

  // Copy into C
  transform_to_vector(C, rotation, translation, scale);

  // Form the pixel vector
  int pixel_vec_len = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good)
      pixel_vec_len += pix[it].size() * 2;
  }
  Vector<double> pixel_vec;
  pixel_vec.set_size(pixel_vec_len);
  int count = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) {
      for (size_t c = 0; c < pix[it].size(); c++) {
	Vector2 pixel = pix[it][c];
	pixel_vec[2*count  ] = pixel[0];
	pixel_vec[2*count+1] = pixel[1];
	count++;
      }
    }
  }
  if (2*count != pixel_vec_len)
    vw_throw( LogicErr() << "Book-keeping failure in cam_gen.\n");
  
  // Optimize the transform
  double abs_tolerance  = 1e-24;
  double rel_tolerance  = 1e-24;
  int    max_iterations = 2000;
  int status = 0;
  CameraSolveRotTransScale<PinholeModel> lma_model(xyz, pixel_vec, sfm_cams);
  Vector<double> final_params
    = vw::math::levenberg_marquardt(lma_model, C, pixel_vec,
				    status, abs_tolerance, rel_tolerance,
				    max_iterations);

  Vector<double>  final_residual = lma_model(final_params, verbose);
  
  // Bring the cameras to world coordinates
  for (int it = 0; it < num_cams; it++) 
    apply_rot_trans_scale(sfm_cams[it], final_params);

  // Unpack the final vector into a rotation + translation + scale
  vector_to_transform(final_params, rotation, translation, scale);

}

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least two images have at least 3 GCP each. Each GCP need
/// not show in multiple images.
void init_pinhole_model_with_mono_gcp(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
				      std::vector<boost::shared_ptr<CameraModel> > & camera_models) {
  
  vw_out() << "Initializing camera positions from ground control points." << std::endl;
  vw_out() << "Assume at least two images have each at least 3 GCP each.\n";

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
  std::vector< std::vector<Vector3> > xyz;
  std::vector< std::vector<Vector2> > pix;
  xyz.resize(num_cams);
  pix.resize(num_cams);
  
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias

  int ipt = - 1;
  for ( ControlNetwork::const_iterator iter = cnet.begin();
	iter != cnet.end(); ++iter ) {

    ipt++;
    
    // Keep only gcp
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint) {
      continue;
    }
        
    for ( ControlPoint::const_iterator measure = (*iter).begin();
	  measure != (*iter).end(); ++measure ) {

      int cam_it = measure->image_id();
      if (cam_it < 0 || cam_it >= num_cams) 
	vw_throw(ArgumentErr() << "Error: cnet index out of range.\n");

      Vector2 pixel( measure->position()[0],  measure->position()[1]);
      pix[cam_it].push_back(pixel);
      xyz[cam_it].push_back(cnet[ipt].position());
    }
  }  


  Matrix3x3 rotation;
  Vector3   translation;
  double    scale;
  align_cameras_to_ground(xyz, pix, pinhole_cams, rotation, translation, scale);

  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);
}

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            boost::shared_ptr<CameraModel> camera_model,
                            vw::cartography::GeoReference const& georef,
                            vw::cartography::GeoReference const& dem_georef) {
  // Get IP coordinate in the DEM
  Vector2 pix(P.x, P.y);
  Vector2 ll      = georef.pixel_to_lonlat(pix);
  Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
  if (!interp_dem.pixel_in_bounds(dem_pix))
    return false;
  // Load the elevation from the DEM
  PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
  if (!is_valid(dem_val))
    return false;
  Vector3 llh(ll[0], ll[1], dem_val.child());
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);

  // Project into the camera
  Vector2 cam_pix;
  try {
   cam_pix = camera_model->point_to_pixel(xyz);
  } catch(...) {
    return false; // Don't update the point.
  }
  P.x  = cam_pix.x();
  P.y  = cam_pix.y();
  P.ix = P.x;
  P.iy = P.y;
  return true;
}

#endif // __BUNDLE_ADJUST_CAMERA_H__
