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

/// \file BaParams.h
///

#ifndef __ASP_CAMERA_BA_PARAMS_H__
#define __ASP_CAMERA_BA_PARAMS_H__

#include <vw/Math/Vector.h>

namespace vw {
  namespace cartography {
    class Datum;
  }
  namespace ba {
    class ControlNetwork;
  }
}

#include <string>
#include <vector>

namespace asp {

// This must be const or else there's a crash
const std::string UNSPECIFIED_DATUM = "unspecified_datum";

// Shared constants
const int PARAMS_PER_POINT  = 3;
const int NUM_CAMERA_PARAMS = 6; // Position and pose
const int NUM_CENTER_PARAMS = 2; // TODO(oalexan1): Use this more widely
const int NUM_FOCUS_PARAMS  = 1;
const int NUM_OPTICAL_BAR_EXTRA_PARAMS = 9; // Stored in the distortion vector

/// Structure to fully describe how the intrinsics are being handled.
/// - Currently only pinhole cameras support intrinsics in bundle_adjust.
struct IntrinsicOptions {
  
  // If to share these intrinsics. Can be per group of cameras or for all cameras.
  bool center_shared;
  bool focus_shared;
  bool distortion_shared;
  
  // If to float these intrinsics. All, none, or per sensor
  std::vector<bool> float_center, float_focus, float_distortion;
  
  bool share_intrinsics_per_sensor;
  std::vector<int> cam2sensor; // cam index to sensor index, when sharing intrinsics per sensor
  int num_sensors; // will be nonzero only if sharing intrinsics per sensor is true
  IntrinsicOptions();

  // Control per each group of cameras or for all cameras which intrinsics
  // should be floated.
  bool float_optical_center(int cam_index) const;
  bool float_focal_length(int cam_index) const;
  bool float_distortion_params(int cam_index) const;
};

/// Class to store parameters as they are being bundle adjusted.
/// - Currently only supports either one camera or all unique cameras.
class BaParams {

public:

  // Constructor
  BaParams(int num_points, int num_cameras,
          // Parameters below here only apply to pinhole models.
          bool using_intrinsics = false,
          int max_num_dist_params = 0,
          IntrinsicOptions intrinsics_opts = IntrinsicOptions()); 

  // Copy constructor
  BaParams(asp::BaParams const& other);

  // Set all camera position and pose values to zero.
  void init_cams_as_zero();

  // When using the copy functions, the sizes must match!
  /// Copy one set of values from another instance.
  void copy_points(asp::BaParams const& other);
  void copy_cameras(asp::BaParams const& other);
  void copy_intrinsics(asp::BaParams const& other);
  void copy_outliers(asp::BaParams const& other);

  /// Apply a random offset to each camera position.
  void randomize_cameras();

  /// Randomly scale each intrinsic value.
  void randomize_intrinsics(std::vector<double> const& intrinsic_limits);
  
  int num_points       () const {return m_num_points; }
  int num_cameras      () const {return m_num_cameras;}
  int num_intrinsics_per_camera() const {return m_num_intrinsics_per_camera;}
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

  // These functions are only needed when solving for intrinsics
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
  
  // End functions needed when solving for intrinsics
  
  void set_point_outlier(int point_index, bool status) {
    m_outlier_points_vec[point_index] = status;
  }
  bool get_point_outlier(int point_index) const {
    return m_outlier_points_vec[point_index];
  }

  /// Return the number of points flagged as outliers.
  int get_num_outliers() const {
    int count = 0;
    for (size_t i = 0; i < m_num_points; i++) {
      if (m_outlier_points_vec[i])
        ++count;
    }
    return count;
  }
  
  /// Get the values for a point
  vw::Vector3 get_point(int point_index)  const{
    double const* ptr = get_point_ptr(point_index);
    vw::Vector3 pt;
    for (int i = 0; i < 3; i++)
      pt[i] = ptr[i];
    return pt;
  }
  
  /// Update the values for a point
  void set_point(int point_index, vw::Vector3 const& pt) {
    double* ptr = get_point_ptr(point_index);
    for (int i = 0; i < 3; i++)
      ptr[i] = pt[i];
  }

  /// Print stats for optimized ground control points.
  void print_gcp_stats(std::string const& out_prefix, 
                       vw::ba::ControlNetwork const& cnet,
                       vw::cartography::Datum const& d) const;

  /// Create a KML file containing the positions of the given points.
  /// - Points are stored as x,y,z in the points vector up to num_points.
  /// - Only every skip'th point is recorded to the file.
  void record_points_to_kml(const std::string &kml_path,
                            const vw::cartography::Datum& datum,
                            size_t skip, const std::string name);

  /// Compute the offset index in the intrinsics
  size_t get_center_offset(int cam_index) const;
  size_t get_focus_offset(int cam_index) const;
  size_t get_distortion_offset(int cam_index) const;

  // Not much of a point of having set/get functions for this one
  int m_max_num_dist_params;
  
private: // Variables

  int m_num_points, m_num_cameras, m_params_per_point, m_num_pose_params;
  
  // m_intrinsics_vec starts out with m_num_shared_intrinsics values which are
  //  shared between all cameras, followed by the per-camera intrinsics for each camera.
  int m_num_shared_intrinsics, m_num_intrinsics_per_camera;
  
  // These store the offset to the focus or distortion data from the start of
  //  either the shared parameters at the start of m_intrinsics_vec or from
  //  the block of intrinsics data for the specified camera.
  int m_focus_offset, m_distortion_offset;
  
  IntrinsicOptions m_intrinsics_opts;
  
  // Raw data storage
  std::vector<double> m_points_vec, m_cameras_vec, m_intrinsics_vec;
  std::vector<bool> m_outlier_points_vec;
}; // End class BaParams

} // end namespace asp

#endif // __ASP_CAMERA_BA_PARAMS_H__
