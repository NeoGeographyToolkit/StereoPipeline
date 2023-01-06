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

// Code which needs the stereo session should go to
// asp/Sessions/CameraUtils.cc. Lower-level code which is not so tied
// to data structures and options used in bundle_adjust can go to
// BundleAdjustUtils.cc.

#ifndef __BUNDLE_ADJUST_CAMERA_H__
#define __BUNDLE_ADJUST_CAMERA_H__

// TODO(oalexan1): Move most of these headers to the .cc file
#include <vw/Camera/CameraUtilities.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Math/Geometry.h>

#include <asp/Core/FileUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/BundleAdjustUtils.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

// TODO(oalexan1): Move all this code to the .cc file and to the asp namespace.

#include <string>

namespace asp {

// Options shared by bundle_adjust and jitter_solve
struct BaBaseOptions: public vw::GdalWriteOptions {
  std::string out_prefix, stereo_session, input_prefix, match_files_prefix,
    clean_match_files_prefix, ref_dem, heights_from_dem, mapproj_dem;
  int overlap_limit, min_matches, max_pairwise_matches, num_iterations,
    ip_edge_buffer_percent;
  bool match_first_to_last, single_threaded_cameras;
  double min_triangulation_angle, max_init_reproj_error, robust_threshold, parameter_tolerance;
  double ref_dem_weight, ref_dem_robust_threshold, heights_from_dem_weight,
    heights_from_dem_robust_threshold, camera_weight, rotation_weight, translation_weight,
    tri_weight, tri_robust_threshold;
  vw::Vector<double, 4> remove_outliers_params;
  
  std::vector<std::string> image_files, camera_files;
  std::vector<boost::shared_ptr<vw::camera::CameraModel>> camera_models;
  std::map<std::pair<int, int>, std::string> match_files;

  BaBaseOptions(): min_triangulation_angle(0.0), camera_weight(-1.0),
                   rotation_weight(0.0), translation_weight(0.0), tri_weight(0.0),
                   robust_threshold(0.0), min_matches(0),
                   num_iterations(0), overlap_limit(0) {}
};
  
// This must be const or else there's a crash
const std::string UNSPECIFIED_DATUM = "unspecified_datum";

// A structure to hold percentiles of given sorted values. This sorts the inputs.
struct MatchPairStats {
  int left_cam_index, right_cam_index, num_vals;
  double val25, val50, val75;
  MatchPairStats(): left_cam_index(0), right_cam_index(0), num_vals(0), val25(0), val50(0),
               val75(0) {}
  void populate(int left_index, int right_index, std::vector<double> & vals) {
    std::sort(vals.begin(), vals.end());
    left_cam_index  = left_index;
    right_cam_index = right_index;
    num_vals = vals.size();
    if (num_vals > 0) {
      val25 = vals[0.25*num_vals];
      val50 = vals[0.50*num_vals];
      val75 = vals[0.75*num_vals];
    }
  }
};
  
} // end namespace asp

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

namespace asp {
/// Class to store parameters as they are being bundle adjusted.
/// - Currently only supports either one camera or all unique cameras.
class BAParams {

public:

  static const int PARAMS_PER_POINT  = 3;
  static const int NUM_CAMERA_PARAMS = 6; // Position and pose.
  // These two are only for pinhole cameras.
  static const int NUM_CENTER_PARAMS = 2; // TODO: Share info with other classes!
  static const int NUM_FOCUS_PARAMS  = 1;

  
  boost::random::mt19937 m_rand_gen;
  
  BAParams(int num_points, int num_cameras,
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
  BAParams(BAParams const& other)
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
  void copy_points(BAParams const& other) {
    for (size_t i=0; i<m_points_vec.size(); ++i)
      m_points_vec[i] = other.m_points_vec[i];
  }
  void copy_cameras(BAParams const& other) {
    for (size_t i=0; i<m_cameras_vec.size(); ++i)
      m_cameras_vec[i] = other.m_cameras_vec[i];
  }
  void copy_intrinsics(BAParams const& other) {
    for (size_t i=0; i<m_intrinsics_vec.size(); ++i)
      m_intrinsics_vec[i] = other.m_intrinsics_vec[i];
  }
  void copy_outliers(BAParams const& other) {
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
              vw::LogicErr() << "Camera parameter length is not a multiple of 6!");
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
                            const std::string icon =
                            "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png");
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
}; // End class BAParams

} // end namespace asp

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
    m_pose_data     = vw::math::axis_angle_to_quaternion(vw::Vector3(array[3], array[4], array[5]));
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
    vw::Vector2 pixel_offset = vw::Vector2();
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

/// Packs info from a pinhole camera into the provided arrays.
/// - It is up to the caller to make sure the arrays are properly sized.
void pack_pinhole_to_arrays(vw::camera::PinholeModel const& camera,
                            int camera_index,
                            asp::BAParams & param_storage);

void pack_optical_bar_to_arrays(vw::camera::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BAParams & param_storage);
// Given an input pinhole camera and param changes, apply those, returning
// the new camera.
vw::camera::PinholeModel transformedPinholeCamera(int camera_index,
                                                  asp::BAParams const& param_storage,
                                                  vw::camera::PinholeModel const& in_cam);

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::camera::OpticalBarModel transformedOpticalBarCamera(int camera_index,
                                                        asp::BAParams const& param_storage,
                                                        vw::camera::OpticalBarModel const& in_cam);

/// Given a transform with origin at the planet center, like output
/// by pc_align, read the adjustments from cameras_vec, apply this
/// transform on top of them, and write the adjustments back to the vector.
/// - Works for pinhole and non-pinhole case.
void apply_transform_to_cameras(vw::Matrix4x4 const& M, asp::BAParams &param_storage,
                                std::vector<vw::CamPtr>
                                const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BAParams & param_storage,
                                        std::vector<vw::CamPtr>
                                        const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BAParams & param_storage,
                                            std::vector<vw::CamPtr>
                                            const& cam_ptrs);

//=================================================================

/// Load all of the reference disparities specified in the input text file
/// and store them in the vectors.  Return the number loaded.
// TODO(oalexan1): This need not be a template. The disparity type
// is always PixelMask<Vector2f>.
template <class DispPixelT>
int load_reference_disparities(std::string const& disp_list_filename,
                               std::vector< vw::ImageView   <DispPixelT>> &disp_vec,
                               std::vector< vw::ImageViewRef<DispPixelT>> &interp_disp) {
  // TODO: Disparities can be large, but if small it is better to
  // read them in memory.
  std::istringstream is(disp_list_filename);
  std::string disp_file;
  while (is >> disp_file) {
    if (disp_file != "none") {
      vw::vw_out() << "Reading: " << disp_file << std::endl;
      disp_vec.push_back(copy(vw::DiskImageView<DispPixelT>(disp_file)));
    }else{
      // Read in an empty disparity
      disp_vec.push_back(vw::ImageView<DispPixelT>());
    }
    interp_disp.push_back(interpolate(disp_vec.back(),
                                      vw::BilinearInterpolation(), vw::ConstantEdgeExtension()));
  }
  return static_cast<int>(disp_vec.size());
}

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<vw::CamPtr> &camera_models,
                           boost::shared_ptr<vw::ba::ControlNetwork> const& cnet);

/// Generate a warning if the GCP's are really far from the IP points
/// - This is intended to help catch the common lat/lon swap in GCP files.
void check_gcp_dists(std::vector<vw::CamPtr> const& camera_models,
                     boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
                     double forced_triangulation_distance);

namespace asp {

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided camera positions.
/// - This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions
(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet, 
 std::vector<vw::CamPtr> & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<vw::Vector3> const & estimated_camera_gcc);

/// Initialize the position and orientation of a pinhole camera model using
/// GCP. It invokes OpenCV's PnP functionality.
void init_camera_using_gcp(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
                           std::vector<vw::CamPtr> & camera_models);
  
/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void transform_cameras_with_shared_gcp(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
				       std::vector<vw::CamPtr> & camera_models);
  
// Given at least two images, each having at least 3 GCP that are not seen in other
// images, find and apply a transform to the camera system based on them.
void transform_cameras_with_indiv_image_gcp(boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
                                            std::vector<vw::CamPtr> & camera_models);

// Given original cams in sfm_cams and individually scaled cameras in
// aux_cams, get the median scale change from the first set to the second one.
// It is important to do the median, since scaling the cameras individually
// is a bit of a shaky business.
double find_median_scale_change(std::vector<vw::camera::PinholeModel> const & sfm_cams,
				std::vector<vw::camera::PinholeModel> const & aux_cams,
				std::vector< std::vector<vw::Vector3>> const& xyz);
// Given some GCP so that at least two images have at at least three GCP each,
// but each GCP is allowed to show in one image only, use the GCP
// to transform cameras to ground coordinates.
void align_cameras_to_ground(std::vector< std::vector<vw::Vector3>> const& xyz,
			     std::vector< std::vector<vw::Vector2>> const& pix,
			     std::vector<vw::camera::PinholeModel> & sfm_cams,
			     vw::Matrix3x3 & rotation, 
			     vw::Vector3 & translation,
			     double & scale);

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            vw::CamPtr camera_model,
                            vw::cartography::GeoReference const& georef,
                            vw::cartography::GeoReference const& dem_georef);

// TODO(oalexan1): Move the asp namespace to encompass the whole header file
// Save convergence angle percentiles for each image pair having matches
void saveConvergenceAngles(std::string const& conv_angles_file,
                           std::vector<asp::MatchPairStats> const& convAngles,
                           std::vector<std::string> const& imageFiles);

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in DEM pixel units. It is assumed that dem_georef
// was created by bilinear interpolation.
void calcPairMapprojOffsets(std::vector<vw::CamPtr> const& optimized_cams,
                            int left_cam_index, int right_cam_index,
                            std::vector<vw::ip::InterestPoint> const& left_ip,
                            std::vector<vw::ip::InterestPoint> const right_ip,
                            vw::cartography::GeoReference const& dem_georef,
                            vw::ImageViewRef<vw::PixelMask<double>> interp_dem,
                            std::vector<vw::Vector<double, 4>> & mapprojPoints, 
                            std::vector<double> & mapproj_offsets);

// Save mapprojected matches offsets for each image pair having matches
void saveMapprojOffsets(std::string const& mapproj_offsets_stats_file,
                        std::string const& mapproj_offsets_file,
                        vw::cartography::GeoReference const& mapproj_dem_georef,
                        std::vector<vw::Vector<double, 4>> const & mapprojPoints,
                        std::vector<asp::MatchPairStats> const& mapprojOffsets,
                        std::vector<std::vector<double>> & mapprojOffsetsPerCam, // will change
                        std::vector<std::string> const& imageFiles);

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.
void matchFilesProcessing(vw::ba::ControlNetwork const& cnet,
                          asp::BaBaseOptions const& opt,
                          std::vector<vw::CamPtr> const& optimized_cams,
                          bool remove_outliers, std::set<int> const& outliers,
                          std::vector<asp::MatchPairStats> & convAngles,
                          std::string const& mapproj_dem,
                          std::vector<vw::Vector<double, 4>> & mapprojPoints,
                          std::vector<asp::MatchPairStats> & mapprojOffsets,
                          std::vector<std::vector<double>> & mapprojOffsetsPerCam);
  
}
#endif // __BUNDLE_ADJUST_CAMERA_H__
