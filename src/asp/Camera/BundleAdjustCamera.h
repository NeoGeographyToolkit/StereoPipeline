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

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/BaParams.h>
#include <asp/Core/Bathymetry.h>

#include <vw/Cartography/Datum.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/FileIO/DiskImageView.h>

#include <string>

namespace vw {
  namespace ip {
    class InterestPoint;
  }
  namespace camera {
    class PinholeModel;
    class OpticalBarModel;
    class AdjustedCameraModel;
  }
}

namespace asp {

/// These are the different camera modes that bundle_adjust supports.
enum BACameraType {BaCameraType_Pinhole    = 0,
                   BaCameraType_OpticalBar = 1,
                   BaCameraType_CSM        = 2,
                   BaCameraType_Other      = 3};

// Options shared by bundle_adjust and jitter_solve
struct BaBaseOptions: public vw::GdalWriteOptions {
  std::string out_prefix, stereo_session, input_prefix, match_files_prefix,
    clean_match_files_prefix, heights_from_dem, reference_terrain, mapproj_dem, weight_image,
    isis_cnet, nvm, nvm_no_shift, output_cnet_type,
    image_list, camera_list, mapprojected_data_list,
    fixed_image_list, camera_position_uncertainty_str;
  int overlap_limit, min_matches, max_pairwise_matches, num_iterations,
    ip_edge_buffer_percent, max_num_reference_points, num_passes;
  std::set<std::pair<std::string, std::string>> overlap_list;
  std::string overlap_list_file, auto_overlap_params, datum_str, proj_str,
    csv_format_str, csv_srs, csv_proj4_str, disparity_list, stereo_prefix_list,
    match_pair_sigma;
  bool have_overlap_list, propagate_errors, match_first_to_last, single_threaded_cameras,
    update_isis_cubes_with_csm_state, save_adjusted_rpc, fix_gcp_xyz, use_llh_error;
  double forced_triangulation_distance, min_triangulation_angle, max_triangulation_angle,
    max_init_reproj_error, robust_threshold, parameter_tolerance;
  double heights_from_dem_uncertainty, reference_terrain_weight,
    reference_terrain_uncertainty, reference_terrain_robust_threshold,
    heights_from_dem_robust_threshold, camera_weight, rotation_weight,
    camera_position_weight, camera_position_robust_threshold,
    tri_weight, tri_robust_threshold, camera_position_uncertainty_power,
    max_disp_error;
  std::vector<vw::Vector2> camera_position_uncertainty;
  vw::Vector<double, 4> remove_outliers_params;
  BACameraType camera_type;
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::vector<vw::CamPtr> camera_models;
  std::map<std::pair<int, int>, std::string> match_files;
  std::map<std::pair<int, int>, double> match_sigmas;

  vw::cartography::Datum datum;
  vw::BBox2 proj_win; // Limit input triangulated points to this projwin
  double horizontal_stddev;
  vw::Vector<double> horizontal_stddev_vec; // may come from cameras or user
  vw::BathyData bathy_data;

  BaBaseOptions():
   forced_triangulation_distance(-1),
   min_triangulation_angle(0.0), camera_position_weight(0.0),
   camera_position_robust_threshold(0.0), camera_weight(-1.0),
   rotation_weight(0.0), tri_weight(0.0),
   robust_threshold(0.0), min_matches(0),
   num_iterations(0), num_passes(0),
   overlap_limit(0), have_overlap_list(false), propagate_errors(false),
   match_first_to_last(false), single_threaded_cameras(false),
   update_isis_cubes_with_csm_state(false),
   fix_gcp_xyz(false), use_llh_error(false),
   camera_type(BaCameraType_Other), max_num_reference_points(-1),
   datum(vw::cartography::Datum(asp::UNSPECIFIED_DATUM,
                                "User Specified Spheroid",
                                "Reference Meridian", 1, 1, 0)) {}
};

// A structure to hold percentiles of given sorted values. This sorts the inputs.
// The input can be float or double. We will keep the result as double.
struct MatchPairStats {
  int left_cam_index, right_cam_index, num_vals;
  double val25, val50, val75, val85, val95;
  MatchPairStats(): left_cam_index(0), right_cam_index(0), num_vals(0), val25(0), val50(0),
                    val75(0), val85(0), val95(0) {}
  template<class T>
  void populate(int left_index, int right_index, std::vector<T> & vals) {
    std::sort(vals.begin(), vals.end());
    left_cam_index  = left_index;
    right_cam_index = right_index;
    num_vals = vals.size();
    if (num_vals > 0) {
      val25 = vals[0.25*num_vals];
      val50 = vals[0.50*num_vals];
      val75 = vals[0.75*num_vals];
      val85 = vals[0.85*num_vals];
      val95 = vals[0.95*num_vals];
    }
  }
};

struct HorizVertErrorStats {
  int left_cam_index, right_cam_index;
  float horiz_error_median, vert_error_median;
  float horiz_error_mean, vert_error_mean;
  float horiz_error_stddev, vert_error_stddev;
  int num_errors;
  HorizVertErrorStats(): left_cam_index(0), right_cam_index(0),
                         horiz_error_median(0), vert_error_median(0),
                         horiz_error_mean(0), vert_error_mean(0),
                         horiz_error_stddev(0), vert_error_stddev(0),
                         num_errors(0) {}
};

// When distortion params are shared, their number must agree
void distortion_sanity_check(std::vector<int> const& num_dist_params,
                             IntrinsicOptions const& intrinsics_opts,
                             std::vector<double> const& intrinsics_limits);

// Read image and camera lists. Can have several comma-separated lists
// in image_list and camera_list, when sharing intrinsics per sensor.
void read_image_cam_lists(std::string const& image_list,
                std::string const& camera_list,
                std::vector<std::string> & images,
                std::vector<std::string> & cameras,
                asp::IntrinsicOptions & intrinsics_opts);

/// Load all of the reference disparities specified in the input text file
/// and store them in the vectors.  Return the number loaded.
int loadRefDisp(std::string const& disp_list_filename,
                std::vector<vw::ImageView<vw::PixelMask<vw::Vector2f>>> &
                   disp_vec,
                std::vector<vw::ImageViewRef<vw::PixelMask<vw::Vector2f>>> &
                   interp_disp);

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
                           std::vector<float>                      & mapprojOffsets);

/// Simple class to manage position/rotation information.
/// - This is the data type stored in pc_align output files,
///   bundle adjustment files, and the position of pinhole cameras.
class CameraAdjustment {
public:

  // Constructors
  CameraAdjustment();
  CameraAdjustment(double const* array);

  // Data access
  vw::Vector3 position() const;
  vw::Quat    pose    () const;

  /// Populate from a six element array.
  void read_from_array(double const* array);

  /// Populate from an AdjustedCameraModel
  void copy_from_adjusted_camera(vw::camera::AdjustedCameraModel const& cam);

  /// Populate from a PinholeModel
  void copy_from_pinhole(vw::camera::PinholeModel const& cam);

  /// Populate from OpticalBarModel
  void copy_from_optical_bar(vw::camera::OpticalBarModel const& cam);

  /// Populate from CSM. Since with CSM we apply adjustments to existing
  /// cameras, these start as 0.
  void copy_from_csm(asp::CsmModel const& cam);

  /// Populate from an adjustment file on disk.
  void read_from_adjust_file(std::string const& filename);

  /// Pack the data to a six element array.
  void pack_to_array(double* array) const;

private:
  vw::Vector3 m_position_data;
  vw::Quat    m_pose_data;

}; // End class CameraAdjustment

/// Packs info from various camera models into the provided arrays.
/// - It is up to the caller to make sure the arrays are properly sized.
void pack_pinhole_to_arrays(vw::camera::PinholeModel const& camera,
                            int camera_index,
                            asp::BaParams & param_storage);
void pack_optical_bar_to_arrays(vw::camera::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BaParams & param_storage);
// This does not copy the camera position and orientation
void pack_csm_to_arrays(asp::CsmModel const& camera,
                        int camera_index,
                        asp::BaParams & param_storage);

// Given an input pinhole camera and param changes, apply those, returning
// the new camera.
vw::camera::PinholeModel transformedPinholeCamera(int camera_index,
                                                  asp::BaParams const& param_storage,
                                                  vw::camera::PinholeModel const& in_cam);

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::camera::OpticalBarModel transformedOpticalBarCamera(int camera_index,
                                                        asp::BaParams const& param_storage,
                                                        vw::camera::OpticalBarModel const& in_cam);

// Given an input CSM camera and param changes, apply those, returning
// the new camera.
boost::shared_ptr<asp::CsmModel> transformedCsmCamera(int camera_index,
                                                      asp::BaParams const& param_storage,
                                                      asp::CsmModel const& in_cam);

/// Given a transform with origin at the planet center, like output by pc_align,
/// read the adjustments from param storage, apply this transform on top of
/// them, and write the adjustments back to the param storage. Cameras
/// do not change.
void apply_transform_to_params(vw::Matrix4x4 const& M, asp::BaParams &param_storage,
                                std::vector<vw::CamPtr>
                                const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BaParams & param_storage,
                                        std::vector<vw::CamPtr>
                                        const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BaParams & param_storage,
                                            std::vector<vw::CamPtr>
                                            const& cam_ptrs);

void apply_transform_to_cameras_csm(vw::Matrix4x4 const& M,
                                    asp::BaParams & param_storage,
                                    std::vector<vw::CamPtr>
                                    const& cam_ptrs);

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
void transform_cameras_with_indiv_image_gcp
  (boost::shared_ptr<vw::ba::ControlNetwork> const& cnet_ptr,
   std::vector<vw::CamPtr> & camera_models);

// TODO(oalexan1): Move the asp namespace to encompass the whole header file
// Save convergence angle percentiles for each image pair having matches
void saveConvergenceAngles(std::string const& conv_angles_file,
                           std::vector<asp::MatchPairStats> const& convAngles,
                           std::vector<std::string> const& imageFiles);

// Save propagated horizontal and vertical errors
void saveHorizVertErrors(std::string const& horiz_vert_errors_file,
                         std::vector<asp::HorizVertErrorStats> const& horizVertErrors,
                         std::vector<std::string> const& imageFiles);

// Save mapprojected matches offsets for each image pair having matches
void saveMapprojOffsets(std::string                       const& out_prefix,
                        vw::cartography::GeoReference     const& mapproj_dem_georef,
                        std::vector<vw::Vector<float, 4>> const& mapprojPoints,
                        std::vector<asp::MatchPairStats>  const& mapprojOffsets,
                        std::vector<std::vector<float>>        & mapprojOffsetsPerCam,
                        std::vector<std::string>          const& imageFiles);

// Write a pinhole camera file to disk after updating the intrinsics and
// extrinsics. Return the path to the saved file.
std::string savePinholeCam(asp::BaBaseOptions const& opt, int icam,
                           vw::cartography::Datum const& datum,
                           asp::BaParams const& param_storage);

// Write an optical bar camera file to disk after updating the intrinsics and
// extrinsics. Return the path to the saved file.
std::string saveOpticalBarCam(asp::BaBaseOptions const& opt, int icam,
                              vw::cartography::Datum const& datum,
                              asp::BaParams const& param_storage);

// Write a CSM camera file to disk. Assumes that the intrinsics are optimized.
std::string saveCsmCamUpdateIntr(asp::BaBaseOptions const& opt, int icam,
                                 vw::cartography::Datum const& datum,
                                 asp::BaParams const& param_storage);

// Write a camera adjustment file to disk, and potentially a camera file with
// the adjustments applied to it. Return the path to the saved file.
std::string saveAdjustedCam(asp::BaBaseOptions const& opt, int icam,
                            asp::BaParams const& param_storage);

// Write updated camera models to disk
void saveUpdatedCameras(asp::BaBaseOptions const& opt,
                        asp::BaParams const& param_storage);

// Save CSM cameras
void saveCsmCameras(std::string const& out_prefix,
                    std::string const& stereo_session,
                    std::vector<std::string> const& image_files,
                    std::vector<std::string> const& camera_files,
                    std::vector<vw::CamPtr>  const& camera_models,
                    bool update_isis_cubes_with_csm_state);

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
                          std::map<std::pair<int, int>, std::string> const& match_files);

/// This is for the BundleAdjustmentModel class where the camera parameters
/// are a rotation/offset that is applied on top of the existing camera model.
/// First read initial adjustments, if any, and apply perhaps a pc_align transform.
/// We assume the initial transform was already read and validated.
bool init_cams(asp::BaBaseOptions const& opt, asp::BaParams & param_storage,
       std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
       std::vector<vw::CamPtr> & new_cam_models);

/// Specialization for pinhole cameras.
bool init_cams_pinhole(asp::BaBaseOptions const& opt, asp::BaParams & param_storage,
     std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
     std::vector<vw::CamPtr> & new_cam_models);

// TODO: Share more code with the similar pinhole case.
/// Specialization for optical bar cameras.
bool init_cams_optical_bar(asp::BaBaseOptions const& opt, asp::BaParams & param_storage,
                    std::string const& initial_transform_file,
                    vw::Matrix<double> const& initial_transform,
                    std::vector<vw::CamPtr> &new_cam_models);

// TODO: Share more code with the similar pinhole case.
/// Specialization for CSM cameras.
bool init_cams_csm(asp::BaBaseOptions const& opt, asp::BaParams & param_storage,
                   std::string const& initial_transform_file,
                   vw::Matrix<double> const& initial_transform,
                   std::vector<vw::CamPtr> &new_cam_models);

// Save pinhole camera positions and orientations in a single file.
// Only works with Pinhole cameras.
void saveCameraReport(asp::BaBaseOptions const& opt, asp::BaParams const& param_storage,
                      vw::cartography::Datum const& datum,
                      std::string const& prefix);

/// For each option, the string must include a subset of the entries:
///  "focal_length, optical_center, distortion_params"
/// - Need the extra boolean to handle the case where --intrinsics-to-share
///   is provided as "" in order to share none of them.
void load_intrinsics_options(bool        solve_intrinsics,
                             bool        shared_is_specified,
                             std::string intrinsics_to_float_str, // make a copy
                             std::string intrinsics_to_share_str, // make a copy
                             asp::IntrinsicOptions & intrinsics_options);

/// Attempt to automatically create the overlap list file estimated
///  footprints for each of the input images.
/// - Currently this only supports cameras with Worldview style XML files.
void auto_build_overlap_list(asp::BaBaseOptions &opt, double lonlat_buffer);

// Parse data needed for error propagation. Note that horizontal_stddevs
// comes from the user, or is otherwise populated from cameras.
void setup_error_propagation(std::string const& session_name,
                             double horizontal_stddev,
                             std::vector<vw::CamPtr> const& cameras,
                             vw::Vector<double> & horizontal_stddev_vec);

// Find stats of propagated errors
void propagatedErrorStats(size_t left_cam_index, size_t right_cam_index,
                          vw::camera::CameraModel const * left_cam,
                          vw::camera::CameraModel const * right_cam,
                          std::vector<vw::ip::InterestPoint> const& left_ip,
                          std::vector<vw::ip::InterestPoint> const& right_ip,
                          double stddev1, double stddev2,
                          vw::cartography::Datum const& datum,
                          // Output
                          asp::HorizVertErrorStats & stats);

// Find the cameras with the latest adjustments. Note that we do not modify
// opt.camera_models, but make copies as needed.
void calcOptimizedCameras(asp::BaBaseOptions const& opt,
                          asp::BaParams const& param_storage,
                          std::vector<vw::CamPtr> & optimized_cams);

// Find the average for the gsd for all pixels whose rays intersect at the given
// triangulated point. This is used in jitter solving.
void estimateGsdPerTriPoint(std::vector<std::string> const& images,
                            std::vector<vw::CamPtr>  const& cameras,
                            asp::CRN                const& crn,
                            asp::BaParams            const& param_storage,
                            // Output
                            std::vector<double>     & gsds);

// This is a version of the above used in jitter solving.
void estimateGsdPerTriPoint(std::vector<std::string> const& images,
                            std::vector<vw::CamPtr>  const& cameras,
                            asp::CRN                const& crn,
                            std::set<int>            const& outliers,
                            std::vector<double>      const& tri_points_vec,
                            // Output
                            std::vector<double>     & gsds);

// Parse the string of limits and make sure they are all valid pairs.
void parse_intrinsics_limits(std::string const& intrinsics_limits_str,
                             std::vector<double> & intrinsics_limits);

// This function returns only one camera center per camera
void calcCameraCenters(std::vector<vw::CamPtr>  const& cams,
                       std::vector<vw::Vector3>      & cam_positions);

// This function returns all camera center samples for linescan cameras
void calcCameraCenters(std::string const& stereo_session,
                       std::vector<vw::CamPtr>  const& cams,
                       std::vector<std::vector<vw::Vector3>> & cam_positions);

// Interface for setting/getting intrinsics for all supported camera models
void get_optical_center(vw::camera::CameraModel const* cam, vw::Vector2 & center);
void set_optical_center(vw::camera::CameraModel* cam, vw::Vector2 const& center);
void get_focal_length(vw::camera::CameraModel const* cam, double & focal);
void set_focal_length(vw::camera::CameraModel* cam, double const& focal);
void get_distortion(vw::camera::CameraModel const* cam, vw::Vector<double> &dist);
void set_distortion(vw::camera::CameraModel* cam, vw::Vector<double> const& dist);

// If some cameras share an intrinsic parameter, that parameter must start with
// the same value for all cameras sharing it. This is a bugfix. Return
// true if the cameras were modified.
bool syncUpInitialSharedParams(BACameraType camera_type,
                               asp::BaParams const& param_storage,
                               std::vector<vw::CamPtr>& camera_models);

// This is needed to allocate enough storage for the distortion parameters.
int calcMaxNumDistParams(std::vector<vw::CamPtr> const& camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<double> const& intrinsics_limits);

// This is needed to ensure distortion coefficients are not so small
// that they don't get optimized. This modifies the camera models in place.
void ensureMinDistortion(std::vector<vw::CamPtr> & camera_models,
                         BACameraType camera_type,
                         IntrinsicOptions const& intrinsics_opts,
                         std::vector<int> const& fixed_distortion_indices,
                         int max_num_dist_params,
                         double min_distortion);

// Sanity check. This does not prevent the user from setting the wrong datum,
// but it can catch unreasonable height values for GCP.
void checkGcpRadius(vw::cartography::Datum const& datum,
                    vw::ba::ControlNetwork const& cnet);

// Some logic for camera position uncertainty, used in bundle_adjust and jitter_solve
void handleCameraPositionUncertainty(asp::BaBaseOptions & opt, bool have_datum);

} // end namespace asp

#endif // __BUNDLE_ADJUST_CAMERA_H__
