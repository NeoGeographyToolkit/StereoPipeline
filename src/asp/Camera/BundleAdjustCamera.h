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

#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Camera/BundleAdjustIntrinsics.h>
#include <asp/Camera/BundleAdjustMatchFiles.h>
#include <asp/Camera/BundleAdjustGcp.h>
#include <asp/Camera/BundleAdjustTransform.h>

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/Bathymetry.h>

#include <vw/Cartography/Datum.h>
#include <vw/FileIO/DiskImageView.h>

#include <map>
#include <set>
#include <string>

namespace vw {
    class InterestPoint;
    class PinholeModel;
    class OpticalBarModel;
    class AdjustedCameraModel;
}

namespace asp {

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
                std::vector<vw::ImageView<vw::PixelMask<vw::Vector2f>>> & disp_vec,
                std::vector<vw::ImageViewRef<vw::PixelMask<vw::Vector2f>>> & interp_disp);

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
                        vw::GeoReference     const& mapproj_dem_georef,
                        std::vector<vw::Vector<float, 4>> const& mapprojPoints,
                        std::vector<asp::MatchPairStats>  const& mapprojOffsets,
                        std::vector<std::vector<float>>        & mapprojOffsetsPerCam,
                        std::vector<std::string>          const& imageFiles);

// Write a pinhole camera file to disk after updating the intrinsics and
// extrinsics. Return the path to the saved file.
std::string savePinholeCam(asp::BaBaseOptions const& opt, int icam,
                           vw::Datum const& datum,
                           asp::BaState const& ba_state);

// Write an optical bar camera file to disk after updating the intrinsics and
// extrinsics. Return the path to the saved file.
std::string saveOpticalBarCam(asp::BaBaseOptions const& opt, int icam,
                              vw::Datum const& datum,
                              asp::BaState const& ba_state);

// Write a CSM camera file to disk. Assumes that the intrinsics are optimized.
std::string saveCsmCamUpdateIntr(asp::BaBaseOptions const& opt, int icam,
                                 vw::Datum const& datum,
                                 asp::BaState const& ba_state);

// Write a camera adjustment file to disk, and potentially a camera file with
// the adjustments applied to it. Return the path to the saved file.
std::string saveAdjustedCam(asp::BaBaseOptions const& opt, int icam,
                            asp::BaState const& ba_state);

// Write updated camera models to disk
void saveUpdatedCameras(asp::BaBaseOptions const& opt,
                        asp::BaState const& ba_state);

// Save CSM cameras
void saveCsmCameras(std::string const& out_prefix,
                    std::string const& stereo_session,
                    std::vector<std::string> const& image_files,
                    std::vector<std::string> const& camera_files,
                    std::vector<vw::CamPtr>  const& camera_models,
                    bool update_isis_cubes_with_csm_state);

// Save pinhole camera positions and orientations in a single file.
// Only works with Pinhole cameras.
void saveCameraReport(asp::BaBaseOptions const& opt, asp::BaState const& ba_state,
                      vw::Datum const& datum,
                      std::string const& prefix);

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
                          vw::CameraModel const * left_cam,
                          vw::CameraModel const * right_cam,
                          std::vector<vw::InterestPoint> const& left_ip,
                          std::vector<vw::InterestPoint> const& right_ip,
                          double stddev1, double stddev2,
                          vw::Datum const& datum,
                          vw::BathyData const& bathy_data,
                          // Outputs
                          std::vector<vw::Vector<double, 5>> & uncertPoints,
                          asp::HorizVertErrorStats & stats);

// Find the average for the gsd for all pixels whose rays intersect at the given
// triangulated point. This is used in jitter solving.
void estimateGsdPerTriPoint(std::vector<std::string> const& images,
                            std::vector<vw::CamPtr>  const& cameras,
                            asp::CRN                const& crn,
                            asp::BaState             const& ba_state,
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

// This function returns only one camera center per camera
void calcCameraCenters(std::vector<vw::CamPtr>  const& cams,
                       std::vector<vw::Vector3>      & cam_positions);

// This function returns all camera center samples for linescan cameras
void calcCameraCenters(std::string const& stereo_session,
                       std::vector<vw::CamPtr>  const& cams,
                       std::vector<std::vector<vw::Vector3>> & cam_positions);

// Some logic for camera position uncertainty, used in bundle_adjust and jitter_solve
void handleCameraPositionUncertainty(asp::BaBaseOptions & opt, bool have_datum);

} // end namespace asp

#endif // __BUNDLE_ADJUST_CAMERA_H__
