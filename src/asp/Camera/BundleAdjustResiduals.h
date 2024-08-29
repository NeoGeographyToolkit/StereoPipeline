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

/// \file BundleAdjustResiduals.h
///
// Logic for computing residuals for bundle adjustment. 

#ifndef __BUNDLE_ADJUST_RESIDUALS_H__
#define __BUNDLE_ADJUST_RESIDUALS_H__

#include <asp/Camera/BundleAdjustCamera.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <string>

namespace asp {

// Compute the bundle_adjust residuals
void compute_residuals(asp::BaBaseOptions const& opt,
                       asp::CRNJ const& crn,
                       asp::BAParams const& param_storage,
                       std::vector<size_t> const& cam_residual_counts,
                       std::vector<std::map<int, vw::Vector2>> const& pixel_sigmas,
                       size_t num_gcp_or_dem_residuals,
                       size_t num_uncertainty_residuals,
                       size_t num_tri_residuals,
                       size_t num_cam_position_residuals,
                       std::vector<vw::Vector3> const& reference_vec,
                       ceres::Problem & problem,
                       // Output
                       std::vector<double> & residuals);

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(asp::CRNJ const& crn,
                                  std::vector<double> const& residuals,
                                  asp::BAParams const& param_storage,
                                  // outputs
                                  std::vector<double> & mean_residuals,
                                  std::vector<int>  & num_point_observations);

/// Write out a .csv file recording the residual error at each location on the ground
void write_residual_map(std::string const& output_prefix,
                        // Mean residual of each point
                        std::vector<double> const& mean_residuals,
                        // Num non-outlier pixels per point
                        std::vector<int> const& num_point_observations, 
                        asp::BAParams const& param_storage,
                        vw::ba::ControlNetwork const& cnet,
                        asp::BaBaseOptions const& opt);

/// Write log files describing all residual errors. The order of data stored
/// in residuals must mirror perfectly the way residuals were created. 
void write_residual_logs(std::string const& residual_prefix,
                         asp::BaBaseOptions const& opt,
                         asp::BAParams const& param_storage,
                         std::vector<size_t> const& cam_residual_counts,
                         std::vector<std::map<int, vw::Vector2>> const& pixel_sigmas,
                         size_t num_gcp_or_dem_residuals,
                         size_t num_uncertainty_residuals,
                         size_t num_tri_residuals,
                         size_t num_cam_position_residuals,
                         std::vector<vw::Vector3> const& reference_vec,
                         vw::ba::ControlNetwork const& cnet, 
                         asp::CRNJ const& crn, 
                         ceres::Problem &problem);

// Find and save the offsets between initial and final triangulated points
void saveTriOffsetsPerCamera(std::vector<std::string> const& image_files,
                             asp::BAParams const& orig_params,
                             asp::BAParams const& param_storage,
                             asp::CRNJ const& crn,
                             std::string const& tri_offsets_file);

// Analogous version to the above, but keep the original and current triangulated points
// in std<vector>, for use in jitter_solve.
void saveTriOffsetsPerCamera(std::vector<std::string> const& image_files,
                             std::set<int>            const& outliers,
                             std::vector<double> const& orig_tri_points_vec,
                             std::vector<double> const& tri_points_vec, 
                             asp::CRNJ const& crn,
                             std::string const& tri_offsets_file);

// Write the offsets between initial and final triangulated points
void writeTriOffsetsPerCamera(int num_cams,
                              std::vector<std::string> const& image_files,
                              std::vector<std::vector<double>> & tri_offsets,
                              std::string const& tri_offsets_file);

// Compute the horizontal and vertical change in camera positions
void saveCameraOffsets(vw::cartography::Datum   const& datum,
                       std::vector<std::string> const& image_files,
                       std::vector<vw::Vector3>  const& orig_cam_positions,
                       std::vector<vw::Vector3>  const& opt_cam_positions,
                       std::string              const& camera_offset_file);

// This is used in jitter_solve
void saveJitterResiduals(ceres::Problem                             & problem, 
                         std::string                           const& residual_prefix,
                         asp::BaBaseOptions                    const& opt,
                         vw::ba::ControlNetwork                const& cnet,
                         asp::CRNJ                             const& crn,
                         vw::cartography::Datum                const& datum,
                         std::vector<double>                   const& tri_points_vec,
                         std::set<int>                         const& outliers,
                         std::vector<double>                   const& weight_per_residual,
                         std::vector<std::vector<vw::Vector2>> const& pixel_vec,
                         std::vector<std::vector<double>>      const& weight_vec,
                         std::vector<std::vector<int>>         const& isAnchor_vec,
                         std::vector<std::vector<int>>         const& pix2xyz_index,
                         std::vector<vw::Vector3>              const& reference_vec,
                         std::vector<std::vector<int>>         const& ref_indices);

// This is used in jitter_solve
void compute_residuals(asp::BaBaseOptions const& opt,
                       ceres::Problem & problem,
                       // Output
                       std::vector<double> & residuals);

} // end namespace asp

#endif // __BUNDLE_ADJUST_RESIDUALS_H__
