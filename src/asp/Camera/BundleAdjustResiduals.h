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
void compute_residuals(bool apply_loss_function,
                       asp::BaBaseOptions const& opt,
                       asp::BAParams const& param_storage,
                       std::vector<size_t> const& cam_residual_counts,
                       size_t num_gcp_or_dem_residuals,
                       size_t num_uncertainty_residuals,
                       size_t num_tri_residuals,
                       std::vector<vw::Vector3> const& reference_vec,
                       ceres::Problem & problem,
                       // Output
                       std::vector<double> & residuals);

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
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
void write_residual_logs(std::string const& residual_prefix, bool apply_loss_function,
                         asp::BaBaseOptions const& opt,
                         asp::BAParams const& param_storage,
                         std::vector<size_t> const& cam_residual_counts,
                         size_t num_gcp_or_dem_residuals,
                         size_t num_uncertainty_residuals,
                         size_t num_tri_residuals,
                         std::vector<vw::Vector3> const& reference_vec,
                         vw::ba::ControlNetwork const& cnet, 
                         vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn, 
                         ceres::Problem &problem);

} // end namespace asp

#endif // __BUNDLE_ADJUST_RESIDUALS_H__
