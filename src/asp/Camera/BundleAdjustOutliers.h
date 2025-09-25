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

/// \file BundleAdjustOutliers.h
///
// Logic for handling outliers in bundle adjustment. 

#ifndef __BUNDLE_ADJUST_OUTLIERS_H__
#define __BUNDLE_ADJUST_OUTLIERS_H__

#include <asp/Core/BundleAdjustUtils.h>

#include <string>

namespace ceres {
  class Problem;
}

namespace asp {

class BaBaseOptions;
class BaOptions;
class BAParams;

// Update the set of outliers based on param_storage
void updateOutliers(vw::ba::ControlNetwork const& cnet, 
                      asp::BAParams const& param_storage,
                      std::set<int> & outliers);

// Filter matches by projection window.
// TODO(oalexan1): Use this in jitter_solve.
// TODO(oalexan1): This needs to be done before subsampling the matches
void filterOutliersProjWin(asp::BaBaseOptions          & opt,
                           asp::BAParams               & param_storage, 
                           vw::ba::ControlNetwork const& cnet);

void filterOutliersByConvergenceAngle(asp::BaBaseOptions const& opt,
                                      vw::ba::ControlNetwork const& cnet,
                                      asp::BAParams & param_storage);

/// Add to the outliers based on the large residuals
int add_to_outliers(vw::ba::ControlNetwork & cnet,
                    asp::CRN const& crn,
                    asp::BAParams & param_storage,
                    asp::BaOptions const& opt,
                    std::vector<size_t> const& cam_residual_counts,
                    std::vector<std::map<int, vw::Vector2>> const& pixel_sigmas,
                    size_t num_gcp_or_dem_residuals,
                    size_t num_uncertainty_residuals,
                    size_t num_tri_residuals,
                    size_t num_cam_pos_residuals,
                    std::vector<vw::Vector3> const& reference_vec,
                    ceres::Problem &problem);

} // end namespace asp

#endif // __BUNDLE_ADJUST_OUTLIERS_H__
