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

/// \file OptimizerOptions.h
///
/// Base options shared by bundle_adjust, jitter_solve, and rig_calibrator.

#ifndef __ASP_CORE_OPTIMIZER_OPTIONS_H__
#define __ASP_CORE_OPTIMIZER_OPTIONS_H__

#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Math/Vector.h>

#include <string>
#include <vector>

namespace asp {

struct OptimizerOptions: public vw::GdalWriteOptions {
  std::string out_prefix, nvm, heights_from_dem,
    camera_position_uncertainty_str, fixed_image_list_str;
  int num_iterations, num_passes;
  double robust_threshold, tri_weight, tri_robust_threshold,
    min_triangulation_angle,
    parameter_tolerance, heights_from_dem_uncertainty,
    heights_from_dem_robust_threshold;
  std::vector<vw::Vector2> camera_position_uncertainty;

  OptimizerOptions():
    num_iterations(0), num_passes(0),
    robust_threshold(0.5), tri_weight(0.1), tri_robust_threshold(0.1),
    min_triangulation_angle(0.01) {}
};

} // namespace asp

#endif // __ASP_CORE_OPTIMIZER_OPTIONS_H__
