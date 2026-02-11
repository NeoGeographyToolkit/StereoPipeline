// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file SfsOptions.vv
// Command-line options for sfs

#include <asp/SfS/SfsOptions.h>

#include <string>
#include <map>

namespace asp {

SfsOptions::SfsOptions():
  max_iterations(0), reflectance_type(0),
  blending_dist(0), blending_power(2.0),
  min_blend_size(0), num_haze_coeffs(0),
  num_samples_for_estim(0),
  float_albedo(false), float_exposure(false), model_shadows(false),
  save_sim_intensity_only(false), save_meas_intensity_only(false),
  estimate_height_errors(false),
  compute_exposures_only(false),
  estim_exposure_haze_albedo(false),
  save_dem_with_nodata(false),
  use_approx_camera_models(false),
  crop_input_images(false),
  allow_borderline_data(false), fix_dem(false),
  float_reflectance_model(false), query(false),
  save_sparingly(false), float_haze(false),
  save_variances(false), save_covariances(false),
  smoothness_weight(0), steepness_factor(1.0),
  curvature_in_shadow(0), curvature_in_shadow_weight(0.0),
  lit_curvature_dist(0.0), shadow_curvature_dist(0.0),
  gradient_weight(0.0), integrability_weight(0), smoothness_weight_pq(0),
  initial_dem_constraint_weight(0.0),
  albedo_constraint_weight(0.0), albedo_robust_threshold(0.0),
  camera_position_step_size(1.0), low_light_threshold(0.0),
  low_light_weight_power(4.0), low_light_blur_sigma(0.0),
  crop_win(vw::BBox2i(0, 0, 0, 0)) {}

} // end namespace asp
