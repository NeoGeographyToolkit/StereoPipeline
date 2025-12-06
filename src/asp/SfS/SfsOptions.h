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

// \file SfsOptions.h
// Command-line options for sfs

#ifndef __ASP_SFS_SFS_OPTIONS_H__
#define __ASP_SFS_SFS_OPTIONS_H__

#include <vw/Cartography/GeoReferenceUtils.h>

namespace asp {

struct SfsOptions: public vw::GdalWriteOptions {
  std::string input_dem, image_list, camera_list, out_prefix, stereo_session, bundle_adjust_prefix, input_albedo;
  std::vector<std::string> input_images, input_cameras;
  std::string shadow_thresholds, custom_shadow_threshold_list, max_valid_image_vals, skip_images_str, image_exposures_prefix, model_coeffs_prefix, model_coeffs, image_haze_prefix, sun_positions_list, sun_angles_list, ref_map;
  std::vector<float> shadow_threshold_vec, max_valid_image_vals_vec;
  std::vector<double> image_exposures_vec;
  std::vector<std::vector<double>> image_haze_vec;
  std::vector<double> model_coeffs_vec;
  std::set<int> skip_images;
  int max_iterations, reflectance_type, blending_dist, min_blend_size, num_haze_coeffs,
    num_samples_for_estim;
  bool float_albedo, float_exposure, model_shadows,
    save_sim_intensity_only, save_meas_intensity_only, estimate_height_errors,
    compute_exposures_only, estim_exposure_haze_albedo,
    save_dem_with_nodata, use_approx_camera_models,
    crop_input_images, allow_borderline_data, fix_dem, float_reflectance_model,
    query, save_sparingly, float_haze, read_exposures, read_haze, read_albedo,
    erode_seams;

  double smoothness_weight, steepness_factor, gradient_weight,
    blending_power, integrability_weight, smoothness_weight_pq, init_dem_height,
    nodata_val, initial_dem_constraint_weight, albedo_constraint_weight,
    albedo_robust_threshold, camera_position_step_size, 
    robust_threshold, shadow_threshold, low_light_threshold,
    low_light_weight_power, low_light_blur_sigma,
    curvature_in_shadow, curvature_in_shadow_weight, lit_curvature_dist, 
    shadow_curvature_dist;
    
  vw::BBox2 crop_win;
  vw::Vector2 height_error_params;

  SfsOptions();
};

} // end namespace asp

#endif // __ASP_SFS_SFS_OPTIONS_H__

