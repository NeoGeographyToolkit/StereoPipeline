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

#ifndef __ASP_TOOLS_BUNDLEADJUST_H__
#define __ASP_TOOLS_BUNDLEADJUST_H__

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#pragma GCC diagnostic pop

#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Cartography/Datum.h>
#include <vw/Camera/OpticalBarModel.h>

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Tools/bundle_adjust_cost_functions.h> // Ceres included in this file.

#include <stdlib.h>
#include <iostream>

// This file contains the bundle adjust options and some other needed functions.

/// The big bag of parameters needed by bundle_adjust.cc
/// The ones shared with jitter_solve.cc are in asp::BaBaseOptions.
struct Options: public asp::BaBaseOptions {
  std::vector<std::string>  gcp_files;
  std::string cnet_file, vwip_prefix,
    cost_function, mapprojected_data, gcp_from_mapprojected;
  int ip_per_tile, ip_per_image, matches_per_tile;
  double forced_triangulation_distance, overlap_exponent, ip_triangulation_max_error;
  int    instance_count, instance_index, num_random_passes, ip_num_ransac_iterations;
  bool   save_intermediate_cameras, approximate_pinhole_intrinsics,
    init_camera_using_gcp, disable_pinhole_gcp_init,
    transform_cameras_with_shared_gcp, transform_cameras_using_gcp,
    fix_gcp_xyz, solve_intrinsics, 
    ip_normalize_tiles, ip_debug_images, stop_after_stats, stop_after_matching,
    skip_matching, apply_initial_transform_only, save_vwip, propagate_errors;
  std::string camera_position_file, initial_transform_file,
    csv_format_str, csv_proj4_str, reference_terrain, disparity_list,
    proj_str, dem_file_for_overlap;
  double semi_major, semi_minor, position_filter_dist;
  int    num_ba_passes, max_num_reference_points;
  std::string remove_outliers_params_str;
  std::vector<double> intrinsics_limits;
  boost::shared_ptr<vw::ba::ControlNetwork> cnet;
  int    ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error,
    reference_terrain_weight, auto_overlap_buffer, pct_for_overlap;
  bool skip_rough_homography, enable_rough_homography, disable_tri_filtering,
    enable_tri_filtering, no_datum, individually_normalize, use_llh_error,
    force_reuse_match_files, save_cnet_as_csv, aster_use_csm;
  vw::Vector2 elevation_limit;   // Expected range of elevation to limit results to.
  vw::BBox2 lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  vw::BBox2 proj_win; // Limit input triangulated points to this projwin
  vw::Matrix<double> initial_transform;
  std::string   fixed_cameras_indices_str;
  std::set<int> fixed_cameras_indices;
  asp::IntrinsicOptions intrinsics_options;
  vw::Vector2i matches_per_tile_params;
  double horizontal_stddev;
  vw::Vector<double> horizontal_stddev_vec; // may come from cameras or user
  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), ip_per_image(0), 
             forced_triangulation_distance(-1), overlap_exponent(0), 
             save_intermediate_cameras(false),
             fix_gcp_xyz(false), solve_intrinsics(false), 
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(2), max_num_reference_points(-1),
             ip_detect_method(0), num_scales(-1), 
             pct_for_overlap(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false), 
             force_reuse_match_files(false) {}

  /// Bundle adjustment settings that must be passed to the asp settings
  void copy_to_asp_settings() const {
    asp::stereo_settings().ip_matching_method         = ip_detect_method;
    asp::stereo_settings().epipolar_threshold         = epipolar_threshold;
    asp::stereo_settings().ip_inlier_factor           = ip_inlier_factor;
    asp::stereo_settings().ip_uniqueness_thresh       = ip_uniqueness_thresh;
    asp::stereo_settings().num_scales                 = num_scales;
    asp::stereo_settings().nodata_value               = nodata_value;
    
    asp::stereo_settings().aster_use_csm = aster_use_csm;
    asp::stereo_settings().ip_per_tile = ip_per_tile;
    asp::stereo_settings().ip_per_image = ip_per_image;
    asp::stereo_settings().matches_per_tile = matches_per_tile;
    asp::stereo_settings().matches_per_tile_params = matches_per_tile_params;
    asp::stereo_settings().no_datum = no_datum;
    asp::stereo_settings().use_least_squares = false; // never true with ba
    
    // Note that by default rough homography and tri filtering are disabled
    // as input cameras may be too inaccurate for that.
    asp::stereo_settings().skip_rough_homography      = !enable_rough_homography;
    asp::stereo_settings().disable_tri_filtering      = !enable_tri_filtering;

    // Do not pass this as it will results in filtering by elevation and lonlat
    // with unoptimized cameras. We will do that filtering with optimized
    // cameras later.
    //asp::stereo_settings().elevation_limit            = elevation_limit;
    //asp::stereo_settings().lon_lat_limit              = lon_lat_limit;
    
    asp::stereo_settings().individually_normalize     = individually_normalize;
    asp::stereo_settings().force_reuse_match_files    = force_reuse_match_files;
    asp::stereo_settings().min_triangulation_angle    = min_triangulation_angle;
    asp::stereo_settings().ip_triangulation_max_error = ip_triangulation_max_error;
    asp::stereo_settings().ip_num_ransac_iterations   = ip_num_ransac_iterations;
    asp::stereo_settings().ip_edge_buffer_percent     = ip_edge_buffer_percent;
    asp::stereo_settings().ip_debug_images            = ip_debug_images;
    asp::stereo_settings().ip_normalize_tiles         = ip_normalize_tiles;
    asp::stereo_settings().propagate_errors           = propagate_errors;
    // The setting below is not used, but populate it for completeness
    asp::stereo_settings().horizontal_stddev          = vw::Vector2(horizontal_stddev,
                                                                  horizontal_stddev);
  }
  
  /// Just parse the string of limits and make sure they are all valid pairs.
  // TODO(oalexan1): This function should not be a member. Also move it out of .h.
  void parse_intrinsics_limits(std::string const& intrinsics_limits_str) {
    intrinsics_limits.clear();
    std::istringstream is(intrinsics_limits_str);
    double val;
    int    count = 0;
    while (is >> val) {
      intrinsics_limits.push_back(val);
      if (count % 2 == 1) {
        if (intrinsics_limits[count] < intrinsics_limits[count-1])
          vw_throw(vw::ArgumentErr()
                   << "Error: Intrinsic limit pairs must be min before max.\n");
      }
      count++;
    }
    if (count % 2 != 0)
      vw_throw( vw::ArgumentErr()
                << "Error: Intrinsic limits must always be provided in min max pairs.\n");
  }
 
}; // End class Options

/// From the input options select the correct Ceres loss function.
ceres::LossFunction* get_loss_function(Options const& opt, double th = 0.0){

  // By default use opt.robust_threshold, unless overwritten above
  if (th == 0) 
    th = opt.robust_threshold;

  ceres::LossFunction* loss_function;
  if      ( opt.cost_function == "l2"     )
    loss_function = NULL;
  else if ( opt.cost_function == "trivial"  )
    loss_function = new ceres::TrivialLoss();
  else if ( opt.cost_function == "huber"  )
    loss_function = new ceres::HuberLoss(th);
  else if ( opt.cost_function == "cauchy" )
    loss_function = new ceres::CauchyLoss(th);
  else if ( opt.cost_function == "l1"     )
    loss_function = new ceres::SoftLOneLoss(th);
  else{
    vw_throw( ArgumentErr() << "Unknown cost function: " << opt.cost_function << ".\n" );
  }
  return loss_function;
}

#endif // __ASP_TOOLS_BUNDLEADJUST_H__
