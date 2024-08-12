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

#include <asp/Camera/BundleAdjustCamera.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Cartography/Datum.h>

namespace asp {
  
// This file contains the bundle adjust options and some other needed functions.
// The ones shared with jitter_solve.cc are in asp::BaBaseOptions.
struct BaOptions: public asp::BaBaseOptions {
  std::string cnet_file, vwip_prefix,
    cost_function, mapprojected_data, gcp_from_mapprojected;
  int ip_per_tile, ip_per_image, matches_per_tile;
  double forced_triangulation_distance, overlap_exponent, ip_triangulation_max_error;
  int instance_count, instance_index, num_random_passes, ip_num_ransac_iterations;
  bool save_intermediate_cameras, approximate_pinhole_intrinsics,
    init_camera_using_gcp, disable_pinhole_gcp_init,
    transform_cameras_with_shared_gcp, transform_cameras_using_gcp,
    fix_gcp_xyz, solve_intrinsics, 
    ip_normalize_tiles, ip_debug_images, stop_after_stats, stop_after_matching,
    skip_matching, apply_initial_transform_only, save_vwip, propagate_errors;
  std::string camera_position_file, initial_transform_file,
    csv_format_str, csv_srs, csv_proj4_str, disparity_list,
    dem_file_for_overlap;
  double semi_major, semi_minor, position_filter_dist;
  int num_ba_passes;
  std::string remove_outliers_params_str;
  std::vector<double> intrinsics_limits;
  boost::shared_ptr<vw::ba::ControlNetwork> cnet;
  int ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error,
    auto_overlap_buffer, pct_for_overlap, min_distortion;
  bool skip_rough_homography, enable_rough_homography, disable_tri_filtering,
    enable_tri_filtering, no_datum, individually_normalize, use_llh_error,
    force_reuse_match_files, no_poses_from_nvm, save_cnet_as_csv, aster_use_csm;
  vw::Vector2 elevation_limit;   // Expected range of elevation to limit results to.
  vw::BBox2 lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  vw::Matrix<double> initial_transform;
  std::string   fixed_cameras_indices_str, flann_method;
  std::set<int> fixed_cameras_indices;
  asp::IntrinsicOptions intrinsics_options;
  vw::Vector2i matches_per_tile_params;
  double horizontal_stddev;
  vw::Vector<double> horizontal_stddev_vec; // may come from cameras or user
  
  // Make sure all values are initialized, even though they will be
  // over-written later.
  BaOptions(): ip_per_tile(0), ip_per_image(0), 
             forced_triangulation_distance(-1), overlap_exponent(0), 
             save_intermediate_cameras(false),
             fix_gcp_xyz(false), solve_intrinsics(false), 
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(2), 
             ip_detect_method(0), num_scales(-1), 
             pct_for_overlap(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false), 
             force_reuse_match_files(false), no_poses_from_nvm(false),
             save_cnet_as_csv(false), aster_use_csm(false) {}

  /// Bundle adjustment settings that must be passed to the asp settings
  void copy_to_asp_settings() const;
  
}; // End class Options

} // end namespace asp

#endif // __ASP_TOOLS_BUNDLEADJUST_H__
