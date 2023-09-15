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

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>

#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Cartography/Datum.h>
#include <vw/Camera/OpticalBarModel.h>

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Tools/bundle_adjust_cost_functions.h> // Ceres included in this file.

#include <stdlib.h>
#include <iostream>

// This file contains the bundle adjust options and some other needed functions.

/// These are the different camera modes that bundle_adjust supports.
enum BACameraType {BaCameraType_Pinhole    = 0,
                   BaCameraType_OpticalBar = 1,
                   BaCameraType_CSM        = 2,
                   BaCameraType_Other      = 3};

/// The big bag of parameters needed by bundle_adjust.cc
/// The ones shared with jitter_solve.cc are in asp::BaBaseOptions.
struct Options: public asp::BaBaseOptions {
  std::vector<std::string>  gcp_files;
  std::string cnet_file, vwip_prefix,
    cost_function, mapprojected_data, gcp_from_mapprojected,
    image_list, camera_list, mapprojected_data_list,
    fixed_image_list;
  int ip_per_tile, ip_per_image, matches_per_tile;
  double forced_triangulation_distance, overlap_exponent, ip_triangulation_max_error;
  int    instance_count, instance_index, num_random_passes, ip_num_ransac_iterations;
  bool   save_intermediate_cameras, approximate_pinhole_intrinsics,
    init_camera_using_gcp, disable_pinhole_gcp_init,
    transform_cameras_with_shared_gcp, transform_cameras_using_gcp,
    fix_gcp_xyz, solve_intrinsics, 
    ip_normalize_tiles, ip_debug_images, stop_after_stats, stop_after_matching,
    skip_matching, apply_initial_transform_only, save_vwip;
  BACameraType camera_type;
  std::string datum_str, camera_position_file, initial_transform_file,
    csv_format_str, csv_proj4_str, reference_terrain, disparity_list,
    proj_str;
  double semi_major, semi_minor, position_filter_dist;
  int    num_ba_passes, max_num_reference_points;
  std::string remove_outliers_params_str;
  std::vector<double> intrinsics_limits;
  boost::shared_ptr<vw::ba::ControlNetwork> cnet;
  vw::cartography::Datum datum;
  int    ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error,
    reference_terrain_weight, auto_overlap_buffer;
  bool   skip_rough_homography, enable_rough_homography, disable_tri_filtering,
    enable_tri_filtering, no_datum, individually_normalize, use_llh_error,
    force_reuse_match_files, save_cnet_as_csv,
    enable_correct_velocity_aberration, enable_correct_atmospheric_refraction, dg_use_csm;
  vw::Vector2 elevation_limit;   // Expected range of elevation to limit results to.
  vw::BBox2 lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  vw::BBox2 proj_win; // Limit input triangulated points to this projwin
  std::string overlap_list_file, auto_overlap_params;
  bool have_overlap_list;
  std::set<std::pair<std::string, std::string>> overlap_list;
  vw::Matrix<double> initial_transform;
  std::string   fixed_cameras_indices_str;
  std::set<int> fixed_cameras_indices;
  asp::IntrinsicOptions intrinsics_options;
  vw::Vector2i matches_per_tile_params;

  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), ip_per_image(0), 
             forced_triangulation_distance(-1), overlap_exponent(0), 
              save_intermediate_cameras(false),
             fix_gcp_xyz(false), solve_intrinsics(false), 
             camera_type(BaCameraType_Other),
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(2), max_num_reference_points(-1),
             datum(vw::cartography::Datum(asp::UNSPECIFIED_DATUM, "User Specified Spheroid",
                                          "Reference Meridian", 1, 1, 0)),
             ip_detect_method(0), num_scales(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false), force_reuse_match_files(false){}

  /// Bundle adjustment settings that must be passed to the asp settings
  void copy_to_asp_settings() const{
    asp::stereo_settings().ip_matching_method         = ip_detect_method;
    asp::stereo_settings().epipolar_threshold         = epipolar_threshold;
    asp::stereo_settings().ip_inlier_factor           = ip_inlier_factor;
    asp::stereo_settings().ip_uniqueness_thresh       = ip_uniqueness_thresh;
    asp::stereo_settings().num_scales                 = num_scales;
    asp::stereo_settings().nodata_value               = nodata_value;
    asp::stereo_settings().enable_correct_atmospheric_refraction
      = enable_correct_atmospheric_refraction;
    asp::stereo_settings().enable_correct_velocity_aberration
      = enable_correct_velocity_aberration;
    asp::stereo_settings().dg_use_csm = dg_use_csm;
    asp::stereo_settings().ip_per_tile = ip_per_tile;
    asp::stereo_settings().ip_per_image = ip_per_image;
    asp::stereo_settings().matches_per_tile = matches_per_tile;
    asp::stereo_settings().matches_per_tile_params = matches_per_tile_params;
    asp::stereo_settings().no_datum = no_datum;

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
  
  /// For each option, the string must include a subset of the entries:
  ///  "focal_length, optical_center, distortion_params"
  /// - Need the extra boolean to handle the case where --intrinsics-to-share
  ///   is provided as "" in order to share none of them.
  // TODO(oalexan1): This logic would be more clear if this function was not a member.
  // Also move it out of .h.
  void load_intrinsics_options(std::string intrinsics_to_float_str, // make a copy
                               std::string intrinsics_to_share_str, // make a copy
                               bool        shared_is_specified) {

    // Float and share everything unless specific options are provided.
    intrinsics_options.focus_constant      = true;
    intrinsics_options.center_constant     = true;
    intrinsics_options.distortion_constant = true;
    intrinsics_options.focus_shared        = true;
    intrinsics_options.center_shared       = true;
    intrinsics_options.distortion_shared   = true;

    if (((intrinsics_to_float_str != "") || (intrinsics_to_share_str != "")) 
        && !solve_intrinsics) {
      vw_throw( ArgumentErr() << "To be able to specify only certain intrinsics, "
                              << "the option --solve-intrinsics must be on.\n" );
    }

    if (!solve_intrinsics)
      return;
    
    // If the user did not specify which intrinsics to float, float all of them.
    boost::to_lower(intrinsics_to_float_str);
    if (intrinsics_to_float_str == "" || intrinsics_to_float_str == "all")
      intrinsics_to_float_str = "focal_length optical_center other_intrinsics";

    // If the user did not specify which intrinsics to share, share all of them.
    boost::to_lower(intrinsics_to_share_str);
    if (!shared_is_specified) {
      intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
    } else {
      // Otherwise, 'all' also means share all of them
      if (intrinsics_to_share_str == "all") 
        intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
    }

    if (intrinsics_options.share_intrinsics_per_sensor && shared_is_specified) 
      vw_out() << "When sharing intrinsics per sensor, option "
               << "--intrinsics-to-share is ignored. The intrinsics will "
               << "always be shared for a sensor and never across sensors.\n";

    // By default solve for everything
    intrinsics_options.focus_constant      = false;
    intrinsics_options.center_constant     = false;
    intrinsics_options.distortion_constant = false;

    if (intrinsics_to_float_str != "") {
      intrinsics_options.focus_constant      = true;
      intrinsics_options.center_constant     = true;
      intrinsics_options.distortion_constant = true;
      // These will be individually changed further down
    }

    // If sharing intrinsics per sensor, the only supported mode is that 
    // the intrinsics are always shared per sensor and never across sensors.
    if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
      intrinsics_options.focus_shared      = false;
      intrinsics_options.center_shared     = false;
      intrinsics_options.distortion_shared = false;
    }

    // This is the right place in which to turn 'none' to empty string,
    // which now will mean float nothing.
    if (intrinsics_to_float_str == "none") 
      intrinsics_to_float_str = "";
    // Parse the values  
    std::istringstream is(intrinsics_to_float_str);
    std::string val;
    while (is >> val) {

      if (val != "focal_length" && val != "optical_center" && val != "other_intrinsics")
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to float: " << val << ".\n");
      
      if (val == "focal_length")
        intrinsics_options.focus_constant = false;
      if (val == "optical_center")
        intrinsics_options.center_constant = false;
      if (val == "other_intrinsics")
        intrinsics_options.distortion_constant = false;
    }

    // No parsing is done when sharing intrinsics per sensor, per above 
    if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
      std::istringstream is2(intrinsics_to_share_str);
      while (is2 >> val) {
        if (val == "focal_length")
          intrinsics_options.focus_shared = true;
        if (val == "optical_center")
          intrinsics_options.center_shared = true;
        if (val == "other_intrinsics")
          intrinsics_options.distortion_shared = true;
      }
    }

  } // End function load_intrinsics_options

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

/// Attempt to automatically create the overlap list file estimated
///  footprints for each of the input images.
/// - Currently this only supports cameras with Worldview style XML files.
void auto_build_overlap_list(Options &opt, double lonlat_buffer) {

  typedef std::pair<std::string, std::string> StringPair;

  const size_t num_images = opt.camera_files.size();
  opt.overlap_list.clear();

  vw_out() << "Attempting to automatically estimate image overlaps...\n";
  int  num_overlaps = 0;
  bool read_success = false;

  // Loop through all image pairs
  for (size_t i=0; i<num_images-1; ++i) {

    // Try to get the lonlat bounds for this image
    std::vector<vw::Vector2> pixel_corners_i, lonlat_corners_i;
    try {
      read_success = asp::read_WV_XML_corners(opt.camera_files[i], pixel_corners_i,
                                              lonlat_corners_i);
    } catch(...) {
      read_success = false;
    }
    if (!read_success) {
      vw_throw( ArgumentErr() << "Unable to get corner estimate from file: "
                              << opt.camera_files[i] << ".\n" );
    }

    vw::BBox2 bbox_i; // Convert to BBox
    for (size_t p=0; p<lonlat_corners_i.size(); ++p)
      bbox_i.grow(lonlat_corners_i[p]);
    bbox_i.expand(lonlat_buffer); // Only expand this bounding box by the buffer.

    for (size_t j=i+1; j<num_images; ++j) {

      std::vector<vw::Vector2> pixel_corners_j, lonlat_corners_j;
      try {
        read_success = asp::read_WV_XML_corners(opt.camera_files[j], pixel_corners_j,
                                                lonlat_corners_j);
      } catch(...) {
        read_success = false;
      }
      if (!read_success) {
        vw_throw( ArgumentErr() << "Unable to get corner estimate from file: "
                                << opt.camera_files[j] << ".\n" );
      }

      vw::BBox2 bbox_j; // Convert to BBox
      for (size_t p=0; p<lonlat_corners_j.size(); ++p)
        bbox_j.grow(lonlat_corners_j[p]);

      // Record the files if the bboxes overlap
      // - TODO: Use polygon intersection instead of bounding boxes!
      if (bbox_i.intersects(bbox_j)) {
        vw_out() << "Predicted overlap between images " << opt.image_files[i]
                 << " and " << opt.image_files[j] << std::endl;
        opt.overlap_list.insert(StringPair(opt.image_files[i], opt.image_files[j]));
        opt.overlap_list.insert(StringPair(opt.image_files[j], opt.image_files[i]));
        ++num_overlaps;
      }
    } // End inner loop through cameras
  } // End outer loop through cameras

  if (num_overlaps == 0)
    vw_throw( ArgumentErr() << "Failed to automatically detect any overlapping images!" );

  vw_out() << "Will try to match at " << num_overlaps << " detected overlaps\n.";
} // End function auto_build_overlap_list

#endif // __ASP_TOOLS_BUNDLEADJUST_H__
