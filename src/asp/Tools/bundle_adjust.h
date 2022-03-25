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

#include <vw/BundleAdjustment/ModelBase.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/BundleAdjustment/AdjustSparse.h>
#include <vw/BundleAdjustment/AdjustRobustSparse.h>
#include <vw/BundleAdjustment/AdjustRobustRef.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Cartography/Datum.h>
#include <vw/FileIO/KML.h>

#include <stdlib.h>
#include <iostream>

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <vw/Camera/OpticalBarModel.h>
#include <asp/Tools/bundle_adjust_misc_functions.h>
#include <asp/Tools/bundle_adjust_cost_functions.h> // Ceres included in this file.

// This file contains the bundle adjust options and some other needed functions.

/// These are the different camera modes that bundle_adjust supports.
enum BACameraType {BaCameraType_Pinhole    = 0,
                   BaCameraType_OpticalBar = 1,
                   BaCameraType_Other      = 2
                  };

/// The big bag of parameters needed by bundle_adjust.cc
struct Options : public vw::cartography::GdalWriteOptions {
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::string cnet_file, out_prefix, input_prefix, stereo_session,
    cost_function, match_files_prefix, clean_match_files_prefix,
    mapprojected_data, gcp_from_mapprojected;
  int ip_per_tile, ip_per_image, ip_edge_buffer_percent;
  double min_triangulation_angle, forced_triangulation_distance,
    lambda, camera_weight, rotation_weight, 
    translation_weight, overlap_exponent, robust_threshold, parameter_tolerance,
    ip_triangulation_max_error;
  int    min_matches, max_pairwise_matches, num_iterations, overlap_limit,
         instance_count, instance_index, num_random_passes, ip_num_ransac_iterations;
  bool   save_intermediate_cameras, approximate_pinhole_intrinsics,
    disable_pinhole_gcp_init, transform_cameras_using_gcp, fix_gcp_xyz, solve_intrinsics,
    ip_normalize_tiles, ip_debug_images, stop_after_stats, stop_after_matching, skip_matching, match_first_to_last,
    apply_initial_transform_only;
  BACameraType camera_type;
  std::string datum_str, camera_position_file, initial_transform_file,
    csv_format_str, csv_proj4_str, reference_terrain, disparity_list,
    heights_from_dem;
  double semi_major, semi_minor, position_filter_dist;
  int    num_ba_passes, max_num_reference_points;
  std::string remove_outliers_params_str;
  std::vector<double> intrinsics_limits;
  vw::Vector<double, 4> remove_outliers_params;
  vw::Vector2 remove_outliers_by_disp_params;
  boost::shared_ptr<vw::ba::ControlNetwork> cnet;
  std::vector<boost::shared_ptr<vw::camera::CameraModel>> camera_models;
  vw::cartography::Datum datum;
  int    ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error,
    reference_terrain_weight, heights_from_dem_weight, heights_from_dem_robust_threshold,
    auto_overlap_buffer;
  bool   skip_rough_homography, enable_rough_homography, disable_tri_filtering, enable_tri_filtering, no_datum, individually_normalize, use_llh_error,
    force_reuse_match_files, save_cnet_as_csv,
    disable_correct_velocity_aberration, disable_correct_atmospheric_refraction;
  vw::Vector2  elevation_limit;     // Expected range of elevation to limit results to.
  vw::BBox2    lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  std::string           overlap_list_file, auto_overlap_params;
  std::set<std::pair<std::string, std::string>> overlap_list;
  vw::Matrix4x4 initial_transform;
  std::string   fixed_cameras_indices_str;
  std::set<int> fixed_cameras_indices;
  IntrinsicOptions intrinisc_options;
  std::map< std::pair<int, int>, std::string> match_files;
  bool single_threaded_cameras; // Set to true if any sessions are single threaded.
  
  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), ip_per_image(0), min_triangulation_angle(0),
             forced_triangulation_distance(-1),
             lambda(-1.0), camera_weight(-1),
             rotation_weight(0), translation_weight(0), overlap_exponent(0), 
             robust_threshold(0), min_matches(0),
             num_iterations(0), overlap_limit(0), save_intermediate_cameras(false),
             fix_gcp_xyz(false), solve_intrinsics(false), camera_type(BaCameraType_Other),
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(2), max_num_reference_points(-1),
             datum(vw::cartography::Datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
                                          "Reference Meridian", 1, 1, 0)),
             ip_detect_method(0), num_scales(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false), force_reuse_match_files(false){}

  /// Duplicate info to asp settings where it needs to go.
  void copy_to_asp_settings() const{
    asp::stereo_settings().ip_matching_method         = ip_detect_method;
    asp::stereo_settings().epipolar_threshold         = epipolar_threshold;
    asp::stereo_settings().ip_inlier_factor           = ip_inlier_factor;
    asp::stereo_settings().ip_uniqueness_thresh       = ip_uniqueness_thresh;
    asp::stereo_settings().num_scales                 = num_scales;
    asp::stereo_settings().nodata_value               = nodata_value;
    asp::stereo_settings().disable_correct_atmospheric_refraction
      = disable_correct_atmospheric_refraction;
    asp::stereo_settings().disable_correct_velocity_aberration
      = disable_correct_velocity_aberration;
    asp::stereo_settings().ip_per_image = ip_per_image;

    // Note that by default rough homography and tri filtering are disabled
    // as input cameras may be too inaccurate for that.
    asp::stereo_settings().skip_rough_homography      = !enable_rough_homography;
    asp::stereo_settings().disable_tri_filtering      = !enable_tri_filtering;

    asp::stereo_settings().no_datum                   = no_datum;

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
  
  /// Ensure that no camera files have duplicate names.  This will cause the output files
  /// to overwrite each other!
  void check_for_duplicate_camera_names() const {
    for (int i=0; i< int(camera_files.size()) - 1; ++i) {   
      std::string filename_1 = asp::bundle_adjust_file_name(out_prefix,
                                              image_files[i], camera_files[i]);
      for (int j=i+1; j< int(camera_files.size()); ++j) {
        std::string filename_2 = asp::bundle_adjust_file_name(out_prefix,
                                                image_files[j], camera_files[j]);
        if (filename_1 == filename_2)
          vw_throw( vw::ArgumentErr() << "All camera model files must have unique names!\n" );
      }
    }
  }

  /// Just parse the string of limits and make sure they are all valid pairs.
  void parse_intrinsics_limits(std::string const& intrinsics_limits_str) {
    intrinsics_limits.clear();
    std::istringstream is(intrinsics_limits_str);
    double val;
    int    count = 0;
    while (is >> val) {
      //std::cout << "val = " << val << std::endl;
      intrinsics_limits.push_back(val);
      if (count % 2 == 1) {
        if (intrinsics_limits[count] < intrinsics_limits[count-1])
          vw_throw( vw::ArgumentErr() << "Error: Intrinsic limit pairs must be min before max.\n" );
      }
      ++count;
    }
    if (count % 2 != 0)
      vw_throw( vw::ArgumentErr() << "Error: Intrinsic limits must always be provided in min max pairs.\n" );
  }
  
  /// For each option, the string must include a subset of the entries:
  ///  "focal_length, optical_center, distortion_params"
  /// - Need the extra boolean to handle the case where --intrinsics-to-share
  ///   is provided as "" in order to share none of them.
  void load_intrinsics_options(std::string const& intrinsics_to_float_str,
                               std::string const& intrinsics_to_share_str,
                               bool               shared_is_specified) {

    // Float everything unless a list was provided.
    intrinisc_options.focus_constant      = true;
    intrinisc_options.center_constant     = true;
    intrinisc_options.distortion_constant = true;
    intrinisc_options.focus_shared        = true;
    intrinisc_options.center_shared       = true;
    intrinisc_options.distortion_shared   = true;

    if (  ((intrinsics_to_float_str != "") || (intrinsics_to_share_str != "")) 
        && !solve_intrinsics) {
      vw_throw( ArgumentErr() << "To be able to specify only certain intrinsics, "
                              << "the option --solve-intrinsics must be on.\n" );
    }

    if (!solve_intrinsics)
      return;

    intrinisc_options.focus_constant      = false; // Default: solve everything!
    intrinisc_options.center_constant     = false;
    intrinisc_options.distortion_constant = false;

    if (intrinsics_to_float_str != "") {
      intrinisc_options.focus_constant      = true;
      intrinisc_options.center_constant     = true;
      intrinisc_options.distortion_constant = true;
    }
    if (shared_is_specified) {
      intrinisc_options.focus_shared      = false;
      intrinisc_options.center_shared     = false;
      intrinisc_options.distortion_shared = false;
    }

    std::istringstream is(intrinsics_to_float_str);
    std::string val;
    while (is >> val) {

      if (val != "focal_length" && val != "optical_center" && val != "other_intrinsics") {
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to float: " << val << ".\n");
      }
      
      if (val == "focal_length")
        intrinisc_options.focus_constant = false;
      if (val == "optical_center")
        intrinisc_options.center_constant = false;
      if (val == "other_intrinsics")
        intrinisc_options.distortion_constant = false;
    }
    std::istringstream is2(intrinsics_to_share_str);
    while (is2 >> val) {
      if (val == "focal_length")
        intrinisc_options.focus_shared = true;
      if (val == "optical_center")
        intrinisc_options.center_shared = true;
      if (val == "other_intrinsics")
        intrinisc_options.distortion_shared = true;
    }

  } // End function load_intrinsics_options

}; // End class Options

/// This is for the BundleAdjustmentModel class where the camera parameters
/// are a rotation/offset that is applied on top of the existing camera model.
/// First read initial adjustments, if any, and apply perhaps a pc_align transform.
bool init_cams(Options & opt, BAParamStorage & param_storage,
       std::vector<boost::shared_ptr<camera::CameraModel> > &new_cam_models){

  bool cameras_changed = false;
  
  // Initialize all of the camera adjustments to zero.
  param_storage.clear_cameras();
  const size_t num_cameras = param_storage.num_cameras();

  // Read the adjustments from a previous run, if present
  if (opt.input_prefix != "") {
    for (size_t icam = 0; icam < num_cameras; icam++){
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      double * cam_ptr = param_storage.get_camera_ptr(icam);
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);
      adjustment.pack_to_array(cam_ptr);
    }
    cameras_changed = true;
  }

  // Get an updated list of camera models
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){
    CameraAdjustment correction(param_storage.get_camera_ptr(icam));
    camera::CameraModel* cam = new camera::AdjustedCameraModel(opt.camera_models[icam],
                                        correction.position(), correction.pose());
    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(cam);
  }

  // Apply any initial transform to the pinhole cameras
  if (opt.initial_transform_file != "") {
    apply_transform_to_cameras(opt.initial_transform, param_storage, new_cam_models);
    cameras_changed = true;
  }
  
  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){
    CameraAdjustment correction(param_storage.get_camera_ptr(icam));
    camera::CameraModel* cam = new camera::AdjustedCameraModel(opt.camera_models[icam],
                                        correction.position(), correction.pose());
    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(cam);
  }

  return cameras_changed;
}

/// Specialization for pinhole cameras.
bool init_cams_pinhole(Options & opt, BAParamStorage & param_storage,
        std::vector<boost::shared_ptr<camera::CameraModel> > &new_cam_models){

  bool cameras_changed = false;
  
  // Copy the camera parameters from the models to param_storage
  const size_t num_cameras = param_storage.num_cameras();
  for (int icam=0; icam < num_cameras; ++icam) {
    PinholeModel* pin_ptr = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());
    vw::vw_out() << "Loading input model: " << *pin_ptr << std::endl;
    pack_pinhole_to_arrays(*pin_ptr, icam, param_storage);
  } // End loop through cameras

  // Apply any initial transform to the pinhole cameras
  if (opt.initial_transform_file != "") {
    apply_transform_to_cameras_pinhole(opt.initial_transform, param_storage, opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){

    PinholeModel* in_cam  = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());
    PinholeModel* out_cam = new PinholeModel(*in_cam); // Start with a copy of the input camera.
    populate_pinhole_from_arrays(icam, param_storage, *out_cam);

    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for optical bar cameras.
bool init_cams_optical_bar(Options & opt, BAParamStorage & param_storage,
        std::vector<boost::shared_ptr<camera::CameraModel> > &new_cam_models){

  bool cameras_changed = false;
  
  // Copy the camera parameters from the models to param_storage
  const size_t num_cameras = param_storage.num_cameras();
  for (int icam=0; icam < num_cameras; ++icam) {
    vw::camera::OpticalBarModel* bar_ptr = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());
    vw::vw_out() << "Loading input model: " << *bar_ptr << std::endl;
    pack_optical_bar_to_arrays(*bar_ptr, icam, param_storage);
  } // End loop through cameras

  // Apply any initial transform to the pinhole cameras
  if (opt.initial_transform_file != "") {
    apply_transform_to_cameras_optical_bar(opt.initial_transform, param_storage, opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){

    vw::camera::OpticalBarModel* in_cam  = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());
    vw::camera::OpticalBarModel* out_cam = new vw::camera::OpticalBarModel(*in_cam); // Start with a copy of the input camera.
    populate_optical_bar_from_arrays(icam, param_storage, *out_cam);

    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(out_cam);
  }

  return cameras_changed;
}


/// Write a pinhole camera file to disk.
void write_pinhole_output_file(Options const& opt, int icam,
                               BAParamStorage const& param_storage) {

  // Get the output file path
  std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                      opt.image_files [icam],
                                                      opt.camera_files[icam]);
  cam_file = boost::filesystem::path(cam_file).replace_extension("tsai").string();

  // Get the final camera model
  vw::camera::PinholeModel* pin_ptr
    = dynamic_cast<vw::camera::PinholeModel*>(opt.camera_models[icam].get());
  populate_pinhole_from_arrays(icam, param_storage, *pin_ptr);

  vw::vw_out() << "Writing: " << cam_file << std::endl;
  pin_ptr->write(cam_file);
  vw::vw_out() << "Writing output model: " << *pin_ptr << std::endl;

  bool has_datum = (opt.datum.name() != UNSPECIFIED_DATUM);
  if (has_datum) {
    vw::vw_out() << "Camera center for " << cam_file << ": "
                  << opt.datum.cartesian_to_geodetic(pin_ptr->camera_center())
                  << " (longitude, latitude, height above datum(m))\n\n";
  }
}


/// Write an optical bar camera file to disk.
void write_optical_bar_output_file(Options const& opt, int icam,
                                   BAParamStorage const& param_storage) {

  // Get the output file path
  std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                      opt.image_files [icam],
                                                      opt.camera_files[icam]);
  cam_file = boost::filesystem::path(cam_file).replace_extension("tsai").string();

  // Get the final camera model
  vw::camera::OpticalBarModel* bar_ptr
    = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());
  populate_optical_bar_from_arrays(icam, param_storage, *bar_ptr);

  vw::vw_out() << "Writing: " << cam_file << std::endl;
  bar_ptr->write(cam_file);
  vw::vw_out() << "Writing output model: " << *bar_ptr << std::endl;

  bool has_datum = (opt.datum.name() != UNSPECIFIED_DATUM);
  if (has_datum) {
    vw::vw_out() << "Camera center for " << cam_file << ": "
                  << opt.datum.cartesian_to_geodetic(bar_ptr->camera_center())
                  << " (longitude, latitude, height above datum(m))\n\n";
  }
}

/// Write a pinhole camera file to disk.
void write_csm_output_file(Options const& opt, int icam,
                           std::string const& adjust_file, 
                           BAParamStorage const& param_storage) {
  
  CameraAdjustment cam_adjust(param_storage.get_camera_ptr(icam));
  
  AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                              cam_adjust.position(), cam_adjust.pose());
  
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  
  // Manufacture the transformed json state file
  std::string json_state = adjust_file;
  
  // If the suffix we want to add is already present, remove it first
  std::string suff = ".adjusted_state";
  auto it = json_state.find(suff);
  if (it != std::string::npos)
    json_state.replace(it, suff.size(), "");
  
  json_state = boost::filesystem::path(json_state).replace_extension(suff + ".json").string();
  
  asp::CsmModel * csm_model = dynamic_cast<asp::CsmModel*>
    (vw::camera::unadjusted_model(opt.camera_models[icam].get()));
  if (csm_model == NULL) 
          vw::vw_throw(vw::ArgumentErr() << "Expected a csm camera model.");
  
  csm_model->save_transformed_json_state(json_state, ecef_transform);
}

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
