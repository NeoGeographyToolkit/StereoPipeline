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
#include <asp/Tools/bundle_adjust_misc_functions.h>
#include <asp/Tools/bundle_adjust_cost_functions.h> // Ceres included in this file.

// This file contains the bundle adjust options and some other needed functions.


//==========================================================================

/// The big bag of parameters needed by bundle_adjust.cc
struct Options : public vw::cartography::GdalWriteOptions {
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::string cnet_file, out_prefix, input_prefix, stereo_session_string,
    cost_function, mapprojected_data, gcp_data;
  int    ip_per_tile, ip_edge_buffer_percent;
  double min_triangulation_angle, lambda, camera_weight, rotation_weight, 
    translation_weight, overlap_exponent, robust_threshold, parameter_tolerance,
    ip_triangulation_max_error;
  int    report_level, min_matches, max_iterations, overlap_limit;
  bool   save_iteration, create_pinhole, approximate_pinhole_intrinsics,
         disable_pinhole_gcp_init, fix_gcp_xyz, solve_intrinsics,
         disable_tri_filtering, ip_normalize_tiles, ip_debug_images;
  std::string datum_str, camera_position_file, initial_transform_file,
    csv_format_str, csv_proj4_str, reference_terrain, disparity_list,
    heights_from_dem;
  double semi_major, semi_minor, position_filter_dist;
  int    num_ba_passes, max_num_reference_points;
  std::string remove_outliers_params_str;
  vw::Vector<double, 4> remove_outliers_params;
  vw::Vector2 remove_outliers_by_disp_params;
  boost::shared_ptr<vw::ba::ControlNetwork> cnet;
  std::vector<boost::shared_ptr<vw::camera::CameraModel> > camera_models;
  vw::cartography::Datum datum;
  int    ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error,
    reference_terrain_weight;
  bool   skip_rough_homography, individually_normalize, use_llh_error, save_cnet_as_csv;
  vw::Vector2  elevation_limit;     // Expected range of elevation to limit results to.
  vw::BBox2    lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  std::string           overlap_list_file;
  std::set< std::pair<std::string, std::string> > overlap_list;
  vw::Matrix4x4 initial_transform;
  std::string   fixed_cameras_indices_str;
  std::set<int> fixed_cameras_indices;
  IntrinsicOptions intrinisc_options;
  std::map< std::pair<int, int>, std::string> match_files;
  
  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), min_triangulation_angle(0), lambda(-1.0), camera_weight(-1),
             rotation_weight(0), translation_weight(0), overlap_exponent(0), 
             robust_threshold(0), report_level(0), min_matches(0),
             max_iterations(0), overlap_limit(0), save_iteration(false),
             create_pinhole(false), fix_gcp_xyz(false), solve_intrinsics(false),
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(1), max_num_reference_points(-1),
             datum(vw::cartography::Datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
                                          "Reference Meridian", 1, 1, 0)),
             ip_detect_method(0), num_scales(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false){}

  /// Duplicate info to asp settings where it needs to go.
  void copy_to_asp_settings() const{
    asp::stereo_settings().ip_matching_method         = ip_detect_method;
    asp::stereo_settings().epipolar_threshold         = epipolar_threshold;
    asp::stereo_settings().ip_inlier_factor           = ip_inlier_factor;
    asp::stereo_settings().ip_uniqueness_thresh       = ip_uniqueness_thresh;
    asp::stereo_settings().num_scales                 = num_scales;
    asp::stereo_settings().nodata_value               = nodata_value;
    asp::stereo_settings().skip_rough_homography      = skip_rough_homography;
    asp::stereo_settings().elevation_limit            = elevation_limit;
    asp::stereo_settings().lon_lat_limit              = lon_lat_limit;
    asp::stereo_settings().individually_normalize     = individually_normalize;
    asp::stereo_settings().min_triangulation_angle    = min_triangulation_angle;
    asp::stereo_settings().ip_triangulation_max_error = ip_triangulation_max_error;
    asp::stereo_settings().disable_tri_filtering      = disable_tri_filtering;
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

  /// For each option, the string must include a subset of the entries:
  ///  "focal_length, optical_center, distortion_params"
  void load_intrinsics_options(std::string const& intrinsics_to_float_str,
                               std::string const& intrinsics_to_share_str) {

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
    if (intrinsics_to_share_str != "") {
      intrinisc_options.focus_shared      = false;
      intrinisc_options.center_shared     = false;
      intrinisc_options.distortion_shared = false;
    }

    std::istringstream is(intrinsics_to_float_str);
    std::string val;
    while (is >> val) {

      if (val != "focal_length" && val != "optical_center" && val != "distortion_params") {
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to float: " << val << ".\n");
      }
      
      if (val == "focal_length")
        intrinisc_options.focus_constant = false;
      if (val == "optical_center")
        intrinisc_options.center_constant = false;
      if (val == "distortion_params")
        intrinisc_options.distortion_constant = false;
    }
    std::istringstream is2(intrinsics_to_share_str);
    while (is2 >> val) {
      if (val == "focal_length")
        intrinisc_options.focus_shared = true;
      if (val == "optical_center")
        intrinisc_options.center_shared = true;
      if (val == "distortion_params")
        intrinisc_options.distortion_shared = true;
    }

  } // End function load_intrinsics_options

}; // End class Options

//==================================================================================
// Mapprojected image functions.

/// If the user map-projected the images and created matches by hand
/// (this is useful when the illumination conditions are too different,
/// and automated matching fails), project those matching ip back
/// into the cameras, creating matches between the raw images
/// that then bundle_adjust can use.
/// - The output matches are written to file.
void create_matches_from_mapprojected_images(Options const& opt){
  
  std::istringstream is(opt.mapprojected_data);
  std::vector<std::string> map_files;
  std::string file;
  while (is >> file){
    map_files.push_back(file); 
  }
  std::string dem_file = map_files.back();
  map_files.erase(map_files.end() - 1);

  if ( opt.camera_models.size() != map_files.size()) 
    vw_throw(ArgumentErr() << "Error: Expecting as many input cameras as map-projected images.\n");

  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> > interp_dem;
  create_interp_dem(dem_file, dem_georef, interp_dem);

  for (size_t i = 0; i < map_files.size(); i++) {
    for (size_t j = i+1; j < map_files.size(); j++) {

      vw::cartography::GeoReference georef1, georef2;
      vw_out() << "Reading georef from " << map_files[i] << ' ' << map_files[j] << std::endl;
      bool is_good1 = vw::cartography::read_georeference(georef1, map_files[i]);
      bool is_good2 = vw::cartography::read_georeference(georef2, map_files[j]);
      if (!is_good1 || !is_good2) {
        vw_throw(ArgumentErr() << "Error: Cannot read georeference.\n");
      }

      std::string match_filename = ip::match_filename(opt.out_prefix,
                                                      map_files[i], map_files[j]);
      if (!boost::filesystem::exists(match_filename)) {
        vw_out() << "Missing: " << match_filename << "\n";
        continue;
      }
      vw_out() << "Reading: " << match_filename << std::endl;
      std::vector<ip::InterestPoint> ip1,     ip2;
      std::vector<ip::InterestPoint> ip1_cam, ip2_cam;
      ip::read_binary_match_file( match_filename, ip1, ip2 );

      // Undo the map-projection
      for (size_t ip_iter = 0; ip_iter < ip1.size(); ip_iter++) {

        // TODO: Does the logic here fail if P1 succeeds but P2 fails???
        vw::ip::InterestPoint P1 = ip1[ip_iter];
        vw::ip::InterestPoint P2 = ip2[ip_iter];
        if (!projected_ip_to_raw_ip(P1, interp_dem, opt.camera_models[i], georef1, dem_georef))
          continue;
        if (!projected_ip_to_raw_ip(P2, interp_dem, opt.camera_models[j], georef2, dem_georef))
          continue;

        ip1_cam.push_back(P1);
        ip2_cam.push_back(P2);
      }

      // TODO: There is a problem if the number of matches changes!!!
      vw_out() << "Saving " << ip1_cam.size() << " matches.\n";
      std::string image1_path  = opt.image_files[i];
      std::string image2_path  = opt.image_files[j];
      match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);

      vw_out() << "Writing: " << match_filename << std::endl;
      ip::write_binary_match_file(match_filename, ip1_cam, ip2_cam);

    }
  }
} // End function create_matches_from_mapprojected_images

/// If the user map-projected the images and created matches by hand
/// from each map-projected image to the DEM it was map-projected onto,
/// project those matches back into the camera image, and create gcp
/// tying each camera image match to its desired location on the DEM.
void create_gcp_from_mapprojected_images(Options const& opt){

  // Read the map-projected images and the dem
  std::istringstream is(opt.gcp_data);
  std::vector<std::string> image_files;
  std::string file;
  while (is >> file){
    image_files.push_back(file); 
  }
  std::string dem_file = image_files.back();
  image_files.erase(image_files.end() - 1); // wipe the dem from the list

  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> > interp_dem;
  create_interp_dem(dem_file, dem_georef, interp_dem);

  int num_images = image_files.size();
  std::vector<std::vector<vw::ip::InterestPoint> > matches;
  std::vector<vw::cartography::GeoReference> img_georefs;
  matches.resize(num_images + 1); // the last match will be for the DEM

  // Read the matches and georefs
  for (int i = 0; i < num_images; i++) {

    vw::cartography::GeoReference img_georef;
    vw_out() << "Reading georef from " << image_files[i]  << std::endl;
    bool is_good_img = vw::cartography::read_georeference(img_georef, image_files[i]);
    if (!is_good_img) {
      vw_throw(ArgumentErr() << "Error: Cannot read georeference.\n");
    }
    img_georefs.push_back(img_georef);

    std::string match_filename = ip::match_filename(opt.out_prefix,
                                                    image_files[i], dem_file);
    if (!boost::filesystem::exists(match_filename)) 
      vw_throw(ArgumentErr() << "Missing: " << match_filename << ".\n");

    vw_out() << "Reading: " << match_filename << std::endl;
    std::vector<ip::InterestPoint> ip1, ip2;
    ip::read_binary_match_file( match_filename, ip1, ip2 );

    if (matches[num_images].size() > 0 && matches[num_images].size() != ip2.size()) {
      vw_throw(ArgumentErr() << "All match files must have the same number of IP.\n");
    }
    matches[i]          = ip1;
    matches[num_images] = ip2;
  }

  std::vector<std::vector<vw::ip::InterestPoint> > cam_matches = matches;

  std::string gcp_file;
  for (int i = 0; i < num_images; i++) {
    gcp_file += boost::filesystem::basename(opt.image_files[i]);
    if (i < num_images - 1) gcp_file += "__"; 
  }
  gcp_file = opt.out_prefix + "-" + gcp_file + ".gcp";

  vw_out() << "Writing: " << gcp_file << std::endl;
  std::ofstream output_handle(gcp_file.c_str());

  int num_ips = matches[0].size();
  int pts_count = 0;
  for (int p = 0; p < num_ips; p++) { // Loop through IPs

    // Compute the GDC coordinate of the point
    ip::InterestPoint dem_ip = matches[num_images][p];
    Vector2 dem_pixel(dem_ip.x, dem_ip.y);
    Vector2 lonlat = dem_georef.pixel_to_lonlat(dem_pixel);

    if (!interp_dem.pixel_in_bounds(dem_pixel)) {
      vw_out() << "Skipping pixel outside of DEM: " << dem_pixel << std::endl;
      continue;
    }

    PixelMask<float> mask_height = interp_dem(dem_pixel[0], dem_pixel[1])[0];
    if (!is_valid(mask_height)) continue;

    Vector3 llh(lonlat[0], lonlat[1], mask_height.child());
    //Vector3 dem_xyz = dem_georef.datum().geodetic_to_cartesian(llh);

    // The ground control point ID
    output_handle << pts_count;
    // Lat, lon, height
    output_handle << ", " << lonlat[1] << ", " << lonlat[0] << ", " << mask_height.child();
    // Sigma values
    output_handle << ", " << 1 << ", " << 1 << ", " << 1;

    // Write the per-image information
    for (int i = 0; i < num_images; i++) {

      // Take the ip in the map-projected image, and back-project it into the camera
      ip::InterestPoint ip = matches[i][p];
      if (!projected_ip_to_raw_ip(ip, interp_dem, opt.camera_models[i], img_georefs[i], dem_georef))
          continue;

      // TODO: Here we can have a book-keeping problem!
      cam_matches[i][p] = ip;

      output_handle << ", " << opt.image_files[i];
      output_handle << ", " << ip.x << ", " << ip.y; // IP location in image
      output_handle << ", " << 1 << ", " << 1; // Sigma values
    } // End loop through IP sets
    output_handle << std::endl; // Finish the line
    pts_count++;

  } // End loop through IPs
  output_handle.close();

  // Write out match files for each pair of images.
  for (int i = 0; i < num_images; i++) {
    for (int j = i; j < num_images; j++) { // write also for i, i. Useful for only 1 image.
      std::string image1_path    = opt.image_files[i];
      std::string image2_path    = opt.image_files[j];
      std::string match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);

      vw_out() << "Writing: " << match_filename << std::endl;
      ip::write_binary_match_file(match_filename, cam_matches[i], cam_matches[j]);
    }
  }

}

// End map projection functions
//===============================================================================



/// This is for the BundleAdjustmentModel class where the camera parameters
/// are a rotation/offset that is applied on top of the existing camera model.
/// First read initial adjustments, if any, and apply perhaps a pc_align transform.
void init_cams(Options & opt, BAParamStorage & param_storage){

  // Initialize all of the camera adjustments to zero.
  param_storage.clear_cameras();

  // Read the adjustments from a previous run, if present
  if (opt.input_prefix != "") {
    for (size_t icam = 0; icam < param_storage.num_cameras(); icam++){
      std::string adjust_file = asp::bundle_adjust_file_name(opt.input_prefix,
                                                             opt.image_files[icam],
                                                             opt.camera_files[icam]);
      double * cam_ptr = param_storage.get_camera_ptr(icam);
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);
      adjustment.pack_to_array(cam_ptr);
    }
  }

  // Apply any initial transform to the pinhole cameras
  if (opt.initial_transform_file != "") {
    apply_transform_to_cameras(opt.initial_transform, param_storage);
  }
}

/// Specialization for pinhole cameras.
void init_cams_pinhole(Options & opt, BAParamStorage & param_storage){

  // Copy the camera parameters from the models to param_storage
  for (int i=0; i < param_storage.num_cameras(); ++i) {
    PinholeModel* pin_ptr = dynamic_cast<PinholeModel*>(opt.camera_models[i].get());
    vw::vw_out() << "Loading input model: " << *pin_ptr << std::endl;
    pack_pinhole_to_arrays(*pin_ptr, i, param_storage);
  } // End loop through cameras

  // Apply any initial transform to the pinhole cameras
  if (opt.initial_transform_file != "") 
    apply_transform_to_cameras(opt.initial_transform, param_storage);

  return;
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


/// From the input options select the correct Ceres loss function.
ceres::LossFunction* get_loss_function(Options const& opt ){
  double th = opt.robust_threshold;
  ceres::LossFunction* loss_function;
  if      ( opt.cost_function == "l2"     )
    loss_function = NULL;
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
      read_success = asp::read_WV_XML_corners(opt.camera_files[i], pixel_corners_i, lonlat_corners_i);
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
        read_success = asp::read_WV_XML_corners(opt.camera_files[j], pixel_corners_j, lonlat_corners_j);
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
