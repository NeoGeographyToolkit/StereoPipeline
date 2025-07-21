// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

/// \file gcp_gen.cc
///
/// A program for finding GCP given a camera image, a similar looking
/// orthoimage, and a DEM.

#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/MatchList.h>
#include <asp/Core/GCP.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Math/Geometry.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/FileUtils.h>

using namespace vw;
namespace po = boost::program_options;

namespace asp {

struct Options: vw::GdalWriteOptions {
  std::string camera_image, ortho_image, mapproj_image, dem, output_gcp, output_prefix, match_file;
  double inlier_threshold, gcp_sigma;
  int ip_detect_method, ip_per_image, ip_per_tile, matches_per_tile, num_ransac_iterations;
  vw::Vector2i matches_per_tile_params;
  bool individually_normalize;
  Options(): ip_per_image(0), num_ransac_iterations(0.0), inlier_threshold(0) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  general_options.add_options()
    ("camera-image", po::value(&opt.camera_image)->default_value(""),
     "The camera image.")
    ("ortho-image", po::value(&opt.ortho_image)->default_value(""),
     "The ortho image to geolocate the interest points in.")
    ("dem", po::value(&opt.dem)->default_value(""),
     "The DEM to infer the elevations from.")
    ("output-gcp,o", po::value(&opt.output_gcp)->default_value(""),
     "The output GCP file.")
    ("gcp-sigma", po::value(&opt.gcp_sigma)->default_value(1.0),
     "The sigma (uncertainty, in meters) to use for the GCPs. A smaller sigma suggests "
      "a more accurate GCP. See also option --fix-gcp-xyz in bundle adjustment.")
    ("ip-detect-method", po::value(&opt.ip_detect_method)->default_value(0),
     "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("ip-per-image", po::value(&opt.ip_per_image)->default_value(20000),
     "How many interest points to detect in each image (the resulting number of "
      "matches will be much less).")
    ("ip-per-tile", po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic "
     "determination). This is before matching. Not all interest points will have a match. "
     "See also --matches-per-tile.")
    ("matches-per-tile",  po::value(&opt.matches_per_tile)->default_value(0),
     "How many interest point matches to compute in each image tile (of size "
     "normally 1024^2 pixels). Use a value of --ip-per-tile a few times larger "
     "than this. See also --matches-per-tile-params.")
    ("matches-per-tile-params",  po::value(&opt.matches_per_tile_params)->default_value(Vector2(1024, 1280), "1024 1280"),
     "To be used with --matches-per-tile. The first value is the image tile size for both "
     "images. A larger second value allows each right tile to further expand to this size, "
     "resulting in the tiles overlapping. This may be needed if the homography alignment "
     "between these images is not great, as this transform is used to pair up left and "
     "right image tiles.")
    ("mapproj-image", po::value(&opt.mapproj_image)->default_value(""),
     "If interest point matching of the camera image to the orthoimage fails, and this "
     "image is provided, which is produced by mapprojecting the camera image with a camera "
     "model onto a DEM, use this for matching to the orthoimage instead, then transfer the "
     "matches to the camera image. The camera model file and DEM must be available and "
     "their names will be read from the geoheader of the mapprojected image.")
    ("individually-normalize",
     po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
    ("num-ransac-iterations", po::value(&opt.num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&opt.inlier_threshold)->default_value(0.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when "
     "computing interest point matches. A smaller threshold will result in fewer "
     "inliers. The default is 10% of the image diagonal.")
    ("output-prefix", po::value(&opt.output_prefix)->default_value(""),
     "Save intermediate data, including match files, in this directory. Ths will "
     "cache any matches found, and those will be used to create the GCP file. "
     "The match file needs to be deleted if desired to recompute it.")
    
    ("match-file", po::value(&opt.match_file)->default_value(""),
     "If set, use this match file instead of creating one.")
  ;

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--camera-image image.tif --ortho-image ortho.tif "
                    "--dem dem.tif -o output.gcp");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Sanity checks
  if (opt.camera_image == "")
    vw_throw(ArgumentErr() << "Missing camera image.\n");
  if (opt.ortho_image == "")
    vw_throw(ArgumentErr() << "Missing ortho image.\n");
  if (opt.dem == "")
    vw_throw(ArgumentErr() << "Missing DEM.\n");
  if (opt.output_gcp == "")
    vw_throw(ArgumentErr() << "Missing output GCP file.\n");
  if (opt.output_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix must be non-empty.\n");

  // GCP sigma must be positive
  if (opt.gcp_sigma <= 0.0)
    vw_throw(ArgumentErr() << "The GCP sigma must be positive.\n");
    
  // Create the output directory
  vw::create_out_dir(opt.output_prefix);
  vw::create_out_dir(opt.output_gcp);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.output_prefix);

  return;
}

// Get a list of matched IP between two images
void find_matches(Options & opt,
                  std::string const& camera_image_name, 
                  std::string const& ortho_image_name,
                  bool is_mapproj,
                  std::vector<vw::ip::InterestPoint> & matched_ip1,
                  std::vector<vw::ip::InterestPoint> & matched_ip2) {

  if (!is_mapproj)
    vw::vw_out() << "Camera image: " << camera_image_name << "\n";
  else 
    vw::vw_out() << "Mapprojected camera image: " << camera_image_name << "\n";
    
  vw::vw_out() << "Ortho image: " << ortho_image_name << "\n";
  vw::vw_out() << "DEM: " << opt.dem << "\n";

  // Load the DEM
  ImageViewRef<double> dem;
  double dem_nodata; // will be initialized next
  bool has_georef_dem = false;
  vw::cartography::GeoReference georef_dem;
  asp::load_image(opt.dem, dem, dem_nodata, has_georef_dem, georef_dem);
  if (!has_georef_dem)
    vw_throw(ArgumentErr() << "The DEM must have a georeference.\n");

  // Read the images and get the nodata values
  float camera_nodata, ortho_nodata;
  boost::shared_ptr<DiskImageResource>
    camera_rsrc(vw::DiskImageResourcePtr(camera_image_name)),
    ortho_rsrc(vw::DiskImageResourcePtr(ortho_image_name));
  asp::get_nodata_values(camera_rsrc, ortho_rsrc, camera_nodata, ortho_nodata);

  // Get masked views of the images to get statistics from
  DiskImageView<float> camera_image(camera_rsrc), ortho_image(ortho_rsrc);
  ImageViewRef<PixelMask<float>> masked_camera = create_mask(camera_image, camera_nodata);
  ImageViewRef<PixelMask<float>> masked_ortho = create_mask(ortho_image, ortho_nodata);

  // The ortho and DEM must have a geo reference
  vw::cartography::GeoReference georef_ortho;
  bool has_georef_ortho = vw::cartography::read_georeference(georef_ortho, ortho_image_name);
  if (!has_georef_ortho)
    vw_throw(ArgumentErr() << "The ortho image must have a georeference.\n");

  // Clear the outputs
  matched_ip1.clear();
  matched_ip2.clear();

  // If the inlier factor is not set, use 10% of the image diagonal.
  if (opt.inlier_threshold <= 0.0) {
    BBox2i bbox = bounding_box(camera_image);
    opt.inlier_threshold = norm_2(Vector2(bbox.width(), bbox.height())) / 10.0;
    vw::vw_out() << "Setting inlier threshold to: " << opt.inlier_threshold << " pixels.\n";
  } else {
    vw::vw_out() << "Using specified inlier threshold: " << opt.inlier_threshold
                << " pixels.\n";
  }

  if (opt.ip_per_tile > 0) {
    vw::vw_out() << "Since --ip-per-tile was set, not using the --ip-per-image option.\n";
    opt.ip_per_image = 0;
  }

  if (opt.ip_per_image > 0)
    vw::vw_out() << "Searching for " << opt.ip_per_image
                  << " interest points in each image.\n";

  vw_out() << "Matching interest points between: " << camera_image_name << " and "
           << ortho_image_name << "\n";

  // These need to be passed to the ip matching function
  asp::stereo_settings().ip_detect_method        = opt.ip_detect_method;
  asp::stereo_settings().ip_per_image            = opt.ip_per_image;
  asp::stereo_settings().ip_per_tile             = opt.ip_per_tile;
  asp::stereo_settings().ip_per_image            = opt.ip_per_image;
  asp::stereo_settings().matches_per_tile        = opt.matches_per_tile;
  asp::stereo_settings().matches_per_tile_params = opt.matches_per_tile_params;
  stereo_settings().ip_num_ransac_iterations     = opt.num_ransac_iterations;
  stereo_settings().epipolar_threshold           = opt.inlier_threshold;
  asp::stereo_settings().correlator_mode         = true; // no cameras
  asp::stereo_settings().alignment_method        = "none"; // no alignment
  asp::stereo_settings().individually_normalize  = opt.individually_normalize;

  std::string match_file 
    = vw::ip::match_filename(opt.output_prefix, camera_image_name, ortho_image_name);

  vw::Vector<vw::float32,6> camera_stats, ortho_stats;
  camera_stats = asp::gather_stats(masked_camera, camera_image_name,
                                   opt.output_prefix, camera_image_name);
  ortho_stats = asp::gather_stats(masked_ortho, ortho_image_name,
                                   opt.output_prefix, ortho_image_name);

  asp::SessionPtr session(NULL);
  std::string stereo_session = "", input_dem = "";
  bool allow_map_promote = false, total_quiet = true;
  session.reset(asp::StereoSessionFactory::create
                        (stereo_session, // may change
                        opt, 
                        camera_image_name, ortho_image_name,
                        camera_image_name, ortho_image_name,
                        opt.output_prefix, input_dem, allow_map_promote, total_quiet));

  // The match files (.match) are cached unless the images or camera
  // are newer than them.
  std::string camera_vwip_file = "", ortho_vwip_file = "";
  vw::camera::CameraModel* camera_model = NULL, *ortho_model = NULL;
  vw::BBox2 camera_bbox, ortho_bbox;
  session->ip_matching(camera_image_name, ortho_image_name,
                       Vector2(camera_image.cols(), camera_image.rows()),
                       camera_stats, ortho_stats,
                       camera_nodata, ortho_nodata,
                       camera_model, ortho_model,
                       match_file,
                       camera_vwip_file, ortho_vwip_file,
                       camera_bbox, ortho_bbox);

  // Read the match files from disk
  vw::vw_out() << "Reading matches from: " << match_file << "\n";
  vw::ip::read_binary_match_file(match_file, matched_ip1, matched_ip2);
   
  return;
}

// If the matching is done with the mapprojected camera image, undo it. This applies
// only to ip1, as ip2 is for the ortho image. Both of these may be modified if the
// mapprojection is not undone successfully for all matches.
void undo_mapproj(Options & opt,
                  std::vector<vw::ip::InterestPoint> & ip1,
                  std::vector<vw::ip::InterestPoint> & ip2) {
  
  // Read the needed information from the mapproj header
  std::string adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key;
  std::string adj_prefix, image_file, cam_type, cam_file, dem_file;
  read_mapproj_header(opt.mapproj_image, 
                      adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key,
                      adj_prefix, image_file, cam_type, cam_file, dem_file);  

  // The image file must match
  if (image_file != opt.camera_image)
    vw_throw(ArgumentErr() << "The image file in the mapproj header of " 
              << opt.mapproj_image << " does not match the camera image.\n");

  // The dem file must be non-empty
  if (dem_file == "")
    vw_throw(ArgumentErr() << "The DEM file in the mapproj header of " 
              << opt.mapproj_image << " is empty.\n");
  
  // Load the camera model  
  asp::SessionPtr session(asp::StereoSessionFactory::create
                          (cam_type, // may change
                            opt, image_file, image_file, cam_file, cam_file,
                            opt.output_prefix));
  const Vector2 zero_pixel_offset(0,0);
  vw::CamPtr map_proj_cam = session->load_camera_model(image_file, cam_file,
                                                       adj_prefix, zero_pixel_offset);
  
  // Prepare the DEM for interpolation  
  vw::cartography::GeoReference dem_georef;
  ImageViewRef<PixelMask<double>> interp_dem;
  asp::create_interp_dem(dem_file, dem_georef, interp_dem);

  // Read the mapproj_image georef
  vw::cartography::GeoReference map_georef;
  bool has_mapproj_georef 
    = vw::cartography::read_georeference(map_georef, opt.mapproj_image);
  if (!has_mapproj_georef)
    vw_throw(ArgumentErr() << "No georeference found in: " << opt.mapproj_image << ".\n");
  
  // Undo the mapprojection. Skip the matches for which cannot do that.
  std::vector<vw::ip::InterestPoint> local_ip1, local_ip2;
  for (size_t ip_iter = 0; ip_iter < ip1.size(); ip_iter++) {
    if (!asp::projected_ip_to_raw_ip(ip1[ip_iter], interp_dem, map_proj_cam,
                                    map_georef, dem_georef))
      continue;
    local_ip1.push_back(ip1[ip_iter]);
    local_ip2.push_back(ip2[ip_iter]);
  }
  
  // Replace the old IP with the new ones
  ip1 = local_ip1;
  ip2 = local_ip2;
}

void gcp_gen(Options & opt) {

  std::vector<vw::ip::InterestPoint> ip1, ip2;
  if (opt.match_file == "") {

    bool is_mapproj = false;
    if (opt.mapproj_image == "") {
      find_matches(opt, opt.camera_image, opt.ortho_image, is_mapproj, ip1, ip2);
    } else {
      // Consider the advanced --mapproj-image option
      is_mapproj = true;
      find_matches(opt, opt.mapproj_image, opt.ortho_image, is_mapproj, ip1, ip2);
    }
    
  } else {
    vw::vw_out() << "Reading matches from: " << opt.match_file << "\n";
    vw::ip::read_binary_match_file(opt.match_file, ip1, ip2);
  }

  if (opt.mapproj_image != "")
    undo_mapproj(opt, ip1, ip2);

  // Populate from two vectors of matched interest points
  asp::MatchList matchList;
  matchList.populateFromIpPair(ip1, ip2);

  // Write the GCP file
  std::vector<std::string> image_files = {opt.camera_image, opt.ortho_image};
  asp::writeGCP(image_files, opt.output_gcp, opt.dem, matchList, opt.gcp_sigma);

  return;
}

} // end namespace asp

int main(int argc, char *argv[]) {

  asp::Options opt;

  try {

    // Process command line options
    asp::handle_arguments(argc, argv, opt);

    asp::gcp_gen(opt);

  } ASP_STANDARD_CATCHES;
  return 0;
}
