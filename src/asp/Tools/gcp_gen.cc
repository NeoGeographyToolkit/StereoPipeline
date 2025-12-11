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

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/MatchList.h>
#include <asp/Core/GCP.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/AspProgramOptions.h>
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
  int min_matches;
  double gcp_sigma;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  const double g_nan_val = std::numeric_limits<double>::quiet_NaN();

  po::options_description general_options("");
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  // Pass interest point matching options directly to stereo_settings
  auto & ip_opt = asp::stereo_settings(); // alias
  
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
    ("ip-detect-method", po::value(&ip_opt.ip_detect_method)->default_value(0),
     "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("ip-per-image", po::value(&ip_opt.ip_per_image)->default_value(20000),
     "How many interest points to detect in each image (the resulting number of "
      "matches will be much less).")
    ("ip-per-tile", po::value(&ip_opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic "
     "determination). This is before matching. Not all interest points will have a match. "
     "See also --matches-per-tile.")
    ("matches-per-tile",  po::value(&ip_opt.matches_per_tile)->default_value(0),
     "How many interest point matches to compute in each image tile (of size "
     "normally 1024^2 pixels). Use a value of --ip-per-tile a few times larger "
     "than this. See also --matches-per-tile-params.")
    ("matches-per-tile-params",  
     po::value(&ip_opt.matches_per_tile_params)->default_value(Vector2(1024, 1280), 
                                                               "1024 1280"),
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
     po::bool_switch(&ip_opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
    ("num-ransac-iterations", 
     po::value(&ip_opt.ip_num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&ip_opt.epipolar_threshold)->default_value(0.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when "
     "computing interest point matches. A smaller threshold will result in fewer "
     "inliers. The default is auto-determined.")
    ("min-matches", po::value(&opt.min_matches)->default_value(10),
     "Set the minimum  number of inlier matches between images for successful matching.")
    ("nodata-value", 
     po::value(&ip_opt.nodata_value)->default_value(g_nan_val),
     "Pixels with values less than or equal to this number are treated as no-data. This "
     "overrides the no-data values from input images.")
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

  // The ortho must have a geo reference
  vw::cartography::GeoReference georef_ortho;
  bool has_georef_ortho = vw::cartography::read_georeference(georef_ortho, opt.ortho_image);
  if (!has_georef_ortho)
    vw_throw(ArgumentErr() << "The ortho image must have a georeference.\n");
    
  // Same for the DEM
  vw::cartography::GeoReference georef_dem;
  bool has_georef_dem = vw::cartography::read_georeference(georef_dem, opt.dem);
  if (!has_georef_dem)
    vw_throw(ArgumentErr() << "The DEM must have a georeference.\n");

  // Sanity checks  
  if (ip_opt.ip_per_image > 0 && ip_opt.ip_per_tile > 0)
    vw_throw(ArgumentErr() << "Can set only one of --ip-per-image and --ip-per-tile.\n");

  // Create the output directory
  vw::create_out_dir(opt.output_prefix);
  vw::create_out_dir(opt.output_gcp);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.output_prefix);

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
      matchIpNoCams(opt.camera_image, opt.ortho_image, opt.output_prefix, ip1, ip2);
    } else {
      // Consider the advanced --mapproj-image option
      is_mapproj = true;
      matchIpNoCams(opt.mapproj_image, opt.ortho_image, opt.output_prefix, ip1, ip2);
    }
    
  } else {
    vw::vw_out() << "Reading matches from: " << opt.match_file << "\n";
    vw::ip::read_binary_match_file(opt.match_file, ip1, ip2);
  }

  // If too few matches, fail, rather than give incorrect results
  if (int(ip1.size()) < opt.min_matches)
    vw::vw_throw(ArgumentErr() << "Found only " << ip1.size() << " matches, "
              << "fewer than the minimum of " << opt.min_matches
              << " (option --min-matches). Cannot create GCP file.\n");
  
  if (opt.mapproj_image != "")
    undo_mapproj(opt, ip1, ip2);

  // Populate from two vectors of matched interest points
  asp::MatchList matchList;
  matchList.populateFromIpPair(ip1, ip2);

  // Write the GCP file
  std::vector<std::string> image_files = {opt.camera_image, opt.ortho_image};
  asp::writeGcp(image_files, opt.output_gcp, opt.dem, matchList, opt.gcp_sigma);

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
