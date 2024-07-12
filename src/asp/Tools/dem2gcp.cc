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

// TODO(oalexan1): The hillshaded file shifts everything by 0.5 pixels.

// \file dem2gcp.cc
// See the documentation on readthedocs.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef vw::PixelMask<vw::Vector2f> Dpix;

// Given a pixel (x, y), and a length l, enumerate all the pixels
// on the boundary of the square of side length 2*l centered at (x, y).
void enumerate_boundary_pixels(int x, int y, int l, std::vector<vw::Vector2> & pixels) {
  pixels.clear();
  for (int i = -l; i <= l; i++) {
    pixels.push_back(vw::Vector2(x-l, y+i));
    pixels.push_back(vw::Vector2(x+l, y+i));
    pixels.push_back(vw::Vector2(x+i, y-l));
    pixels.push_back(vw::Vector2(x+i, y+l));
  }
}

// Find the disparity value at a given pixel. If not valid, look around
// on the perimeter of a square of side length 2. Then increase the square size.
// If the disparity is still invalid, return an invalid value.
Dpix find_disparity(vw::ImageViewRef<Dpix> const& disparity,
                 int x, int y, int max_l) {
  if (x < 0 || y < 0 || x >= disparity.cols() || y >= disparity.rows())
    return Dpix();
  
  if (is_valid(disparity(x, y)))
    return disparity(x, y);  

  // Iterate from 1 to max_l with an empty loop
  for (int l = 1; l <= max_l; l++) {

    // Will keep the pixel closest to the center
    double max_dist = std::numeric_limits<double>::max();
    vw::Vector2 best_pix;
    bool found = false;

    std::vector<vw::Vector2> pixels;
    enumerate_boundary_pixels(x, y, l, pixels);
    
    for (int i = 0; i < (int)pixels.size(); i++) {
      int px = pixels[i].x(), py = pixels[i].y();
      if (px < 0 || py < 0 || px >= disparity.cols() || py >= disparity.rows())
        continue;
      if (!is_valid(disparity(px, py)))
        continue;
      double dist = norm_2(vw::Vector2(px-x, py-y));
      if (dist >= max_dist)
        continue;
      max_dist = dist;
      best_pix = vw::Vector2(px, py);
      found = true;
    } // end loop over pixels on the boundary of the square of side length 2*l
    if (found) {
      return disparity(best_pix.x(), best_pix.y());
    }
  } // end loop over ever-growing squares
  
  // Give up
  return Dpix();
}

// Put the variables below in a struct
struct Options : public vw::GdalWriteOptions {
std::string warped_dem_file, ref_dem_file, 
  warped_to_ref_disp_file, left_img, right_img, 
  left_cam, right_cam, 
  match_file, out_gcp;
  int search_len;
  double gcp_sigma;
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("");
  general_options.add_options()
    ("warped-dem", po::value(&opt.warped_dem_file), 
     "The DEM file produced by stereo, that may have warping due to unmodeled distortion.")
    ("ref-dem", po::value(&opt.ref_dem_file), 
     "The reference DEM file, which is assumed to be accurate.")
    ("warped-to-ref-disparity", po::value(&opt.warped_to_ref_disp_file), 
     "The stereo disparity from the warped DEM to the reference DEM (use the first band "
     "of the hillshaded DEMs as inputs for the disparity).")
    ("left-image", po::value(&opt.left_img), 
     "The left raw image that produced the warped DEM.")
    ("right-image", po::value(&opt.right_img), 
     "The right raw image that produced the warped DEM.")
    ("left-camera", po::value(&opt.left_cam), 
     "The left camera that was used for stereo.")
    ("right-camera", po::value(&opt.right_cam), 
     "The right camera that was used for stereo.")
    ("match-file", 
     po::value(&opt.match_file), 
     "A match file between the left and right raw images with many dense matches.")
    ("search-len", po::value(&opt.search_len)->default_value(5), 
     "How many DEM pixels to search around to find a valid DEM disparity "
     "(pick the closest).")
    ("gcp-sigma", po::value(&opt.gcp_sigma)->default_value(1.0),
     "The sigma to use for the GCP points. A smaller value will give to GCP more weight.")
    ("output-gcp", po::value(&opt.out_gcp), 
     "The produced GPP file with ground coordinates from the reference DEM.")
    ("help,h", "Display this help message");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
    
  // Create the output directory
  vw::create_out_dir(opt.out_gcp);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_gcp);
}

int run_dem2gcp(int argc, char * argv[]) {
  
  Options opt;
  
  // Parse and validate the input options  
  handle_arguments(argc, argv, opt);
  
  // Load the cameras  
  std::vector<vw::CamPtr> camera_models(2);
  std::string stereo_session, out_prefix;
  asp::SessionPtr session(NULL);
  session.reset(asp::StereoSessionFactory::create
                        (stereo_session, // will change
                        opt, opt.left_img, opt.right_img,
                        opt.left_cam, opt.right_cam,
                        out_prefix));
  session->camera_models(camera_models[0], camera_models[1]);

  // Load the control network  
  std::vector<std::string> image_files;
  image_files.push_back(opt.left_img);
  image_files.push_back(opt.right_img);
  vw::ba::ControlNetwork cnet("asp");
  bool triangulate_control_points = true;
  std::map<std::pair<int, int>, std::string> match_files;
  match_files[std::make_pair(0, 1)] = opt.match_file;
  int min_matches = 0;
  double min_triangulation_angle = 1e-10;
  bool forced_triangulation_distance = -1.0;
  int max_pairwise_matches = 1e6;
  bool success = vw::ba::build_control_network(triangulate_control_points,
                                               cnet, camera_models,
                                               image_files,
                                               match_files,
                                               min_matches,
                                               min_triangulation_angle,
                                               forced_triangulation_distance,
                                               max_pairwise_matches);
  if (!success)
    vw::vw_throw(vw::ArgumentErr() << "Failed to load the interest points.\n");
  
  // Load the DEM and georef, and prepare for interpolation  
  vw::cartography::GeoReference warped_dem_georef;
  vw::ImageViewRef<vw::PixelMask<double>> interp_warped_dem;
  asp::create_interp_dem(opt.warped_dem_file, warped_dem_georef, interp_warped_dem);

  // Load the reference DEM
  vw::cartography::GeoReference ref_dem_georef;
  vw::ImageViewRef<vw::PixelMask<double>> interp_ref_dem;
  asp::create_interp_dem(opt.ref_dem_file, ref_dem_georef, interp_ref_dem);
  
  // Must ensure these have the same projection and grid size, as then it ist
  // most likely the disparity will be computed accurately.
  if (warped_dem_georef.get_wkt() != ref_dem_georef.get_wkt())
    vw::vw_throw( vw::ArgumentErr() 
                << "The warped DEM and reference DEM have different projections."
                << "Use gdalwarp -t_srs to convert them to the same projection.\n");
    
  // The grid size is based on the transform
  auto warped_trans = warped_dem_georef.transform();
  auto ref_trans = ref_dem_georef.transform();
  
  double tol = 1e-6;
  if (std::abs(warped_trans(0, 0) - ref_trans(0, 0)) > tol ||
      std::abs(warped_trans(1, 1) - ref_trans(1, 1)) > tol)
    vw::vw_throw( vw::ArgumentErr() 
                << "The warped DEM and reference DEM have different grid sizes. Use "
                << "gdalwarp -r cubicspline to resample the warped DEM.\n");
  
  // Load the disparity
  vw::DiskImageView<Dpix> disparity(opt.warped_to_ref_disp_file);
  
  // Warped size must equal disparity size, otherwise there is a mistake
  if (interp_warped_dem.cols() != disparity.cols() || 
      interp_warped_dem.rows() != disparity.rows())
    vw::vw_throw( vw::ArgumentErr() 
                << "Error: The warped DEM and disparity sizes do not match.\n" );
  
  // TODO(oalexan1): This must be a function
  // Write the GCP file

  vw::vw_out() << "Writing: " << opt.out_gcp << "\n";
  std::ofstream ofs(opt.out_gcp.c_str());
  ofs.precision(17); // max precision
  ofs << "# WKT: " << ref_dem_georef.get_wkt() << "\n";
  ofs << "# id lat lon height_above_datum sigma_x sigma_y sigma_z image_name "
      << "pixel_x pixel_y sigma_x sigma_y, etc.\n";
  
  // Iterate over the cnet
  int num_pts = cnet.size();
  int gcp_id = 0;
  for (int ipt = 0; ipt < num_pts; ipt++) {
    vw::ba::ControlPoint & cp = cnet[ipt];
    
    // Find the pixel in the warped DEM
    vw::Vector3 xyz = cnet[ipt].position();
    vw::Vector3 llh;
    llh = warped_dem_georef.datum().cartesian_to_geodetic(xyz);
    vw::Vector2 dem_pix = warped_dem_georef.lonlat_to_pixel(vw::Vector2(llh.x(), llh.y()));
    
    // Find the corresponding pixel in the reference DEM
    Dpix disp = find_disparity(disparity, dem_pix.x(), dem_pix.y(), opt.search_len);
    if (!is_valid(disp))
      continue; 
    vw::Vector2 ref_pix = dem_pix + disp.child();
    
    // Find height at the reference pixel
    double ref_height = interp_ref_dem(ref_pix.x(), ref_pix.y());
    if (!vw::is_valid(ref_height)) 
      continue;
    
    // Find lon-lat-height at the reference pixel
    vw::Vector2 ref_lonlat = ref_dem_georef.pixel_to_lonlat(ref_pix);
    vw::Vector3 ref_llh(ref_lonlat.x(), ref_lonlat.y(), ref_height);

    // Set the sigmas to 1. The gcp file will be used with --fix-gcp-xyz,
    // so the sigmas won't matter.
    vw::Vector3 sigma(opt.gcp_sigma, opt.gcp_sigma, opt.gcp_sigma);
    vw::Vector2 pix_sigma(1, 1);
    
    // Write the id, lat, lon, height, sigmas
    ofs << gcp_id << " " << ref_llh.y() << " " << ref_llh.x() << " " << ref_llh.z() << " ";
    ofs << sigma[0] << " " << sigma[1] << " " << sigma[2] << " ";
        
    for (int im = 0; im < cp.size(); im++) {
      vw::ba::ControlMeasure & cm = cp[im];
      ofs << image_files[cm.image_id()] << " " 
          << cm.position()[0] << " " << cm.position()[1] << " "
          << pix_sigma[0] << " " << pix_sigma[1] << " ";
    }
    ofs << "\n";
    
    gcp_id++;
  }
  
  return 0;
}

int main(int argc, char * argv[]) {
  try {
    run_dem2gcp(argc, argv);    
  } ASP_STANDARD_CATCHES;
   
  return 0;      

}
