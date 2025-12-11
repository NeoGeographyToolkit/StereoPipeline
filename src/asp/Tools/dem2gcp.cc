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

// \file dem2gcp.cc
// See the documentation on readthedocs.

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Math/RandomSet.h>
#include <vw/Image/Filter.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/FileIO/FileUtils.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef vw::PixelMask<vw::Vector2f> DpixT;

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

// Invalidate disparity whose norm is more than this
class FilterDisp: public vw::ReturnFixedType<DpixT> {
public:

  FilterDisp(double max_disp): m_max_disp(max_disp) {}
  
  DpixT operator()(DpixT const& disp) const {
    if (is_valid(disp) && vw::math::norm_2(disp.child()) > m_max_disp) {
      // Invalidate the disparity
      DpixT invalid_disp = disp;
      invalid_disp.invalidate();
      return invalid_disp;
    }
    return disp;
  }
  double m_max_disp;
};

// Find the interpolated disparity value at a given pixel. If not valid, look in
// a small neighborhood. That should not be overused. 
DpixT find_disparity(vw::ImageViewRef<DpixT> disparity,
                     vw::ImageViewRef<DpixT> interp_disp,
                     double x, double y, int max_l) {
  if (x < 0 || y < 0 || x >= disparity.cols() || y >= disparity.rows())
    return DpixT();

  // Try to find the interpolated disparity
  DpixT idisp = interp_disp(x, y);
  
  if (is_valid(idisp))
    return idisp;
  
  // Otherwise try without interpolation. This casts x and y to integers.
  if (is_valid(disparity(x, y)))
    return disparity(x, y);  

  // Otherwise search in the neighborhood. The default value of the neighborhood
  // is 0. This is is likely a desperate measure when the disparity has very few
  // good values.
  for (int l = 1; l <= max_l; l++) {

    // Will keep the pixel closest to the center
    double max_dist = std::numeric_limits<double>::max();
    vw::Vector2 best_pix;
    bool found = false;

    std::vector<vw::Vector2> pixels;
    enumerate_boundary_pixels(int(x), int(y), l, pixels);
    
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
    
    if (found)
      return disparity(best_pix.x(), best_pix.y());

  } // end loop over ever-growing squares
  
  // Give up
  return DpixT();
}

// A structure for storing a GCP and the index in the cnet
struct Gcp {
  int cnet_pt_id;
  vw::Vector3 llh;
  vw::Vector3 sigma;
};

void writeGcp(vw::cartography::GeoReference const& ref_dem_georef,
              vw::ba::ControlNetwork const& cnet,
              vw::ImageViewRef<DpixT> const& disparity,
              vw::ImageViewRef<vw::PixelMask<double>> const& interp_ref_dem,
              vw::cartography::GeoReference const& warped_dem_georef,
              std::vector<std::string> const& image_files,
              double gcp_sigma, int search_len, 
              int max_num_gcp,
              vw::ImageViewRef<vw::PixelMask<float>> const& gcp_sigma_image,
              vw::cartography::GeoReference          const& gcp_sigma_image_georef,

              std::string const& out_gcp) {

  int num_pts = cnet.size();

  // Collect all GCP, in case we need to reduce their number
  std::vector<Gcp> gcp_vec;
  gcp_vec.reserve(num_pts);
  gcp_vec.clear();
  
  // Form the interpolated disparity
  DpixT nodata_pix;
  nodata_pix.invalidate();
  vw::ImageViewRef<DpixT> interp_disp
    = interpolate(disparity, vw::BilinearInterpolation(), 
                  vw::ValueEdgeExtension<DpixT>(nodata_pix));

  bool have_gcp_sigma_image = (gcp_sigma_image.cols() > 0 &&
                               gcp_sigma_image.rows() > 0);
  
  // Iterate over the cnet. Keep track of progress.
  vw::TerminalProgressCallback tpc("asp", "Producing GCP --> ");
  int progress_steps = 100; 
  double inc_amount = double(progress_steps) / std::max(num_pts, 1);
  tpc.report_progress(0);
  for (int ipt = 0; ipt < num_pts; ipt++) {
    
    // Report progress once this many points are processed
    if (ipt % progress_steps == 0)
      tpc.report_incremental_progress(inc_amount);
    
    vw::ba::ControlPoint const& cp = cnet[ipt]; // alias
    
    // Find the pixel in the warped DEM
    vw::Vector3 xyz = cnet[ipt].position();
    vw::Vector3 llh;
    llh = warped_dem_georef.datum().cartesian_to_geodetic(xyz);
    vw::Vector2 dem_pix = warped_dem_georef.lonlat_to_pixel(vw::Vector2(llh.x(), llh.y()));
    
    // Find the corresponding pixel in the reference DEM
    DpixT disp = find_disparity(disparity, interp_disp, dem_pix.x(), dem_pix.y(), search_len);
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

    // The case when the GCP sigma comes from a file
    if (have_gcp_sigma_image) {
      vw::PixelMask<float> img_gcp_sigma 
        = vw::cartography::closestPixelVal(gcp_sigma_image, gcp_sigma_image_georef, xyz);
        
      // Flag bad gcp_sigmas as outliers
      if (!is_valid(img_gcp_sigma) || std::isnan(img_gcp_sigma.child()) || 
          img_gcp_sigma.child() <= 0.0)
        continue;
       
      // Use this sigma
      gcp_sigma = img_gcp_sigma.child();
    }

    // Use given gcp sigma for ground points, and use 1 for pixel sigma.
    vw::Vector3 sigma(gcp_sigma, gcp_sigma, gcp_sigma);

    Gcp gcp;
    gcp.cnet_pt_id = ipt;
    gcp.llh = ref_llh;
    gcp.sigma = sigma;
    gcp_vec.push_back(gcp);
  }
  tpc.report_finished();

  // See if to reduce the number of GCP
  if (max_num_gcp > 0 && (int)gcp_vec.size() > max_num_gcp) {
    vw::vw_out() << "Reducing the number of GCP from "
                << gcp_vec.size() << " to " << max_num_gcp << ".\n";
    std::vector<int> indices;
    vw::math::pick_random_indices_in_range(gcp_vec.size(), max_num_gcp, indices);
    std::vector<Gcp> out_gcp;
    for (size_t i = 0; i < indices.size(); i++)
      out_gcp.push_back(gcp_vec[indices[i]]);
    // Swap the two
    gcp_vec.swap(out_gcp);
  }
  
  // Write to disk
  vw::vw_out() << "Writing: " << out_gcp << "\n";
  std::ofstream ofs(out_gcp.c_str());
  ofs.precision(17); // full precision
  ofs << "# WKT: " << ref_dem_georef.get_wkt() << "\n";
  ofs << "# id lat lon height_above_datum sigma_x sigma_y sigma_z image_name "
      << "pixel_x pixel_y sigma_x sigma_y, etc.\n";

  // Iterate over the gcp_vec
  for (size_t gcp_id = 0; gcp_id < gcp_vec.size(); gcp_id++) {

    auto const& gcp   = gcp_vec[gcp_id]; // alias
    auto const& cp    = cnet[gcp.cnet_pt_id]; // alias
    auto const& llh   = gcp.llh; // alias
    auto const& sigma = gcp.sigma; // alias
    vw::Vector2 pix_sigma(1, 1);

    // Write the id, lat, lon, height, sigmas
    ofs << gcp_id << " " << llh.y() << " " << llh.x() << " " << llh.z() << " "
        << sigma[0] << " " << sigma[1] << " " << sigma[2] << " ";
        
    for (int im = 0; im < cp.size(); im++) {
      auto const& cm = cp[im]; // measure
      ofs << image_files[cm.image_id()] << " " 
          << cm.position()[0] << " " << cm.position()[1] << " "
          << pix_sigma[0] << " " << pix_sigma[1] << " ";
    }
    ofs << "\n";
  }
  
  ofs.close();
}

// Put the variables below in a struct
struct Options: public vw::GdalWriteOptions {
  std::string warped_dem_file, ref_dem_file, warped_to_ref_disp_file, left_img, right_img,
    left_cam, right_cam,  match_file, out_gcp, gcp_sigma_image, image_list, camera_list,
    match_files_prefix, clean_match_files_prefix;
  int search_len, max_num_gcp;
  double gcp_sigma, max_disp;
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
    ("gcp-sigma", po::value(&opt.gcp_sigma)->default_value(1.0),
     "The sigma to use for the GCP points. A smaller value will give to GCP more weight. "
     "See also --gcp-sigma-image.")
    ("max-num-gcp", po::value(&opt.max_num_gcp)->default_value(-1),
     "The maximum number of GCP to write. If negative, all GCP are written. If more than "
     "this number, a random subset will be picked. The same subset will be selected if the "
     "program is called again.")
    ("output-gcp", po::value(&opt.out_gcp), 
     "The produced GPP file with ground coordinates from the reference DEM.")
    ("max-disp", po::value(&opt.max_disp)->default_value(-1), 
     "If positive, flag a disparity whose norm is larger than this as erroneous and do not "
     "use it for creating GCP.")
    ("gcp-sigma-image", po::value(&opt.gcp_sigma_image)->default_value(""),
     "Given a georeferenced image with float values, for each GCP find its location in "
     "this image and closest pixel value. Let the GCP sigma be that value. Skip GCP that "
     "result in values that are are no-data or are not positive. This overrides --gcp-sigma.")
    ("search-len", po::value(&opt.search_len)->default_value(0), 
     "How many DEM pixels to search around a given interest point to find a valid DEM "
     "disparity (pick the closest). This may help with a spotty disparity but should not "
     "be overused.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images. This can be used with more than two images.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
      "A file containing the list of cameras. This can be used with more than two images.")
    ("match-files-prefix",  po::value(&opt.match_files_prefix)->default_value(""),
     "Use the match files with this prefix.")
    ("clean-match-files-prefix",  po::value(&opt.clean_match_files_prefix)->default_value(""),
     "Use as input *-clean.match files with this prefix.")
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

  //  Only one of --match-file, --match-files-prefix, or
  //  --clean-match-files-prefix can be used
  int num = int(!opt.match_file.empty()) +
            int(!opt.match_files_prefix.empty()) +
            int(!opt.clean_match_files_prefix.empty());
  if (num != 1)
    vw::vw_throw(vw::ArgumentErr()
                 << "Exactly one of --match-file, --match-files-prefix, or "
                 << "--clean-match-files-prefix must be specified.\n");
  
  //  Can have either left and right image, or image-list
  if ((!opt.left_img.empty() || !opt.right_img.empty()) &&
      !opt.image_list.empty())
    vw::vw_throw(vw::ArgumentErr()
                 << "Either specify both --left-image and --right-image, "
                 << "or --image-list, but not both options.\n");
  // Same for left and right camera
  if ((!opt.left_cam.empty() || !opt.right_cam.empty()) &&
      !opt.camera_list.empty())
    vw::vw_throw(vw::ArgumentErr()
                 << "Either specify both --left-camera and --right-camera, "
                 << "or --camera-list, but not both options.\n");
    
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_gcp);
}

int run_dem2gcp(int argc, char * argv[]) {
  
  Options opt;
  
  // Parse and validate the input options  
  handle_arguments(argc, argv, opt);
  
  // Prepare the images  
  std::vector<std::string> image_files;
  // See if to read from image list
  if (!opt.image_list.empty()) {
    asp::read_list(opt.image_list, image_files);
  } else {
    image_files.push_back(opt.left_img);
    image_files.push_back(opt.right_img);
  }
  // Prepare camera lists
  std::vector<std::string> camera_files;
  // See if to read from camera list
  if (!opt.camera_list.empty()) {
    asp::read_list(opt.camera_list, camera_files);
  } else {
    camera_files.push_back(opt.left_cam);
    camera_files.push_back(opt.right_cam);
  }

  // Load the cameras  
  std::string stereo_session, out_prefix;
  std::vector<vw::CamPtr> camera_models;
  bool approximate_pinhole_intrinsics = false, single_threaded_cameras = false;
  asp::load_cameras(image_files, camera_files, out_prefix, opt,
                      approximate_pinhole_intrinsics,
                      // Outputs
                      stereo_session, single_threaded_cameras, camera_models);
    
  // Load the control network. Consider the case of one match files or a prefix
  // pointing to many match files.  
  vw::ba::ControlNetwork cnet("asp");
  bool triangulate_control_points = true;
  std::map<std::pair<int, int>, std::string> match_files;
  if (!opt.match_file.empty()) {
    match_files[std::make_pair(0, 1)] = opt.match_file;
  } else {
    // Load all match files with no constraints
    int overlap_limit = image_files.size();
    bool match_first_to_last = true;
    asp::findMatchFiles(overlap_limit, match_first_to_last,
                        image_files, opt.clean_match_files_prefix,
                        opt.match_files_prefix, out_prefix,
                        // Outputs
                        match_files);
  }
  
  int min_matches = 0; // Not used as we are just loading
  double min_triangulation_angle = 1e-10;
  double forced_triangulation_distance = -1.0;
  int max_pairwise_matches = 1e6; // Not used as we are just loading
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
  vw::ImageViewRef<DpixT> disparity = vw::DiskImageView<DpixT>(opt.warped_to_ref_disp_file);
  
  if (opt.max_disp > 0) {
    vw::vw_out() << "Filtering the disparity to remove values larger than "
                 << opt.max_disp << ".\n";
    disparity = vw::per_pixel_filter(disparity, FilterDisp(opt.max_disp));
  }
  
  // Warped size must equal disparity size, otherwise there is a mistake
  if (interp_warped_dem.cols() != disparity.cols() || 
      interp_warped_dem.rows() != disparity.rows())
    vw::vw_throw( vw::ArgumentErr() 
                << "Error: The warped DEM and disparity sizes do not match.\n" );
  
  // If to use a gcp_sigma image
  bool have_gcp_sigma_image = (!opt.gcp_sigma_image.empty());
  vw::ImageViewRef<vw::PixelMask<float>> gcp_sigma_image;
  float gcp_sigma_image_nodata = -std::numeric_limits<float>::max();
  vw::cartography::GeoReference gcp_sigma_image_georef;
  if (have_gcp_sigma_image) {
    vw::cartography::readGeorefImage(opt.gcp_sigma_image,
      gcp_sigma_image_nodata, gcp_sigma_image_georef, gcp_sigma_image);
    vw::vw_out() << "Using GCP sigma image: " << opt.gcp_sigma_image << "\n";
  }
  
  writeGcp(ref_dem_georef, cnet, disparity,
           interp_ref_dem, warped_dem_georef, image_files, 
           opt.gcp_sigma, opt.search_len, opt.max_num_gcp,
           gcp_sigma_image, gcp_sigma_image_georef,
           opt.out_gcp);
  
  return 0;
}

int main(int argc, char * argv[]) {
  try {
    run_dem2gcp(argc, argv);    
  } ASP_STANDARD_CATCHES;
   
  return 0;      

}
