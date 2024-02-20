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

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Math/Geometry.h>

using namespace vw;
namespace po = boost::program_options;

namespace asp {
  
struct Options: vw::GdalWriteOptions {
  std::string camera_image, ortho_image, dem, output_gcp, output_prefix, match_file;
  double inlier_threshold;
  int ip_per_image, num_ransac_iterations;
  Options(): ip_per_image(0), num_ransac_iterations(0.0), inlier_threshold(0){}
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
    ("ip-per-image", po::value(&opt.ip_per_image)->default_value(20000),
     "How many interest points to detect in each image (the resulting number of "
      "matches will be much less).")
    ("num-ransac-iterations", po::value(&opt.num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&opt.inlier_threshold)->default_value(0.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when "
     "computing interest point matches. A smaller threshold will result in fewer "
     "inliers. The default is 10% of the image diagonal.")
    ("output-prefix", po::value(&opt.output_prefix)->default_value(""),
     "If set, save the interest point matches using this prefix (for inspection).")
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
  
  // Create the output directory
  vw::create_out_dir(opt.output_gcp);
  
  return;  
}

// Filter outliers with RANSAC
void do_ransac(Options const& opt, 
               std::vector<ip::InterestPoint> & ip1, 
               std::vector<ip::InterestPoint> & ip2) {
  
  // Convert to 3D points with the third coordinate being 1, obtaining
  // homogeneous coordinates.
  std::vector<vw::Vector3> ip1_vec3 = vw::ip::iplist_to_vectorlist(ip1);
  std::vector<vw::Vector3> ip2_vec3 = vw::ip::iplist_to_vectorlist(ip2);
  
  // RANSAC parameters
  int  min_num_output_inliers = ip1.size()/2;
  bool reduce_num_inliers_if_no_fit = true;
  vw::vw_out() << "Using RANSAC with " << opt.num_ransac_iterations << " iterations.\n";

  vw::Matrix<double> tf;
  std::vector<size_t> indices;
  try {
    vw::math::RandomSampleConsensus<vw::math::HomographyFittingFunctor, 
                                    vw::math::InterestPointErrorMetric>
      ransac(vw::math::HomographyFittingFunctor(), 
             vw::math::InterestPointErrorMetric(),
             opt.num_ransac_iterations, opt.inlier_threshold,
             min_num_output_inliers, reduce_num_inliers_if_no_fit);
    tf = ransac(ip2_vec3, ip1_vec3);
    indices = ransac.inlier_indices(tf, ip2_vec3, ip1_vec3);
  } catch (std::exception const& e) {
    vw_throw(ArgumentErr() << e.what() << "\n"
             << "RANSAC failed.\n");
  }
  vw::vw_out() << "Homography transform: " << tf << std::endl;
  
  // Keeping only inliers. Overwrite them in place.
  for (size_t i = 0; i < indices.size(); i++) {
    ip1[i] = ip1[indices[i]];
    ip2[i] = ip2[indices[i]];
  }
  ip1.resize(indices.size());
  ip2.resize(indices.size());

  vw_out() << "Found " << ip1.size() << " inlier matches using RANSAC.\n";
}

/// Get a list of matched IP between two images
void find_matches(Options & opt,
                  std::string const& camera_image_name, std::string const& ortho_image_name,
                  ImageViewRef<double> camera_image, ImageViewRef<double> ortho_image,
                  double nodata1, double nodata2,
                  std::vector<vw::ip::InterestPoint> & matched_ip1,
                  std::vector<vw::ip::InterestPoint> & matched_ip2) {
  
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
  vw::vw_out() << "Searching for " << opt.ip_per_image 
                << " interest points in each image.\n";
  
  vw_out() << "Matching interest points between: " << camera_image_name << " and "
           << ortho_image_name << "\n";

  // Now find and match interest points. Use ip per image rather than ip per
  // tile as it is more intuitive that way
  int ip_per_tile = 0;
  asp::stereo_settings().ip_per_image = opt.ip_per_image;
  size_t number_of_jobs = 1;
  std::string match_file = ""; // so we do not yet write to disk
  asp::detect_match_ip(matched_ip1, matched_ip2,
                       vw::pixel_cast<float>(camera_image), // cast to float so it compiles
                       vw::pixel_cast<float>(ortho_image),
                       ip_per_tile, number_of_jobs,
                       "", "", // Do not read ip from disk
                       nodata1, nodata2, match_file);
  
  // Filter outliers with RANSAC
  do_ransac(opt, matched_ip1, matched_ip2);

  if (opt.output_prefix != "") {
    // Write a match file for debugging
    match_file = ip::match_filename(opt.output_prefix, camera_image_name, ortho_image_name);
    vw_out() << "Writing inlier matches to: " << match_file << std::endl;
    ip::write_binary_match_file(match_file, matched_ip1, matched_ip2);
  }
      
  return;
}

void gcp_gen(Options & opt) {

  // Load the images, georef, and nodata values
  ImageViewRef<double> camera_image, ortho_image, dem;
  double camera_nodata, ortho_nodata, dem_nodata; // will be initialized next
  bool has_georef_camera = false, has_georef_ortho = false, has_georef_dem = false;
  vw::cartography::GeoReference georef_camera, georef_ortho, georef_dem;
  asp::load_image(opt.camera_image, camera_image, camera_nodata, 
                  has_georef_camera, georef_camera);  
  asp::load_image(opt.ortho_image, ortho_image, ortho_nodata, 
                  has_georef_ortho, georef_ortho);
  asp::load_image(opt.dem, dem, dem_nodata, 
                  has_georef_dem, georef_dem);

  vw::vw_out() << "Camera image: " << opt.camera_image << "\n";
  vw::vw_out() << "Ortho image: " << opt.ortho_image << "\n";
  vw::vw_out() << "DEM: " << opt.dem << "\n";
  
  // The ortho and DEM must have a geo reference
  if (!has_georef_ortho)
    vw_throw(ArgumentErr() << "The ortho image must have a georeference.\n");
  if (!has_georef_dem)
    vw_throw(ArgumentErr() << "The DEM must have a georeference.\n");
    
  std::vector<vw::ip::InterestPoint> ip1, ip2;
  if (opt.match_file == "") {
    find_matches(opt, opt.camera_image, opt.ortho_image, camera_image, ortho_image,
                 camera_nodata, ortho_nodata, ip1, ip2);
  } else {
    vw::vw_out() << "Reading matches from: " << opt.match_file << "\n";
    vw::ip::read_binary_match_file(opt.match_file, ip1, ip2);
  }
  
  // Populate from two vectors of matched interest points
  asp::MatchList matchList;
  matchList.populateFromIpPair(ip1, ip2);

  // Write the GCP file
  std::vector<std::string> image_files = {opt.camera_image, opt.ortho_image};
  asp::writeGCP(image_files, opt.output_gcp, opt.dem, matchList);
  
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
