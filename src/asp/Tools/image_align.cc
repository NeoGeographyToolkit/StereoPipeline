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

/// \file image_align.cc
///
/// Tool for aligning a second image to a first image.

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Manipulation.h>

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/InterestPointMatching.h>

using namespace vw;
namespace po = boost::program_options;

struct Options: vw::cartography::GdalWriteOptions {
  std::vector<std::string> input_images;
  std::string alignment_transform, output_image, output_prefix;
  bool has_input_nodata_value, has_output_nodata_value;
  double input_nodata_value, output_nodata_value, inlier_threshold;
  int band, ip_per_image, num_ransac_iterations;
  Options(): has_input_nodata_value(false), has_output_nodata_value(false),
             input_nodata_value (std::numeric_limits<double>::quiet_NaN()),
             output_nodata_value(std::numeric_limits<double>::quiet_NaN()),
             band(0), ip_per_image(0), num_ransac_iterations(0.0), inlier_threshold(0){}
};

/// Load an input image, respecting the user parameters.
void get_input_image(std::string const& image_file, int band,
                     ImageViewRef<float> &image,
                     double &nodata) {
  // Extract the desired band
  int num_bands = get_num_channels(image_file);
  if (band > num_bands || band <= 0) 
    vw_throw(ArgumentErr() << "The desired band is out of range.");
  
  if (num_bands > 6) 
    vw_throw(ArgumentErr() << "Only images with at most 6 bands are supported.");
  
  if (num_bands == 1){
    image = DiskImageView<float>(image_file);
  }else{
    // Multi-band image. Pick the desired band.
    int channel = band - 1;  // In VW, bands start from 0, not 1.
    image = select_channel(read_channels<1, float>(image_file, channel), 0);
  }

  // Read nodata-value from disk.
  DiskImageResourceGDAL in_rsrc(image_file);
  bool has_nodata = in_rsrc.has_nodata_read();
  if (has_nodata) {
    nodata = in_rsrc.nodata_read();
    vw_out() << "Read no-data value for image " << image_file << ": " << nodata << ".\n";
  } else {
    nodata = std::numeric_limits<double>::quiet_NaN();
  }
}

/// Get a list of matched IP, looking in certain image regions.
void find_matches(std::string const& image_file1, std::string const& image_file2,
                  ImageViewRef<float> image1, ImageViewRef<float> image2,
                  double nodata1, double nodata2,
                  std::vector<ip::InterestPoint> &matched_ip1,
                  std::vector<ip::InterestPoint> &matched_ip2,
                  Options const& opt) {
  
  // Clear the outputs
  matched_ip1.clear();
  matched_ip2.clear();
  
  vw_out() << "Matching interest points between: " << image_file1 << " and "
           << image_file2 << "\n";
  
  std::string match_file = "";
  if (opt.output_prefix != "") {
    // Write a match file for debugging
    match_file = ip::match_filename(opt.output_prefix, image_file1, image_file2);
  }

  // Use ip per image rather than ip per tile as it is more intuitive that way
  int ip_per_tile = 0;
  asp::stereo_settings().ip_per_image = opt.ip_per_image;

  // Now find and match interest points in the selected regions
  asp::detect_match_ip(matched_ip1, matched_ip2, image1, image2, ip_per_tile,
                       "", "", // Do not read ip from disk
                       nodata1, nodata2, match_file);
} // End function match_ip_in_regions

template<class FunctorT>
Matrix<double> do_ransac(std::vector<Vector3> const& ransac_ip1,
                         std::vector<Vector3> const& ransac_ip2,
                         Options const& opt,
                         int min_num_output_inliers, bool reduce_num_inliers_if_no_fit,
                         // Output
                         std::vector<size_t> & indices) {
  indices.clear();
  Matrix<double> tf;
  
  try {
    vw::math::RandomSampleConsensus<FunctorT, vw::math::InterestPointErrorMetric>
      ransac(FunctorT(), vw::math::InterestPointErrorMetric(),
             opt.num_ransac_iterations, opt.inlier_threshold,
             min_num_output_inliers, reduce_num_inliers_if_no_fit);
    tf      = ransac(ransac_ip2, ransac_ip1);
    indices = ransac.inlier_indices(tf, ransac_ip2, ransac_ip1);
    return tf;
  } catch (std::exception const& e) {
    vw_throw(ArgumentErr() << e.what() << "\n"
             << "Alignment transform computation failed.\n");
  }
}
    
/// Compute a matrix transform between images, searching for IP in
///  the specified regions.
Matrix<double>
calc_alignment_transform(std::string const& image_file1,
                         std::string const& image_file2,
                         std::vector<ip::InterestPoint> &matched_ip1,
                         std::vector<ip::InterestPoint> &matched_ip2,
                         Options const& opt) {
  
  // Convert to 3D points with the third coordinate being 1, obtaining
  // homogeneous coordinates.
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);

  // RANSAC parameters.
  const int    min_num_output_inliers = ransac_ip1.size()/2;
  const bool   reduce_num_inliers_if_no_fit = true;

  Matrix<double> tf;
  std::vector<size_t> indices;
  if (opt.alignment_transform == "translation") {
    tf = do_ransac<vw::math::TranslationFittingFunctor>
      (ransac_ip1, ransac_ip2, opt, min_num_output_inliers,
       reduce_num_inliers_if_no_fit, indices);
  } else if (opt.alignment_transform == "rigid") {
    tf = do_ransac<vw::math::TranslationRotationFittingFunctorN<2>>
      (ransac_ip1, ransac_ip2, opt, min_num_output_inliers,
       reduce_num_inliers_if_no_fit, indices);
  } else if (opt.alignment_transform == "similarity") {
    tf = do_ransac<vw::math::SimilarityFittingFunctor>
      (ransac_ip1, ransac_ip2, opt, min_num_output_inliers,
       reduce_num_inliers_if_no_fit, indices);
  } else if (opt.alignment_transform == "affine") {
    tf = do_ransac<vw::math::AffineFittingFunctor>
      (ransac_ip1, ransac_ip2, opt, min_num_output_inliers,
       reduce_num_inliers_if_no_fit, indices);
  } else if (opt.alignment_transform == "homography") {
    tf = do_ransac<vw::math::HomographyFittingFunctor>
      (ransac_ip1, ransac_ip2, opt, min_num_output_inliers,
       reduce_num_inliers_if_no_fit, indices);
  } else {
    vw_throw(ArgumentErr() << "Unknown alignment transform: "
             << opt.alignment_transform << ".\n");
  }
    
  // Keeping only inliers
  std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
  for (size_t i = 0; i < indices.size(); i++) {
    inlier_ip1.push_back(matched_ip1[indices[i]]);
    inlier_ip2.push_back(matched_ip2[indices[i]]);
  } 

  vw_out() << "Found " << inlier_ip1.size() << " / " << ransac_ip1.size()
           << " inliers using RANSAC.\n";
  
  if (opt.output_prefix != "") {
    // Write a match file for debugging
    std::string clean_match_file
      = ip::clean_match_filename(ip::match_filename(opt.output_prefix, image_file1, image_file2));
    vw_out() << "Writing inlier matches after RANSAC to: " << clean_match_file << std::endl;
    ip::write_binary_match_file(clean_match_file, inlier_ip1, inlier_ip2);
  }

  vw_out() << "Alignment transform:\n" << tf << std::endl;
  
  return tf;
}

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  // Add the reverse option
  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));
  
  general_options.add_options()
    ("output-image,o", po::value(&opt.output_image)->default_value(""),
     "Specify the output image.")
    ("alignment-transform", po::value(&opt.alignment_transform)->default_value("translation"),
     "Specify the transform to use to align the second image to the first. Options: translation, "
     "rigid (translation + rotation), similarity (translation + rotation + scale), affine, homography.")
    ("band", po::value(&opt.band)->default_value(1),
     "Which band to use for interest point matching (for multi-spectral images).")
    ("ip-per-image", po::value(&opt.ip_per_image)->default_value(0),
     "How many interest points to detect in each image (default: automatic determination).")
    ("output-prefix", po::value(&opt.output_prefix)->default_value(""),
     "If set, save the interest point matches and computed transform at this prefix.")
    ("num-ransac-iterations", po::value(&opt.num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&opt.inlier_threshold)->default_value(10.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when computing interest point matches. A smaller threshold will result in fewer inliers.");
  
  po::options_description positional("");
  positional.add_options()
    ("input-images", po::value(&opt.input_images));

  po::positional_options_description positional_desc;
  positional_desc.add("input-images", -1);

  std::string usage("[options] <reference image> <source image> -o "
                    " <aligned source image>");
  
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  
  opt.has_input_nodata_value  = vm.count("input-nodata-value");
  opt.has_output_nodata_value = vm.count("output-nodata-value");

  if (opt.input_images.size() != 2)
    vw_throw(ArgumentErr() << "Expecting two input images.\n" << usage << general_options);

  if (opt.output_image.empty())
    vw_throw(ArgumentErr() << "Missing output image name.\n" << usage << general_options);

  if (opt.alignment_transform != "translation" && opt.alignment_transform != "rigid" &&
      opt.alignment_transform != "similarity" && opt.alignment_transform != "affine" &&
      opt.alignment_transform != "homography") {
    vw_throw(ArgumentErr() << "The alignment transform must be one of: translation, "
             "rigid, similarity, affine, homography.\n" << usage << general_options);    
  }

  // Create the output directory
  vw::create_out_dir(opt.output_image);

  if (opt.output_prefix != "") // for saving match files, etc.
    vw::create_out_dir(opt.output_prefix);
  
  return;  
}

int main(int argc, char *argv[]) {

  Options opt;
  
  try {

    // Find command line options
    handle_arguments(argc, argv, opt);

    std::string image_file1 = opt.input_images[0], image_file2 = opt.input_images[1];
    ImageViewRef<float> image1,  image2;
    double              nodata1, nodata2;
    get_input_image(image_file1, opt.band, image1, nodata1);
    get_input_image(image_file2, opt.band, image2, nodata2);
    
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    find_matches(image_file1, image_file2, image1, image2,  
                 nodata1, nodata2, matched_ip1, matched_ip2, opt);
    
    Matrix<double> tf = calc_alignment_transform(image_file1, image_file2,  
                                                 matched_ip1, matched_ip2, opt);
    

    // TODO(oalexan1): Save the aligned image
    
  } ASP_STANDARD_CATCHES;
  return 0;
}
