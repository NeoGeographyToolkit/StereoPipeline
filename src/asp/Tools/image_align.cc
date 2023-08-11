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
#include <vw/Image/PixelTypeInfo.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Math/Geometry.h>

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/InterestPointMatching.h>

using namespace vw;
namespace po = boost::program_options;

struct Options: vw::GdalWriteOptions {
  std::vector<std::string> input_images;
  std::string alignment_transform, output_image, output_prefix, output_data_string,
    input_transform, disparity_params, ecef_transform_type, dem1, dem2;
  bool has_input_nodata_value, has_output_nodata_value;
  double input_nodata_value, output_nodata_value, inlier_threshold;
  int ip_per_image, num_ransac_iterations, output_data_type;
  Options(): has_input_nodata_value(false), has_output_nodata_value(false),
             input_nodata_value (std::numeric_limits<double>::quiet_NaN()),
             output_nodata_value(std::numeric_limits<double>::quiet_NaN()),
             ip_per_image(0), num_ransac_iterations(0.0), inlier_threshold(0){}
};


/// Load an input image, respecting the user parameters.
void load_image(std::string const& image_file,
                ImageViewRef<double> & image, double & nodata,
                bool & has_georef, vw::cartography::GeoReference & georef) {
  
  has_georef = false; // ensure this is initialized

  image = vw::load_image_as_double(image_file);

  // Read nodata-value from disk.
  DiskImageResourceGDAL in_rsrc(image_file);
  bool has_nodata = in_rsrc.has_nodata_read();
  if (has_nodata) {
    nodata = in_rsrc.nodata_read();
    vw_out() << "Read no-data value for image " << image_file << ": " << nodata << ".\n";
  } else {
    nodata = vw::get_default_nodata(in_rsrc.channel_type());
  }
  
  has_georef = vw::cartography::read_georeference(georef, image_file);
}

/// Get a list of matched IP, looking in certain image regions.
void find_matches(std::string const& image_file1, std::string const& image_file2,
                  ImageViewRef<double> image1, ImageViewRef<double> image2,
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
  size_t number_of_jobs = 1;
  // Now find and match interest points in the selected regions
  asp::detect_match_ip(matched_ip1, matched_ip2,
                       vw::pixel_cast<float>(image1), // cast to float so it compiles
                       vw::pixel_cast<float>(image2),
                       ip_per_tile, number_of_jobs,
                       "", "", // Do not read ip from disk
                       nodata1, nodata2, match_file);

  return;
}

/// Get a list of matched IP based on an input disparity
void find_matches_from_disp(std::vector<ip::InterestPoint> &matched_ip1,
                            std::vector<ip::InterestPoint> &matched_ip2,
                            Options const& opt) {

  // Clear the outputs
  matched_ip1.clear();
  matched_ip2.clear();
  
  std::string disp_file;
  int num_samples;
  std::istringstream iss(opt.disparity_params);
  if (!(iss >> disp_file >> num_samples)) 
    vw_throw(ArgumentErr() << "Could not parse correctly the option --disparity-params.\n");

  DiskImageView<PixelMask<Vector2f>> disp(disp_file);

  if (num_samples <= 0) 
    vw_throw(ArgumentErr() << "Expecting a positive number of samples in --disparity-params.\n");

  if (disp.cols() == 0 || disp.rows() == 0) 
    vw_throw(ArgumentErr() << "Empty disparity specified in --disparity-params.\n");    
  
  // Careful here to not overflow an int32
  double num_pixels = double(disp.cols()) * double(disp.rows());
  int sample_rate = std::max(round(sqrt(num_pixels / double(num_samples))), 1.0);
  
  vw_out() << "Creating interest point matches from disparity: " << disp_file << ".\n";
  vw_out() << "Using a row and column sampling rate of: " << sample_rate << ".\n";
  
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = sample_rate / double(disp.cols());
  tpc.report_progress(0);
  
  for (int col = 0; col < disp.cols(); col += sample_rate) {
    for (int row = 0; row < disp.rows(); row += sample_rate) {
      
      vw::PixelMask<vw::Vector2f> d = disp(col, row);
      if (!is_valid(d)) 
        continue;
      
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + d.child();
      
      vw::ip::InterestPoint lip(left_pix.x(), left_pix.y());
      vw::ip::InterestPoint rip(right_pix.x(), right_pix.y());
      matched_ip1.push_back(lip); 
      matched_ip2.push_back(rip);
    }

    tpc.report_incremental_progress(inc_amount);
  }
  tpc.report_finished();
  
  return;
}

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

// Compute the ECEF transform (around planet center) given the
// interest point matches
void calc_ecef_transform(std::vector<ip::InterestPoint> const& inlier_ip1,
                         std::vector<ip::InterestPoint> const& inlier_ip2,
                         Options const& opt,
                         Matrix<double> & ecef_transform) { // output

  // Go from pixels to 3D points
  int num_matches = inlier_ip1.size();
  vw::Matrix<double> points_ref(3, num_matches), points_src(3, num_matches);

  // The value of the invalid pixel used in interpolation
  PixelMask<float> invalid_pix; invalid_pix.invalidate();
  vw::ValueEdgeExtension<PixelMask<float>> invalid_ext(invalid_pix);

  vw::cartography::GeoReference img1_geo, img2_geo, dem1_geo, dem2_geo;

  // Read first image and DEM, and set up interpolation.  Out-of-range
  // values are set to invalid.
  bool has_img1_geo = vw::cartography::read_georeference(img1_geo, opt.input_images[0]);
  if (!has_img1_geo)
    vw::vw_throw(vw::ArgumentErr() << "The first image does not have a georeference.\n");
  bool has_dem1_geo = vw::cartography::read_georeference(dem1_geo, opt.dem1);
  if (!has_dem1_geo)
    vw::vw_throw(vw::ArgumentErr() << "The first DEM does not have a georeference.\n");
  double dem1_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(opt.dem1, dem1_nodata);
  DiskImageView<float> dem1(opt.dem1);
  auto interp_dem1 = interpolate(create_mask(dem1, dem1_nodata),
                                 BilinearInterpolation(), invalid_ext);

  // Repeat for 2nd DEM and image
  bool has_img2_geo = vw::cartography::read_georeference(img2_geo, opt.input_images[1]);
  if (!has_img2_geo)
    vw::vw_throw(vw::ArgumentErr() << "The second image does not have a georeference.\n");
  bool has_dem2_geo = vw::cartography::read_georeference(dem2_geo, opt.dem2);
  if (!has_dem2_geo)
    vw::vw_throw(vw::ArgumentErr() << "The second DEM does not have a georeference.\n");
  double dem2_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(opt.dem2, dem2_nodata);
  DiskImageView<float> dem2(opt.dem2);
  auto interp_dem2 = interpolate(create_mask(dem2, dem2_nodata),
                                 BilinearInterpolation(), invalid_ext);

  // Find the 3D coordinates
  for (size_t ip_it = 0; ip_it < inlier_ip1.size(); ip_it++) {

    // ECEF point for first image ip
    vw::Vector2 pix1(inlier_ip1[ip_it].x, inlier_ip1[ip_it].y);
    vw::Vector2 img1_lonlat = img1_geo.pixel_to_lonlat(pix1);
    vw::Vector2 dem1_pix = dem1_geo.lonlat_to_pixel(img1_lonlat);
    PixelMask<float> dem1_val = interp_dem1(dem1_pix.x(), dem1_pix.y());
    if (!is_valid(dem1_val)) 
      continue;
    vw::Vector3 xyz1 = dem1_geo.datum().geodetic_to_cartesian
      (vw::Vector3(img1_lonlat.x(), img1_lonlat.y(), dem1_val.child()));
    
    // ECEF point for second image ip
    vw::Vector2 pix2(inlier_ip2[ip_it].x, inlier_ip2[ip_it].y);
    vw::Vector2 img2_lonlat = img2_geo.pixel_to_lonlat(pix2);
    vw::Vector2 dem2_pix = dem2_geo.lonlat_to_pixel(img2_lonlat);
    PixelMask<float> dem2_val = interp_dem2(dem2_pix.x(), dem2_pix.y());
    if (!is_valid(dem2_val)) 
      continue;
    vw::Vector3 xyz2 = dem2_geo.datum().geodetic_to_cartesian
      (vw::Vector3(img2_lonlat.x(), img2_lonlat.y(), dem2_val.child()));
    
    // Store in matrices
    typedef vw::math::MatrixCol<vw::Matrix<double>> ColView;
    ColView col_ref(points_ref, ip_it); 
    ColView col_src(points_src, ip_it);
    col_ref = xyz1;
    col_src = xyz2;
  }

  // Find the 3D transform from second to first set of ECEF points
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  bool filter_outliers = true;
  Vector2 ransac_params;
  ransac_params[0] = opt.num_ransac_iterations;
  ransac_params[1] = 1.0; // factor; not the same as opt.inlier_threshold
  vw::math::find_3D_transform(points_src, points_ref,
                              rotation, translation, scale,
                              opt.ecef_transform_type,
                              filter_outliers,
                              ransac_params);

  // Convert to pc_align transform format
  ecef_transform = identity_matrix(4);
  submatrix(ecef_transform, 0, 0, 3, 3) = rotation * scale;
  for (int row = 0; row < 3; row++)
    ecef_transform(row, 3) = translation[row];
}

/// Compute a matrix transform between images, searching for IP in
///  the specified regions.
Matrix<double>
calc_alignment_transform(std::string const& image_file1,
                         std::string const& image_file2,
                         std::vector<ip::InterestPoint> &matched_ip1,
                         std::vector<ip::InterestPoint> &matched_ip2,
                         Options const& opt,
                         Matrix<double> & ecef_transform) { // potential output
  
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

  if (opt.ecef_transform_type != "") 
    calc_ecef_transform(inlier_ip1, inlier_ip2, opt, ecef_transform);
  
  return tf;
}

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add(vw::GdalWriteOptionsDescription(opt));
  
  general_options.add_options()
    ("output-image,o", po::value(&opt.output_image)->default_value(""),
     "Specify the output image.")
    ("alignment-transform", po::value(&opt.alignment_transform)->default_value("translation"),
     "Specify the transform to use to align the second image to the first. Options: translation, "
     "rigid (translation + rotation), similarity (translation + rotation + scale), affine, homography.")
    ("ip-per-image", po::value(&opt.ip_per_image)->default_value(0),
     "How many interest points to detect in each image (default: automatic determination).")
    ("output-prefix", po::value(&opt.output_prefix)->default_value(""),
     "If set, save the interest point matches and computed transform using this prefix.")
    ("output-data-type,d",  po::value(&opt.output_data_string)->default_value("float32"),
     "The data type of the output file. Options: uint8, uint16, uint32, int16, int32, "
     "float32, float64. The values are clamped (and also rounded for integer types) to avoid "
     "overflow.")
    ("num-ransac-iterations", po::value(&opt.num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&opt.inlier_threshold)->default_value(5.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when computing interest point matches. A smaller threshold will result in fewer inliers.")
    ("input-transform", po::value(&opt.input_transform)->default_value(""),
     "Instead of computing an alignment transform, read and apply the one from this file. Must be stored as a 3x3 matrix.")
    ("ecef-transform-type", po::value(&opt.ecef_transform_type)->default_value(""), "Save the ECEF transform corresponding to the image alignment transform to <output prefix>-ecef-transform.txt. The type can be: 'translation', 'rigid' (rotation + translation), or 'similarity' (rotation + translation + scale).")
    ("dem1", po::value(&opt.dem1)->default_value(""), "The DEM associated with the first image. To be used with --ecef-transform-type.")
    ("dem2", po::value(&opt.dem2)->default_value(""), "The DEM associated with the second image. To be used with --ecef-transform-type.")
    ("disparity-params", po::value(&opt.disparity_params)->default_value(""),
     "Find the alignment transform by using, instead of interest points, a disparity, such as produced by 'parallel_stereo --correlator-mode'. Specify as a string in quotes, in the format: 'disparity.tif num_samples'.");
    
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

  if (opt.ecef_transform_type != "") {
    if (opt.ecef_transform_type != "translation" && opt.ecef_transform_type != "rigid" &&
        opt.ecef_transform_type != "similarity") 
      vw_throw(ArgumentErr() << "The value of --ecef-transform-type must be one of: translation, "
               "rigid, or similarity.\n");    
    if (opt.dem1 == "" || opt.dem2 == "") 
      vw::vw_throw(vw::ArgumentErr() << "When using the option --ecef-transform-type, "
                   "the options --dem1 and --dem2 must be set.\n");
    if (opt.output_prefix == "") 
      vw::vw_throw(vw::ArgumentErr() << "When using the option --ecef-transform-type, "
                   "the option --output-prefix must be set.\n");
  }
    
  // Determining the format of the second input image
  boost::shared_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResourcePtr(opt.input_images[1]));
  ChannelTypeEnum input_data_type = rsrc->channel_type();

  // By default, use the same type on output as on input
  opt.output_data_type = input_data_type;
  if      (opt.output_data_string == "uint8"  ) opt.output_data_type = VW_CHANNEL_UINT8;
  else if (opt.output_data_string == "uint16" ) opt.output_data_type = VW_CHANNEL_UINT16;
  else if (opt.output_data_string == "uint32" ) opt.output_data_type = VW_CHANNEL_UINT32;
  // GDAL does not support int8
  //else if (opt.output_data_string == "int8"   ) opt.output_data_type = VW_CHANNEL_INT8;
  else if (opt.output_data_string == "int16"  ) opt.output_data_type = VW_CHANNEL_INT16;
  else if (opt.output_data_string == "int32"  ) opt.output_data_type = VW_CHANNEL_INT32;
  else if (opt.output_data_string == "float32") opt.output_data_type = VW_CHANNEL_FLOAT32;
  else if (opt.output_data_string == "float64") opt.output_data_type = VW_CHANNEL_FLOAT64;
  else
    vw_throw(ArgumentErr() << "Invalid value for the output type: " << opt.output_data_string
             << ".\n");
  
  // Create the output directory
  vw::create_out_dir(opt.output_image);

  if (opt.output_prefix != "") // for saving match files, etc.
    vw::create_out_dir(opt.output_prefix);
  
  return;  
}

void save_output(ImageViewRef<PixelMask<double>> aligned_image2,
                 bool has_nodata2, double nodata2,
                 bool has_georef2, vw::cartography::GeoReference const& georef2,
                 Options const& opt) {
  
  vw_out() << "Writing: " << opt.output_image << "\n";
  // Note that output int types get rounded before being clamped and
  // cast, but not output float or double types.
  switch (opt.output_data_type) {
  case VW_CHANNEL_UINT8:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampRoundAndCastToInt<vw::uint8, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_UINT16:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampRoundAndCastToInt<vw::uint16, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_UINT32:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampRoundAndCastToInt<vw::uint32, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
    // GDAL does not support int8
    //case VW_CHANNEL_INT8:
    //block_write_gdal_image(opt.output_image,
    //                       per_pixel_filter(apply_mask(aligned_image2, nodata2),
    //                                       vw::ClampRoundAndCastToInt<vw::int8, double>()),
    //                      has_georef2, georef2, has_nodata2, nodata2, opt,
    //                      TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_INT16:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampRoundAndCastToInt<vw::int16, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_INT32:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampRoundAndCastToInt<vw::int32, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_FLOAT32:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampAndCast<vw::float32, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
    break;
  case VW_CHANNEL_FLOAT64:
    block_write_gdal_image(opt.output_image,
                           per_pixel_filter(apply_mask(aligned_image2, nodata2),
                                            vw::ClampAndCast<vw::float64, double>()),
                           has_georef2, georef2, has_nodata2, nodata2, opt,
                           TerminalProgressCallback("asp","\t  Aligned image:  "));
  default:
    vw_throw(ArgumentErr() << "Invalid value for the output pixel format.\n");

  };
}

int main(int argc, char *argv[]) {

  Options opt;
  
  try {

    // Process command line options
    handle_arguments(argc, argv, opt);

    std::string image_file1 = opt.input_images[0], image_file2 = opt.input_images[1];
    ImageViewRef<double> image1,  image2;
    double              nodata1, nodata2;
    bool has_georef1 = false, has_georef2 = false;
    vw::cartography::GeoReference georef1, georef2;
    load_image(image_file1, image1, nodata1, has_georef1, georef1);
    load_image(image_file2, image2, nodata2, has_georef2, georef2);

    Matrix<double> tf, ecef_transform;
    if (opt.input_transform.empty()) {
      std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
      if (opt.disparity_params == "")
        find_matches(image_file1, image_file2, image1, image2,  
                     nodata1, nodata2, matched_ip1, matched_ip2, opt);
      else
        find_matches_from_disp(matched_ip1, matched_ip2, opt);
      
      tf = calc_alignment_transform(image_file1, image_file2,  
                                    matched_ip1, matched_ip2, opt, ecef_transform);
    } else {
      vw_out() << "Reading the alignment transform from: " << opt.input_transform << "\n";
      read_matrix_as_txt(opt.input_transform, tf);
    }
    
    if (opt.output_prefix != "") {
      std::string transform_file = opt.output_prefix + "-transform.txt";
      vw_out() << "Writing the transform to: " << transform_file << std::endl;
      write_matrix_as_txt(transform_file, tf);
    }
    
    if (opt.output_prefix != "" && opt.ecef_transform_type != "") {
      std::string ecef_transform_file = opt.output_prefix + "-ecef-transform.txt";
      vw_out() << "Writing the ECEF transform to: " << ecef_transform_file << std::endl;
      write_matrix_as_txt(ecef_transform_file, ecef_transform);
    }
    
    // Any transforms supported by this tool fit in a homography transform object
    vw::HomographyTransform T(tf);

    // Find the domain of the transformed image
    BBox2i trans_box = T.forward_bbox(vw::bounding_box(image2));

    // Adjust the box to consistent with what vw::transform expects below
    trans_box.min() = vw::Vector2(0, 0);

    PixelMask<double> nodata_mask = PixelMask<double>(); // invalid value for a PixelMask
    ImageViewRef<PixelMask<double>> aligned_image2 =
      vw::transform(create_mask(image2, nodata2), HomographyTransform(tf),
                    trans_box.width(), trans_box.height(),
                    ValueEdgeExtension<PixelMask<double>>(nodata_mask),
                    BilinearInterpolation());

    // Almost always the alignment will result in pixels with no data
    bool has_nodata2 = true;

    if (has_georef1) {
      // Borrow the georef from the first image, since we are in its coordinates
      has_georef2 = has_georef1;
      georef2 = georef1;
    } else{
      // Write no georef
      has_georef1 = false;
      has_georef2 = false;
    }
      
    save_output(aligned_image2, has_nodata2, nodata2, has_georef2, georef2, opt);
    
  } ASP_STANDARD_CATCHES;
  return 0;
}
