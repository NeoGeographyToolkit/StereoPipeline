// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionIsis.cc
///

// Vision Workbench
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography.h>
// Stereo Pipeline
#include <asp/Sessions/ISIS/StereoSessionIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/Sessions/ISIS/PhotometricOutlier.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/shared_ptr.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace asp;

// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
void remove_duplicates(std::vector<ip::InterestPoint> &ip1,
                       std::vector<ip::InterestPoint> &ip2) {
  std::vector<ip::InterestPoint> new_ip1, new_ip2;

  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j &&
          ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
           (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)) ) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }

  ip1 = new_ip1;
  ip2 = new_ip2;
}

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

vw::math::Matrix<double>
StereoSessionIsis::determine_image_alignment(std::string const& input_file1,
                                             std::string const& input_file2, float lo, float hi) {

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  std::string match_filename = fs::path( input_file1 ).replace_extension("").string() + "__" + fs::path( input_file2 ).stem() + ".match";
  if ( fs::exists( match_filename ) ) {
    // Is there a match file linking these 2 image?

    vw_out() << "\t--> Found cached interest point match file: "
             << match_filename << "\n";
    read_binary_match_file( match_filename,
                            matched_ip1, matched_ip2 );

    // Fitting a matrix immediately (no RANSAC)
    math::HomographyFittingFunctor fitting;
    remove_duplicates( matched_ip1, matched_ip2 );
    std::vector<Vector3> list1 = iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> list2 = iplist_to_vectorlist(matched_ip2);
    math::AffineFittingFunctor aff_fit;
    Matrix<double> seed = aff_fit( list2, list1 );
    Matrix<double> align = fitting( list2, list1, seed );  // LMA optimization second
    vw_out() << "\tFit = " << align << std::endl;
    return align;

  } else {

    // Next best thing.. VWIPs?
    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    std::string ip1_filename = fs::path( input_file1 ).replace_extension("vwip").string();
    std::string ip2_filename = fs::path( input_file2 ).replace_extension("vwip").string();
    if ( fs::exists( ip1_filename ) &&
         fs::exists( ip2_filename ) ) {
      // Found VWIPs already done before
      vw_out() << "\t--> Found cached interest point files: "
                << ip1_filename << "\n"
                << "\t                                       "
                << ip2_filename << "\n";
      ip1_copy = ip::read_binary_ip_file( ip1_filename );
      ip2_copy = ip::read_binary_ip_file( ip2_filename );

    } else {
      // Worst case, no interest point operations have been performed before
      vw_out() << "\t--> Locating Interest Points\n";
      ip::InterestPointList ip1, ip2;
      DiskImageView<PixelGray<float> > left_disk_image(input_file1);
      DiskImageView<PixelGray<float> > right_disk_image(input_file2);
      ImageViewRef<PixelGray<float> > left_image =
        normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
      ImageViewRef<PixelGray<float> > right_image =
        normalize(remove_isis_special_pixels(right_disk_image, lo), lo, hi, 0, 1.0);

      // Interest Point module detector code.
      float ipgain = 0.08;
      while ( ip1.size() < 1500 || ip2.size() < 1500 ) {
        ip1.clear(); ip2.clear();
        ip::OBALoGInterestOperator interest_operator( ipgain );
        ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator, 0 );
        vw_out() << "\t    Processing " << input_file1 << "\n";
        ip1 = detect_interest_points( left_image, detector );
        vw_out() << "\t    Processing " << input_file2 << "\n";
        ip2 = detect_interest_points( right_image, detector );

        ipgain *= 0.75;
      }
      vw_out() << "\t    Located " << ip1.size() << " points.\n";
      vw_out() << "\t    Located " << ip2.size() << " points.\n";

      vw_out() << "\t    Generating descriptors...\n";
      ip::SGradDescriptorGenerator descriptor;
      descriptor( left_image, ip1 );
      descriptor( right_image, ip2 );
      vw_out() << "\t    done.\n";

      // Writing out the results
      vw_out() << "\t    Caching interest points: "
                << ip1_filename << ", "
                << ip2_filename << "\n";
      ip::write_binary_ip_file(ip1_filename, ip1);
      ip::write_binary_ip_file(ip2_filename, ip2);

      // Reading back into the vector interestpoint format
      ip1_copy = ip::read_binary_ip_file( ip1_filename );
      ip2_copy = ip::read_binary_ip_file( ip2_filename );
    }

    vw_out() << "\t--> Matching interest points\n";
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);

    matcher(ip1_copy, ip2_copy,
            matched_ip1, matched_ip2,
            false,
            TerminalProgressCallback( "asp", "\t    Matching: "));

  } // End matching

  vw_out(InfoMessage) << "\t--> " << matched_ip1.size()
                      << " putative matches.\n";

  vw_out() << "\t--> Rejecting outliers using RANSAC.\n";
  remove_duplicates(matched_ip1, matched_ip2);
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  vw_out(DebugMessage) << "\t--> Removed "
                       << matched_ip1.size() - ransac_ip1.size()
                       << " duplicate matches.\n";

  Matrix<double> T;
  std::vector<int> indices;
  try {

    math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 10 );
    T = ransac( ransac_ip2, ransac_ip1 );
    indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
    vw_out(DebugMessage) << "\t--> AlignMatrix: " << T << std::endl;

  } catch (...) {
    vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
    T.set_size(3,3);
    T.set_identity();
  }

  vw_out() << "\t    Caching matches: "
              << match_filename << "\n";

  { // Keeping only inliers
    std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
    for ( unsigned i = 0; i < indices.size(); i++ ) {
      inlier_ip1.push_back( matched_ip1[indices[i]] );
      inlier_ip2.push_back( matched_ip2[indices[i]] );
    }
    matched_ip1 = inlier_ip1;
    matched_ip2 = inlier_ip2;
  }

  write_binary_match_file( match_filename,
                           matched_ip1, matched_ip2);

  return T;
}

// Do not attempt interest point alignment; assume ISIS images are already map projected.
// Simply remove the special pixels and normalize between 0 and 1 (so that the image masks
// are found properly)
void
StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                          std::string & output_file1, std::string & output_file2) {

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  try {
    DiskImageView<PixelGray<float32> > out1(output_file1);
    DiskImageView<PixelGray<float32> > out2(output_file2);
    vw_out(InfoMessage) << "Skipping normalization step, using cached images: " <<
      output_file1 << " and " << output_file2 << "\n";
    return;
  } catch (vw::Exception const& e) {}

  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);
  DiskImageResourceIsis left_rsrc(input_file1);
  DiskImageResourceIsis right_rsrc(input_file2);

  float left_lo, left_hi, right_lo, right_hi;
  float left_mean, left_std, right_mean, right_std;

  // Calculating statistics. We subsample the images so statistics
  // only does about a million samples.
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for the left image\n";
    int left_stat_scale = int(ceil(sqrt(float(left_disk_image.cols())*float(left_disk_image.rows()) / 1000000)));
    ImageViewRef<PixelMask<PixelGray<float> > > left_valid =
      subsample(create_mask( left_disk_image, left_rsrc.valid_minimum(), left_rsrc.valid_maximum() ),
                left_stat_scale );
    min_max_channel_values( left_valid, left_lo, left_hi);
    left_mean = mean_channel_value( left_valid );
    left_std = stddev_channel_value( left_valid );
    vw_out(InfoMessage) << "\t    Left: [ lo:" << left_lo << " hi:" << left_hi
                        << " m: " << left_mean << " s: " << left_std <<  "]\n";
  }
  {
    vw_out(InfoMessage) << "\t--> Computing statistics values for the right image\n";
    int right_stat_scale = int(ceil(sqrt(float(right_disk_image.cols())*float(right_disk_image.rows()) / 1000000)));
    ImageViewRef<PixelMask<PixelGray<float> > > right_valid =
      subsample(create_mask( right_disk_image, right_rsrc.valid_minimum(), right_rsrc.valid_maximum() ),
                right_stat_scale );
    min_max_channel_values( right_valid, right_lo, right_hi);
    right_mean = mean_channel_value( right_valid );
    right_std = stddev_channel_value( right_valid );
    vw_out(InfoMessage) << "\t    Right: [ lo:" << right_lo << " hi:" << right_hi
                        << " m: " << right_mean << " s: " << right_std << "]\n";
  }

  // Normalizing to -+2 sigmas around mean
  if ( stereo_settings().force_max_min == 0 ) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    if ( left_lo < left_mean - 2*left_std )
      left_lo = left_mean - 2*left_std;
    if ( right_lo < right_mean - 2*right_std )
      right_lo = right_mean - 2*right_std;
    if ( left_hi > left_mean + 2*left_std )
      left_hi = left_mean + 2*left_std;
    if ( right_hi > right_mean + 2*right_std )
      right_hi = right_mean + 2*right_std;

    vw_out(InfoMessage) << "\t    Left changed: [ lo:"<<left_lo<<" hi:"<<left_hi<<"]\n";
    vw_out(InfoMessage) << "\t    Right changed: [ lo:"<<right_lo<<" hi:"<<right_hi<<"]\n";
  }

  // Working out alignment
  float lo = std::min (left_lo, right_lo);  // Finding global
  float hi = std::max (left_hi, right_hi);
  Matrix<double> align_matrix(3,3);
  align_matrix.set_identity();
  if ( stereo_settings().keypoint_alignment) {
    if ( left_rsrc.is_map_projected() ||
         right_rsrc.is_map_projected() ) {
      vw_out(WarningMessage,"console") << "One or more of the input files is Map Projected.\n"
                                       << "\tInterest Point alignment is not recommend in this case.\n";
    }
    align_matrix = determine_image_alignment(input_file1, input_file2,
                                             lo, hi );
  }
  write_matrix( m_out_prefix + "-align.exr", align_matrix );

  // Apply alignment and normalization
  ImageViewRef<PixelGray<float> > Limg;
  ImageViewRef<PixelGray<float> > Rimg;
  if (stereo_settings().individually_normalize == 0 ) {
    vw_out() << "\t--> Normalizing globally to: ["<<lo<<" "<<hi<<"]\n";
    Limg = clamp(normalize(remove_isis_special_pixels(left_disk_image, left_lo, left_hi, lo),
                           lo, hi, 0.0, 1.0));
    Rimg = clamp(transform(normalize(remove_isis_special_pixels(right_disk_image, right_lo, right_hi, lo),
                                     lo, hi, 0.0, 1.0),
                           HomographyTransform(align_matrix),
                           left_disk_image.cols(), left_disk_image.rows()));
  } else {
    vw_out() << "\t--> Individually normalizing.\n";
    Limg = clamp(normalize(remove_isis_special_pixels(left_disk_image, left_lo,
                                                      left_hi, left_lo),
                           left_lo, left_hi, 0.0, 1.0));
    Rimg = clamp(transform(normalize(remove_isis_special_pixels(right_disk_image, right_lo,
                                                                right_hi, right_lo),
                                     right_lo, right_hi, 0.0, 1.0),
                           HomographyTransform(align_matrix),
                           left_disk_image.cols(), left_disk_image.rows()));
  }
  // Write the results to disk.
  vw_out() << "\t--> Writing normalized images.\n";
  DiskImageResourceGDAL left_out_rsrc( output_file1, Limg.format(),
                                       Vector2i(vw_settings().default_tile_size(),
                                                vw_settings().default_tile_size()) );
  block_write_image( left_out_rsrc, Limg,
                     TerminalProgressCallback("asp", "\t    Left:  "));

  DiskImageResourceGDAL right_out_rsrc( output_file2, Rimg.format(),
                                        Vector2i(vw_settings().default_tile_size(),
                                                 vw_settings().default_tile_size()) );
  block_write_image( right_out_rsrc, Rimg,
                     TerminalProgressCallback("asp", "\t    Right: "));
}

inline std::string write_shadow_mask( std::string const& output_prefix,
                                      std::string const& input_image,
                                      std::string const& mask_postfix ) {
  // This thresholds at -25000 as the input sub4s for Apollo that I've
  // processed have a range somewhere between -32000 and +32000. -ZMM
  DiskImageView<PixelGray<float> > disk_image( input_image );
  DiskImageView<uint8> disk_mask( output_prefix + mask_postfix );
  ImageViewRef<uint8> mask =
    apply_mask(intersect_mask(create_mask(disk_mask),
                              create_mask(threshold(disk_image,-25000,0,1.0))));
  std::string output_mask =
    output_prefix+mask_postfix.substr(0,mask_postfix.size()-4)+"Debug.tif";

  DiskImageResourceGDAL out_mask_rsrc( output_mask, mask.format(),
                                       Vector2i(vw_settings().default_tile_size(),
                                                vw_settings().default_tile_size()) );
  block_write_image( out_mask_rsrc, mask );
  return output_mask;
}

// Stage 2: Correlation
//
// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
// Post file is a disparity map.            ( ImageView<PixelMask<Vector2f> > )
void
StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                                      std::string & output_file) {
  output_file = input_file;

  // ****************************************************
  // The following code is for Apollo Metric Camera ONLY!
  // (use at your own risk)
  // ****************************************************
  if (stereo_settings().mask_flatfield) {
    vw_out() << "\t--> Masking pixels that are less than 0.0.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = m_out_prefix + "-R-masked.exr";

    std::string shadowLmask_name =
      write_shadow_mask( m_out_prefix, m_left_image_file,
                         "-lMask.tif" );
    std::string shadowRmask_name =
      write_shadow_mask( m_out_prefix, m_right_image_file,
                         "-rMask.tif" );

    DiskImageView<uint8> shadowLmask( shadowLmask_name );
    DiskImageView<uint8> shadowRmask( shadowRmask_name );

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef<PixelMask<Vector2f> > disparity_map =
      stereo::disparity_mask(disparity_disk_image,
                             shadowLmask, shadowRmask );

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(vw_settings().default_tile_size(),disparity_map.cols()),
                                       std::min(vw_settings().default_tile_size(),disparity_map.rows()));
    block_write_image( disparity_map_rsrc, disparity_map,
                       TerminalProgressCallback( "asp", "\t--> Saving Mask :") );
  }
}

// Reverse any pre-alignment that was done to the images.
void
StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file,
                                       std::string & output_file) {

  // ****************************************************
  // The following code is for Apollo Metric Camera ONLY!
  // (use at your own risk)
  // ****************************************************
  std::string dust_result = input_file;
  if ( stereo_settings().mask_flatfield ) {
    vw_out() << "\t--> Masking pixels that appear to be dust.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection( m_out_prefix, input_file,
                                   dust_result, stereo_settings().h_kern );
  }

  DiskImageView<PixelMask<Vector2f> > disparity_map(dust_result);
  ImageViewRef<PixelMask<Vector2f> > result = disparity_map;
  output_file = m_out_prefix + "-F-corrected.exr";

  // We used a homography to line up the images, we may want
  // to generate pre-alignment disparities before passing this information
  // onto the camera model in the next stage of the stereo pipeline.
  Matrix<double> align_matrix;
  try {
    read_matrix(align_matrix, m_out_prefix + "-align.exr");
    vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
  } catch (vw::IOErr &e) {
    vw_out() << "\nCould not read in aligment matrix: " << m_out_prefix << "-align.exr.  Exiting. \n\n";
    exit(1);
  }

  result = stereo::transform_disparities(disparity_map, HomographyTransform(align_matrix));

  // Remove pixels that are outside the bounds of the secondary image.
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  result = stereo::disparity_range_mask(result,
                                        Vector2f(0,0),
                                        Vector2f( right_disk_image.cols(),
                                                  right_disk_image.rows() ) );

  DiskImageResourceOpenEXR disparity_corrected_rsrc(output_file, result.format() );
  disparity_corrected_rsrc.set_tiled_write(std::min(vw_settings().default_tile_size(),disparity_map.cols()),
                                           std::min(vw_settings().default_tile_size(),disparity_map.rows()));
  block_write_image( disparity_corrected_rsrc, result,
                     TerminalProgressCallback("asp", "\t    Processing:"));
}

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::camera_model(std::string image_file,
                                std::string camera_file) {

  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    vw_out() << "\t--> Using adjusted Isis Camera Model: " << camera_file << "\n";

    // Creating Equations for the files
    std::ifstream input( camera_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = read_equation( input );
    boost::shared_ptr<asp::BaseEquation> poseF = read_equation( input );
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<camera::CameraModel>(new IsisAdjustCameraModel( image_file, posF, poseF ));

  } else {
    vw_out() << "\t--> Using standard Isis camera model: " << image_file << "\n";
    return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
  }

}


