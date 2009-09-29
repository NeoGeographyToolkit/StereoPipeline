// __BEGIN_LICENSE__
//
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2008 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::stereo;
using namespace vw::ip;


// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}


static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
static void remove_duplicates(std::vector<Vector3> &ip1, std::vector<Vector3> &ip2) {
  std::vector<Vector3> new_ip1, new_ip2;

  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j &&
          (ip1[i] == ip1[j] || ip2[i] == ip2[j])) {
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

vw::math::Matrix<double> StereoSessionIsis::determine_image_alignment(std::string const& input_file1, std::string const& input_file2, float lo, float hi) {

  std::vector<InterestPoint> matched_ip1, matched_ip2;
  if ( exists( prefix_from_filename( input_file1 ) + "__" +
               prefix_from_filename( input_file2 ) + ".match" ) ) {
    // Is there a match file linking these 2 image?

    vw_out(0) << "\t--> Found cached interest point match file: "
              << ( prefix_from_filename(input_file1) + "__" +
                   prefix_from_filename(input_file2) + ".match" ) << "\n";
    read_binary_match_file( ( prefix_from_filename(input_file1) + "__" +
                              prefix_from_filename(input_file2) + ".match" ),
                            matched_ip1, matched_ip2 );
  } else {

    // Next best thing.. VWIPs?
    std::vector<InterestPoint> ip1_copy, ip2_copy;

    if ( exists( prefix_from_filename(input_file1) + ".vwip" ) &&
         exists( prefix_from_filename(input_file2) + ".vwip" ) ) {
      // Found VWIPs already done before
      vw_out(0) << "\t--> Found cached interest point files: "
                << ( prefix_from_filename(input_file1) + ".vwip" ) << "\n"
                << "\t                                       "
                << ( prefix_from_filename(input_file2) + ".vwip" ) << "\n";
      ip1_copy = read_binary_ip_file( prefix_from_filename(input_file1) +
                                      ".vwip" );
      ip2_copy = read_binary_ip_file( prefix_from_filename(input_file2) +
                                      ".vwip" );

    } else {
      // Worst case, no interest point operations have been performed before
      vw_out(0) << "\t--> Locating Interest Points\n";
      InterestPointList ip1, ip2;
      DiskImageView<PixelGray<float> > left_disk_image(input_file1);
      DiskImageView<PixelGray<float> > right_disk_image(input_file2);
      ImageViewRef<PixelGray<float> > left_image = normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
      ImageViewRef<PixelGray<float> > right_image = normalize(remove_isis_special_pixels(right_disk_image, lo), lo, hi, 0, 1.0);

      // Interest Point module detector code.
      LogInterestOperator log_detector;
      ScaledInterestPointDetector<LogInterestOperator> detector(log_detector, 500);
      vw_out(0) << "\t    Processing " << input_file1 << "\n";
      ip1 = detect_interest_points( left_image, detector );
      vw_out(0) << "\t    Located " << ip1.size() << " points.\n";
      vw_out(0) << "\t    Processing " << input_file2 << "\n";
      ip2 = detect_interest_points( right_image, detector );
      vw_out(0) << "\t    Located " << ip2.size() << " points.\n";

      vw_out(0) << "\t    Generating descriptors...\n";
      PatchDescriptorGenerator descriptor;
      descriptor( left_image, ip1 );
      descriptor( right_image, ip2 );
      vw_out(0) << "\t    done.\n";

      // Writing out the results
      vw_out(0) << "\t    Caching interest points: "
                << (prefix_from_filename(input_file1) + ".vwip") << ", "
                << (prefix_from_filename(input_file2) + ".vwip") << "\n";
      write_binary_ip_file(prefix_from_filename(input_file1)+".vwip", ip1);
      write_binary_ip_file(prefix_from_filename(input_file2)+".vwip", ip2);

      // Reading back into the vector interestpoint format
      ip1_copy = read_binary_ip_file( prefix_from_filename(input_file1) + ".vwip" );
      ip2_copy = read_binary_ip_file( prefix_from_filename(input_file2) + ".vwip" );

    }

    vw_out(0) << "\t--> Matching interest points\n";
    InterestPointMatcher<L2NormMetric,NullConstraint> matcher(0.8);

    matcher(ip1_copy, ip2_copy,
            matched_ip1, matched_ip2,
            false,
            TerminalProgressCallback( InfoMessage, "\t    Matching: "));

    vw_out(0) << "\t    Caching matches: "
              << ( prefix_from_filename(input_file1) + "__" +
                   prefix_from_filename(input_file2) + ".match") << "\n";

    write_binary_match_file( prefix_from_filename(input_file1) + "__" +
                             prefix_from_filename(input_file2) + ".match",
                             matched_ip1, matched_ip2);

  } // End matching

  vw_out(InfoMessage) << "\t--> " << matched_ip1.size()
                      << " putative matches.\n";

  vw_out(0) << "\t--> Rejecting outliers using RANSAC.\n";
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  remove_duplicates(ransac_ip1, ransac_ip2);
  vw_out(DebugMessage) << "\t--> Removed "
                       << matched_ip1.size() - ransac_ip1.size()
                       << " duplicate matches.\n";

  Matrix<double> T;
  try {

    math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 10 );
    T = ransac( ransac_ip2, ransac_ip1 );
    vw_out(DebugMessage) << "\t--> AlignMatrix: " << T << std::endl;

  } catch (...) {
    vw_out(0) << "\n*************************************************************\n";
    vw_out(0) << "WARNING: Automatic Alignment Failed!  Proceed with caution...\n";
    vw_out(0) << "*************************************************************\n\n";
    T.set_size(3,3);
    T.set_identity();
  }

  return T;
}

// Pre-align the ISIS images.  If the ISIS images are map projected,
// we can perform pre-alignment by transforming them both into a
// common map projection.  Otherwise, we resort to feature-based image
// matching techniques to align the right image to the left image.
void StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                               std::string & output_file1, std::string & output_file2) {

  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);
  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  DiskImageResourceIsis left_rsrc(input_file1);
  DiskImageResourceIsis right_rsrc(input_file2);

  // Make sure the images are normalized
  vw_out(InfoMessage) << "\t--> Computing min/max values for normalization.\n";
  float left_lo, left_hi, right_lo, right_hi;
  float left_mean, left_std, right_mean, right_std;
  min_max_channel_values(create_mask(left_disk_image, left_rsrc.valid_minimum(),
                                     left_rsrc.valid_maximum()), left_lo, left_hi);
  left_mean = mean_channel_value(create_mask(left_disk_image, left_rsrc.valid_minimum(),
                                             left_rsrc.valid_maximum()));
  left_std = stddev_channel_value(create_mask(left_disk_image,left_rsrc.valid_minimum(),
                                              left_rsrc.valid_maximum()));
  vw_out(InfoMessage) << "\t    Left: [ lo:" << left_lo << " hi:" << left_hi
                      << " m:" << left_mean << " s:" << left_std << "]\n";
  min_max_channel_values(create_mask(right_disk_image, right_rsrc.valid_minimum(),
                                     right_rsrc.valid_maximum()), right_lo, right_hi);
  right_mean = mean_channel_value(create_mask(right_disk_image, right_rsrc.valid_minimum(),
                                              right_rsrc.valid_maximum()));
  right_std = stddev_channel_value(create_mask(right_disk_image,right_rsrc.valid_minimum(),
                                                 right_rsrc.valid_maximum()));
  vw_out(InfoMessage) << "\t    Right: [ lo:" << right_lo << " hi:" << right_hi
                      << " m:" << right_mean << " s:" << right_std << "]\n";

   // Normalizing to -+2 sigmas around mean
  if ( stereo_settings().force_max_min == 0 ) {
    if ( left_lo < left_mean - 2*left_std )
      left_lo = left_mean - 2*left_std;
    if ( right_lo < right_mean - 2*right_std )
      right_lo = right_mean - 2*right_std;
    if ( left_hi > left_mean + 2*left_std )
      left_hi = left_mean + 2*left_std;
    if ( right_hi > right_mean + 2*right_std )
      right_hi = right_mean + 2*right_std;

    vw_out(DebugMessage, "stereo") << "\t    Left changed: [ lo:"<<left_lo<<" hi:"<<left_hi<<"]\n";
    vw_out(DebugMessage, "stereo") << "\t    Right changed: [ lo:"<<right_lo<<" hi:"<<right_hi<<"]\n";
  }

  // Picking Global
  float lo = std::min (left_lo, right_lo);
  float hi = std::max (left_hi, right_hi);

  Matrix<double> align_matrix(3,3);
  align_matrix.set_identity();
  if (stereo_settings().keypoint_alignment)
    align_matrix = determine_image_alignment(input_file1, input_file2,
                                             lo, hi);
  ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

  // Apply the alignment transformation to the right image.
  ImageViewRef<PixelGray<float> > Limg;
  ImageViewRef<PixelGray<float> > Rimg;
  if (stereo_settings().individually_normalize == 0 ) {
    vw_out(0) << "\t--> Normalizing globally to: ["<<lo<<" "<<hi<<"]\n";
    Limg = normalize(remove_isis_special_pixels(left_disk_image, left_lo, left_hi, lo),
                     lo,hi,0.0,1.0);
    Rimg = transform(normalize(remove_isis_special_pixels(right_disk_image,right_lo,right_hi,lo),
                               lo,hi,0.0,1.0),
                     HomographyTransform(align_matrix),
                     left_disk_image.cols(), left_disk_image.rows());
  } else {
    vw_out(0) << "\t--> Individually normalizing.\n";
    Limg = normalize(remove_isis_special_pixels(left_disk_image, left_lo, left_hi, left_lo),
                     left_lo,left_hi,0.0,1.0);
    Rimg = transform(normalize(remove_isis_special_pixels(right_disk_image,right_lo,right_hi,right_lo),
                               right_lo,right_hi,0.0,1.0),
                     HomographyTransform(align_matrix),
                     left_disk_image.cols(), left_disk_image.rows());
  }

  // Write the results to disk.
  vw_out(0) << "\t--> Writing pre-aligned images.\n";
  {
    DiskImageResourceGDAL left_rsrc( output_file1, Limg.format(), Vector2i(256,256) );
    block_write_image( left_rsrc, Limg, TerminalProgressCallback(InfoMessage, "\t    Left:  "));
  }
  {
    DiskImageResourceGDAL right_rsrc( output_file2, Rimg.format(), Vector2i(256,256) );
    block_write_image( right_rsrc, Rimg, TerminalProgressCallback(InfoMessage, "\t    Right: "));
  }
}

// Stage 2: Correlation
//
// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
// Post file is a disparity map.            ( ImageView<PixelDisparity<float> > )
void StereoSessionIsis::pre_filtering_hook(std::string const& input_file, std::string & output_file) {
  output_file = input_file;

  // ****************************************************
  // The following code is for Apollo Metric Camera ONLY!
  // (use at your own risk)
  // ****************************************************
  if (stereo_settings().mask_flatfield) {
    vw_out(0) << "\t--> Masking pixels that are less than 0.0.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = m_out_prefix + "-R-masked.exr";
    DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
    DiskImageView<uint8> left_disk_mask( m_out_prefix + "-lMask.tif");
    DiskImageView<uint8> right_disk_mask( m_out_prefix + "-rMask.tif");

    ImageViewRef<PixelMask<uint8> > Lmask = intersect_mask(create_mask(left_disk_mask),
                                                           create_mask(clamp(left_disk_image,0,1e6)));
    ImageViewRef<PixelMask<uint8> > Rmask = intersect_mask(create_mask(right_disk_mask),
                                                           create_mask(clamp(right_disk_image,0,1e6)));

    write_image(m_out_prefix + "-lMaskDebug.tif", Lmask);
    write_image(m_out_prefix + "-rMaskDebug.tif", Rmask);

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef<PixelMask<Vector2f> > disparity_map = disparity_mask(disparity_disk_image, Lmask, Rmask);

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(512,disparity_map.cols()),
                                       std::min(512,disparity_map.rows()));
    block_write_image( disparity_map_rsrc, disparity_map, TerminalProgressCallback(InfoMessage, "\t--> Saving Mask :") );
  }
}

// Reverse any pre-alignment that was done to the images.
void StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {

  DiskImageView<PixelMask<Vector2f> > disparity_map(input_file);
  output_file = m_out_prefix + "-F-corrected.exr";
  ImageViewRef<PixelMask<Vector2f> > result;

  // We used a homography to line up the images, we may want
  // to generate pre-alignment disparities before passing this information
  // onto the camera model in the next stage of the stereo pipeline.
  Matrix<double> align_matrix;
  try {
    read_matrix(align_matrix, m_out_prefix + "-align.exr");
    vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
  } catch (vw::IOErr &e) {
    vw_out(0) << "\nCould not read in aligment matrix: " << m_out_prefix << "-align.exr.  Exiting. \n\n";
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
  disparity_corrected_rsrc.set_tiled_write(std::min(512,disparity_map.cols()),
                                     std::min(512,disparity_map.rows()));
  block_write_image( disparity_corrected_rsrc, result,
                     TerminalProgressCallback(InfoMessage, "\t    Processing:"));
}


boost::shared_ptr<vw::camera::CameraModel> StereoSessionIsis::camera_model(std::string image_file,
                                                                           std::string camera_file) {

  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    vw_out(0) << "\t--> Using adjusted Isis Camera Model: " << camera_file << "\n";

    // Creating Equations for the files
    std::ifstream input( camera_file.c_str() );
    boost::shared_ptr<BaseEquation> posF = read_equation( input );
    boost::shared_ptr<BaseEquation> poseF = read_equation( input );
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<camera::CameraModel>(new IsisAdjustCameraModel( image_file, posF, poseF ));

  } else {
    vw_out(0) << "\t--> Using standard Isis camera model: " << image_file << "\n";
    return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
  }

}


