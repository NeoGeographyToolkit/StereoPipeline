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

#include <boost/shared_ptr.hpp>

#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include "Isis/StereoSessionIsis.h"
#include "Isis/IsisCameraModel.h"
#include "StereoSettings.h"

// For support of IsisAdjust camera model
#include "Isis/Equations.h"
#include "Isis/IsisAdjustCameraModel.h"

// Boost
#include "boost/filesystem.hpp"   
using namespace boost::filesystem; 

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::stereo;
using namespace vw::ip;


// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
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
  std::string left_align_image_file(input_file1), right_align_image_file(input_file2);

  // Load the two images
  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);

  // Image Alignment
  //
  // Images are aligned by computing interest points, matching
  // them using a standard 2-Norm nearest-neighor metric, and then
  // rejecting outliers by fitting a similarity between the
  // putative matches using RANSAC.  

  // Interest points are matched in image chunk of <= 2048x2048
  // pixels to conserve memory.
  InterestPointList ip1, ip2;
  
  // If the interest points are cached in a file, read that file in here.
  if ( boost::filesystem::exists( prefix_from_filename(input_file1) + ".vwip" ) && 
              boost::filesystem::exists( prefix_from_filename(input_file2) + ".vwip" )) {
    vw_out(0) << "\t--> Found cached interest point files: "
              << (prefix_from_filename(input_file1) + ".vwip") << ", "
              << (prefix_from_filename(input_file2) + ".vwip") << "\n";
    vw_out(0) << "\t    (Skipping interest point detection step)\n";
  } else {
    vw_out(0) << "\t--> Locating Interest Points \n";
    ImageViewRef<PixelGray<float> > left_image = normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
    ImageViewRef<PixelGray<float> > right_image = normalize(remove_isis_special_pixels(right_disk_image, lo), lo, hi, 0, 1.0);
    
    // Interest Point module detector code.
    LogInterestOperator log_detector;
    ScaledInterestPointDetector<LogInterestOperator> detector(log_detector, 500);
    vw_out(0) << "\t    Processing " << input_file1 << "\n";
    ip1 = detect_interest_points(left_image, detector);
    vw_out(0) << "\t    " << ip1.size() << " points.\n";
    vw_out(0) << "\t    Processing " << input_file2 << "\n";
    ip2 = detect_interest_points(right_image, detector);
    vw_out(0) << "\t    Located " << ip2.size() << " points.\n"; 

    // Generate descriptors for interest points.
    // TODO: Switch to a more sophisticated descriptor
    vw_out(0) << "\t    Generating descriptors... " << std::flush;
    PatchDescriptorGenerator descriptor;
    descriptor(left_image, ip1);
    descriptor(right_image, ip2);
    vw_out(0) << "done.\n";
  
    // Write out the results
    vw_out(0) << "\t    Caching interest points: " 
              << (prefix_from_filename(input_file1)+".vwip") << ", " 
              << (prefix_from_filename(input_file2)+".vwip") << "\n"; 
    write_binary_ip_file(prefix_from_filename(input_file1)+".vwip", ip1);
    write_binary_ip_file(prefix_from_filename(input_file2)+".vwip", ip2);
  }

  // Checking once again to see if the match file exists. This could
  // be done better.
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  if ( boost::filesystem::exists( prefix_from_filename(input_file1) + "__" +
				  prefix_from_filename(input_file2) + ".match" ) ) {
    vw_out(0) << "\t--> Found cached interest point match file: " << ( prefix_from_filename(input_file1) + "__" +
                                                                  prefix_from_filename(input_file2) + ".match" ) << "\n";
    read_binary_match_file( ( prefix_from_filename(input_file1) + "__" +
			      prefix_from_filename(input_file2) + ".match" ),
			    matched_ip1, matched_ip2 );
  } else { 
    vw_out(0) << "\t--> Matching interest points\n";
    // The basic interest point matcher does not impose any
    // constraints on the matched interest points.
    double matcher_threshold = 0.8;

    // RANSAC needs the matches as a vector, and so does the matcher.
    // this is messy, but for now we simply make a copy.
    std::vector<InterestPoint> ip1_copy, ip2_copy;
    ip1_copy = read_binary_ip_file(prefix_from_filename(input_file1)+".vwip");
    ip2_copy = read_binary_ip_file(prefix_from_filename(input_file2)+".vwip");

    InterestPointMatcher<L2NormMetric,NullConstraint> matcher(matcher_threshold);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2, false, TerminalProgressCallback(ErrorMessage, "\t    Matching: "));
    write_binary_match_file(prefix_from_filename(input_file1) + "__" +
                            prefix_from_filename(input_file2) + ".match",
                            matched_ip1, matched_ip2);
    vw_out(0) << "\t    Caching matches: " << ( prefix_from_filename(input_file1) + "__" +
                                                prefix_from_filename(input_file2) + ".match" ) << "\n";
  }
  vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";

  // RANSAC is used to fit a similarity transform between the
  // matched sets of points  
  vw_out(0) << "\t--> Rejecting outliers using RANSAC.\n";
  std::vector<Vector3> ransac_ip1(matched_ip1.size());
  std::vector<Vector3> ransac_ip2(matched_ip2.size());
  for (unsigned i = 0; i < matched_ip1.size();++i ) {
    ransac_ip1[i] = Vector3(matched_ip1[i].x, matched_ip1[i].y,1);
    ransac_ip2[i] = Vector3(matched_ip2[i].x, matched_ip2[i].y,1);
  }  
  remove_duplicates(ransac_ip1, ransac_ip2);
  RandomSampleConsensus<math::HomographyFittingFunctor, InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(),
                                                                                          InterestPointErrorMetric(), 
                                                                                          10 ); // inlier_threshold
  std::vector<Vector3> result_ip1, result_ip2;
  Matrix<double> T;
  try {
    T = ransac(ransac_ip2,ransac_ip1);
  } catch (vw::ip::RANSACErr &e) {
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

  GeoReference input_georef1, input_georef2;
  // Disabled for now since we haven't really figured how to
  // capitalize on the map projected images... -mbroxton
  //
  //   try {
  //     // Read georeferencing information (if it exists...)
  //     DiskImageResourceGDAL file_resource1( input_file1 );
  //     DiskImageResourceGDAL file_resource2( input_file2 );
  //     read_georeference( input_georef1, file_resource1 );
  //     read_georeference( input_georef2, file_resource2 );
  //   } catch (ArgumentErr &e) {
  //     vw_out(0) << "Warning: Couldn't read georeference data from input images using the GDAL driver.\n";
  //   }

  // Make sure the images are normalized
  vw_out(InfoMessage) << "\t--> Computing min/max values for normalization.  " << std::flush;
  float left_lo, left_hi, right_lo, right_hi;
  isis_min_max_channel_values(left_disk_image, left_lo, left_hi);
  vw_out(InfoMessage) << "Left: [" << left_lo << " " << left_hi << "]    " << std::flush;
  isis_min_max_channel_values(right_disk_image, right_lo, right_hi);
  vw_out(InfoMessage) << "Right: [" << right_lo << " " << right_hi << "]\n";
  float lo = std::min (left_lo, right_lo);
  float hi = std::min (left_hi, right_hi);

  // If this is a map projected cube, we skip the step of aligning the
  // images, because the map projected images are probable very nearly
  // aligned already.  For unprojected cubes, we align in the "usual"
  // way using interest points.
  if (input_georef1.transform() != math::identity_matrix<3>() && 
      input_georef2.transform() != math::identity_matrix<3>() ) {
    vw_out(0) << "\t--> Map projected ISIS cubes detected.  Placing both images into the same map projection.\n";
    
    // If we are using map-projected cubes, we need to put them into a
    // common projection.  We adopt the projection of the first image.
    GeoReference common_georef = input_georef1;
    BBox2i common_bbox(0,0,left_disk_image.cols(), left_disk_image.rows());

    // Create the geotransform objects and determine the common size
    // of the output images and apply it to the right image.
    GeoTransform trans2(input_georef2, common_georef);
    ImageViewRef<PixelGray<float> > Limg = normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
    ImageViewRef<PixelGray<float> > Rimg = crop(transform(normalize(remove_isis_special_pixels(right_disk_image, lo),lo,hi,0.0,1.0),trans2),common_bbox);

    // Write the results to disk.
    write_image(output_file1, channel_cast_rescale<uint8>(Limg), TerminalProgressCallback());
    write_image(output_file2, channel_cast_rescale<uint8>(Rimg), TerminalProgressCallback()); 
  
  } else {
    // For unprojected ISIS images, we resort to the "old style" of
    // image alignment: determine the alignment matrix using keypoint
    // matching techniques.
    vw_out(0) << "\t--> Unprojected ISIS cubes detected.  Aligning images using feature-based matching techniques.\n";

    Matrix<double> align_matrix(3,3);
    align_matrix.set_identity();
    if (stereo_settings().keypoint_alignment)
      align_matrix = determine_image_alignment(input_file1, input_file2, lo, hi);
    ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

    // Apply the alignment transformation to the right image.
    ImageViewRef<PixelGray<float> > Limg = normalize(remove_isis_special_pixels(left_disk_image, lo),lo,hi,0.0,1.0);
    ImageViewRef<PixelGray<float> > Rimg = transform(normalize(remove_isis_special_pixels(right_disk_image,lo),lo,hi,0.0,1.0), 
                                                     HomographyTransform(align_matrix),
                                                     left_disk_image.cols(), left_disk_image.rows());

    // Write the results to disk.
    vw_out(0) << "\t--> Writing pre-aligned images.\n";
    write_image(output_file1, channel_cast_rescale<uint8>(Limg), TerminalProgressCallback(ErrorMessage, "\t    Left:  "));
    write_image(output_file2, channel_cast_rescale<uint8>(Rimg), TerminalProgressCallback(ErrorMessage, "\t    Right: ")); 
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
  // (uncomment at your own risk)
  // ****************************************************

//   output_file = m_out_prefix + "-R-masked.exr";
//   ImageView<uint8> Lmask, Rmask;
//   DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
//   DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  
//   read_image(Lmask,m_out_prefix + "-lMask.tif");
//   read_image(Rmask,m_out_prefix + "-rMask.tif");
//   disparity::mask_black_pixels(clamp(left_disk_image,0,1e6), Lmask);
//   disparity::mask_black_pixels(clamp(right_disk_image,0,1e6), Rmask);
//   write_image(m_out_prefix + "-lMaskDebug.tif", Lmask);
//   write_image(m_out_prefix + "-rMaskDebug.tif", Rmask);
  
//   DiskImageView<PixelDisparity<float> > disparity_disk_image(input_file);
//   ImageViewRef<PixelDisparity<float> > disparity_map = disparity::mask(disparity_disk_image, Lmask, Rmask);

//   DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
//   disparity_map_rsrc.set_tiled_write(std::min(512,disparity_map.cols()),std::min(512, disparity_map.rows()));
//   block_write_image( disparity_map_rsrc, disparity_map, TerminalProgressCallback() );
}

// Reverse any pre-alignment that was done to the images.
void StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {

  DiskImageView<PixelDisparity<float> > disparity_map(input_file);
  output_file = m_out_prefix + "-F-corrected.exr";
  ImageViewRef<PixelDisparity<float> > result;

  // Read georeferencing information (if it exists...)
  GeoReference input_georef1, input_georef2;
  // Disabled for now since we haven't really figured how to
  // capitalize on the map projected images... -mbroxton
  //
  //   try {
  //     DiskImageResourceGDAL file_resource1( m_left_image_file );
  //     DiskImageResourceGDAL file_resource2( m_right_image_file );
  //     read_georeference( input_georef1, file_resource1 );
  //     read_georeference( input_georef2, file_resource2 );
  //   } catch (ArgumentErr &e) {
  //     vw_out(0) << "\t--> Warning: Couldn't read georeference data from input images using the GDAL driver.\n";
  //   }
  
  // If this is a map projected cube, we skip the step of aligning the
  // images, because the map projected images are probable very nearly
  // aligned already.  For unprojected cubes, we align in the "usual"
  // way using interest points.
  if (input_georef1.transform() != math::identity_matrix<3>() && 
      input_georef2.transform() != math::identity_matrix<3>() ) {
    vw_out(0) << "\t--> Map projected ISIS cubes detected.\n"
              << "\t--> Placing both images into the same map projection.\n";

    // If we are using map-projected cubes, we need to put them into a
    // common projection.  We adopt the projection of the first image.
    result = stereo::disparity::transform_disparities(disparity_map, GeoTransform(input_georef2, input_georef1));

  } else {
    vw_out(0) << "\t--> Unprojected ISIS cubes detected.\n"
              << "\t--> Processing disparity map to remove the earlier effects of interest point alignment.\n";  

    // We used a homography to line up the images, we may want 
    // to generate pre-alignment disparities before passing this information
    // onto the camera model in the next stage of the stereo pipeline.
    vw::Matrix<double> align_matrix;
    try {
      ::read_matrix(align_matrix, m_out_prefix + "-align.exr");
      vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
    } catch (vw::IOErr &e) {
      vw_out(0) << "\nCould not read in aligment matrix: " << m_out_prefix << "-align.exr.  Exiting. \n\n";
      exit(1);
    }
    
    result = stereo::disparity::transform_disparities(disparity_map, HomographyTransform(align_matrix));

    // Remove pixels that are outside the bounds of the secondary image.
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
    result = stereo::disparity::remove_invalid_pixels(result, right_disk_image.cols(), right_disk_image.rows());
  }
  
  write_image(output_file, result, TerminalProgressCallback(ErrorMessage, "\t    Processing: ") );
}


boost::shared_ptr<vw::camera::CameraModel> StereoSessionIsis::camera_model(std::string image_file, 
                                                                           std::string camera_file) {
  
  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    vw_out(0) << "\t--> Using adjusted Isis Camera Model: " << camera_file << "\n";

    // Creating Equations for the files
    boost::shared_ptr<PositionZeroOrder> posF( new PositionZeroOrder() );
    boost::shared_ptr<PoseZeroOrder> poseF( new PoseZeroOrder() );
    std::ifstream input( camera_file.c_str() );
    double val;
    for ( unsigned n = 0; n < posF->size(); ++n) {
      input >> val;
      posF->set_constant(n,val);
    }
    for ( unsigned n = 0; n < poseF->size(); ++n) {
      input >> val;
      poseF->set_constant(n,val);
    }
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<camera::CameraModel>(new IsisAdjustCameraModel( image_file, posF, poseF ));

  } else {
    vw_out(0) << "\t--> Using standard Isis camera model: " << image_file << "\n";
    return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
  }

}

