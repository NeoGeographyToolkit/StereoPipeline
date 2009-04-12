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

/// \file StereoSessionPinhole.cc
///

// Ames Stereo Pipeline
#include "StereoSettings.h"
#include "StereoSessionPinhole.h"

// Vision Workbench
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/CameraTransform.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
using namespace boost::filesystem;

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

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

vw::math::Matrix<double> StereoSessionPinhole::determine_keypoint_alignment( std::string const& input_file1, std::string const& input_file2 ) {

  std::vector<InterestPoint> matched_ip1, matched_ip2;
  if ( boost::filesystem::exists( prefix_from_filename( input_file1 ) + "__" +
				  prefix_from_filename( input_file2 ) + ".match" ) ) {
    // Is there a match file linking these 2 images?
    
    vw_out(0) << "\t--> Found cached interest point match file: " 
	      << ( prefix_from_filename(input_file1) + "__" + 
		   prefix_from_filename(input_file2) + ".match" ) << "\n";
    read_binary_match_file( ( prefix_from_filename( input_file1 ) + "__" +
			      prefix_from_filename( input_file2 ) + ".match" ),
			    matched_ip1, matched_ip2 );
  } else {
    
    // Need to at least match the files
    std::vector<InterestPoint> ip1_copy, ip2_copy;
    
    if ( exists( prefix_from_filename(input_file1) + ".vwip") &&
	 exists( prefix_from_filename(input_file2) + ".vwip") ) {
      // Is there at least VWIP already done for both images?
      vw_out(0) << "\t--> Found cached interest point files: "
		<< ( prefix_from_filename(input_file1) + ".vwip" ) << "\n"
		<< "\t                                       "
		<< ( prefix_from_filename(input_file2) + ".vwip" ) << "\n";
      ip1_copy = read_binary_ip_file( prefix_from_filename(input_file1)+
				      ".vwip");
      ip2_copy = read_binary_ip_file( prefix_from_filename(input_file2)+
				      ".vwip");
      	
    } else {
      // Performing interest point detector
      vw_out(0) << "\t--> Locating Interest Points\n";
      InterestPointList ip1, ip2;
      DiskImageView<PixelGray<float> > left_disk_image( input_file1 );
      DiskImageView<PixelGray<float> > right_disk_image( input_file2 );
      ImageViewRef<PixelGray<float> > left_image = left_disk_image;
      ImageViewRef<PixelGray<float> > right_image = right_disk_image;
      
      // Interest Point Module Detector Code
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
		<< (prefix_from_filename(input_file1)+".vwip") << ", "
		<< (prefix_from_filename(input_file2)+".vwip") << "\n";
      write_binary_ip_file(prefix_from_filename(input_file1)+".vwip", ip1);
      write_binary_ip_file(prefix_from_filename(input_file2)+".vwip", ip2);
      
      // Reading back into the vector format
      ip1_copy = read_binary_ip_file( prefix_from_filename(input_file1)+
				      ".vwip");
      ip2_copy = read_binary_ip_file( prefix_from_filename(input_file2)+
				      ".vwip");
    }

    vw_out(0) << "\t--> Matching interest points\n";
    InterestPointMatcher<L2NormMetric,NullConstraint> matcher(0.8);
    
    matcher(ip1_copy, ip2_copy, 
	    matched_ip1, matched_ip2, 
	    false, 
	    TerminalProgressCallback( InfoMessage, "\t    Matching: "));

    vw_out(0) << "\t    Caching matches: " 
	      << ( prefix_from_filename(input_file1)+"__"+
		   prefix_from_filename(input_file2)+".match") << "\n";

    write_binary_match_file(prefix_from_filename(input_file1) + "__" +
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

    math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 10);
    T = ransac(ransac_ip2, ransac_ip1);
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

boost::shared_ptr<vw::camera::CameraModel> StereoSessionPinhole::camera_model(std::string image_file, 
                                                                              std::string camera_file) {
  // Epipolar Alignment
  if ( stereo_settings().epipolar_alignment ) {
    // Load the image
    DiskImageView<PixelGray<float> > left_image(m_left_image_file);
    DiskImageView<PixelGray<float> > right_image(m_right_image_file);

    bool is_left_camera = true;
    if (camera_file == m_left_camera_file) 
      is_left_camera = true;
    else if (camera_file == m_right_camera_file) 
      is_left_camera = false;
    else 
      (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

    // Return the appropriate camera model object
    CAHVModel left_cahv, right_cahv;
    if (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahvor")  || 
        boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      left_cahv = linearize_camera(left_cahvor, 
                                   left_image.cols(), left_image.rows(),
                                   left_image.cols(), left_image.rows());
      right_cahv = linearize_camera(right_cahvor, 
                                    right_image.cols(), right_image.rows(),
                                    right_image.cols(), right_image.rows());
    } else if ( boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahv") ||
                boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".pin" )) {
      left_cahv = CAHVModel(m_left_camera_file);
      right_cahv = CAHVModel(m_right_camera_file);

    } else if ( boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      left_cahv = linearize_camera(left_pin);
      right_cahv = linearize_camera(right_pin);

    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported cameara file type.\n");
    }

    // Create epipolar recitified camera views
    boost::shared_ptr<CAHVModel> epipolar_left_cahv(new CAHVModel);
    boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
    epipolar(left_cahv, right_cahv, *epipolar_left_cahv, *epipolar_right_cahv);

    if (is_left_camera)
      return epipolar_left_cahv;
    else 
      return epipolar_right_cahv;
  }

  // Keypoint alignment
  else if ( stereo_settings().keypoint_alignment ) {
    if (boost::ends_with(boost::to_lower_copy(camera_file),".cahvor") ||
	boost::ends_with(boost::to_lower_copy(camera_file),".cmod") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVORModel(camera_file) );
    } else if ( boost::ends_with(boost::to_lower_copy(camera_file),".cahv") ||
		boost::ends_with(boost::to_lower_copy(camera_file),".pin") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVModel(camera_file) );
    } else if ( boost::ends_with(boost::to_lower_copy(camera_file),"tsai") ) {
      return boost::shared_ptr<vw::camera::CameraModel> ( new PinholeModel(camera_file) );
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported cameara file type.\n");
    }

   
  } else {
    vw_throw(ArgumentErr() << "PinholeStereoSession: no alignment method was selected in your stereo.default file..\n");
  } 
  return boost::shared_ptr<vw::camera::CameraModel>(); // Never reached
}

void StereoSessionPinhole::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                  std::string &output_file1, std::string &output_file2) {

  // Load the images
  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelGray<float> > Limg, Rimg;

  if ( stereo_settings().epipolar_alignment ) {

    vw_out(0) << "\t--> Performing epipolar alignment\n";

    // Load the two images and fetch the two camera models  
    boost::shared_ptr<camera::CameraModel> left_camera = this->camera_model(input_file1, m_left_camera_file);
    boost::shared_ptr<camera::CameraModel> right_camera = this->camera_model(input_file2, m_right_camera_file);
    CAHVModel* left_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*left_camera));
    CAHVModel* right_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*right_camera));
    
    // Remove lens distortion and create epipolar rectified images.
    if (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahvor")  || 
	boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<CAHVORModel, CAHVModel>(left_cahvor, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, *right_epipolar_cahv));
      
    } else if ( boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahv") ||
		boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".pin" )) {
      CAHVModel left_cahv(m_left_camera_file);
      CAHVModel right_cahv(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<CAHVModel, CAHVModel>(left_cahv, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<CAHVModel, CAHVModel>(right_cahv, *right_epipolar_cahv));
      
    } else if ( boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<PinholeModel, CAHVModel>(left_pin, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<PinholeModel, CAHVModel>(right_pin, *right_epipolar_cahv));
      
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

  } else if ( stereo_settings().keypoint_alignment ) {

    Matrix<double> align_matrix(3,3);
    align_matrix = determine_keypoint_alignment( input_file1, input_file2 );
    write_matrix( m_out_prefix + "-align.exr", align_matrix );
    
    // Applying alignment transform
    Limg = left_disk_image;
    Rimg = transform(right_disk_image,
		     HomographyTransform(align_matrix),
		     left_disk_image.cols(), left_disk_image.rows());

  } else {
    vw_throw(ArgumentErr() << "PinholeStereoSession: no alignment method was selected in your stereo.default file..\n");
  }

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";
  vw_out(0) << "\t--> Writing pre-aligned images.\n";
  write_image(output_file1, channel_cast_rescale<uint8>(Limg));
  write_image(output_file2, channel_cast_rescale<uint8>(Rimg));
}

// Reverse any pre-alignment that might have been done to the disparity map
void StereoSessionPinhole::pre_pointcloud_hook(std::string const& input_file,
					       std::string & output_file ) {

  if ( stereo_settings().epipolar_alignment ) {
    output_file = input_file;
  } else if ( stereo_settings().keypoint_alignment ) {
    
    DiskImageView<PixelDisparity<float> > disparity_map( input_file );
    output_file = m_out_prefix + "-F-corrected.exr";
    ImageViewRef<PixelDisparity<float> > result;

    vw::Matrix<double> align_matrix;
    try {
      read_matrix(align_matrix, m_out_prefix + "-align.exr");
      vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
    } catch ( vw::IOErr &e ) {
      vw_out(0) << "\nCould not read in alignment matrix: " << m_out_prefix << "-align.exr. Exiting. \n\n";
      exit(1);
    }

    result = stereo::disparity::transform_disparities( disparity_map,HomographyTransform(align_matrix));

    // Remove pixels that are outside the bounds of the second image
    DiskImageView<PixelGray<float> > right_disk_image( m_right_image_file );
    result = stereo::disparity::remove_invalid_pixels( result, right_disk_image.cols(), right_disk_image.rows()); 

    write_image(output_file, result, TerminalProgressCallback(ErrorMessage, "\t    Saving: ") );

  } else {
    vw_throw(ArgumentErr() << "PinholeStereoSession: unselected alignment option.\n");
  }
}
