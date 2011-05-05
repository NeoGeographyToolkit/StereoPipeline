// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionPinhole.cc
///

// Ames Stereo Pipeline
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

// Vision Workbench
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/CAHVOREModel.h>
#include <vw/Camera/CameraTransform.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Math.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionPinhole::camera_model(std::string /*image_file*/,
                                   std::string camera_file) {
  // Epipolar Alignment
  if ( stereo_settings().epipolar_alignment ) {
    // Load the image
    DiskImageView<PixelGray<float> > left_image(m_left_image_file);
    DiskImageView<PixelGray<float> > right_image(m_right_image_file);
    Vector2i left_image_size( left_image.cols(), left_image.rows() ),
      right_image_size( right_image.cols(), right_image.rows() );

    bool is_left_camera = true;
    if (camera_file == m_left_camera_file)
      is_left_camera = true;
    else if (camera_file == m_right_camera_file)
      is_left_camera = false;
    else
      (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

    // Return the appropriate camera model object
    std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
    CAHVModel left_cahv, right_cahv;
    if (boost::ends_with(lcase_file, ".cahvore") ) {
      CAHVOREModel left_cahvore(m_left_camera_file);
      CAHVOREModel right_cahvore(m_right_camera_file);
      left_cahv =
        linearize_camera(left_cahvore, left_image_size, left_image_size);
      right_cahv =
        linearize_camera(right_cahvore, right_image_size, right_image_size);
    } else if (boost::ends_with(lcase_file, ".cahvor")  ||
               boost::ends_with(lcase_file, ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      left_cahv =
        linearize_camera(left_cahvor, left_image_size, left_image_size);
      right_cahv =
        linearize_camera(right_cahvor, right_image_size, right_image_size);
    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      left_cahv = CAHVModel(m_left_camera_file);
      right_cahv = CAHVModel(m_right_camera_file);

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      left_cahv = linearize_camera(left_pin);
      right_cahv = linearize_camera(right_pin);
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

    // Create epipolar recitified camera views
    boost::shared_ptr<CAHVModel> epipolar_left_cahv(new CAHVModel);
    boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
    epipolar(left_cahv, right_cahv, *epipolar_left_cahv, *epipolar_right_cahv);

    if (is_left_camera)
      return epipolar_left_cahv;
    else
      return epipolar_right_cahv;
  } else {
    // Keypoint alignment and everything else just gets camera models
    std::string lcase_file = boost::to_lower_copy(camera_file);
    if (boost::ends_with(lcase_file,".cahvore") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVOREModel(camera_file) );
    } else if (boost::ends_with(lcase_file,".cahvor") ||
               boost::ends_with(lcase_file,".cmod") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVORModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".cahv") ||
                boost::ends_with(lcase_file,".pin") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".pinhole") ||
                boost::ends_with(lcase_file,".tsai") ) {
      return boost::shared_ptr<vw::camera::CameraModel> ( new PinholeModel(camera_file) );
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }


  }
  return boost::shared_ptr<vw::camera::CameraModel>(); // Never reached
}

void StereoSessionPinhole::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                  std::string &output_file1, std::string &output_file2) {

  // Load the images
  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelGray<float> > Limg, Rimg;
  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

  if ( stereo_settings().epipolar_alignment ) {

    vw_out() << "\t--> Performing epipolar alignment\n";

    // Load the two images and fetch the two camera models
    boost::shared_ptr<camera::CameraModel> left_camera =
      this->camera_model(input_file1, m_left_camera_file);
    boost::shared_ptr<camera::CameraModel> right_camera =
      this->camera_model(input_file2, m_right_camera_file);
    CAHVModel* left_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*left_camera));
    CAHVModel* right_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*right_camera));

    // Remove lens distortion and create epipolar rectified images.
    if (boost::ends_with(lcase_file, ".cahvore")) {
      CAHVOREModel left_cahvore(m_left_camera_file);
      CAHVOREModel right_cahvore(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<CAHVOREModel, CAHVModel>(left_cahvore, *left_epipolar_cahv));

      Rimg = transform(right_disk_image, CameraTransform<CAHVOREModel, CAHVModel>(right_cahvore, *right_epipolar_cahv));
    } else if (boost::ends_with(lcase_file, ".cahvor") ||
               boost::ends_with(lcase_file, ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<CAHVORModel, CAHVModel>(left_cahvor, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      CAHVModel left_cahv(m_left_camera_file);
      CAHVModel right_cahv(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<CAHVModel, CAHVModel>(left_cahv, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<CAHVModel, CAHVModel>(right_cahv, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      Limg = transform(left_disk_image, CameraTransform<PinholeModel, CAHVModel>(left_pin, *left_epipolar_cahv));
      Rimg = transform(right_disk_image, CameraTransform<PinholeModel, CAHVModel>(right_pin, *right_epipolar_cahv));

    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

  } else if ( stereo_settings().keypoint_alignment ) {

    Matrix<double> align_matrix;
    align_matrix = determine_image_align( input_file1, input_file2,
                                          left_disk_image, right_disk_image);
    write_matrix( m_out_prefix + "-align.exr", align_matrix );

    // Applying alignment transform
    Limg = left_disk_image;
    Rimg = transform(right_disk_image,
                     HomographyTransform(align_matrix),
                     left_disk_image.cols(), left_disk_image.rows());

  } else {
    // Do nothing just provide the original files.
    Limg = left_disk_image;
    Rimg = right_disk_image;
  }

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";
  vw_out() << "\t--> Writing pre-aligned images.\n";
  write_image(output_file1, Limg);
  write_image(output_file2, Rimg);
}

// Reverse any pre-alignment that might have been done to the disparity map
void StereoSessionPinhole::pre_pointcloud_hook(std::string const& input_file,
                                               std::string & output_file ) {

  if ( stereo_settings().keypoint_alignment ) {

    DiskImageView<PixelMask<Vector2f> > disparity_map( input_file );
    output_file = m_out_prefix + "-F-corrected.tif";

    vw::Matrix<double> align_matrix;
    try {
      read_matrix(align_matrix, m_out_prefix + "-align.exr");
      vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
    } catch ( vw::IOErr &e ) {
      vw_out() << "\nCould not read in alignment matrix: " << m_out_prefix
               << "-align.exr. Exiting. \n\n";
      exit(1);
    }

    // Remove pixels that are outside the bounds of the second image
    DiskImageView<PixelGray<float> > right_disk_image( m_right_image_file );
    ImageViewRef<PixelMask<Vector2f> > result =
      stereo::disparity_range_mask( stereo::transform_disparities( disparity_map,
                                             HomographyTransform(align_matrix)),
                                    Vector2f(0,0),
                                    Vector2f( right_disk_image.cols(),
                                              right_disk_image.rows()) );

    write_image(output_file, result,
                TerminalProgressCallback("asp", "\t    Saving: ") );

  } else {
    output_file = input_file;
  }
}
