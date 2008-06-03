#include <boost/shared_ptr.hpp>

#include "StereoSessionPinhole.h"
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Camera/CameraTransform.h>

using namespace vw;
using namespace vw::camera;

boost::shared_ptr<vw::camera::CameraModel> StereoSessionPinhole::camera_model(std::string image_file, 
                                                                              std::string camera_file) {
  bool is_left_camera = true;
  if (camera_file == m_left_camera_file) 
    is_left_camera = true;
  else if (camera_file == m_right_camera_file) 
    is_left_camera = false;
  else 
    (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

  // Load the image
  DiskImageView<PixelGray<float> > left_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_image(m_right_image_file);

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

void StereoSessionPinhole::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                  std::string &output_file1, std::string &output_file2) {

  // Load the images
  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);

  // Load the two images and fetch the two camera models  
  boost::shared_ptr<camera::CameraModel> left_camera = this->camera_model(input_file1, m_left_camera_file);
  boost::shared_ptr<camera::CameraModel> right_camera = this->camera_model(input_file2, m_right_camera_file);
  CAHVModel* left_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*left_camera));
  CAHVModel* right_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*right_camera));

  // Remove lens distortion and create epipolar rectified images.
  ImageViewRef<PixelGray<float> > Limg, Rimg;
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
    vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported cameara file type.\n");
  }

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";
  write_image(output_file1, channel_cast_rescale<uint8>(Limg));
  write_image(output_file2, channel_cast_rescale<uint8>(Rimg));
}

