#include <boost/shared_ptr.hpp>

#include "StereoSessionPinhole.h"
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Camera/CameraTransform.h>

using namespace vw;
using namespace vw::camera;

void StereoSessionPinhole::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                         boost::shared_ptr<camera::CameraModel> &cam2) {
    
    // Load the two images
    DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);

    // Read Camera Models, removing lens distortion if necessary.
    CAHVModel left_cahv, right_cahv;
    if ((boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahvor") &&
         boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".cahvor"))  || 
        (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cmod") &&
         boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".cmod"))  ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      left_cahv = linearize_camera(left_cahvor, left_disk_image.cols(), left_disk_image.rows(), left_disk_image.cols(), left_disk_image.rows());
      right_cahv = linearize_camera(right_cahvor, left_disk_image.cols(), left_disk_image.rows(), left_disk_image.cols(), left_disk_image.rows());
      
    } else if ( (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahv") &&
                 boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".cahv")) || 
                (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".pin") &&
                 boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".pin")) ) {
      
      left_cahv = CAHVModel(m_left_camera_file);
      right_cahv = CAHVModel(m_right_camera_file);
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported cameara file type.\n");
    }
        
    // Epipolar rectify the two camera models
    camera::CAHVModel* left_result = new CAHVModel;
    camera::CAHVModel* right_result = new CAHVModel;
    epipolar(left_cahv, right_cahv, *left_result, *right_result);

    // Save the recitified camera model in the return value.
    cam1 = boost::shared_ptr<camera::CameraModel>(left_result);
    cam2 = boost::shared_ptr<camera::CameraModel>(right_result);
  }


void StereoSessionPinhole::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                  std::string &output_file1, std::string &output_file2) {
  
    boost::shared_ptr<camera::CameraModel> left_camera, right_camera;
    this->camera_models(left_camera, right_camera);
    CAHVModel* left_cahv = dynamic_cast<CAHVModel*>(&(*left_camera));
    CAHVModel* right_cahv = dynamic_cast<CAHVModel*>(&(*right_camera));

    if (!left_cahv || !right_cahv) 
      vw_throw(LogicErr() << "PinholeStereoSession: an error occurred when converting to the CAHV camera model.");

    // Load the two images
    DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);

    // Read Camera Models, removing lens distortion if necessary.
    std::cout << "Removing lens distortion and applying epipolar alignment.\n";
    if ((boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahvor") &&
         boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".cahvor"))  || 
        (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cmod") &&
         boost::ends_with(boost::to_lower_copy(m_right_camera_file), ".cmod"))  ) {

      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      ImageView<PixelGray<float> > Limg = transform(left_disk_image, CameraTransform<CAHVORModel, CAHVModel>(left_cahvor, *left_cahv));
      ImageView<PixelGray<float> > Rimg = transform(right_disk_image, CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, *right_cahv));

      output_file1 = m_out_prefix + "-L.tif";
      output_file2 = m_out_prefix + "-R.tif";
      write_image(output_file1, channel_cast_rescale<uint8>(Limg));
      write_image(output_file2, channel_cast_rescale<uint8>(Rimg));
    } else if ( ((boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".cahv") &&
                  boost::ends_with(boost::to_lower_copy(m_right_image_file), ".cahv")) || 
                 (boost::ends_with(boost::to_lower_copy(m_left_camera_file), ".pin") &&
                  boost::ends_with(boost::to_lower_copy(m_right_image_file), ".pin"))) ) {
      CAHVModel left_unrectified_cahv(m_left_camera_file);
      CAHVModel right_unrectified_cahv(m_right_image_file);

      ImageViewRef<PixelGray<float> > Limg = transform(left_disk_image, CameraTransform<CAHVModel, CAHVModel>(left_unrectified_cahv, *left_cahv));
      ImageViewRef<PixelGray<float> > Rimg = transform(right_disk_image, CameraTransform<CAHVModel, CAHVModel>(right_unrectified_cahv, *right_cahv));
      output_file1 = m_out_prefix + "-L.tif";
      output_file2 = m_out_prefix + "-R.tif";
      write_image(output_file1, channel_cast_rescale<uint8>(Limg));
      write_image(output_file2, channel_cast_rescale<uint8>(Rimg));
    } else {
      output_file1 = input_file1;
      output_file2 = input_file2;
    }
  }
