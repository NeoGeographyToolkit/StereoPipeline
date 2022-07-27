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


/// \file StereoSessionPinhole.cc
///

#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Core/InterestPointMatching.h>

#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Stereo/DisparityMap.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

// This class assumes pinhole cameras with no datum, such as on a rover.
// For pinhole satellite images the nadirpinhole mode is suggested.

namespace asp {

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionPinhole::load_camera_model
  (std::string const& image_file, std::string const& camera_file, Vector2 pixel_offset) const{

  return load_adj_pinhole_model(image_file, camera_file,
                                m_left_image_file,  m_right_image_file,
                                m_left_camera_file, m_right_camera_file,
                                m_input_dem);
}

// Apply epipolar alignment to images, if the camera models are pinhole
void StereoSessionPinhole::
epipolar_alignment(vw::ImageViewRef<vw::PixelMask<float>> left_masked_image,
                   vw::ImageViewRef<vw::PixelMask<float>> right_masked_image,
                   vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata,
                   // Outputs
                   vw::ImageViewRef<vw::PixelMask<float>> & Limg, 
                   vw::ImageViewRef<vw::PixelMask<float>> & Rimg) {

  // Load the two images and fetch the two camera models
  boost::shared_ptr<camera::CameraModel> left_cam, right_cam;

  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
  if (boost::ends_with(lcase_file, ".pinhole") || boost::ends_with(lcase_file, ".tsai")) {
      
    // This loads epipolar-aligned camera models.
    // - The out sizes incorporate the crop amount if any, the camera models 
    Vector2i left_out_size, right_out_size;
    load_camera_models(left_cam, right_cam, left_out_size, right_out_size);
      
    // Get the input image crop regions, if any.
    BBox2i left_image_in_roi, right_image_in_roi;
    get_input_image_crops(left_image_in_roi, right_image_in_roi);

    // Write out the camera models used to generate the aligned images.
    // - Currently this won't work if we used .adjust files from bundle_adjust.
    PinholeModel* left_pin_model  = dynamic_cast<PinholeModel*>(left_cam.get ());
    PinholeModel* right_pin_model = dynamic_cast<PinholeModel*>(right_cam.get());
    if (left_pin_model)
      left_pin_model->write(m_out_prefix + "-L.tsai");
    if (right_pin_model)
      right_pin_model->write(m_out_prefix + "-R.tsai");

    // Transform the input images to be as if they were captured by the
    //  epipolar-aligned camera models, aligning the two images.
    get_epipolar_transformed_pinhole_images(m_left_camera_file, m_right_camera_file,
                                            left_cam, right_cam,
                                            left_masked_image, right_masked_image,
                                            left_image_in_roi, right_image_in_roi,
                                            left_out_size, right_out_size,
                                            Limg, Rimg,
                                            ext_nodata,
                                            BilinearInterpolation());

  } else { // Handle CAHV derived models
    camera_models(left_cam, right_cam);
    get_epipolar_transformed_images(m_left_camera_file, m_right_camera_file,
                                    left_cam, right_cam,
                                    left_masked_image, right_masked_image,
                                    Limg, Rimg, ext_nodata);
  }
}

void StereoSessionPinhole::get_unaligned_camera_models(
                                 boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                                 boost::shared_ptr<vw::camera::CameraModel> &right_cam) const{

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 left_pixel_offset  = camera_pixel_offset(m_input_dem, m_left_image_file,
                                                       m_right_image_file, m_left_image_file);
  vw::Vector2 right_pixel_offset = camera_pixel_offset(m_input_dem, m_left_image_file,
                                                       m_right_image_file, m_right_image_file);

  // Load the camera models adjusted for cropping
  left_cam  = load_adjusted_model(vw::camera::load_pinhole_camera_model(m_left_camera_file),
                                  m_left_image_file, m_left_camera_file, left_pixel_offset);
  right_cam = load_adjusted_model(vw::camera::load_pinhole_camera_model(m_right_camera_file),
                                  m_right_image_file, m_right_camera_file, right_pixel_offset);
}


boost::shared_ptr<vw::camera::CameraModel>
StereoSessionPinhole::load_adj_pinhole_model(std::string const& image_file,
                                             std::string const& camera_file,
                                             std::string const& left_image_file,
                                             std::string const& right_image_file,
                                             std::string const& left_camera_file,
                                             std::string const& right_camera_file,
                                             std::string const& input_dem){

  // Unfortunately the pinhole case is more complicated since the left
  // and right files are inter-dependent.

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = camera_pixel_offset(input_dem,
                                                 left_image_file,
                                                 right_image_file,
                                                 image_file);
  
  if ( stereo_settings().alignment_method != "epipolar" ) {
    // Not epipolar, just load the camera model.
    return load_adjusted_model(vw::camera::load_pinhole_camera_model(camera_file),
                               image_file, camera_file, pixel_offset);
  }
  // Otherwise handle the epipolar case

  bool is_left_camera = true;
  if (image_file == left_image_file)
    is_left_camera = true;
  else if (image_file == right_image_file)
    is_left_camera = false;
  else
    (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename "
     << "does not match the name supplied in the constructor.");

  std::string lcase_file = boost::to_lower_copy(left_camera_file);
  if (boost::ends_with(lcase_file, ".pinhole") || boost::ends_with(lcase_file, ".tsai")) {
    // Use PinholeModel epipolar code

    PinholeModel left_pin (left_camera_file );
    PinholeModel right_pin(right_camera_file);

    // Create epipolar rectified camera views
    boost::shared_ptr<PinholeModel> epipolar_left_pin (new PinholeModel);
    boost::shared_ptr<PinholeModel> epipolar_right_pin(new PinholeModel);
    epipolar(left_pin,  right_pin, *epipolar_left_pin, *epipolar_right_pin);

    // Expand epipolar cameras to contain the entire source images.
    Vector2i left_size  = file_image_size(left_image_file );
    Vector2i right_size = file_image_size(right_image_file);
    Vector2i epi_size1, epi_size2; // TODO: Use these!
    resize_epipolar_cameras_to_fit(left_pin, right_pin,
                                   *(epipolar_left_pin.get()), *(epipolar_right_pin.get()),
                                   BBox2i(Vector2i(0,0), left_size),
                                   BBox2i(Vector2i(0,0), right_size),
                                   epi_size1, epi_size2);

    if (is_left_camera)
      return load_adjusted_model(epipolar_left_pin, image_file, camera_file, pixel_offset);
    // Right camera
    return load_adjusted_model(epipolar_right_pin, image_file, camera_file, pixel_offset);
       
  } else { // Not PinholeModel, use CAHV epipolar code.

    // Fetch CAHV version of the two input pinhole files
    boost::shared_ptr<CAHVModel> left_cahv
      = vw::camera::load_cahv_pinhole_camera_model(left_image_file,  left_camera_file );
    boost::shared_ptr<CAHVModel> right_cahv
      = vw::camera::load_cahv_pinhole_camera_model(right_image_file, right_camera_file);

    // Create epipolar rectified camera views
    boost::shared_ptr<CAHVModel> epipolar_left_cahv (new CAHVModel);
    boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
    epipolar(*(left_cahv.get()),  *(right_cahv.get()),
             *epipolar_left_cahv, *epipolar_right_cahv);

    if (is_left_camera)
      return load_adjusted_model(epipolar_left_cahv, image_file, camera_file, pixel_offset);
    // Right camera
    return load_adjusted_model(epipolar_right_cahv, image_file, camera_file, pixel_offset);
  }
}

void StereoSessionPinhole::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                         boost::shared_ptr<vw::camera::CameraModel> &cam2) {
  vw::Vector2i left_out_size, right_out_size;
  load_camera_models(cam1, cam2, left_out_size, right_out_size);
}

void StereoSessionPinhole::load_camera_models(
                   boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                   boost::shared_ptr<vw::camera::CameraModel> &right_cam,
                   Vector2i &left_out_size, Vector2i &right_out_size) {

  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
  if ( (stereo_settings().alignment_method != "epipolar") ||
       ( !boost::ends_with(lcase_file, ".pinhole") &&
         !boost::ends_with(lcase_file, ".tsai"   )   ) ) {
    // Non-PinholeModel and non-epipolar case, just use the simpler handling method
    // and leave the sizes unset, they won't be used.
    left_cam  = camera_model(m_left_image_file,  m_left_camera_file );
    right_cam = camera_model(m_right_image_file, m_right_camera_file);
    return;
  }

  // PinholeModel case is more complicated. The camera models
  // returned do not include a crop offset, but the aligned camera
  // models have been shifted so that they are aligned after the crop
  // has been applied.

  PinholeModel left_pin (m_left_camera_file );
  PinholeModel right_pin(m_right_camera_file);

  // Create epipolar rectified camera views
  boost::shared_ptr<PinholeModel> epipolar_left_pin (new PinholeModel);
  boost::shared_ptr<PinholeModel> epipolar_right_pin(new PinholeModel);
  epipolar(left_pin,  right_pin, *epipolar_left_pin, *epipolar_right_pin);

  // Get the input image crop regions, if any.
  BBox2i left_bbox, right_bbox;
  get_input_image_crops(left_bbox, right_bbox);

  // Shift the epipolar cameras to line up with the top left corner of the image and also
  //  get the pixel sizes of -L.tif and -R.tif.
  // - These camera model still represent the entire image as if no cropping occurred.
  resize_epipolar_cameras_to_fit(left_pin, right_pin,
                                 *(epipolar_left_pin.get()), *(epipolar_right_pin.get()),
                                 left_bbox, right_bbox, left_out_size, right_out_size);

  // The pinhole epipolar case is incompatible with adjustment files so they are not loaded.
  left_cam  = epipolar_left_pin;
  right_cam = epipolar_right_pin;
}

// Return the left transform used in alignment
StereoSessionPinhole::tx_type StereoSessionPinhole::tx_left() const {

  if (stereo_settings().alignment_method != "epipolar")
    return StereoSession::tx_left_homography();
  
  // TODO(oalexan1): Figure out if things can work without casting
  // away the const.
  StereoSession::tx_type trans_left, trans_right;
  ((StereoSessionPinhole*)this)->pinhole_cam_trans(trans_left, trans_right);
  return trans_left;
}

// Return the right transform used in alignment
StereoSessionPinhole::tx_type StereoSessionPinhole::tx_right() const {

  if (stereo_settings().alignment_method != "epipolar")
    return StereoSession::tx_right_homography();
  
  // TODO(oalexan1): Figure out if things can work without casting
  // away the const.
  StereoSession::tx_type trans_left, trans_right;
  ((StereoSessionPinhole*)this)->pinhole_cam_trans(trans_left, trans_right);
  return trans_right;
}

void StereoSessionPinhole::pinhole_cam_trans(tx_type & left_trans,
                                             tx_type & right_trans) {

  // Load the epipolar aligned camera models
  boost::shared_ptr<camera::CameraModel> left_aligned_model, right_aligned_model;

  // TODO(oalexan1): Models must be loaded by now, presumably. Then
  // pinhole_cam_trans can be const, and we don't to cast away the const in tx_left()
  // and tx_right() above.
  this->camera_models(left_aligned_model, right_aligned_model);
  
  boost::shared_ptr<camera::CameraModel> left_input_model, right_input_model;
  this->get_unaligned_camera_models(left_input_model, right_input_model);

  // Check the type, CAHV* type models are not supported!
  typedef vw::camera::PinholeModel PinModel;
  PinModel* left_in_ptr = dynamic_cast<PinModel*>(&(*left_input_model));
  if (!left_in_ptr)
    vw_throw(NoImplErr() << "Detected CAHV-type cameras. Use an alignment "
             << "method different than 'epipolar'.\n" );

  // Set up transform objects
  left_trans.reset (new PinholeCamTrans(*dynamic_cast<PinModel*>(&(*left_input_model)),
                                        *dynamic_cast<PinModel*>(&(*left_aligned_model))));
  right_trans.reset(new PinholeCamTrans(*dynamic_cast<PinModel*>(&(*right_input_model)),
                                        *dynamic_cast<PinModel*>(&(*right_aligned_model))));
}

} // end namespace asp

