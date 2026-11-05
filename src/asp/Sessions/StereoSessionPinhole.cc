// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/EpipolarUtils.h>
#include <vw/Stereo/DisparityMap.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;

// This class assumes pinhole cameras with no datum, such as on a rover.
// For pinhole satellite images the nadirpinhole mode is suggested.

namespace asp {

vw::CamPtr load_adj_pinhole_model(std::string const& image_file,
                                  std::string const& camera_file,
                                  std::string const& left_image_file,
                                  std::string const& right_image_file,
                                  std::string const& left_camera_file,
                                  std::string const& right_camera_file,
                                  std::string const& ba_prefix,
                                  bool isMapProjected) { 

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset 
    = asp::camera_pixel_offset(isMapProjected, left_image_file, right_image_file, 
                                         image_file);
  
  // The rectification for epipolar alignment is applied as a transform (see
  // tx_left/tx_right), not baked into the camera model, consistent with the other
  // alignment methods. So always return the unaligned camera model.
  std::string lcase_file = boost::to_lower_copy(camera_file);
  bool is_pinhole = boost::ends_with(lcase_file, ".pinhole") ||
                    boost::ends_with(lcase_file, ".tsai");

  if (stereo_settings().alignment_method == "epipolar" && !is_pinhole)
    // CAHV-type camera: use the linearized CAHV model, which is what the epipolar
    // rectification is built from.
    return asp::load_adjusted_model(
      vw::load_cahv_pinhole_camera_model(image_file, camera_file),
      image_file, camera_file, ba_prefix, pixel_offset);

  return asp::load_adjusted_model(
    vw::load_pinhole_camera_model(camera_file),
    image_file, camera_file, ba_prefix, pixel_offset);
}

// The pinhole session is used with rovers and other non-satellite cameras,
// so there is no datum, unless explicitly provided.
bool StereoSessionPinhole::have_datum() const {
  return !stereo_settings().datum.empty() && 
         StereoSession::have_datum();
}

boost::shared_ptr<vw::CameraModel>
StereoSessionPinhole::load_camera_model
  (std::string const& image_file, std::string const& camera_file, 
   std::string const& ba_prefix, Vector2 pixel_offset) const {

  return load_adj_pinhole_model(image_file, camera_file,
                                m_left_image_file,  m_right_image_file,
                                m_left_camera_file, m_right_camera_file,
                                ba_prefix, isMapProjected());
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
  boost::shared_ptr<vw::CameraModel> left_cam, right_cam;

  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
  if (boost::ends_with(lcase_file, ".pinhole") || boost::ends_with(lcase_file, ".tsai")) {
      
    // This loads epipolar-aligned camera models.
    // - The out sizes incorporate the crop amount if any, the camera models
    Vector2i left_out_size, right_out_size;
    load_aligned_camera_models(left_cam, right_cam, left_out_size, right_out_size);
      
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
    epipolar_transformed_images(m_left_camera_file, m_right_camera_file,
                                left_cam, right_cam,
                                left_masked_image, right_masked_image,
                                left_image_in_roi, right_image_in_roi,
                                left_out_size, right_out_size,
                                Limg, Rimg,
                                ext_nodata);

  } else { // Handle CAHV derived models
    Vector2i left_out_size, right_out_size;
    load_aligned_camera_models(left_cam, right_cam, left_out_size, right_out_size);
    epipolar_transformed_cahv_images(m_left_camera_file, m_right_camera_file,
                                    left_cam, right_cam,
                                    left_masked_image, right_masked_image,
                                    Limg, Rimg, ext_nodata);
  }
}

void StereoSessionPinhole::get_unaligned_camera_models(
                                 boost::shared_ptr<vw::CameraModel> &left_cam,
                                 boost::shared_ptr<vw::CameraModel> &right_cam,
                                 std::string ba_prefix) const {

  // The unaligned, crop-adjusted camera models. Handles pinhole and CAHV uniformly.
  left_cam  = load_adj_pinhole_model(m_left_image_file, m_left_camera_file,
                                     m_left_image_file, m_right_image_file,
                                     m_left_camera_file, m_right_camera_file,
                                     ba_prefix, isMapProjected());
  right_cam = load_adj_pinhole_model(m_right_image_file, m_right_camera_file,
                                     m_left_image_file, m_right_image_file,
                                     m_left_camera_file, m_right_camera_file,
                                     ba_prefix, isMapProjected());
}


void StereoSessionPinhole::camera_models(boost::shared_ptr<vw::CameraModel> &cam1,
                                         boost::shared_ptr<vw::CameraModel> &cam2) {

  // For epipolar alignment return the unaligned input cameras (pinhole or CAHV). The
  // rectification is a transform (tx_left/tx_right), not baked into the cameras, and
  // triangulation un-does the transform and uses these cameras. The epipolar image warp
  // uses the raw cameras (no bundle adjustment), so the triangulation cameras match.
  if (stereo_settings().alignment_method == "epipolar") {
    get_unaligned_camera_models(cam1, cam2, "");
    return;
  }

  // Non-epipolar: the usual camera models, unchanged.
  cam1 = camera_model(m_left_image_file,  m_left_camera_file);
  cam2 = camera_model(m_right_image_file, m_right_camera_file);
}

// Load the epipolar-rectified (aligned) camera models and the sizes of the aligned
// images. These are the cameras used to warp the input images into -L.tif and -R.tif.
void StereoSessionPinhole::load_aligned_camera_models(
                   boost::shared_ptr<vw::CameraModel> &left_cam,
                   boost::shared_ptr<vw::CameraModel> &right_cam,
                   Vector2i &left_out_size, Vector2i &right_out_size) {

  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
  bool is_pinhole = boost::ends_with(lcase_file, ".pinhole") ||
                    boost::ends_with(lcase_file, ".tsai");

  if (!is_pinhole) {
    // CAHV-type cameras: build the aligned CAHV models used to warp the images. The
    // output sizes are not used for the CAHV warp.
    boost::shared_ptr<CAHVModel> left_cahv
      = vw::load_cahv_pinhole_camera_model(m_left_image_file,  m_left_camera_file);
    boost::shared_ptr<CAHVModel> right_cahv
      = vw::load_cahv_pinhole_camera_model(m_right_image_file, m_right_camera_file);
    boost::shared_ptr<CAHVModel> epi_left (new CAHVModel);
    boost::shared_ptr<CAHVModel> epi_right(new CAHVModel);
    epipolar(*left_cahv, *right_cahv, *epi_left, *epi_right);
    left_cam = epi_left;
    right_cam = epi_right;
    left_out_size = right_out_size = Vector2i();
    return;
  }

  // Pinhole epipolar. The aligned camera models are shifted so that they line up after
  // any crop has been applied.
  PinholeModel left_pin (m_left_camera_file);
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

// Return the left transform used in alignment. For epipolar alignment this maps an
// aligned-image pixel back to the unaligned input image, so triangulation can use the
// unaligned cameras, like the other alignment methods. Otherwise it is the usual
// homography transform.
vw::TransformPtr StereoSessionPinhole::tx_left() const {
  if (stereo_settings().alignment_method == "epipolar") {
    vw::TransformPtr trans_left, trans_right;
    ((StereoSessionPinhole*)this)->epipolar_cam_trans(trans_left, trans_right);
    return trans_left;
  }
  return StereoSession::tx_left_homography();
}

// Return the right transform used in alignment. See tx_left() above.
vw::TransformPtr StereoSessionPinhole::tx_right() const {
  if (stereo_settings().alignment_method == "epipolar") {
    vw::TransformPtr trans_left, trans_right;
    ((StereoSessionPinhole*)this)->epipolar_cam_trans(trans_left, trans_right);
    return trans_right;
  }
  return StereoSession::tx_right_homography();
}

void StereoSessionPinhole::epipolar_cam_trans(vw::TransformPtr & left_trans,
                                              vw::TransformPtr & right_trans) {

  // The epipolar-rectified (aligned) camera models, the same ones used to warp the
  // input images into the aligned -L.tif and -R.tif.
  boost::shared_ptr<vw::CameraModel> left_aligned_model, right_aligned_model;
  vw::Vector2i left_size, right_size;
  this->load_aligned_camera_models(left_aligned_model, right_aligned_model,
                                   left_size, right_size);

  // The unaligned input camera models. No bundle adjustment is applied, matching the
  // epipolar image warp and the cameras returned by camera_models() for triangulation.
  boost::shared_ptr<vw::CameraModel> left_input_model, right_input_model;
  this->get_unaligned_camera_models(left_input_model, right_input_model, "");

  // Build a transform whose reverse() maps an aligned-image pixel back to the
  // unaligned input-image pixel (source = input camera, destination = aligned camera).
  // vw::epipolar preserves each camera center, so this exactly reproduces the ray the
  // aligned camera would cast, letting triangulation use the unaligned cameras. The
  // smart-pointer form works for any camera type, including cropped (adjusted) models.
  left_trans.reset (new vw::SmartPtrCameraTransform(left_input_model,  left_aligned_model));
  right_trans.reset(new vw::SmartPtrCameraTransform(right_input_model, right_aligned_model));
}

} // end namespace asp
