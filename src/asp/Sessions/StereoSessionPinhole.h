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


/// \file StereoSessionPinhole.h
///

#ifndef __STEREO_SESSION_PINHOLE_H__
#define __STEREO_SESSION_PINHOLE_H__

#include <asp/Sessions/StereoSession.h>
#include <vw/Camera/CameraTransform.h>
#include <vw/Camera/PinholeModel.h>

namespace asp {

// Pinhole camera model loading function which handles the case of epipolar alignment.
vw::CamPtr load_adj_pinhole_model(std::string const& image_file,      
                                  std::string const& camera_file,
                                  std::string const& left_image_file,
                                  std::string const& right_image_file,
                                  std::string const& left_camera_file, 
                                  std::string const& right_camera_file,
                                  std::string const& ba_prefix, 
                                  bool isMapProjected);

 class StereoSessionPinhole: public StereoSession {
  public:
    StereoSessionPinhole() {}
    virtual ~StereoSessionPinhole() {}

    virtual std::string name() const { return "pinhole"; }

    // The pinhole session is used with rovers and other non-satellite cameras,
    // so there is no datum, unless explicitly provided.
    virtual bool have_datum() const;

    static SessionPtr construct() { return SessionPtr(new StereoSessionPinhole); }


    /// Return the camera models used for triangulation. For epipolar alignment with
    /// pinhole cameras these are the unaligned input cameras (the rectification is a
    /// transform, see tx_left/tx_right); for the legacy CAHV epipolar path they are the
    /// aligned cameras.
    virtual void camera_models(boost::shared_ptr<vw::CameraModel> &cam1,
                               boost::shared_ptr<vw::CameraModel> &cam2);

    /// Load the epipolar-rectified (aligned) camera models and the sizes of the aligned
    /// images. These are the cameras used to warp the input images into -L.tif/-R.tif.
    void load_aligned_camera_models(boost::shared_ptr<vw::CameraModel> &left_cam,
                                    boost::shared_ptr<vw::CameraModel> &right_cam,
                                    vw::Vector2i &left_out_size, vw::Vector2i &right_out_size);

    /// Return the input camera models with no alignment applied.
    void get_unaligned_camera_models(boost::shared_ptr<vw::CameraModel> &left_cam,
                                     boost::shared_ptr<vw::CameraModel> &right_cam,
                                     std::string ba_prefix) const;

    /// Transforms from the aligned image coordinates back to coordinates in the camera
    /// models. For epipolar alignment this is a camera transform mapping aligned pixels
    /// to the unaligned input cameras; otherwise it is the usual homography transform.
    virtual vw::TransformPtr tx_left () const;
    virtual vw::TransformPtr tx_right() const;

    /// Build the transforms mapping the epipolar aligned images back to the unaligned
    /// input cameras. Works for pinhole and CAHV-type cameras.
    void epipolar_cam_trans(vw::TransformPtr & left_trans, vw::TransformPtr & right_trans);
    
  protected:
    /// Function to load a camera model of the particular type.
   virtual boost::shared_ptr<vw::CameraModel>
   load_camera_model(std::string const& image_file, 
                     std::string const& camera_file,
                     std::string const& ba_prefix, 
                     vw::Vector2 pixel_offset) const;

   // Apply epipolar alignment to images, if the camera models are pinhole
   virtual void epipolar_alignment(vw::ImageViewRef<vw::PixelMask<float>> left_masked_image,
                                   vw::ImageViewRef<vw::PixelMask<float>> right_masked_image,
                                   vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata,
                                   // Outputs
                                   vw::ImageViewRef<vw::PixelMask<float>> & Limg, 
                                   vw::ImageViewRef<vw::PixelMask<float>> & Rimg);
 };

}

#endif // __STEREO_SESSION_PINHOLE_H__
