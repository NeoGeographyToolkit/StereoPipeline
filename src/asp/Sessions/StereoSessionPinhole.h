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

#include <asp/Core/InterestPointMatching.h>
#include <vw/Stereo/StereoModel.h>
#include <asp/Sessions/StereoSession.h>
#include <vw/Camera/CameraTransform.h>
#include <vw/Camera/PinholeModel.h>

namespace asp {

  typedef vw::camera::CameraTransform<vw::camera::PinholeModel, vw::camera::PinholeModel> PinholeCamTrans;
  typedef boost::shared_ptr<PinholeCamTrans> PinholeCamTransPtr;

 class StereoSessionPinhole: public StereoSession {
  public:
    StereoSessionPinhole() {}
    virtual ~StereoSessionPinhole() {}

    virtual std::string name() const { return "pinhole"; }

    virtual bool have_datum() const {return false;}

    // Stage 1: Preprocessing
    //
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    // Post file is a grayscale images.         ( ImageView<PixelGray<float> > )
    virtual void pre_preprocessing_hook(bool adjust_left_image_size,
                                        std::string const& left_input_file,
                                        std::string const& right_input_file,
                                        std::string      & left_output_file,
                                        std::string      & right_output_file);

    static StereoSession* construct() { return new StereoSessionPinhole; }


    /// Override this function to make sure it properly generates the
    ///  aligned camera models.
    virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                               boost::shared_ptr<vw::camera::CameraModel> &cam2);

    /// Specialized function to load both camera models and find their output sizes
    void load_camera_models(boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                            boost::shared_ptr<vw::camera::CameraModel> &right_cam,
                            vw::Vector2i &left_out_size, vw::Vector2i &right_out_size);

    /// Return the input camera models with no alignment applied.
    /// - This only matters in the epipolar alignment case, where the normal camera model
    ///   functions return the aligned camera models.
    void get_unaligned_camera_models(boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                                     boost::shared_ptr<vw::camera::CameraModel> &right_cam) const;


    /// Pinhole camera model loading function which handles the case of epipolar alignment.
    static boost::shared_ptr<vw::camera::CameraModel>
    load_adj_pinhole_model(std::string const& image_file,      std::string const& camera_file,
                           std::string const& left_image_file,  std::string const& right_image_file,
                           std::string const& left_camera_file, std::string const& right_camera_file,
                           std::string const& input_dem);

    /// Transforms from the aligned image coordinates back to coordinates in the camera models.
    /// - Note that for epipolar aligned images these return identity transforms since the 
    ///   epipolar aligned images are consisted with the (new epipolar) camera models returned
    ///   from this class.
    virtual tx_type tx_left () const;
    virtual tx_type tx_right() const;

    /// Get the transforms from the unaligned input images to the epipolar aligned images.
    /// - CAHV* type models are not currently supported!
    void pinhole_cam_trans(tx_type & left_trans, tx_type & right_trans);
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const;
 private:
    /// Helper function for determining image alignment.
    /// - Only used in pre_preprocessing_hook()
    vw::Matrix3x3 determine_image_align( std::string  const& out_prefix,
                                         std::string  const& input_file1,
                                         std::string  const& input_file2,
                                         vw::Vector2  const& uncropped_image_size,
                                         Vector6f const& stats1,
                                         Vector6f const& stats2,
                                         float nodata1, float nodata2);
 };

}

#endif // __STEREO_SESSION_PINHOLE_H__
