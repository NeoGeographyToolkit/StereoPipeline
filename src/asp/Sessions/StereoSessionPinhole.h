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
  
 class StereoSessionPinhole: public StereoSession {
  public:
    StereoSessionPinhole() {}
    virtual ~StereoSessionPinhole() {}

    virtual std::string name() const { return "pinhole"; }

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

    /// Override this function
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "");

    /// Specialized function to load both camera models and find their output sizes
    void load_camera_models(boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                            boost::shared_ptr<vw::camera::CameraModel> &right_cam,
                            vw::Vector2i &left_out_size, vw::Vector2i &right_out_size);

    /// Return the input camera models with no alignment applied.
    /// - This only matters in the epipolar alignment case, where the normal camera model
    ///   functions return the aligned camera models.
    void get_unaligned_camera_models(boost::shared_ptr<vw::camera::CameraModel> &left_cam,
                                     boost::shared_ptr<vw::camera::CameraModel> &right_cam);

    /// Transforms from pixel coordinates on disk to original unwarped image coordinates.
    /// - For reversing our arithmetic applied in preprocessing.
    typedef vw::HomographyTransform tx_type;
    tx_type tx_left () const;
    tx_type tx_right() const;

   typedef vw::stereo::StereoModel stereo_model_type;

   static bool isMapProjected() { return false; }

   // TODO: Need tx_left to return a pointer, and then the logic of the function below
   // needs to be incorporated into tx_left(). This because for epipolar alignment
   // the camera transform type is not a homography transform.
   void pinhole_cam_trans(PinholeCamTrans & left_trans, PinholeCamTrans & right_trans);
   
   // TODO: Clean these up!

   // Override the base class functions according to the class paramaters
   virtual bool uses_map_projected_inputs() const {return  isMapProjected();}
   virtual bool requires_input_dem       () const {return  isMapProjected();}
   virtual bool supports_image_alignment () const {return !isMapProjected();}
   virtual bool is_nadir_facing          () const {return false;}

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

  // TODO: Move this to a Pinhole loader class
  boost::shared_ptr<vw::camera::CameraModel>
  load_adj_pinhole_model(std::string const& image_file, std::string const& camera_file,
                         std::string const& left_image_file, std::string const& right_image_file,
                         std::string const& left_camera_file, std::string const& right_camera_file,
                         std::string const& input_dem);

}

#endif // __STEREO_SESSION_PINHOLE_H__
