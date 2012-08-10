// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file StereoSessionDG.h
///
/// This a session to hopefully support Digital Globe images from
/// Quickbird and World View.

#ifndef __STEREO_SESSION_DG_H__
#define __STEREO_SESSION_DG_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

namespace asp {

  // Forward declaration
  class RPCModel;
  
  class StereoSessionDG : public StereoSessionPinhole {

  public:
    StereoSessionDG();
    virtual ~StereoSessionDG();

    // Initializer that determines if our input images are map
    // projected or if this is just a straight Digitial Globe session.
    virtual void initialize(BaseOptions const& options,
                            std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file,
                            std::string const& out_prefix,
                            std::string const& extra_argument1,
                            std::string const& extra_argument2,
                            std::string const& extra_argument3,
                            std::string const& extra_argument4);
    
    // Produces a camera model from the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model( std::string const& image_file,
                  std::string const& camera_file = "" );

    // LUT access (enabled only when working with images map projected by RPC)
    virtual bool has_lut_images() const;
    virtual vw::ImageViewRef<vw::Vector2f> lut_image_left() const;
    virtual vw::ImageViewRef<vw::Vector2f> lut_image_right() const;

    // Stage 1: Preprocessing
    //
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    // Post file is a pair of grayscale images. ( ImageView<PixelGray<flaot> > )
    virtual void pre_preprocessing_hook(std::string const& input_file1,
                                        std::string const& input_file2,
                                        std::string &output_file1,
                                        std::string &output_file2);

    static StereoSession* construct() { return new StereoSessionDG; }

  protected:

    bool m_rpc_map_projected;

    vw::ImageViewRef<vw::Vector2f> generate_lut_image( std::string const&, std::string const& ) const;
    static RPCModel* read_rpc_model( std::string const& image_file, std::string const& camera_file );
    
  };

}

#endif//__STEREO_SESSION_DG_H__
