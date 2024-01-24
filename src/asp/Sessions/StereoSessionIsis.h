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


/// \file StereoSessionIsis.h
///

#ifndef __STEREO_SESSION_ISIS_H__
#define __STEREO_SESSION_ISIS_H__

#include <asp/Sessions/StereoSession.h>
#include <vw/Stereo/StereoModel.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

namespace asp {

  /// Derived StereoSession class for ISIS images.
  class StereoSessionIsis : public StereoSession {
  public:
    StereoSessionIsis();
    virtual ~StereoSessionIsis() {}

    virtual std::string name() const { return "isis"; }
    
    virtual bool supports_multi_threading() const;
    
    /// Returns the target datum to use for a given camera model
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_non_earth) const;

    /// Stage 1: Preprocessing
    ///
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    virtual void preprocessing_hook(bool adjust_left_image_size,
                                        std::string const& left_input_file,
                                        std::string const& right_input_file,
                                        std::string      & left_output_file,
                                        std::string      & right_output_file);

    /// Stage 2: Correlation
    ///
    /// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
    /// Post file is a disparity map.            ( ImageView<PixelDisparity> > )
    virtual void pre_filtering_hook(std::string const& input_file,
                                    std::string      & output_file);

    /// Stage 4: Point cloud generation
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f> >
    pre_pointcloud_hook(std::string const& input_file);

    /// Simple factory function.
    static StereoSession* construct() { return new StereoSessionIsis; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const;
  };

} // end namespace asp

#endif  // ASP_HAVE_PKG_ISISIO

#endif // __STEREO_SESSION_ISIS_H__
