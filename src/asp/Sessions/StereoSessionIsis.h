// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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
#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

namespace asp {

/// Derived StereoSession class for ISIS images
class StereoSessionIsis: public StereoSession {
public:

  virtual ~StereoSessionIsis() {}

  virtual std::string name() const { return "isis"; }

  virtual bool supports_multi_threading() const;

  /// Returns the target datum to use for a given camera model
  virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                            bool use_sphere_for_non_earth) const;

  /// Simple factory function
  static SessionPtr construct() { return SessionPtr(new StereoSessionIsis); }

protected:
  // Find the masked images and stats. Reimplemented for ISIS to handle special pixels.
  virtual void calcStatsMaskedImages(// Inputs
                                     vw::ImageViewRef<float> const& left_cropped_image,
                                     vw::ImageViewRef<float> const& right_cropped_image,
                                     float left_nodata_value, float right_nodata_value,
                                     std::string const& left_input_file,
                                     std::string const& right_input_file,
                                     std::string const& left_cropped_file,
                                     std::string const& right_cropped_file,
                                     // Outputs
                                     vw::ImageViewRef<vw::PixelMask<float>> & left_masked_image,
                                     vw::ImageViewRef<vw::PixelMask<float>> & right_masked_image,
                                     vw::Vector6f & left_stats, 
                                     vw::Vector6f & right_stats) const;

  /// Function to load a camera model of the particular type
  virtual boost::shared_ptr<vw::camera::CameraModel>
  load_camera_model(std::string const& image_file,
                    std::string const& camera_file,
                    std::string const& ba_prefix,
                    vw::Vector2 pixel_offset) const;
};

} // end namespace asp

#endif  // ASP_HAVE_PKG_ISIS

#endif // __STEREO_SESSION_ISIS_H__
