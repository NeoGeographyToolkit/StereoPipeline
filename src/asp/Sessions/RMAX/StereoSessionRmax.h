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


/// \file StereoSessionRMAX.h
///

#ifndef __RMAX_STEREO_SESSION_H__
#define __RMAX_STEREO_SESSION_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

namespace asp {
  class StereoSessionRmax : public StereoSession {
  public:
    virtual ~StereoSessionRmax() {}

    // Correct lens distortion and epipolar-rectify the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "");

    static StereoSession* construct() { return new StereoSessionRmax; }
  };
} // end namespace asp

#endif // __RMAX_STEREO_SESSION_H__
