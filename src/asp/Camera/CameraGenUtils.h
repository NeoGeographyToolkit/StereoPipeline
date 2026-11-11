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

/// \file CameraGenUtils.h
///
/// Generic camera-construction helpers used by cam_gen. These are kept free of
/// the cam_gen Options struct so the logic can live in the Camera library.

#ifndef __ASP_CAMERA_CAMERA_GEN_UTILS_H__
#define __ASP_CAMERA_CAMERA_GEN_UTILS_H__

#include <vw/Math/Vector.h>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>

namespace vw {
  class CameraModel;
}

namespace asp {

/// Build an initial camera of the given image dimensions. For camera_type
/// "pinhole", use the intrinsics passed in, or read them from a sample file
/// (.tsai, or a CSM .json whose intrinsics are read back through the in-out
/// parameters). For "opticalbar", read the sample optical bar camera and set its
/// image size and optical center.
void manufacture_cam(std::string const& camera_type,
                     std::string const& sample_file,
                     int wid, int hgt,
                     double & focal_length,
                     double & pixel_pitch,
                     vw::Vector2 & optical_center,
                     std::vector<double> & distortion,
                     std::string & distortion_type,
                     boost::shared_ptr<vw::CameraModel> & out_cam);

} // end namespace asp

#endif // __ASP_CAMERA_CAMERA_GEN_UTILS_H__
