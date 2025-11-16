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

/// \file BaParams.h
///

#ifndef __ASP_CAMERA_BA_PARAMS_H__
#define __ASP_CAMERA_BA_PARAMS_H__

#include <string>
#include <vector>

namespace asp {

/// Structure to fully describe how the intrinsics are being handled.
/// - Currently only pinhole cameras support intrinsics in bundle_adjust.
struct IntrinsicOptions {
  
  // If to share these intrinsics. Can be per group of cameras or for all cameras.
  bool center_shared;
  bool focus_shared;
  bool distortion_shared;
  
  // If to float these intrinsics. All, none, or per sensor
  std::vector<bool> float_center, float_focus, float_distortion;
  
  bool share_intrinsics_per_sensor;
  std::vector<int> cam2sensor; // cam index to sensor index, when sharing intrinsics per sensor
  int num_sensors; // will be nonzero only if sharing intrinsics per sensor is true
  IntrinsicOptions();

  // Control per each group of cameras or for all cameras which intrinsics
  // should be floated.
  bool float_optical_center(int cam_index) const;
  bool float_focal_length(int cam_index) const;
  bool float_distortion_params(int cam_index) const;
};


} // end namespace asp

#endif // __ASP_CAMERA_BA_PARAMS_H__
