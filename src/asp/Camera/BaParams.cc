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

/// \file BaParams.cc
///

#include <asp/Camera/BaParams.h>

namespace asp {

IntrinsicOptions::IntrinsicOptions(): 
  center_shared(true), focus_shared(true), distortion_shared(true),
  share_intrinsics_per_sensor(false), num_sensors(0) {}

// Control per each group of cameras or for all cameras which intrinsics
// should be floated.
bool IntrinsicOptions::float_optical_center(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_center[sensor_id];
}

bool IntrinsicOptions::float_focal_length(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_focus[sensor_id];
}

bool IntrinsicOptions::float_distortion_params(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_distortion[sensor_id];
}

} // end namespace asp
