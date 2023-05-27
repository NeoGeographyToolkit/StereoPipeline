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

#ifndef __CORE_CAMERA_TRANSFORMS_H__
#define __CORE_CAMERA_TRANSFORMS_H__
#include <vw/Math/Matrix.h>

// A set of routines for handling camera transformations. It is best to not
// include here any Eigen headers, as that will slow down the compilation.

namespace asp {

// Find the roll-pitch-yaw rotation in ZYX order. The inputs are in
// degrees.
vw::Matrix3x3 rollPitchYaw(double roll, double pitch, double yaw);

} //end namespace asp

#endif//__CORE_CAMERA_TRANSFORMS_H__
