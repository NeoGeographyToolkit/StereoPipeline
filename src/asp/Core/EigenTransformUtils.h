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

#ifndef __CORE_EIGEN_TRANSFORM_UTILS_H__
#define __CORE_EIGEN_TRANSFORM_UTILS_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Very low level transform utilities
namespace asp {

// Calculate an affine transform given position and quaternions (in order x, y, z, w).
Eigen::Affine3d calcTransform(double x, double y, double z,
                              double qx, double qy, double qz, double qw);

} //end namespace asp

#endif//__CORE_EIGEN_TRANSFORM_UTILS_H__
