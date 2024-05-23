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


/// \file EigenTransformUtils.cc
///

#include <asp/Core/EigenTransformUtils.h>

namespace asp {

// Calculate an affine transform given position and quaternions (in order x, y, z, w).
Eigen::Affine3d calcTransform(double x, double y, double z,
                              double qx, double qy, double qz, double qw) {

  Eigen::Vector3d ET(x, y, z);
  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Matrix3d ER = q.toRotationMatrix();
  
  Eigen::Matrix4d E;
  E.setIdentity();
  E.block<3,3>(0,0) = ER;
  E.block<3,1>(0,3) = ET;
  
  Eigen::Affine3d cam_to_world;
  cam_to_world.matrix() = E;
  
  return cam_to_world;
} 

}
