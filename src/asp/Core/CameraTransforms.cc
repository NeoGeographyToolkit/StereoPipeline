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

/// \file CameraTransforms.cc
///

#include <asp/Core/CameraTransforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace asp{

// Find the roll-pitch-yaw rotation in ZYX order. The inputs are in
// degrees.
vw::Matrix3x3 rollPitchYaw(double roll, double pitch, double yaw) {

    // Factor to convert from degrees to radians
    const double DEG_TO_RAD = M_PI/180.0;

    Eigen::AngleAxisd rollAngle (DEG_TO_RAD * roll,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle  (DEG_TO_RAD * yaw,   Eigen::Vector3d::UnitZ());

    // Multiply these returing an Eigen Matrix3d
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    // Convert to Eigen3
    Eigen::Matrix3d M = q.matrix();

    // Convert to vw::Matrix3x3
    vw::Matrix3x3 R;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            R(r, c) = M(r, c);
        }
    }
    
    return R;
}
    
} //end namespace asp

