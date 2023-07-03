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

#include <asp/Core/CameraTransforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace asp {

// A function to convert a 3x3 VW matrix to Eigen
Eigen::Matrix3d vwToEigenMat(vw::Matrix3x3 const& m) {
  Eigen::Matrix3d result;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      result(r, c) = m(r, c);
  return result;
}

// Convert an Eigen 3x3 matrix to a VW 3x3 matrix
vw::Matrix3x3 eigenToVwMat(Eigen::Matrix3d const& m) {
  vw::Matrix3x3 result;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      result(r, c) = m(r, c);
  return result;
}

// A function to go from a VW matrix to a quaternion, represented
// as four values, x, y, z, w. Care with the order of values. It is not
// w, x, y, z as in the Eigen convention.
void matrixToQuaternion(vw::Matrix3x3 const& R, 
                         // Outputs
                         double & x, double & y, double & z, double & w) {

    // Convert to Eigen3
    Eigen::Matrix3d M = vwToEigenMat(R);
    // Convert to quaternion
    Eigen::Quaterniond q(M);
    // Convert to x, y, z, w
    x = q.x(); y = q.y(); z = q.z(); w = q.w();
}

// A function to convert a quaternion given by 4 numbers to a VW matrix. It is
// very important to note that were we assume the order of the quaternion
// numbers is (x, y, z, w) and not (w, x, y, z).
vw::Matrix3x3 quaternionToMatrix(double x, double y, double z, double w) {
  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Matrix3d m = q.toRotationMatrix();

  // Convert to vw::Matrix3x3
  return eigenToVwMat(m);
}

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
    vw::Matrix3x3 R = eigenToVwMat(M);
    
    return R;
}
    
// Given a matrix obtained by multiplying roll, pitch, and yaw rotations, by applying
// then in this order from right to left, find the roll, pitch, and yaw angles in degrees.
// This can return 180 +/- x, if x was the original angle, so it is not a true inverse.
void rollPitchYawFromRotationMatrix(vw::Matrix3x3 const& R, 
  // Outputs
  double & roll, double & pitch, double & yaw) {

    // Convert to Eigen3 and then find the Euler angles in radians
    Eigen::Matrix3d M = vwToEigenMat(R);
    Eigen::Vector3d euler = M.eulerAngles(2, 1, 0);

    // Convert to degrees
    roll  = euler[2]*180.0/M_PI;
    pitch = euler[1]*180.0/M_PI;
    yaw   = euler[0]*180.0/M_PI;
}

} //end namespace asp

