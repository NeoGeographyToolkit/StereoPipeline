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


/// \file Covariance.h

// Logic for propagation of covariance through stereo triangulation 

#ifndef __ASP_CAMERA_COVARIANCE_H__
#define __ASP_CAMERA_COVARIANCE_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>

namespace asp {

  // Given 0 <= num < 15, return a perturbation in position. The
  // starting one is the zero perturbation, then perturb first
  // coordinate in the positive and then negative direction, then same
  // for second and third coordinate. The rest of the perturbations
  // are 0 as those indices are used to perturb the quaternions.  So,
  // return (0, 0, 0), (deltaPosition, 0, 0), (-deltaPosition, 0, 0)
  // (0, deltaPosition, 0), and so on.
  vw::Vector<double, 3> positionDelta(int num);

  // Similar logic as above for the quaternion, for indices 7, ..., 14 (8 of them)
  vw::Vector<double, 4> quatDelta(int num);

  // Number of nominal and perturbed cameras when the covariance is computed
  int numCamsForCovariance();

  // Given two cameras and a pixel in each camera image, consider the
  // following transform. Go from the perturbed joint vector of
  // satellite positions and quaternions for this pixel pair to the
  // perturbed triangulated point. Then, the vector from nominal to
  // perturbed triangulation point is converted to North-East-Down
  // relative to the nominal point. Use numerical differentiation to
  // find the Jacobian of this transform with centered
  // differences. This will be used to find the covariances of the
  // triangulated point in NED coordinates given the input satellite
  // covariances. This works only for Maxar (DigitalGlobe)
  // cameras. This function may throw exceptions. Do not divide the
  // numerical derivatives by deltaPosition and deltaQuat, but only by
  // 2.0 (since these are centered differences). That because the
  // division makes the partial derivatives in quaternions huge and is
  // not good for numerical stability. We will compensate for this
  // when we multiply by the actual covariances, which are huge, so
  // those will be pre-multiplied by the squares of deltaPosition and
  // deltaQuat, with the same final result.
  void scaledTriangulationJacobian(vw::camera::CameraModel const* cam1,
                                   vw::camera::CameraModel const* cam2,
                                   vw::Vector2 const& pix1,
                                   vw::Vector2 const& pix2,
                                   vw::Matrix<double> & J);

  // Based on tabulated satellite position and quaternion covariance
  // for each camera, find the interpolated covariances for cam1 at
  // pix1 (6 for position, 10 for orientation, as just the upper-right
  // corner is used), same for cam2 at pix2, autocomplete these to the
  // full matrices (3x3 and 4x4 for each), create a combined matrix of
  // covariances (14 x 14), and divide the entries by squares of
  // deltaPosition and deltaQuat which normalizes them, and which are
  // compensated by not dividing by these numbers (without the square)
  // what is found in scaledTriangulationJacobian(). Later we will do
  // J * C * J^T. The same order of variables as in
  // scaledTriangulationJacobian must be used.
  void scaledSatelliteCovariance(vw::camera::CameraModel const* cam1,
                                 vw::camera::CameraModel const* cam2,
                                 vw::Vector2 const& pix1,
                                 vw::Vector2 const& pix2,
                                 vw::Matrix<double> & C);

  
} // end namespace asp

#endif//__ASP_CAMERA_COVARIANCE_H__
