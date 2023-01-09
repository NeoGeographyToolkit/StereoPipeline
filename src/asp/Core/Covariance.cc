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

#include <asp/Core/Covariance.h>

// Logic for propagation of covariance through stereo triangulation 

namespace asp {

// Change in satellite position (measured in meters) and satellite orientation
// measured in normalized quaternions, to be used for numerical differencing.
// One has to be mindful of the fact that the positions are on the order of
// 7.0e6 meters given the distance from satellite to Earth center in ECEF,
// so the position delta should not be too tiny.
// TODO(oalexan1): Need to experiment with both of these, especially
// deltaQuat.
const double deltaPosition = 0.01; // measured in meters
const double deltaQuat     = 1.0e-6; // given that quaternions are normalized

// Given 0 <= num < 15, return a perturbation in position.  The
// starting one is the zero perturbation, then perturb first
// coordinate in the positive and then negative direction, then same
// for second and third coordinate. The rest of the perturbations are
// 0 as those indices are used to perturb the quaternions.
// So, return (0, 0, 0), (deltaPosition, 0, 0), (-deltaPosition, 0, 0)
// (0, deltaPosition, 0), and so on.
vw::Vector<double, 3> positionDelta(int num) {

  vw::Vector<double, 3> ans; // this is 0
  if (num == 0)
    return ans; // nominal position value, no perturbation
  if (num > 6) 
    return ans; // nominal position value, will perturb the quaternion then

  double sign = 1.0; // sign of the perturbation: 1, -1, 1, -1, etc.
  if (num % 2 == 0) 
    sign = -1.0;

  int index = (num - 1)/2; // get 0, 1, or 2
  ans[index] = sign * deltaPosition;
  
  return ans;
}

// Similar logic as above for the quaternion, for indices 7, ..., 14 (8 of them)
vw::Vector<double, 4> quatDelta(int num) {

  vw::Vector<double, 4> ans; // this is 0
  if (num <= 6) 
    return ans; // nominal quat value, will perturb the positions then

  num = num - 6; // now num = 1, 2, ... 8
  if (num > 8) 
    vw::vw_throw(vw::ArgumentErr() << "Out of bounds in quatDelta().\n");
  
  double sign = 1.0; // sign of the perturbation: 1, -1, 1, -1, etc.
  if (num % 2 == 0) 
    sign = -1.0;

  int index = (num - 1)/2; // get 0, 1, 2, or 3
  ans[index] = sign * deltaQuat;
  
  return ans;
}

// Number of nominal and perturbed cameras when the covariance is computed.
int numCamsForCovariance() {
  // One nominal camera. Then one positive and negative perturbation
  // for each position (3) and quaternion (4).
  return 15; 
}
  
} // end namespace asp
