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

#ifndef __ASP_CORE_COVARIANCE_H__
#define __ASP_CORE_COVARIANCE_H__

#include <vw/Math/Vector.h>

namespace asp {

  // Given 0 <= num < 15, return a perturbation in position.  The
  // starting one is the zero perturbation, then perturb first
  // coordinate in the positive and then negative direction, then same
  // for second and third coordinate. The rest of the perturbations are
  // 0 as those indices are used to perturb the quaternions.
  // So, return (0, 0, 0), (deltaPosition, 0, 0), (-deltaPosition, 0, 0)
  // (0, deltaPosition, 0), and so on.
  vw::Vector<double, 3> positionDelta(int num);

  // Similar logic as above for the quaternion, for indices 7, ..., 14 (8 of them)
  vw::Vector<double, 4> quatDelta(int num);

  // Number of nominal and perturbed cameras when the covariance is computed
  int numCamsForCovariance();
  
} // end namespace asp

#endif//__ASP_CORE_COVARIANCE_H__
