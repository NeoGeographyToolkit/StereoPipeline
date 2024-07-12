// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as XYZError2represented by the
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

/// \file BaseCostFuns.h

// Cost functions shared by bundle adjustment and camera jitter estimation.

#ifndef __ASP_CAMERA_BASE_COST_FUNS_H__
#define __ASP_CAMERA_BASE_COST_FUNS_H__

#include <asp/Core/BundleAdjustUtils.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <string>
#include <vector>

namespace asp {

/// A ceres cost function. The residual is the difference between the
/// observed 3D point and the current (floating) 3D point, normalized by
/// xyz_sigma. 
struct XYZError {
  XYZError(vw::Vector3 const& observation, vw::Vector3 const& xyz_sigma):
    m_observation(observation), m_xyz_sigma(xyz_sigma) {
      bool is_good = (xyz_sigma[0] > 0 && xyz_sigma[1] > 0 && xyz_sigma[2] > 0);
      if (!is_good) {
        // This will also cover NaNs
        vw::vw_throw(vw::ArgumentErr() << "XYZError: Invalid xyz_sigma: "
                 << xyz_sigma << ". All values must be positive.\n");
      }
    }

  template <typename T>
  bool operator()(const T* point, T* residuals) const {
    for (size_t p = 0; p < m_observation.size(); p++)
      residuals[p] = (point[p] - m_observation[p])/m_xyz_sigma[p]; // Units are meters

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(vw::Vector3 const& observation,
                                     vw::Vector3 const& xyz_sigma) {
    return (new ceres::AutoDiffCostFunction<XYZError, 3, 3>
            (new XYZError(observation, xyz_sigma)));
  }

  vw::Vector3 m_observation;
  vw::Vector3 m_xyz_sigma;
};

} // end namespace asp

#endif //__ASP_CAMERA_BASE_COST_FUNS_H__
