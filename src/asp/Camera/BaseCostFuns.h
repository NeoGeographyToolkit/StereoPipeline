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

// A triangulated point with this sigma will be declared fixed. This should be
// positive and somewhat reasonable, as it will show up in the cost function,
// though, in theory, it should not matter as it shows up as a term like
// (x-x0)^2/sigma^2, with x starting as x0 and kept fixed.
const double FIXED_GCP_SIGMA = 1e-10;

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

/// This cost function imposes a rather hard constraint on camera center
/// horizontal and vertical motion. It does so by knowing how many reprojection
/// errors exist for this camera and making this cost function big enough to
/// overcome then when the motion is going out of bounds. The residual here is
/// raised to 4th power and will be squared when added to the cost function.
/// Two residuals are computed, for horizontal and vertical motion.
struct CamUncertaintyError {
  
  CamUncertaintyError(vw::Vector3 const& orig_ctr, double const* orig_adj,
                      vw::Vector2 const& uncertainty, double num_pixel_obs,
                      vw::cartography::Datum const& datum,
                      double camera_position_uncertainty_power);
    
  bool operator()(const double* cam_adj, double* residuals) const;
  
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* 
    Create(vw::Vector3 const& orig_ctr, double const* orig_adj, int param_len,
           vw::Vector2 const& uncertainty, double num_pixel_obs,
           vw::cartography::Datum const& datum, 
           double camera_position_uncertainty_power) {
    // 2 residuals and 3 translation variables. For bundle_adjust must add the
    // rotation variables, as otherwise CERES says some params have inconsistent
    // sizes. ceres::RIDDER works better than ceres::CENTRAL for this cost
    // function, especially when the uncertainty is 0.1 m or less.
    if (param_len == 3)
     return (new ceres::NumericDiffCostFunction<CamUncertaintyError, ceres::CENTRAL, 3, 3>
            (new CamUncertaintyError(orig_ctr, orig_adj, uncertainty, num_pixel_obs, 
                                     datum, camera_position_uncertainty_power)));
    else if (param_len == 6)
     return (new ceres::NumericDiffCostFunction<CamUncertaintyError, ceres::CENTRAL, 3, 6>
            (new CamUncertaintyError(orig_ctr, orig_adj, uncertainty, num_pixel_obs, 
                                     datum, camera_position_uncertainty_power)));
    else
      vw::vw_throw(vw::ArgumentErr() << "CamUncertaintyError: Invalid param_len: "
               << param_len << ". Must be 3 or 6.\n");
  }

  // orig_ctr is the original camera center, orig_cam_ptr is the original
  // adjustment (resulting in the original center). The uncertainty is
  // in meters.
  vw::Vector3 m_orig_ctr;
  vw::Vector3 m_orig_adj;
  int m_param_len;
  vw::Vector2 m_uncertainty;
  double m_num_pixel_obs; // use double, so we can do a fractional amount of the constraint
  vw::Matrix3x3 m_EcefToNed;
  double m_camera_position_uncertainty_power;
};

} // end namespace asp

#endif //__ASP_CAMERA_BASE_COST_FUNS_H__
