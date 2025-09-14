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

// Cost functions used in bundle adjustment. These need access to the camera
// models, so they are stored in the Camera folder.

#include <asp/Camera/BaseCostFuns.h>

namespace asp {

// Factory to hide the construction of the CostFunction object from
// the client code.
ceres::CostFunction* LLHError::Create(vw::Vector3            const& observation_xyz,
                                      vw::Vector3            const& sigma,
                                      vw::cartography::Datum const& datum) {
  return (new ceres::NumericDiffCostFunction<LLHError, ceres::CENTRAL, 3, 3>
          (new LLHError(observation_xyz, sigma, datum)));
}

bool LLHError::operator()(const double* point, double* residuals) const {
  vw::Vector3 observation_llh, point_xyz, point_llh;
  for (size_t p = 0; p < m_observation_xyz.size(); p++) {
    point_xyz[p] = double(point[p]);
  }

  point_llh       = m_datum.cartesian_to_geodetic(point_xyz);
  observation_llh = m_datum.cartesian_to_geodetic(m_observation_xyz);

  for (size_t p = 0; p < m_observation_xyz.size(); p++) 
    residuals[p] = (point_llh[p] - observation_llh[p])/m_sigma[p]; // Input units are meters

  return true;
}

CamUncertaintyError::CamUncertaintyError(vw::Vector3 const& orig_ctr, double const* orig_adj,
                                         vw::Vector2 const& uncertainty, double weight,
                                         vw::cartography::Datum const& datum):
  m_orig_ctr(orig_ctr), m_uncertainty(uncertainty), m_weight(weight) {
    
  // m_weight must be positive
  if (m_weight <= 0)
    vw::vw_throw(vw::ArgumentErr() << "CamUncertaintyError: Invalid weight: "
              << m_weight << ". It must be positive.\n");
  
  // The first three parameters are the camera center adjustments.
  m_orig_adj = vw::Vector3(orig_adj[0], orig_adj[1], orig_adj[2]);

  // The uncertainty must be positive
  if (m_uncertainty[0] <= 0 || m_uncertainty[1] <= 0)
    vw::vw_throw(vw::ArgumentErr() << "CamUncertaintyError: Invalid uncertainty: "
              << uncertainty << ". All values must be positive.\n");    
  
  // The NED coordinate system, for separating horizontal and vertical components
  vw::Vector3 llh = datum.cartesian_to_geodetic(orig_ctr);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
  m_EcefToNed = vw::math::inverse(NedToEcef);
}

bool CamUncertaintyError::operator()(const double* cam_adj, double* residuals) const {
  
  // The difference between the original and current camera center
  vw::Vector3 diff;
  for (size_t p = 0; p < 3; p++)
    diff[p] = cam_adj[p] - m_orig_adj[p];
  
  // Convert the difference to NED
  vw::Vector3 NedDir = m_EcefToNed * diff;
  
  // Split into horizontal and vertical components
  vw::Vector2 horiz = subvector(NedDir, 0, 2);
  double      vert  = NedDir[2];
  
  // Normalize by uncertainty
  horiz /= m_uncertainty[0];
  vert  /= m_uncertainty[1];
  
  // Regular sum of squares. Multiply by sqrt(m_weight), to give the squared
  // residual the correct weight. This was shown to work to work better than a
  // more abrupt function.
  residuals[0] = sqrt(m_weight) * horiz[0];
  residuals[1] = sqrt(m_weight) * horiz[1];
  residuals[2] = sqrt(m_weight) * vert;

  return true;
}

} // end namespace asp
