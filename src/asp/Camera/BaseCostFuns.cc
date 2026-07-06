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

#include <vw/Math/Quaternion.h>

#include <cmath>

namespace asp {

// Sanitize a per-interest-point pixel sigma. A NaN sigma is treated as a
// sigma of 1.0. A non-positive sigma is skipped (return false so the caller
// can mark the point as an outlier). Shared by bundle_adjust and jitter_solve.
bool sanitizePixelSigma(vw::Vector2 & pixel_sigma) {
  if (std::isnan(pixel_sigma[0]) || std::isnan(pixel_sigma[1]))
    pixel_sigma = vw::Vector2(1, 1);
  if (pixel_sigma[0] <= 0.0 || pixel_sigma[1] <= 0.0)
    return false;
  return true;
}

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

// Validate the weight and uncertainty, and return the ECEF-to-NED rotation anchored
// at the given camera center, used to split a position error into horizontal and
// vertical components. Shared by the two position-uncertainty cost functions.
vw::Matrix3x3 setupPositionUnc(std::string const& tag, vw::Vector3 const& anchor,
                               vw::Vector2 const& uncertainty, double weight,
                               vw::cartography::Datum const& datum) {
  if (weight <= 0)
    vw::vw_throw(vw::ArgumentErr() << tag << ": Invalid weight: "
              << weight << ". It must be positive.\n");

  if (uncertainty[0] <= 0 || uncertainty[1] <= 0)
    vw::vw_throw(vw::ArgumentErr() << tag << ": Invalid uncertainty: "
              << uncertainty << ". All values must be positive.\n");

  vw::Vector3 llh = datum.cartesian_to_geodetic(anchor);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
  return vw::math::inverse(NedToEcef);
}

CamUncertaintyError::CamUncertaintyError(vw::Vector3 const& orig_ctr, double const* orig_adj,
                                         vw::Vector2 const& uncertainty, double weight,
                                         vw::cartography::Datum const& datum,
                                         double camera_position_uncertainty_power):
  m_orig_ctr(orig_ctr), m_uncertainty(uncertainty), m_weight(weight),
  m_camera_position_uncertainty_power(camera_position_uncertainty_power) {

  // The first three parameters are the camera center adjustments.
  m_orig_adj = vw::Vector3(orig_adj[0], orig_adj[1], orig_adj[2]);

  // Validate the inputs and build the ECEF-to-NED rotation, anchored at the center.
  m_EcefToNed = setupPositionUnc("CamUncertaintyError", orig_ctr, uncertainty, 
                                 weight, datum);
}

// The signed power is a better-behaved version of pow that respects the sign of the input.
double signed_power(double val, double power) {
  if (val < 0)
    return -pow(-val, power);
  return pow(val, power);
}

// Fill 3 residuals from an ECEF position error. The error is rotated into NED,
// split into horizontal and vertical components, each normalized by the horizontal
// and vertical uncertainty, then raised to the given power (power 1 gives a plain
// sum of squares) and weighted by sqrt(weight). Shared by CamUncertaintyError and
// GroupCamUncertaintyError.
void positionUncResidual(vw::Vector3 const& diff,
                         vw::Matrix3x3 const& EcefToNed,
                         vw::Vector2 const& uncertainty, double weight,
                         double camera_position_uncertainty_power,
                         double* residuals) {

  // Convert the difference to NED
  vw::Vector3 NedDir = EcefToNed * diff;

  // Split into horizontal and vertical components
  vw::Vector2 horiz = subvector(NedDir, 0, 2);
  double      vert  = NedDir[2];

  // Normalize by uncertainty
  horiz /= uncertainty[0];
  vert  /= uncertainty[1];

  double p = camera_position_uncertainty_power / 2.0;

  // Regular sum of squares, by default, which corresponds to p above being 1.
  // Multiply by sqrt(weight), to give the squared residual the correct weight.
  // This was shown to work better than a more abrupt function.
  if (p == 1.0) {
    residuals[0] = sqrt(weight) * horiz[0];
    residuals[1] = sqrt(weight) * horiz[1];
    residuals[2] = sqrt(weight) * vert;
  } else { // More general power
    residuals[0] = sqrt(weight) * signed_power(horiz[0], p);
    residuals[1] = sqrt(weight) * signed_power(horiz[1], p);
    residuals[2] = sqrt(weight) * signed_power(vert,     p);
  }
}

bool CamUncertaintyError::operator()(const double* cam_adj, double* residuals) const {

  // The difference between the original and current camera center
  vw::Vector3 diff;
  for (size_t p = 0; p < 3; p++)
    diff[p] = cam_adj[p] - m_orig_adj[p];

  positionUncResidual(diff, m_EcefToNed, m_uncertainty, m_weight,
                      m_camera_position_uncertainty_power, residuals);
  return true;
}

GroupCamUncertaintyError::GroupCamUncertaintyError
                          (vw::Vector3 const& init_pos, vw::Vector3 const& centroid,
                           vw::Vector2 const& uncertainty, double weight,
                           vw::cartography::Datum const& datum,
                           double camera_position_uncertainty_power):
  m_init_pos(init_pos), m_centroid(centroid), m_uncertainty(uncertainty), m_weight(weight),
  m_camera_position_uncertainty_power(camera_position_uncertainty_power) {

  // Validate the inputs and build the ECEF-to-NED rotation, anchored at the
  // original camera position.
  m_EcefToNed = setupPositionUnc("GroupCamUncertaintyError", init_pos, uncertainty, 
                                 weight, datum);
}

bool GroupCamUncertaintyError::operator()(const double* group_pose,
                                          double* residuals) const {

  // The current position is derived from the shared group pose applied to the
  // original position, with rotation about the group centroid (same formula as
  // applyOrbitalGroupPose). This is the only difference from CamUncertaintyError.
  vw::Vector3 axis_angle(group_pose[0], group_pose[1], group_pose[2]);
  vw::Vector3 translation(group_pose[3], group_pose[4], group_pose[5]);
  vw::Quat rot = vw::math::axis_angle_to_quaternion(axis_angle);
  vw::Vector3 cur_pos = m_centroid + rot.rotate(m_init_pos - m_centroid) + translation;

  // The difference between the original and current camera center
  vw::Vector3 diff = cur_pos - m_init_pos;

  positionUncResidual(diff, m_EcefToNed, m_uncertainty, m_weight,
                      m_camera_position_uncertainty_power, residuals);
  return true;
}

} // end namespace asp
