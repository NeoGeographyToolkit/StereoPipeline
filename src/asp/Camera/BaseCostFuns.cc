// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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
                                      vw::Datum const& datum) {
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
                               vw::Datum const& datum) {
  if (weight <= 0)
    vw::vw_throw(vw::ArgumentErr() << tag << ": Invalid weight: "
              << weight << ". It must be positive.\n");

  if (uncertainty[0] <= 0 || uncertainty[1] <= 0)
    vw::vw_throw(vw::ArgumentErr() << tag << ": Invalid uncertainty: "
              << uncertainty << ". All values must be positive.\n");

  vw::Vector3 llh = datum.cartesian_to_geodetic(anchor);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
  return vw::inverse(NedToEcef);
}

CamUncertaintyError::CamUncertaintyError(vw::Vector3 const& orig_ctr, double const* orig_adj,
                                         int param_len,
                                         vw::Vector2 const& uncertainty, double weight,
                                         vw::Datum const& datum,
                                         double camera_position_uncertainty_power):
  m_orig_ctr(orig_ctr), m_param_len(param_len), m_uncertainty(uncertainty),
  m_weight(weight),
  m_camera_position_uncertainty_power(camera_position_uncertainty_power) {

  // Three residuals and one parameter block
  set_num_residuals(3);
  mutable_parameter_block_sizes()->push_back(param_len);

  // The first three parameters are the camera center adjustments.
  m_orig_adj = vw::Vector3(orig_adj[0], orig_adj[1], orig_adj[2]);

  // Validate the inputs and build the ECEF-to-NED rotation, anchored at the center.
  m_EcefToNed = setupPositionUnc("CamUncertaintyError", orig_ctr, uncertainty, 
                                 weight, datum);
}

// Residual for a normalized motion x, whose square is the average of x^2 and
// |x|^power. That is x^2 for power 2, and for a higher power it is quadratic near
// zero and rises steeply past one sigma, where it equals 1 for any power.
double posUncResidual(double x, double power) {
  if (power == 2.0)
    return x;
  double a = std::abs(x);
  double r = a * sqrt((1.0 + pow(a, power - 2.0)) / 2.0);
  return (x < 0) ? -r : r;
}

// Derivative of posUncResidual() with respect to x. It is 1/sqrt(2) at zero for a
// power above 2, so the constraint is felt at the initial position.
double posUncDerivative(double x, double power) {
  if (power == 2.0)
    return 1.0;
  double b = pow(std::abs(x), power - 2.0);
  return (1.0 + 0.5 * power * b) / (2.0 * sqrt((1.0 + b) / 2.0));
}

// Fill 3 residuals from an ECEF position error. The error is rotated into NED,
// split into horizontal and vertical components, each normalized by the horizontal
// and vertical uncertainty, passed through posUncResidual() with the given power,
// and weighted by sqrt(weight). Shared by CamUncertaintyError and
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

  // Regular sum of squares, by default, for power 2. Multiply by sqrt(weight), to
  // give the squared residual the correct weight. This was shown to work better
  // than a more abrupt function.
  double power = camera_position_uncertainty_power;
  residuals[0] = sqrt(weight) * posUncResidual(horiz[0], power);
  residuals[1] = sqrt(weight) * posUncResidual(horiz[1], power);
  residuals[2] = sqrt(weight) * posUncResidual(vert,     power);
}

bool CamUncertaintyError::Evaluate(double const* const* parameters, double* residuals,
                                   double** jacobians) const {

  // The difference between the original and current camera center
  double const* cam_adj = parameters[0];
  vw::Vector3 diff;
  for (size_t p = 0; p < 3; p++)
    diff[p] = cam_adj[p] - m_orig_adj[p];

  positionUncResidual(diff, m_EcefToNed, m_uncertainty, m_weight,
                      m_camera_position_uncertainty_power, residuals);

  if (jacobians == NULL || jacobians[0] == NULL)
    return true;

  // Residual i is sqrt(w) * posUncResidual(x_i), with x_i = (EcefToNed * diff)_i / s_i.
  // Its derivative is sqrt(w) * posUncDerivative(x_i) / s_i times row i of EcefToNed.
  // The rotation variables, if present, do not affect the residual.
  vw::Vector3 ned = m_EcefToNed * diff;
  vw::Vector3 sigma(m_uncertainty[0], m_uncertainty[0], m_uncertainty[1]);
  double power = m_camera_position_uncertainty_power;
  double * jac = jacobians[0];
  for (int row = 0; row < 3; row++) {
    double x = ned[row] / sigma[row];
    double factor = sqrt(m_weight) / sigma[row] * posUncDerivative(x, power);
    for (int col = 0; col < m_param_len; col++) {
      if (col < 3)
        jac[row * m_param_len + col] = factor * m_EcefToNed(row, col);
      else
        jac[row * m_param_len + col] = 0.0;
    }
  }

  return true;
}

GroupCamUncertaintyError::GroupCamUncertaintyError
                          (vw::Vector3 const& init_pos, vw::Vector3 const& centroid,
                           vw::Vector2 const& uncertainty, double weight,
                           vw::Datum const& datum,
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
  vw::Quat rot = vw::axis_angle_to_quaternion(axis_angle);
  vw::Vector3 cur_pos = m_centroid + rot.rotate(m_init_pos - m_centroid) + translation;

  // The difference between the original and current camera center
  vw::Vector3 diff = cur_pos - m_init_pos;

  positionUncResidual(diff, m_EcefToNed, m_uncertainty, m_weight,
                      m_camera_position_uncertainty_power, residuals);
  return true;
}

} // end namespace asp
