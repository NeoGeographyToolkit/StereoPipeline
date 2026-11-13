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

// Cost functions that regularize the camera poses in jitter solving: roll and
// yaw priors, quaternion temporal smoothness, and the orientation-uncertainty
// terms. Stored in the Camera folder, as they need the camera models.

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveRigCostFuns.h>
#include <asp/Camera/BundleAdjustCostFuns.h> // for get_loss_function
#include <asp/Rig/TransformUtils.h>
#include <asp/Rig/RigConfig.h>
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/CamPoseUtils.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/DataLoader.h>

#include <vw/Camera/CameraImage.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/Math/Transform.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/RandomSet.h>
#include <vw/Image/Interpolation.h>
#include <vw/Image/ImageChannels.h>
#include <vw/Cartography/Map2CamTrans.h>

#include <ceres/rotation.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <vector>
#include <unordered_map>

namespace asp {

// A Ceres cost function. The residual is the roll and/or yaw component of the camera
// rotation, as measured relative to the initial along-track direction. We assume
// that all positions are along the same segment in projected coordinates, or at
// least that the current position and its nearest neighbors are roughly on
// such a segment. That one is used to measure the roll/yaw from. This is consistent
// with how sat_sim creates the cameras.
struct weightedRollYawError {
  weightedRollYawError(std::vector<double>       const& positions,
                   std::vector<double>           const& quaternions,
                   vw::GeoReference const& georef,
                   int cur_pos, double rollWeight, double yawWeight,
                   bool initial_camera_constraint);

  // Compute the weighted roll/yaw error between the current position and along-track
  // direction. Recall that quaternion = cam2world = sat2World * rollPitchYaw * rotXY.
  // rollPitchYaw is variable and can have jitter. Extract from it roll, pitch,
  bool operator()(double const * const * parameters, double * residuals) const;

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(std::vector<double>           const& positions,
                                     std::vector<double>           const& quaternions,
                                     vw::GeoReference const& georef,
                                     int cur_pos,
                                     double rollWeight, double yawWeight,
                                     bool initial_camera_constraint) {

    ceres::DynamicNumericDiffCostFunction<weightedRollYawError>* cost_function =
          new ceres::DynamicNumericDiffCostFunction<weightedRollYawError>
          (new weightedRollYawError(positions, quaternions, georef, cur_pos,
                                    rollWeight, yawWeight, initial_camera_constraint));

    cost_function->SetNumResiduals(2); // for roll and yaw
    cost_function->AddParameterBlock(NUM_QUAT_PARAMS);

    return cost_function;
  }

  double m_rollWeight, m_yawWeight;
  vw::Matrix3x3 m_rotXY, m_sat2World, m_initCam2World;
  bool m_initial_camera_constraint;
};

// Constructor for weightedRollYawError. See the .h file for the documentation.
weightedRollYawError::weightedRollYawError
                  (std::vector<double>           const& positions,
                   std::vector<double>           const& quaternions,
                   vw::GeoReference const& georef,
                   int cur_pos, double rollWeight, double yawWeight,
                   bool initial_camera_constraint):
                   m_rollWeight(rollWeight), m_yawWeight(yawWeight),
                   m_initial_camera_constraint(initial_camera_constraint) {

    int num_pos = positions.size()/NUM_XYZ_PARAMS;
    int num_quat = quaternions.size()/NUM_QUAT_PARAMS;
    if (num_pos != num_quat)
      vw::vw_throw(vw::ArgumentErr()
        << "weightedRollYawError: Expecting the same number of positions and quaternions.\n");
    if (cur_pos < 0 || cur_pos >= num_pos)
      vw::vw_throw(vw::ArgumentErr()
        << "weightedRollYawError: Expecting position index in range.\n");

    // Find the nearest neighbors of the current position
    int beg_pos = std::max(0, cur_pos - 1);
    int end_pos = std::min(num_pos - 1, cur_pos + 1);
    if (beg_pos >= end_pos)
      vw::vw_throw(vw::ArgumentErr()
        << "weightedRollYawError: Expecting at least 2 camera positions.\n");

    // Find the segment along which the cameras are located, in projected coordinates
    // Here we mirror the logic from SatSim.cc
    int b = beg_pos * NUM_XYZ_PARAMS;
    int c = cur_pos * NUM_XYZ_PARAMS;
    int e = end_pos * NUM_XYZ_PARAMS;
    vw::Vector3 beg_pt(positions[b], positions[b+1], positions[b+2]);
    vw::Vector3 cur_pt(positions[c], positions[c+1], positions[c+2]);
    vw::Vector3 end_pt(positions[e], positions[e+1], positions[e+2]);

    // Orbital points before the current one, the current one, and after the
    // current one, in projected coordinates
    vw::Vector3 beg_proj = vw::ecefToProj(georef, beg_pt);
    vw::Vector3 cur_proj = vw::ecefToProj(georef, cur_pt);
    vw::Vector3 end_proj = vw::ecefToProj(georef, end_pt);

    // Find satellite along and across track directions in projected coordinates
    vw::Vector3 proj_along, proj_across;
    asp::calcProjAlongAcross(beg_proj, end_proj, proj_along, proj_across);

    // Find along and across in ECEF
    vw::Vector3 along, across;
    asp::calcEcefAlongAcross(georef, asp::satSimDelta(),
                              proj_along, proj_across, cur_proj,
                              along, across); // outputs

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::cross_prod(along, across);
    down = down / norm_2(down);

    // Find the rotation matrix from satellite to world coordinates, and 90
    // degree in-camera rotation. It is assumed, as in sat_sim, that:
    // cam2world = sat2World * rollPitchYaw * rotXY.
    asp::assembleCam2WorldMatrix(along, across, down, m_sat2World);
    m_rotXY = asp::rotationXY();

    // Initial camera rotation matrix, before we optimize it
    m_initCam2World = asp::quaternionToMatrix(&quaternions[cur_pos*NUM_QUAT_PARAMS]);
}

// See the .h file for the documentation.
bool weightedRollYawError::operator()(double const * const * parameters,
                                      double * residuals) const {

  // Convert to rotation matrix. Order of quaternion is x, y, z, w.  
  vw::Matrix3x3 cam2world = asp::quaternionToMatrix(parameters[0]);

  if (m_initial_camera_constraint) {
    // Find the new camera orientation relative to the initial camera, not
    // relative to the satellite along-track direction. Then find the roll and
    // yaw from it. This is experimental.
    vw::Matrix3x3 cam2cam =  vw::inverse(cam2world) * m_initCam2World;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    rollPitchYawFromRotationMatrix(cam2cam, roll, pitch, yaw);

    // Fix for roll / yaw being determined with +/- 180 degree ambiguity.
    roll  = roll  - 180.0 * round(roll  / 180.0);
    pitch = pitch - 180.0 * round(pitch / 180.0);
    yaw   = yaw   - 180.0 * round(yaw   / 180.0);

    // Roll, pitch, yaw in camera coordinates are pitch, roll, yaw in satellite
    // coordinates. So adjust below accordingly.
    // CERES is very tolerant if one of the weights used below is 0. So there is
    // no need to use a special cost function for such cases.
    residuals[0] = pitch * m_rollWeight; // per above, swap roll and pitch
    residuals[1] = yaw  * m_yawWeight;

    return true;
  }

  vw::Matrix3x3 rollPitchYaw
    = vw::inverse(m_sat2World) * cam2world * vw::inverse(m_rotXY);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  rollPitchYawFromRotationMatrix(rollPitchYaw, roll, pitch, yaw);

  // Fix for roll / yaw being determined with +/- 180 degree ambiguity.
  roll = roll - 180.0 * round(roll / 180.0);
  pitch = pitch - 180.0 * round(pitch / 180.0);
  yaw  = yaw  - 180.0 * round(yaw  / 180.0);

  // CERES is very tolerant if one of the weights used below is 0. So there is
  // no need to use a special cost function for such cases.
  residuals[0] = roll * m_rollWeight;
  residuals[1] = yaw  * m_yawWeight;

  return true;
}

/// A unary, DEM-free, metric "faux anchor" on one orientation sample. The
/// residual is the ground displacement (meters, over sigma) of a few fixed look
/// rays caused by the sample's rotation drifting from its initial value. Pins
/// the full rotation (yaw included, via off-boresight rays) without a DEM or
/// neighbor samples. Non-robust, so it keeps pulling on runaways.
struct GroundRotationError {
  GroundRotationError(const double * init_quat,
                      std::vector<vw::Vector3> const& body_rays,
                      std::vector<vw::Vector3> const& world_rays,
                      double sigma):
    m_sigma(sigma), m_body(body_rays), m_world(world_rays) {
    for (int it = 0; it < NUM_QUAT_PARAMS; it++)
      m_init_quat[it] = init_quat[it];
  }

  template <typename T>
  bool operator()(const T* quat, T* residuals) const {
    // CSM quaternions are stored [x, y, z, w]; ceres wants [w, x, y, z].
    // Normalize for safety (the norm is also held near 1 by --quat-norm-weight).
    T q[4] = {quat[3], quat[0], quat[1], quat[2]};
    T nrm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= nrm;

    for (size_t k = 0; k < m_body.size(); k++) {
      T b[3] = {T(m_body[k][0]), T(m_body[k][1]), T(m_body[k][2])};
      T rot[3];
      ceres::UnitQuaternionRotatePoint(q, b, rot); // R(quat) * body ray
      // Ground displacement from the initial world ray, in meters, over sigma.
      residuals[3*k + 0] = (rot[0] - T(m_world[k][0])) / T(m_sigma);
      residuals[3*k + 1] = (rot[1] - T(m_world[k][1])) / T(m_sigma);
      residuals[3*k + 2] = (rot[2] - T(m_world[k][2])) / T(m_sigma);
    }
    return true;
  }

  // Factory. Fixed at 3 rays -> 9 residuals, 4 quaternion parameters.
  static ceres::CostFunction* Create(const double * init_quat,
                                     std::vector<vw::Vector3> const& body_rays,
                                     std::vector<vw::Vector3> const& world_rays,
                                     double sigma) {
    return (new ceres::AutoDiffCostFunction<GroundRotationError, 9, NUM_QUAT_PARAMS>
            (new GroundRotationError(init_quat, body_rays, world_rays, sigma)));
  }

  double m_init_quat[NUM_QUAT_PARAMS];
  std::vector<vw::Vector3> m_body, m_world;
  double m_sigma;
};

// Add roll / yaw constraints. For linescan, use the whole set of samples for given
// camera model. For frame cameras, use the trajectory of all cameras in the same orbital
// group as the current camera.
void addRollYawConstraint(asp::BaBaseOptions              const& opt,
                          asp::CRN                       const& crn,
                          std::vector<asp::CsmModel*>     const& csm_models,
                          vw::GeoReference   const& georef,
                          std::map<int, int>              const& cam2group,
                          bool initial_camera_constraint,
                          double roll_weight, double yaw_weight,
                          // Outputs (append to residual)
                          std::vector<double>                  & frame_params,
                          std::vector<double>                  & weight_per_residual,
                          ceres::Problem                       & problem) {

  if (roll_weight <= 0.0 && yaw_weight <= 0.0)
     vw::vw_throw(vw::ArgumentErr()
         << "addRollYawConstraint: The roll or yaw weight must be positive.\n");

  int num_cams = crn.size();

  // Frame cameras can be grouped by orbital portion. Ensure that all cameras
  // belong to a group.
  if (num_cams != int(cam2group.size()))
    vw::vw_throw(vw::ArgumentErr()
         << "addRollYawConstraint: Failed to add each input camera to an orbital group.\n");

  // Create the orbital trajectory for each group of frame cameras
  std::map<int, std::vector<double>> orbital_group_positions;
  std::map<int, std::vector<double>> orbital_group_quaternions;
  formPositionQuatVecPerGroup(cam2group, csm_models,
    orbital_group_positions, orbital_group_quaternions); // outputs

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

    if (ls_model != NULL) {
      // Linescan cameras. Use the full sequence of cameras in the model
      // to enforce the roll/yaw constraint for each camera in the sequence.
      int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;

      // Make positions one-to-one with quaternions
      std::vector<double> interp_positions;
      asp::orbitInterpExtrap(ls_model, georef, interp_positions);

      for (int iq = 0; iq < numQuat; iq++) {
        ceres::CostFunction* roll_yaw_cost_function
          = weightedRollYawError::Create(interp_positions,
                                         ls_model->m_quaternions,
                                         georef, iq,
                                         roll_weight, yaw_weight,
                                         initial_camera_constraint);

        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* roll_yaw_loss_function = NULL;
        problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
        // The recorded weight should not be 0 as we will divide by it
        weight_per_residual.push_back(roll_weight || 1.0);
        weight_per_residual.push_back(yaw_weight  || 1.0);
      } // end loop through quaternions for given camera

    } else if (frame_model != NULL) {
      // Frame cameras. Use the positions and quaternions of the cameras
      // in the same orbital group to enforce the roll/yaw constraint for
      // each camera in the group.
      auto it = cam2group.find(icam);
      if (it == cam2group.end())
        vw::vw_throw(vw::ArgumentErr()
           << "addRollYawConstraint: Failed to find orbital group for camera.\n");
      int group_id = it->second;

      int index_in_group = indexInGroup(icam, cam2group);
      std::vector<double> positions = orbital_group_positions[group_id];
      std::vector<double> quaternions = orbital_group_quaternions[group_id];
      if (positions.size() / NUM_XYZ_PARAMS < 2) {
        // It can happen that we have just one frame camera, but then we just
        // can't add this constraint
        vw::vw_out(vw::WarningMessage) << "Cannot add roll and/or yaw constraint for "
          << "for an orbital group consisting of only one frame camera.\n";
        continue;
      }

      ceres::CostFunction* roll_yaw_cost_function
        = weightedRollYawError::Create(positions, quaternions,
                                   georef, index_in_group,
                                   roll_weight, yaw_weight,
                                   initial_camera_constraint);

      // We use no loss function, as the quaternions have no outliers
      ceres::LossFunction* roll_yaw_loss_function = NULL;

      // Note how we set the quaternions to be optimized from frame_params.
      // Above, we only cared for initial positions and quaternions.
      double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
      problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here

      // The recorded weight should not be 0 as we will divide by it
      weight_per_residual.push_back(roll_weight || 1.0);
      weight_per_residual.push_back(yaw_weight  || 1.0);
    } else {
      vw::vw_throw(vw::ArgumentErr()
         << "addRollYawConstraint: Expecting CSM linescan or frame cameras.\n");
    }

  } // end loop through cameras

  return;
}

// Given the quaternions in a camera model, calculate the curvature for each
// coordinate. This cannot be done at the starting and ending coordinate, as we
// use a centered difference. Make each curvature be at least the median
// curvature. This will ensure that even flat areas are allowed to have some
// curvature when being optimized.
void estimCurvature(UsgsAstroLsSensorModel * ls_model,
                   std::vector<double> & curvature) {

  // There will be as many curvature terms as overall quaternion coefficients.
  int numQuats = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  curvature.resize(numQuats * NUM_QUAT_PARAMS, 0.0);

  // Throw an error if less than 3 quaternions
  if (numQuats < 3)
    vw::vw_throw(vw::ArgumentErr()
        << "Need at least 3 quaternions to calculate curvature. Set the smoothness "
        << "weight to 0.\n");

  // Start iterating from the second quaternion, as we need the previous one
  // shorthand
  double *q = &ls_model->m_quaternions[0];
  for (int it = 1; it < numQuats - 1; it++) {
    for (int c = 0; c < NUM_QUAT_PARAMS; c++) {
        double val
        = q[(it - 1) * NUM_QUAT_PARAMS + c] - 2.0 * q[it * NUM_QUAT_PARAMS + c] +
          q[(it + 1) * NUM_QUAT_PARAMS + c];
        // Take the absolute value
        curvature[it * NUM_QUAT_PARAMS + c] = std::abs(val);
    }
  }

  // Find the median curvature. Ignore the first and last curvatures as those are 0.
  std::vector<double> median_curvature(4);
  for (int c = 0; c < NUM_QUAT_PARAMS; c++) {

    std::vector<double> vals;
    for (int it = 1; it < numQuats - 1; it++)
      vals.push_back(curvature[it * NUM_QUAT_PARAMS + c]);
    median_curvature[c] = vw::median_in_place<double>(vals);
  }

  // For synthetic cameras, the median initial curvature is 0. That is not good.
  // Ensure a decent lower bound for the median curvature by finding the average 
  // of median curvatures and using 10% of that at least for each component.
  double avg = 0.0;
  for (int c = 0; c < NUM_QUAT_PARAMS; c++)
    avg += median_curvature[c];
  avg /= NUM_QUAT_PARAMS;
  for (int c = 0; c < NUM_QUAT_PARAMS; c++)
    median_curvature[c] = std::max(median_curvature[c], 0.1 * avg);

  // Make each curvature, even the ones at endpoints, be at least the median curvature
  for (int it = 0; it < numQuats; it++) {
    for (int c = 0; c < NUM_QUAT_PARAMS; c++) {
      if (curvature[it * NUM_QUAT_PARAMS + c] < median_curvature[c])
        curvature[it * NUM_QUAT_PARAMS + c] = median_curvature[c];
    }
  }

  return;
}

// Add the quaternion smoothness constraint. This prevents high-frequency
// oscillations in the camera orientation.
struct QuatSmoothnessErr {

QuatSmoothnessErr(double const* smoothnessWeight) {

  // Make a local copy of the smoothness weight, so it does not go out of scope.
  m_smoothnessWeight.resize(NUM_QUAT_PARAMS);
  for (int it = 0; it < NUM_QUAT_PARAMS; it++)
    m_smoothnessWeight[it] = smoothnessWeight[it];
}

// The implementation is further down
bool operator()(double const * const * parameters, double * residuals) const;

// Factory to hide the construction of the CostFunction object from the client code.
static ceres::CostFunction* Create(double const* smoothnessWeight,
                                    int begQuatIndex,
                                    int endQuatIndex) {

  // TODO(oalexan1): Try using here the analytical cost function
  ceres::DynamicNumericDiffCostFunction<QuatSmoothnessErr>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<QuatSmoothnessErr>
    (new QuatSmoothnessErr(smoothnessWeight));

  // The residual size is NUM_QUAT_PARAMS: the curvature per each component
  cost_function->SetNumResiduals(NUM_QUAT_PARAMS);

  // Add a parameter block for each quaternion and each position
  for (int it = begQuatIndex; it <= endQuatIndex; it++)
    cost_function->AddParameterBlock(NUM_QUAT_PARAMS);

  return cost_function;
}

private:
  std::vector<double> m_smoothnessWeight;
}; // End class QuatSmoothnessErr

// A function that first increases slowly, then very fast, but without being
// numerically unstable for very small or very large values of the input.
double signed_exp(double val) {
  double ans = val * (std::exp(std::abs(val)) - 1.0);
  return ans;
}

// See the documentation higher up in the file.
bool QuatSmoothnessErr::operator()(double const * const * parameters,
                                   double * residuals) const {

  // parameters[0] has has q[index-1], with four components (x, y, z, w)
  // parameters[1] has has q[index]
  // parameters[2] has has q[index+1]
  for (int it = 0; it < NUM_QUAT_PARAMS; it++) {
    double curvature = parameters[0][it] - 2.0 * parameters[1][it] + parameters[2][it];

    // Use a power of 8 so that this kicks in only when the curvature becomes
    // big, but then it does so big time. Note that smoothnessWeight 
    // takes into account the initial curvature, so this residual is normalized.
    //residuals[it] = m_smoothnessWeight[it] * curvature;
    residuals[it] = asp::signed_exp(m_smoothnessWeight[it] * curvature);
  }

  return true;
}

void addQuatSmoothnessErr(UsgsAstroLsSensorModel   * ls_model,
                          int                        quatIndex,
                          double              const* smoothnessWeight,
                          std::vector<double>      & weight_per_residual,
                          ceres::Problem           & problem) {

  // For a smoothness constraint need the current quaternion and its neighbors.
  int begQuatIndex = quatIndex - 1, endQuatIndex = quatIndex + 1;

  // Sanity check
  int numQuats = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  if (begQuatIndex < 0 || endQuatIndex >= numQuats)
    vw::vw_throw(vw::ArgumentErr()
                 << "Quat index out of bounds in the smoothness constraint.\n");

  ceres::CostFunction* pixel_cost_function =
    QuatSmoothnessErr::Create(smoothnessWeight, begQuatIndex, endQuatIndex);
  ceres::LossFunction* pixel_loss_function = NULL; // no attenuation

  // The variable of optimization are camera quaternions. Notice how the
  // quaternion indices are in the [begQuatIndex, endQuatIndex] range, so
  // inclusive at both ends.
  std::vector<double*> vars;
  for (int it = begQuatIndex; it <= endQuatIndex; it++)
    vars.push_back(&ls_model->m_quaternions[it * NUM_QUAT_PARAMS]);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  // Need this for bookkeeping. Won't be used as do do not save the residuals 
  // for this cost function. If we ever do, need to take into account the internal
  // raising to the power and the smoothness weight.
  for (int it = 0; it < NUM_QUAT_PARAMS; it++)
    weight_per_residual.push_back(1.0);

  return;
}

// Build the 3 cross-track look rays for one orientation sample at the initial
// pose, by intersecting each ray with the mean-terrain sphere. Shared by the
// constraint and the offset report so they cannot drift apart. Also returns the
// body-frame rays (R(init) * body = world). False if a ray cannot be formed.
static bool buildOrientationSampleRays(asp::BaBaseOptions const& opt, int icam,
                                       UsgsAstroLsSensorModel * ls_model,
                                       int iq, int numQuat,
                                       double numSamples, double numLines,
                                       double mean_radius,
                                       // Outputs
                                       std::vector<vw::Vector3> & world_rays,
                                       std::vector<vw::Vector3> & body_rays) {
  world_rays.clear();
  body_rays.clear();

  // Three cross-track rays: ~10% in from each edge gives yaw leverage, the
  // center gives roll/pitch. Keep it at exactly 3 (matches the fixed residual
  // count of GroundRotationError).
  double lever_samples[3] = {0.1*numSamples, 0.5*numSamples, 0.9*numSamples};

  // Approximate image line for this orientation sample (samples are tabulated
  // ~uniformly over the image lines). The ray only needs to sit near the sample.
  double line = (double(iq) / double(numQuat - 1)) * (numLines - 1);

  vw::Matrix3x3 Rinit
    = asp::quaternionToMatrix(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
  vw::Matrix3x3 RinitT = vw::transpose(Rinit);

  for (int s = 0; s < 3; s++) {
    vw::Vector2 pix(lever_samples[s], line);
    vw::Vector3 C, v;
    try {
      C = opt.camera_models[icam]->camera_center(pix);
      v = opt.camera_models[icam]->pixel_to_vector(pix); // unit, world frame
    } catch (...) {
      return false;
    }
    // Ray-sphere intersection with the mean-terrain sphere (centered at the
    // planet center = ECEF origin). This is the cooked-up ground point on the
    // ray; it needs no DEM and works beyond any DEM extent.
    double Cv = dot_prod(C, v);
    double CC = dot_prod(C, C);
    double disc = Cv*Cv - (CC - mean_radius*mean_radius);
    double t;
    if (disc >= 0)
      t = -Cv - std::sqrt(disc); // near intersection
    else
      t = norm_2(C) - mean_radius; // fallback: nadir distance to the sphere
    if (t <= 0)
      return false;

    vw::Vector3 Lw = t * v;       // world look ray (ground point - camera_center)
    vw::Vector3 B  = RinitT * Lw; // body-frame ray, so R(init) * B = Lw
    world_rays.push_back(Lw);
    body_rays.push_back(B);
  }

  return (world_rays.size() == 3);
}

// Add the orientation-uncertainty constraint: a unary GroundRotationError on
// every orientation knot of every linescan camera. sigma is in meters.
void addOrientationUncertaintyConstraint(asp::BaBaseOptions          const& opt,
                                         std::vector<asp::CsmModel*> const& csm_models,
                                         double                             sigma,
                                         double                             mean_radius,
                                         // Outputs
                                         std::vector<double>              & weight_per_residual,
                                         ceres::Problem                   & problem) {

  if (sigma <= 0)
    return;

  vw::vw_out() << "Adding per-pose orientation uncertainty constraint, sigma = "
               << sigma << " m, mean terrain radius = " << mean_radius << " m.\n";

  int num_cams = opt.camera_models.size();
  int num_samples = 0;

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (ls_model == NULL)
      continue; // linescan only

    vw::Vector2 dims = vw::file_image_size(opt.image_files[icam]);
    double numSamples = dims[0], numLines = dims[1];
    int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
    if (numQuat < 2 || numSamples < 2 || numLines < 2)
      continue;

    for (int iq = 0; iq < numQuat; iq++) {

      std::vector<vw::Vector3> world_rays, body_rays;
      bool ok = buildOrientationSampleRays(opt, icam, ls_model, iq, numQuat,
                                           numSamples, numLines, mean_radius,
                                           world_rays, body_rays);
      if (!ok)
        continue;

      ceres::CostFunction* cost_function
        = GroundRotationError::Create(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS],
                                      body_rays, world_rays, sigma);
      // No loss function: a robust loss would release runaways, defeating the guard.
      problem.AddResidualBlock(cost_function, NULL,
                               &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
      for (int c = 0; c < 9; c++)
        weight_per_residual.push_back(1.0 / sigma);
      num_samples++;
    }
  }

  vw::vw_out() << "Added orientation uncertainty constraints to " << num_samples
               << " orientation samples.\n";
  return;
}

// Snapshot the initial look rays (before optimization) for the
// orientation-offset report. See the header for the output layout.
void collectOrientationRays(asp::BaBaseOptions          const& opt,
                            std::vector<asp::CsmModel*> const& csm_models,
                            double                             mean_radius,
                            // Output
                            std::vector<std::vector<asp::OrientationSampleRays>> & rays) {

  int num_cams = opt.camera_models.size();
  rays.clear();
  rays.resize(num_cams);

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (ls_model == NULL)
      continue; // linescan only

    vw::Vector2 dims = vw::file_image_size(opt.image_files[icam]);
    double numSamples = dims[0], numLines = dims[1];
    int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
    if (numQuat < 2 || numSamples < 2 || numLines < 2)
      continue;

    for (int iq = 0; iq < numQuat; iq++) {

      std::vector<vw::Vector3> world_rays, body_rays;
      bool ok = buildOrientationSampleRays(opt, icam, ls_model, iq, numQuat,
                                           numSamples, numLines, mean_radius,
                                           world_rays, body_rays);
      if (!ok)
        continue;

      asp::OrientationSampleRays sample;
      sample.quat_index = iq;
      sample.look_rays = world_rays;
      sample.init_quat.resize(NUM_QUAT_PARAMS);
      for (int it = 0; it < NUM_QUAT_PARAMS; it++)
        sample.init_quat[it] = ls_model->m_quaternions[iq * NUM_QUAT_PARAMS + it];
      rays[icam].push_back(sample);
    }
  }

  return;
}

// Write the per-camera orientation-offset report: for each sample, the largest
// ground displacement (meters) over its 3 look rays between the initial and
// optimized rotation, aggregated per camera (mean, median, min, max, count).
void saveOrientationOffsets(asp::BaBaseOptions          const& opt,
                            std::vector<asp::CsmModel*> const& csm_models,
                            std::vector<std::vector<asp::OrientationSampleRays>> const& rays,
                            double                             sigma,
                            std::string                 const& orientation_offsets_file) {

  int num_cams = opt.camera_models.size();
  if ((int)rays.size() != num_cams)
    return; // no rays were collected (e.g. no linescan cameras)

  vw::vw_out() << "Writing: " << orientation_offsets_file << "\n";
  std::ofstream ofs(orientation_offsets_file.c_str());
  ofs.precision(8);
  ofs << "# Per-image ground displacement (meters) from the change in camera "
      << "orientation\n";
  ofs << "# (initial vs optimized), measured on the mean-terrain sphere with the "
      << "camera center held fixed.\n";
  if (sigma > 0)
    ofs << "# orientation-uncertainty sigma = " << sigma << " m\n";
  ofs << "# Image mean median min max count\n";

  double nan = std::numeric_limits<double>::quiet_NaN();

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());

    // One value per orientation sample: the max over its 3 look rays.
    std::vector<double> offsets;

    if (ls_model != NULL) {
      for (size_t k = 0; k < rays[icam].size(); k++) {

        asp::OrientationSampleRays const& sample = rays[icam][k];
        int iq = sample.quat_index;
        if ((iq + 1) * NUM_QUAT_PARAMS > (int)ls_model->m_quaternions.size())
          continue; // sample no longer present (should not happen)

        vw::Matrix3x3 Ropt
          = asp::quaternionToMatrix(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
        vw::Matrix3x3 Rinit = asp::quaternionToMatrix(&sample.init_quat[0]);
        // The rotation drift applied in the world frame.
        vw::Matrix3x3 Rdelta = Ropt * vw::transpose(Rinit);

        double sample_offset = 0.0;
        for (size_t s = 0; s < sample.look_rays.size(); s++) {
          vw::Vector3 Lw = sample.look_rays[s];
          double disp = norm_2(Rdelta * Lw - Lw); // how far the ground end moved
          sample_offset = std::max(sample_offset, disp);
        }
        offsets.push_back(sample_offset);
      }
    }

    double mean = nan, median = nan, mn = nan, mx = nan, count = 0;
    if (!offsets.empty()) {
      mean  = vw::mean(offsets);
      mn    = *std::min_element(offsets.begin(), offsets.end());
      mx    = *std::max_element(offsets.begin(), offsets.end());
      median = vw::median_in_place<double>(offsets); // reorders offsets
      count = offsets.size();
    }
    ofs << opt.image_files[icam] << " " << mean << " " << median << " "
        << mn << " " << mx << " " << count << "\n";
  }
  ofs.close();

  return;
}

// Add a constraint that the curvature of the sequence of poses does not become
// a lot more than the initial one.
void addSmoothnessConstraint(asp::BaBaseOptions               const& opt,
                             std::vector<asp::CsmModel*>      const& csm_models,
                             double                                  smoothness_weight,
                             bool                                    have_rig,
                             rig::RigSet                      const& rig,
                             std::vector<asp::RigCamInfo>     const& rig_cam_info,
                             // Outputs
                             std::vector<double>                & weight_per_residual,
                             std::vector<std::vector<double>>   & orig_curvatures,
                             ceres::Problem                     & problem) {

  if (smoothness_weight < 0.0 || !std::isfinite(smoothness_weight))
    vw::vw_throw(vw::ArgumentErr()
      << "The smoothness weight must be non-negative.\n");

  // orig_curvatures must be either empty or of size num_cams
  int num_cams = csm_models.size();
  if (orig_curvatures.size() != 0 && orig_curvatures.size() != num_cams)
    vw::vw_throw(vw::ArgumentErr()
      << "The orig_curvatures vector must be empty or of size equal "
      << "to the number of cameras.\n");
   if (orig_curvatures.empty())
    orig_curvatures.resize(num_cams);

  for (int icam = 0; icam < num_cams; icam++) {

    // With a rig, only the ref sensor has rotation constraints 
    if (have_rig && !rig.isRefSensor(rig_cam_info[icam].sensor_id))
      continue;

    // Get the underlying linescan models      
    csm::RasterGM * csm = csm_models[icam]->m_gm_model.get();
    UsgsAstroLsSensorModel * ls_model = dynamic_cast<UsgsAstroLsSensorModel*>(csm);
    if (ls_model == NULL)
      continue; // not a linescan camera

    // Estimate the curvature of the quaternion sequence. Do this only once,
    // with the original cameras, to keep the same constraint for all passes.
    if (orig_curvatures[icam].empty())
      estimCurvature(ls_model, orig_curvatures[icam]);

    // Let the weight be inversely proportional to the curvature. This way the
    // current curvature is normalized by the initial curvature. The multiplier
    // below ensures the optimized curvature can be no more than a factor the
    // initial curvature. This was carefully tested with real and synthetic data.
    std::vector<double> weights(orig_curvatures[icam].size());
    for (size_t it = 0; it < orig_curvatures[icam].size(); it++)
      weights[it] = 0.001 * smoothness_weight / orig_curvatures[icam][it];

    // Add the smoothness constraint for the quaternions. Cannot have such
    // a constraint for the first and last quaternion, as need neighbors.
    int numQuats = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
    for (int it = 1; it < numQuats - 1; it++)
      addQuatSmoothnessErr(ls_model, it, &weights[it * NUM_QUAT_PARAMS],
                           weight_per_residual, problem);

  } // end loop through cameras

  return;
}

} // end namespace asp
