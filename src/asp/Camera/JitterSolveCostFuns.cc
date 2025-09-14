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

// Cost functions used in solving for jitter. These need access to the camera models,
// so they are stored in the Camera folder.

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveRigCostFuns.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/SatSimBase.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/DataLoader.h>

#include <vw/Camera/CameraImage.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/Math/Transform.h>
#include <vw/Image/Interpolation.h>
#include <vw/Cartography/Map2CamTrans.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>

namespace asp {

// An error function minimizing the error of projecting an xyz point
// into a given CSM linescan camera pixel. The variables of optimization are a
// portion of the position and quaternion variables affected by this, and the 
// triangulation point.
struct LsPixelReprojErr {
  LsPixelReprojErr(vw::Vector2 const& observation, double weight,
                   UsgsAstroLsSensorModel* ls_model,
                   int begQuatIndex, int endQuatIndex, 
                   int begPosIndex, int endPosIndex):
    m_observation(observation), m_weight(weight),
    m_begQuatIndex(begQuatIndex), m_endQuatIndex(endQuatIndex),
    m_begPosIndex(begPosIndex),   m_endPosIndex(endPosIndex),
    m_ls_model(ls_model) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const; 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& observation, double weight,
                                     UsgsAstroLsSensorModel* ls_model,
                                     int begQuatIndex, int endQuatIndex,
                                     int begPosIndex, int endPosIndex) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>
      (new LsPixelReprojErr(observation, weight, ls_model,
                                  begQuatIndex, endQuatIndex,
                                  begPosIndex, endPosIndex));

    // The residual size is always the same.
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each quaternion and each position
    for (int it = begQuatIndex; it < endQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    for (int it = begPosIndex; it < endPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);

    // Add a parameter block for the xyz point
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);

    return cost_function;
  }

private:
  vw::Vector2 m_observation; // The pixel observation for this camera/point pair
  double m_weight;
  UsgsAstroLsSensorModel* m_ls_model;
  int m_begQuatIndex, m_endQuatIndex;
  int m_begPosIndex, m_endPosIndex;
}; // End class LsPixelReprojErr

// An error function minimizing the error of projecting an xyz point
// into a given CSM Frame camera pixel. The variables of optimization are 
// the camera position, quaternion, and triangulation point.
struct FramePixelReprojErr {
  FramePixelReprojErr(vw::Vector2 const& observation, double weight,
                   UsgsAstroFrameSensorModel* frame_model):
    m_observation(observation), m_weight(weight),
    m_frame_model(frame_model) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const; 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& observation, double weight,
                                     UsgsAstroFrameSensorModel* frame_model) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<FramePixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<FramePixelReprojErr>
      (new FramePixelReprojErr(observation, weight, frame_model));

    // The residual size is always the same.
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each position and quaternion, in this order
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    cost_function->AddParameterBlock(NUM_QUAT_PARAMS);

    // Add a parameter block for the xyz point
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    
    return cost_function;
  }

private:
  vw::Vector2 m_observation; // The pixel observation for this camera/point pair
  double m_weight;
  UsgsAstroFrameSensorModel* m_frame_model;
}; // End class FramePixelReprojErr

// Update the linescan model with the latest optimized values of the position
// and quaternion parameters. Also update the triangulated point.
void updateLsModelTriPt(double const * const * parameters, 
                        int begQuatIndex, int endQuatIndex,
                        int begPosIndex, int endPosIndex,
                        int & param_shift,
                        UsgsAstroLsSensorModel & cam,
                        csm::EcefCoord & P) {
    
  // Start at the first param
  param_shift = 0;
  
  // Update the relevant quaternions in the local copy
  for (int qi = begQuatIndex; qi < endQuatIndex; qi++) {
    for (int coord = 0; coord < NUM_QUAT_PARAMS; coord++) {
      cam.m_quaternions[NUM_QUAT_PARAMS * qi + coord]
        = parameters[qi + param_shift - begQuatIndex][coord];
    }
  }

  // Same for the positions. Note how we move forward in the parameters array,
  // as this is after the quaternions
  param_shift += (endQuatIndex - begQuatIndex);
  for (int pi = begPosIndex; pi < endPosIndex; pi++) {
    for (int coord = 0; coord < NUM_XYZ_PARAMS; coord++) {
      cam.m_positions[NUM_XYZ_PARAMS * pi + coord]
        = parameters[pi + param_shift - begPosIndex][coord];
    }
  }

  // Move forward in the array of parameters, then recover the triangulated point
  param_shift += (endPosIndex - begPosIndex);
  P.x = parameters[param_shift][0];
  P.y = parameters[param_shift][1];
  P.z = parameters[param_shift][2];
}  

// See the documentation higher up in the file.
bool LsPixelReprojErr::operator()(double const * const * parameters, 
                                  double * residuals) const {

  try {
    // Make a copy of the model, as we will update quaternion and position
    // values that are being modified now. This may be expensive.
    // Update the shift too.
    UsgsAstroLsSensorModel cam = *m_ls_model;
    int shift = 0;
    csm::EcefCoord P;
    updateLsModelTriPt(parameters, m_begQuatIndex, m_endQuatIndex,
                       m_begPosIndex, m_endPosIndex, shift, cam, P);

    // Project in the camera with high precision. Do not use here
    // anything lower than 1e-8, as the linescan model will then
    // return junk.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
    csm::ImageCoord imagePt = cam.groundToImage(P, desired_precision);

    // Convert to what ASP expects
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);

    residuals[0] = m_weight*(pix[0] - m_observation[0]);
    residuals[1] = m_weight*(pix[1] - m_observation[1]);
    
  } catch (std::exception const& e) {
    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return true; // accept the solution anyway
  }

  return true;
}

// See the .h file for the documentation.
bool FramePixelReprojErr::operator()(double const * const * parameters, 
                                     double * residuals) const {

  try {
    // Make a copy of the model, as we will update position and quaternion
    // values that are being modified now. Use the same order as in
    // UsgsAstroFrameSensorModel::m_currentParameterValue.
    UsgsAstroFrameSensorModel cam = *m_frame_model;

    // The latest position is in parameters[0].
    for (int coord = 0; coord < NUM_XYZ_PARAMS; coord++)
      cam.setParameterValue(coord, parameters[0][coord]);

    // The latest quaternion is in parameters[1]. Note how we below
    // move forward when invoking cam.setParameterValue().
    for (int coord = 0; coord < NUM_QUAT_PARAMS; coord++) 
      cam.setParameterValue(coord + NUM_XYZ_PARAMS, parameters[1][coord]);

    // The triangulation parameter is after the position and orientation
    csm::EcefCoord P;
    P.x = parameters[2][0];
    P.y = parameters[2][1];
    P.z = parameters[2][2];

    // Project in the camera with high precision. Do not use here
    // anything lower than 1e-8, as the linescan model will then
    // return junk.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
    csm::ImageCoord imagePt = cam.groundToImage(P, desired_precision);

    // Convert to what ASP expects
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);

    residuals[0] = m_weight*(pix[0] - m_observation[0]);
    residuals[1] = m_weight*(pix[1] - m_observation[1]);
    
  } catch (std::exception const& e) {
    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return true; // accept the solution anyway
  }

  return true;
}

// A Ceres cost function. The residual is the roll and/or yaw component of the camera
// rotation, as measured relative to the initial along-track direction. We assume
// that all positions are along the same segment in projected coordinates, or at
// least that the current position and its nearest neighbors are roughly on
// such a segment. That one is used to measure the roll/yaw from. This is consistent
// with how sat_sim creates the cameras.
struct weightedRollYawError {
  weightedRollYawError(std::vector<double>       const& positions, 
                   std::vector<double>           const& quaternions,
                   vw::cartography::GeoReference const& georef,
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
                                     vw::cartography::GeoReference const& georef,
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
                   vw::cartography::GeoReference const& georef,
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
    vw::Vector3 beg_proj = vw::cartography::ecefToProj(georef, beg_pt);
    vw::Vector3 cur_proj = vw::cartography::ecefToProj(georef, cur_pt);
    vw::Vector3 end_proj = vw::cartography::ecefToProj(georef, end_pt);
    
    // Find satellite along and across track directions in projected coordinates
    vw::Vector3 proj_along, proj_across;
    asp::calcProjAlongAcross(beg_proj, end_proj, proj_along, proj_across);

    // Find along and across in ECEF
    vw::Vector3 along, across;
    asp::calcEcefAlongAcross(georef, asp::satSimDelta(), 
                              proj_along, proj_across, cur_proj,
                              along, across); // outputs

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
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
    vw::Matrix3x3 cam2cam =  vw::math::inverse(cam2world) * m_initCam2World;

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
    = vw::math::inverse(m_sat2World) * cam2world * vw::math::inverse(m_rotXY);

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

// Calc the range of indices in the samples needed to interpolate between time1 and time2.
// Based on lagrangeInterp() in usgscsm.
void calcIndexBounds(double time1, double time2, double t0, double dt, int numVals,
                     // Outputs
                     int & begIndex, int & endIndex) {

  // Order of Lagrange interpolation
  int numInterpSamples = 8;

  // Starting and ending  index (ending is exclusive).
  int index1 = static_cast<int>((time1 - t0) / dt);
  int index2 = static_cast<int>((time2 - t0) / dt);
  
  // TODO(oalexan1): Maybe the indices should be more generous, so not adding 1
  // to begIndex, even though what is here seems correct according to 
  // lagrangeInterp().
  begIndex = std::min(index1, index2) - numInterpSamples / 2 + 1;
  endIndex = std::max(index1, index2) + numInterpSamples / 2 + 1;
  
  // Keep in bounds
  begIndex = std::max(0, begIndex);
  endIndex = std::min(endIndex, numVals);
  if (begIndex >= endIndex)
    vw::vw_throw(vw::ArgumentErr() << "Book-keeping error in interpolation. " 
      << "Likely image order is different than camera order.\n"); 
    
  return;
}

// Find the positions and orientations in the current linescan model
// that can affect the given pixel.
void calcPosQuatIndexBounds(double line_extra,
                            UsgsAstroLsSensorModel * ls_model,
                            vw::Vector2 const& observation,
                            // Outputs
                            int & begPosIndex,  int & endPosIndex,
                            int & begQuatIndex, int & endQuatIndex) {

  // Initialize the outputs
  begPosIndex = -1; endPosIndex = -1;
  begQuatIndex = -1; endQuatIndex = -1;

  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(observation - vw::Vector2(0.0, line_extra), imagePt1);
  asp::toCsmPixel(observation + vw::Vector2(0.0, line_extra), imagePt2);
  double time1 = ls_model->getImageTime(imagePt1);
  double time2 = ls_model->getImageTime(imagePt2);

  // Positions
  int numPos       = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double posT0     = ls_model->m_t0Ephem;
  double posDt     = ls_model->m_dtEphem;
  calcIndexBounds(time1, time2, posT0, posDt, numPos, 
                  begPosIndex, endPosIndex); // outputs

  // Quaternions
  int numQuat       = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double quatT0     = ls_model->m_t0Quat;
  double quatDt     = ls_model->m_dtQuat;
  calcIndexBounds(time1, time2, quatT0, quatDt, numQuat, 
                  begQuatIndex, endQuatIndex); // outputs
  return;
}
   
// Add the linescan model reprojection error to the cost function
void addLsReprojectionErr(asp::BaBaseOptions  const& opt,
                          UsgsAstroLsSensorModel   * ls_model,
                          vw::Vector2         const& observation,
                          double                   * tri_point,
                          double                     weight,
                          ceres::Problem           & problem) {

  // Find all positions and quaternions that can affect the current pixel. Must
  // grow the number of quaternions and positions a bit because during
  // optimization the 3D point and corresponding pixel may move somewhat.
  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  int begPosIndex = -1, endPosIndex = -1;
  int begQuatIndex = -1, endQuatIndex = -1;
  calcPosQuatIndexBounds(line_extra, ls_model, observation, 
                         begPosIndex, endPosIndex, begQuatIndex, endQuatIndex); // outputs
  
  ceres::CostFunction* pixel_cost_function =
    LsPixelReprojErr::Create(observation, weight, ls_model,
                             begQuatIndex, endQuatIndex,
                             begPosIndex, endPosIndex);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera quaternions and positions stored in the
  // camera models, and the triangulated point.
  std::vector<double*> vars;
  for (int it = begQuatIndex; it < endQuatIndex; it++)
    vars.push_back(&ls_model->m_quaternions[it * NUM_QUAT_PARAMS]);
  for (int it = begPosIndex; it < endPosIndex; it++)
    vars.push_back(&ls_model->m_positions[it * NUM_XYZ_PARAMS]);
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

// Add the frame camera model reprojection error to the cost function
void addFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                             UsgsAstroFrameSensorModel * frame_model,
                             vw::Vector2         const & observation,
                             double                    * frame_params,
                             double                    * tri_point,
                             double                      weight,
                             ceres::Problem            & problem) {

  ceres::CostFunction* pixel_cost_function =
    FramePixelReprojErr::Create(observation, weight, frame_model);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera positions and quaternion stored 
  // in frame_cam_params, in this order, and the triangulated point.
  // This is different from the linescan model, where we can directly access
  // these quantities inside the model, so they need not be stored separately.
  std::vector<double*> vars;
  vars.push_back(&frame_params[0]);              // positions start here
  vars.push_back(&frame_params[NUM_XYZ_PARAMS]); // quaternions start here
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

// Add the ls or frame camera model reprojection error to the cost function
void addLsOrFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                                 int                         icam,
                                 UsgsAstroLsSensorModel    * ls_model,
                                 UsgsAstroFrameSensorModel * frame_model,
                                 std::vector<double>       & frame_params,
                                 vw::Vector2         const & pix_obs,
                                 double                      pix_wt,
                                 double                    * tri_point,
                                 ceres::Problem            & problem) {

  // Note how for the frame model we pass the frame_params for the current camera.
  // For the linescan cameras the params are changed inside the model.
  if (ls_model != NULL)
    addLsReprojectionErr(opt, ls_model, pix_obs, tri_point, pix_wt, problem);
  else if (frame_model != NULL)
    addFrameReprojectionErr(opt, frame_model, pix_obs, 
        &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)],
        tri_point, pix_wt, problem);                   
  else
    vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
}

/// A ceres cost function. The residual is the difference between the observed
/// 3D point and the current (floating) 3D point, multiplied by given weight.
struct weightedXyzError {
  weightedXyzError(vw::Vector3 const& observation, double weight):
    m_observation(observation), m_weight(weight){}

  template <typename T>
  bool operator()(const T* point, T* residuals) const {
    for (size_t p = 0; p < m_observation.size(); p++)
      residuals[p] = m_weight * (point[p] - m_observation[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(vw::Vector3 const& observation, double const& weight) {
    return (new ceres::AutoDiffCostFunction<weightedXyzError, 3, 3>
            (new weightedXyzError(observation, weight)));
  }

  vw::Vector3 m_observation;
  double  m_weight;
};

/// A Ceres cost function. The residual is the difference between the
/// initial quaternion and optimized quaternion, multiplied by given weight.
struct weightedRotationError {
  weightedRotationError(const double * init_quat, double weight):
    m_weight(weight) {

    // Make a copy, as later the value at the pointer will change
    m_init_quat.resize(NUM_QUAT_PARAMS);
    for (int it = 0; it < NUM_QUAT_PARAMS; it++)
      m_init_quat[it] = init_quat[it];
  }

  template <typename T>
  bool operator()(const T* quat, T* residuals) const {
    for (size_t p = 0; p < m_init_quat.size(); p++)
      residuals[p] = m_weight * (quat[p] - m_init_quat[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double * init_quat, double weight){
    return (new ceres::AutoDiffCostFunction<weightedRotationError,
            NUM_QUAT_PARAMS, NUM_QUAT_PARAMS>
            (new weightedRotationError(init_quat, weight)));
  }

  std::vector<double> m_init_quat;
  double  m_weight;
};

/// A Ceres cost function. The residual is the difference between the
/// initial position and optimized position, multiplied by given weight.
struct weightedTranslationError {
  weightedTranslationError(const double * init_position, double weight):
    m_weight(weight) {

    // Make a copy, as later the value at the pointer will change
    m_init_position.resize(NUM_XYZ_PARAMS);
    for (int it = 0; it < NUM_XYZ_PARAMS; it++)
      m_init_position[it] = init_position[it];
  }

  template <typename T>
  bool operator()(const T* position, T* residuals) const {
    for (size_t p = 0; p < m_init_position.size(); p++)
      residuals[p] = m_weight * (position[p] - m_init_position[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double * init_position, double weight){
    return (new ceres::AutoDiffCostFunction
            <weightedTranslationError, NUM_XYZ_PARAMS, NUM_XYZ_PARAMS>
            (new weightedTranslationError(init_position, weight)));
  }

  std::vector<double> m_init_position;
  double  m_weight;
};

/// A Ceres cost function. The residual is the weighted difference between 1 and
/// norm of quaternion.
struct weightedQuatNormError {
  weightedQuatNormError(double weight):
    m_weight(weight) {}

  template <typename T>
  bool operator()(const T* quat, T* residuals) const {
    residuals[0] = T(0.0);
    for (size_t p = 0; p < NUM_QUAT_PARAMS; p++)
      residuals[0] += quat[p] * quat[p];

    residuals[0] = m_weight * (residuals[0] - 1.0);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double weight) {
    return (new ceres::AutoDiffCostFunction<weightedQuatNormError, 1, NUM_QUAT_PARAMS>
            (new weightedQuatNormError(weight)));
  }

  double  m_weight;
};

// Add reprojection errors. Collect data that will be used to add camera
// constraints that scale with the number of reprojection errors and GSD.
void addReprojCamErrs(asp::BaBaseOptions                    const & opt,
                      asp::CRNJ                             const & crn,
                      std::vector<std::vector<vw::Vector2>> const & pixel_vec,
                      std::vector<std::vector<double>>      const & weight_vec,
                      std::vector<std::vector<int>>         const & isAnchor_vec,
                      std::vector<std::vector<int>>         const & pix2xyz_index,
                      std::vector<asp::CsmModel*>           const & csm_models,
                      bool                                          have_rig,
                      rig::RigSet                           const & rig,
                      std::vector<RigCamInfo>               const & rig_cam_info,
                      std::map<int, int>                    const & cam2group,
                      TimestampMap                          const & timestamp_map,
                      bool                                          fix_rig_translations,
                      bool                                          fix_rig_rotations,
                      // Outputs
                      std::vector<double>                     & tri_points_vec,
                      std::vector<double>                     & frame_params,
                      std::vector<double>                     & weight_per_residual,
                      std::vector<std::vector<double>>        & weight_per_cam,
                      std::vector<std::vector<double>>        & count_per_cam,
                      std::vector<double>                     & ref_to_curr_sensor_vec,
                      ceres::Problem                          & problem) {

  // Do here two passes, first for non-anchor points and then for anchor ones.
  // This way it is easier to do the bookkeeping when saving the residuals.
  // Note: The same motions as here are repeated in saveJitterResiduals().
  weight_per_cam.resize(2);
  count_per_cam.resize(2);
  for (int pass = 0; pass < 2; pass++) {
    
     weight_per_cam[pass].resize((int)crn.size(), 0.0);
     count_per_cam[pass].resize((int)crn.size(), 0.0);

    for (int icam = 0; icam < (int)crn.size(); icam++) {

      vw::DiskImageView<float> img(opt.image_files[icam]);
      vw::BBox2 image_box = bounding_box(img);
      std::vector<double> this_cam_weights;

      for (size_t ipix = 0; ipix < pixel_vec[icam].size(); ipix++) {

        vw::Vector2 pix_obs = pixel_vec[icam][ipix];
        double * tri_point  = &tri_points_vec[3 * pix2xyz_index[icam][ipix]];
        double pix_wt       = weight_vec[icam][ipix];
        bool isAnchor       = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them
        if ((int)isAnchor != pass) 
          continue;
        
        csm::RasterGM * csm = csm_models[icam]->m_gm_model.get();
        UsgsAstroLsSensorModel * ls_model 
          = dynamic_cast<UsgsAstroLsSensorModel*>(csm);
        UsgsAstroFrameSensorModel * frame_model
          = dynamic_cast<UsgsAstroFrameSensorModel*>(csm);
  
        if (!have_rig) {
          // No rig
          addLsOrFrameReprojectionErr(opt, icam, ls_model, frame_model, frame_params, 
                                      pix_obs, pix_wt, tri_point, problem);
        } else {
          // Have rig
          asp::RigCamInfo rig_info = rig_cam_info[icam];  
          int ref_cam              = rig_info.ref_cam_index;
          int sensor_id            = rig_info.sensor_id;
          double* ref_to_curr_sensor_trans 
              = &ref_to_curr_sensor_vec[rig::NUM_RIGID_PARAMS * sensor_id];
          
          // We can have linescan or frame cameras for ref and curr cameras
          csm::RasterGM * ref_csm = csm_models[ref_cam]->m_gm_model.get();
          UsgsAstroLsSensorModel * ref_ls_model 
            = dynamic_cast<UsgsAstroLsSensorModel*>(ref_csm);
          UsgsAstroFrameSensorModel * ref_frame_model 
            = dynamic_cast<UsgsAstroFrameSensorModel*>(ref_csm);  

          if (rig.isRefSensor(sensor_id)) // Do not need the rig
            addLsOrFrameReprojectionErr(opt, icam, ls_model, frame_model, frame_params, 
                                        pix_obs, pix_wt, tri_point, problem);
          else // Use the rig
             asp::addRigLsOrFrameReprojectionErr(opt, icam, ref_ls_model, ref_frame_model, 
                                    ls_model, frame_model, frame_params, 
                                    csm_models, cam2group, timestamp_map,
                                    pix_obs, pix_wt, tri_point, 
                                    ref_to_curr_sensor_trans, rig_info, 
                                    fix_rig_translations, fix_rig_rotations,
                                    problem);

        } // end condition for having a rig
      
        // Two residuals were added. Save the corresponding weights.
        for (int c = 0; c < asp::PIXEL_SIZE; c++)
          weight_per_residual.push_back(pix_wt);

        // Anchor points are fixed by definition. They try to prevent
        // the cameras from moving too much from original poses.
        if (isAnchor) 
          problem.SetParameterBlockConstant(tri_point);
        
        // Find the weight to use with the camera constraint
        vw::Vector3 xyz_obs(tri_point[0], tri_point[1], tri_point[2]);
        double gsd = 0.0;
        try {
          gsd = vw::camera::estimatedGSD(opt.camera_models[icam].get(), image_box, 
                                         pix_obs, xyz_obs);
        } catch (...) {
          continue;
        }
        if (gsd <= 0) 
          continue; 

        // The camera position weight depends on the input multiplier, pixel weight, and gsd
        double position_wt = opt.camera_position_weight * pix_wt / gsd;
        this_cam_weights.push_back(position_wt);
      } // end iteration through pixels
      
      // Find the median weight and count. The median is more robust to outliers.
      count_per_cam[pass][icam] = this_cam_weights.size();
      if (count_per_cam[pass][icam] > 0)
        weight_per_cam[pass][icam] = vw::math::destructive_median(this_cam_weights);
      else
        weight_per_cam[pass][icam] = 0.0;
    } // end iteration through cameras
  } // end iteration through passes

  return;
}

// Add the constraint based on DEM
void addDemConstraint(asp::BaBaseOptions       const& opt,
                      std::vector<vw::Vector3> const& dem_xyz_vec,
                      std::set<int>            const& outliers,
                      vw::ba::ControlNetwork   const& cnet,
                      // Outputs
                      std::vector<double>           & tri_points_vec,
                      std::vector<double>           & weight_per_residual, // append
                      ceres::Problem                & problem) {
  
  double xyz_weight = -1.0, xyz_threshold = -1.0;
    
  if (!opt.heights_from_dem.empty()) {
    xyz_weight = 1.0/opt.heights_from_dem_uncertainty;
    xyz_threshold = opt.heights_from_dem_robust_threshold;
  } else {
    vw::vw_throw(vw::ArgumentErr() << "No input DEM was provided.\n");
  }
  
  if (dem_xyz_vec.size() != cnet.size()) 
    vw::vw_throw(vw::ArgumentErr() << "Must have as many xyz computed from DEM as xyz "
             << "triangulated from match files.\n");
  if (xyz_weight <= 0 || xyz_threshold <= 0)
    vw::vw_throw(vw::ArgumentErr() << "Detected invalid robust threshold or weights.\n");

  int num_tri_points = cnet.size();
  
  // The tri_points_vec must have at least as many points as cnet. It can have anchor points
  // as well.
  if ((int)tri_points_vec.size() < num_tri_points * NUM_XYZ_PARAMS)
    vw::vw_throw(vw::ArgumentErr() << "Too few triangulated points.\n");
  
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
      
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      continue; // Skip GCPs

    // Note that we get tri points from dem_xyz_vec, based on the input DEM
    vw::Vector3 observation = dem_xyz_vec.at(ipt);
    if (outliers.find(ipt) != outliers.end() || observation == vw::Vector3(0, 0, 0)) 
      continue; // outlier
      
    ceres::CostFunction* xyz_cost_function 
      = weightedXyzError::Create(observation, xyz_weight);
    ceres::LossFunction* xyz_loss_function = new ceres::CauchyLoss(xyz_threshold);
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;

    // Add cost function
    problem.AddResidualBlock(xyz_cost_function, xyz_loss_function, tri_point);

    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(xyz_weight);
  }
}

// Add the constraint to keep triangulated points close to initial values
// This does not need a DEM or alignment
void addTriConstraint(asp::BaBaseOptions     const& opt,
                      std::set<int>          const& outliers,
                      vw::ba::ControlNetwork const& cnet,
                      asp::CRNJ              const& crn,
                      // Outputs
                      std::vector<double>    & tri_points_vec,
                      std::vector<double>    & weight_per_residual, // append
                      ceres::Problem         & problem) {

  // Estimate the GSD for each triangulated point
  std::vector<double> gsds;
  asp::estimateGsdPerTriPoint(opt.image_files, opt.camera_models, crn, 
                              outliers, tri_points_vec, gsds);
  
  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint ||
        cnet[ipt].type() == vw::ba::ControlPoint::PointFromDem)
      continue; // Skip GCPs and height-from-dem points which have their own constraint

    if (outliers.find(ipt) != outliers.end()) 
      continue; // skip outliers
      
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
    
    // The weight must be inversely proportional to the GSD, to ensure
    // this is in pixel units
    double gsd = gsds[ipt];
    if (gsd <= 0) 
      continue; // GSD calculation failed. Do not use a constraint.
    double weight = opt.tri_weight / gsd;
  
    // Use as constraint the initially triangulated point
    vw::Vector3 observation(tri_point[0], tri_point[1], tri_point[2]);

    ceres::CostFunction* cost_function = weightedXyzError::Create(observation, weight);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(opt.tri_robust_threshold);
    problem.AddResidualBlock(cost_function, loss_function, tri_point);
    
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(opt.tri_weight);
      
  } // End loop through xyz
}

// Add the GCP constraint
void addGcpConstraint(asp::BaBaseOptions     const& opt,
                      std::set<int>          const& outliers,
                      bool                          use_llh_error,
                      bool                          fix_gcp_xyz,
                      // Outputs
                      vw::ba::ControlNetwork      & cnet,
                      std::vector<double>         & tri_points_vec,
                      std::vector<double>         & weight_per_residual, // append
                      ceres::Problem              & problem) {
  
  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    
    if (cnet[ipt].type() != vw::ba::ControlPoint::GroundControlPoint)
      continue; // Applies only to GCPs

    if (outliers.find(ipt) != outliers.end()) 
      continue; // skip outliers

    vw::Vector3 observation = cnet[ipt].position();
    vw::Vector3 xyz_sigma = cnet[ipt].sigma();

    // Use same cost function as for bundle adjustment
    ceres::CostFunction* cost_function = NULL;
    if (!use_llh_error) {
      cost_function = XYZError::Create(observation, xyz_sigma);
    } else {
      vw::Vector3 llh_sigma = xyz_sigma;
      // make lat, lon into lon, lat
      std::swap(llh_sigma[0], llh_sigma[1]);
      cost_function = LLHError::Create(observation, llh_sigma, opt.datum);
    }
    
    // No soft cost function for GCP. These are assumed to be accurate.
    ceres::LossFunction* loss_function  = new ceres::TrivialLoss();
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
    problem.AddResidualBlock(cost_function, loss_function, tri_point);
    
    // Ground xyz whose sigma is asp::FIXED_GCP_SIGMA (a tiny positive value) are set to fixed
    double s = asp::FIXED_GCP_SIGMA;
    if (fix_gcp_xyz || xyz_sigma == vw::Vector3(s, s, s)) {
      cnet[ipt].set_sigma(vw::Vector3(s, s, s)); // will be saved in the ISIS cnet
      problem.SetParameterBlockConstant(tri_point);
    }
    
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(1.0 / xyz_sigma[c]);
      
  } // End loop over triangulated points
}

// Add hard camera constraints. Be generous with the uncertainty. 
void addHardCamPositionConstraint(asp::BaBaseOptions               const& opt,
                                  std::set<int>                    const& outliers,
                                  asp::CRNJ                        const& crn,
                                  std::vector<asp::CsmModel*>      const& csm_models,
                                  std::vector<std::vector<double>> const& count_per_cam,
                                  double                                  anchor_weight,
                                  bool                                    have_rig,
                                  rig::RigSet                      const& rig,
                                  std::vector<asp::RigCamInfo>     const& rig_cam_info,
                                  // Outputs
                                  std::vector<double>                & frame_params,
                                  std::vector<double>                & weight_per_residual, 
                                  ceres::Problem                     & problem) {

  // Do not do a second pass for anchor points. A constraint for interest points
  // only seems to be enough.
  for (int pass = 0; pass < 1; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      
      // With a rig, only the ref sensor has rotation constraints 
      if (have_rig && !rig.isRefSensor(rig_cam_info[icam].sensor_id))
        continue;
      
      double count = count_per_cam[pass][icam];
      if (count <= 0) 
        continue; // no reprojection errors for this camera
    
      int param_len = NUM_XYZ_PARAMS;
      
      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
        
      if (ls_model != NULL) {
        
        // There are multiple position parameters per camera. Divide the
        // constraint between them. This was very carefully tested.
        int numPos = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
        double weight = 1.0 / std::max(double(numPos), 1.0);

        for (int pos_it = 0; pos_it < numPos; pos_it++) {

          // Must have both a pointer and the vector, as dictated by the API
          double * cam_ptr = &ls_model->m_positions[pos_it * NUM_XYZ_PARAMS];
          vw::Vector3 orig_cam(cam_ptr[0], cam_ptr[1], cam_ptr[2]);
          ceres::CostFunction* cost_function
                  = CamUncertaintyError::Create(orig_cam, cam_ptr, param_len,
                                        opt.camera_position_uncertainty[icam],
                                        weight, opt.datum);
          // This is a hard constraint, so we use a trivial loss function        
          ceres::LossFunction* loss_function = new ceres::TrivialLoss();
          problem.AddResidualBlock(cost_function, loss_function, cam_ptr);
          
          for (int c = 0; c < NUM_XYZ_PARAMS; c++)
            weight_per_residual.push_back(weight); // To ensure correct bookkeeping
        }
        
      } else if (frame_model != NULL) {
      
        // Same logic as for bundle_adjust
        // There is only one position per camera
        // Must have both a pointer and the vector, as dictated by the API
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
        vw::Vector3 orig_cam(curr_params[0], curr_params[1], curr_params[2]);

        double weight = 1.0;

        ceres::CostFunction* cost_function
          = CamUncertaintyError::Create(orig_cam, curr_params, param_len,
                                        opt.camera_position_uncertainty[icam],
                                        weight, opt.datum);
        // This is a hard constraint, so we use a trivial loss function        
        ceres::LossFunction* loss_function = new ceres::TrivialLoss();
        problem.AddResidualBlock(cost_function, loss_function,
                                &curr_params[0]);
        
        for (int c = 0; c < NUM_XYZ_PARAMS; c++)
          weight_per_residual.push_back(weight); // To ensure correct bookkeeping
          
      } else {
        vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }
    } // end loop through cameras
  } // end loop through passes

  return;
}

// Add camera constraints that are proportional to the number of reprojection errors.
// This requires going through some of the same motions as in addReprojCamErrs().
// Use instead addHardCamPositionConstraint().
void addSoftCamPositionConstraint(asp::BaBaseOptions           const& opt,
                              std::set<int>                    const& outliers,
                              asp::CRNJ                        const& crn,
                              std::vector<asp::CsmModel*>      const& csm_models,
                              std::vector<std::vector<double>> const& weight_per_cam,
                              std::vector<std::vector<double>> const& count_per_cam,
                              bool                                    have_rig,
                              rig::RigSet                      const& rig,
                              std::vector<asp::RigCamInfo>     const& rig_cam_info,
                              // Outputs
                              std::vector<double>                & frame_params,
                              std::vector<double>                & weight_per_residual, 
                              ceres::Problem                     & problem) {

  // First pass is for interest point matches, and second pass is for anchor points
  for (int pass = 0; pass < 2; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      
      // With a rig, only the ref sensor has rotation constraints 
      if (have_rig && !rig.isRefSensor(rig_cam_info[icam].sensor_id))
        continue;
      
      double median_wt = weight_per_cam[pass][icam];
      double count = count_per_cam[pass][icam];
      if (count <= 0) 
        continue; // no reprojection errors for this camera

      // We know the median weight to use, and how many residuals were added.
      // Based on the CERES loss function formula, adding N loss functions each 
      // with weight w and robust threshold t is equivalent to adding one loss 
      // function with weight sqrt(N)*w and robust threshold sqrt(N)*t.
      // For linescan cameras, then need to subdivide this for individual
      // positions for that camera.
      double combined_wt  = sqrt(count * 1.0) * median_wt;
      double combined_th = sqrt(count * 1.0) * opt.camera_position_robust_threshold;
      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
        
      if (ls_model != NULL) {
        // There are multiple position parameters per camera. They divide among
        // them the job of minimizing the reprojection error. So need to divide
        // the weight among them.

        // Divide the weight among the positions
        int numPos = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
        double wt = combined_wt / sqrt(numPos * 1.0);
        double th = combined_th / sqrt(numPos * 1.0);
        for (int ip = 0; ip < numPos; ip++) {
          ceres::CostFunction* cost_function
            = weightedTranslationError::Create(&ls_model->m_positions[ip * NUM_XYZ_PARAMS],
                                               wt);
          ceres::LossFunction* loss_function = new ceres::CauchyLoss(th);
          problem.AddResidualBlock(cost_function, loss_function,
                                  &ls_model->m_positions[ip * NUM_XYZ_PARAMS]);
          
          for (int c = 0; c < NUM_XYZ_PARAMS; c++)
            weight_per_residual.push_back(wt);
        }
        
      } else if (frame_model != NULL) {
      
        // Same logic as for bundle_adjust
        // There is only one position per camera
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
        // we will copy from curr_params the initial position
        ceres::CostFunction* cost_function
          = weightedTranslationError::Create(&curr_params[0], combined_wt);
        ceres::LossFunction* loss_function = new ceres::CauchyLoss(combined_th);
        problem.AddResidualBlock(cost_function, loss_function,
                                &curr_params[0]); // translation starts here

        for (int c = 0; c < NUM_XYZ_PARAMS; c++)
          weight_per_residual.push_back(combined_wt);
            
      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }

    }
  }

  return;
}

void addQuatNormRotationConstraints(
                        asp::BaBaseOptions          const& opt,
                        std::set<int>               const& outliers,
                        asp::CRNJ                   const& crn,
                        std::vector<asp::CsmModel*> const& csm_models,
                        bool                               have_rig,
                        rig::RigSet                 const& rig,
                        std::vector<RigCamInfo>     const& rig_cam_info,
                        double                             quat_norm_weight, 
                        // Outputs
                        std::vector<double>              & frame_params,
                        std::vector<double>              & weight_per_residual,
                        ceres::Problem                   & problem) {
  
  // Constrain the rotations
  // TODO(oalexan1): Make this a standalone function
  if (opt.rotation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      // With a rig, only the ref sensor has rotation constraints 
      if (have_rig && !rig.isRefSensor(rig_cam_info[icam].sensor_id))
        continue;
      
      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {
        // There are multiple quaternion parameters per camera
        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* rotation_cost_function
            = weightedRotationError::Create(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS],
                                            opt.rotation_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* rotation_loss_function = NULL;
          problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          for (int c = 0; c < NUM_QUAT_PARAMS; c++)
            weight_per_residual.push_back(opt.rotation_weight);
        }

      } else if (frame_model != NULL) {
        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
          
        // Copy from curr_params the initial quaternion
        ceres::CostFunction* rotation_cost_function
          = weightedRotationError::Create(&curr_params[NUM_XYZ_PARAMS], // quat starts here
                                          opt.rotation_weight);
        // Pass the quaternion to optimize to the problem                                  
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* rotation_loss_function = NULL;
        problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        for (int c = 0; c < NUM_QUAT_PARAMS; c++)
          weight_per_residual.push_back(opt.rotation_weight);
      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }

    } // end loop through cameras
  }

  // Try to make the norm of quaternions be close to 1
  // TODO(oalexan1): Make this a standalone function
  if (quat_norm_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {

        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* quat_norm_cost_function
            = weightedQuatNormError::Create(quat_norm_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* quat_norm_loss_function = NULL;
          problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          weight_per_residual.push_back(quat_norm_weight); // 1 single residual
        }

      } else if (frame_model != NULL) {

        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];

        ceres::CostFunction* quat_norm_cost_function
          = weightedQuatNormError::Create(quat_norm_weight);
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* quat_norm_loss_function = NULL;
        problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        weight_per_residual.push_back(quat_norm_weight); // 1 single residual

      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }
    }
  }
}

// Add roll / yaw constraints. For linescan, use the whole set of samples for given
// camera model. For frame cameras, use the trajectory of all cameras in the same orbital
// group as the current camera.
void addRollYawConstraint(asp::BaBaseOptions              const& opt,
                          asp::CRNJ                       const& crn,
                          std::vector<asp::CsmModel*>     const& csm_models,
                          vw::cartography::GeoReference   const& georef,
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

// TODO(oalexan1): This file is getting big. Move the code below to
// jitterSolveRefCostFun.cc.

// The disparity-based error measured by --reference-terrain. It is the
// difference between left_pix + disp and right_pix, given the current ground
// pixel and camera models.
bool calcDispBasedError(UsgsAstroLsSensorModel *left_ls,
                        UsgsAstroLsSensorModel *right_ls,
                        vw::TransformPtr left_tx,
                        vw::TransformPtr right_tx,
                        DispPtr disp,
                        csm::EcefCoord const& P,
                        vw::Vector2 & error) {

  // Project into the images
  double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  csm::ImageCoord left_csm_pix = left_ls->groundToImage(P, desired_precision);
  csm::ImageCoord right_csm_pix = right_ls->groundToImage(P, desired_precision);

  // Covert to ASP pixels    
  vw::Vector2 left_raw_pix, right_raw_pix;
  asp::fromCsmPixel(left_raw_pix, left_csm_pix);
  asp::fromCsmPixel(right_raw_pix, right_csm_pix);

  // Convert to aligned pixel
  vw::Vector2 left_trans_pix;
  try {
    left_trans_pix = left_tx->forward(left_raw_pix);
  } catch (...) {
    return false;
  }

  // Typedefs to avoid long lines      
  typedef vw::ValueEdgeExtension<vw::PixelMask<vw::Vector2f>> NoDataType;
  typedef vw::DiskImageView<vw::PixelMask<vw::Vector2f>> DispDiskView;
  
  // Compute the interpolated aligned disparity. Use no-data when out of bounds.
  vw::PixelMask<vw::Vector2f> no_data; 
  no_data.invalidate();
  NoDataType no_data_ext(no_data);
  vw::InterpolationView<vw::EdgeExtensionView<DispDiskView, NoDataType>, 
                        vw::BicubicInterpolation> 
    interp_disp = interpolate(*disp.get(), vw::BicubicInterpolation(), no_data_ext);
  vw::PixelMask<vw::Vector2f> disp_pix = interp_disp(left_trans_pix[0], left_trans_pix[1]);
  
  // Flag invalid pixels
  if (!is_valid(disp_pix))
    return false;

  // Discrepancy between right pixel and left pixel + disparity
  vw::Vector2 right_trans_pix = left_trans_pix + disp_pix.child();
  vw::Vector2 right_raw_derived_pix;
  try {
    right_raw_derived_pix = right_tx->reverse(right_trans_pix);
  } catch (...) {
    return false;
  }
  error = right_raw_derived_pix - right_raw_pix;
  
  return true;
}

// Fallback error value for --reference-terrain. Do not make this big, as when
// the calculation succeeds, the errors are small.
double g_ref_terrain_pix_val = 5.0;

// Reference terrain error. See calcDispBasedError() for the implementation.
struct RefTerrainReprojErr {
  RefTerrainReprojErr(UsgsAstroLsSensorModel *left_ls,
                      UsgsAstroLsSensorModel *right_ls,
                      vw::TransformPtr left_tx,
                      vw::TransformPtr right_tx,
                      DispPtr disp,
                      csm::EcefCoord const& P,
                      double weight,
                      int begLeftPosIndex, int endLeftPosIndex,
                      int begLeftQuatIndex, int endLeftQuatIndex,
                      int begRightPosIndex, int endRightPosIndex,
                      int begRightQuatIndex, int endRightQuatIndex):
    m_left_ls(left_ls), m_right_ls(right_ls),
    m_left_tx(left_tx), m_right_tx(right_tx), m_disp(disp), m_P(P), m_weight(weight),
    m_begLeftPosIndex(begLeftPosIndex), m_endLeftPosIndex(endLeftPosIndex),
    m_begLeftQuatIndex(begLeftQuatIndex), m_endLeftQuatIndex(endLeftQuatIndex),
    m_begRightPosIndex(begRightPosIndex), m_endRightPosIndex(endRightPosIndex),
    m_begRightQuatIndex(begRightQuatIndex), m_endRightQuatIndex(endRightQuatIndex) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const; 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(
                      UsgsAstroLsSensorModel *left_ls,
                      UsgsAstroLsSensorModel *right_ls,
                      vw::TransformPtr left_tx,
                      vw::TransformPtr right_tx,
                      DispPtr disp,
                      csm::EcefCoord const& P,
                      double weight,
                      int begLeftPosIndex, int endLeftPosIndex,
                      int begLeftQuatIndex, int endLeftQuatIndex,
                      int begRightPosIndex, int endRightPosIndex,
                      int begRightQuatIndex, int endRightQuatIndex) {

    ceres::DynamicNumericDiffCostFunction<RefTerrainReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RefTerrainReprojErr>
      (new RefTerrainReprojErr(left_ls, right_ls, left_tx, right_tx, disp, P, weight,
                               begLeftPosIndex, endLeftPosIndex,
                               begLeftQuatIndex, endLeftQuatIndex,
                               begRightPosIndex, endRightPosIndex,
                               begRightQuatIndex, endRightQuatIndex));

    // The residual size is the pixel size
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each position and quaternion
    for (int it = begLeftPosIndex; it < endLeftPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    for (int it = begLeftQuatIndex; it < endLeftQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    for (int it = begRightPosIndex; it < endRightPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    for (int it = begRightQuatIndex; it < endRightQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);

    return cost_function;
  }

private:
  UsgsAstroLsSensorModel *m_left_ls, *m_right_ls;
  vw::TransformPtr m_left_tx, m_right_tx;
  DispPtr m_disp;
  csm::EcefCoord m_P;
  double m_weight;
  int m_begLeftPosIndex, m_endLeftPosIndex;
  int m_begLeftQuatIndex, m_endLeftQuatIndex;
  int m_begRightPosIndex, m_endRightPosIndex;
  int m_begRightQuatIndex, m_endRightQuatIndex;
  
}; // End class RefTerrainReprojErr

// See the documentation higher up in the file.
bool RefTerrainReprojErr::operator()(double const * const * parameters,
                                     double * residuals) const {

  try {
    // Make a copy of the models
    UsgsAstroLsSensorModel local_left_ls = *m_left_ls;
    UsgsAstroLsSensorModel local_right_ls = *m_right_ls;
    
    // Update position and quaternion parameters with the current values
    int param_count = 0;
    for (int it = m_begLeftPosIndex; it < m_endLeftPosIndex; it++) {
      for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        local_left_ls.m_positions[it * NUM_XYZ_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    for (int it = m_begLeftQuatIndex; it < m_endLeftQuatIndex; it++) {
      for (int c = 0; c < NUM_QUAT_PARAMS; c++)
        local_left_ls.m_quaternions[it * NUM_QUAT_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    // Same for the right camera
    for (int it = m_begRightPosIndex; it < m_endRightPosIndex; it++) {
      for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        local_right_ls.m_positions[it * NUM_XYZ_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    for (int it = m_begRightQuatIndex; it < m_endRightQuatIndex; it++) {
      for (int c = 0; c < NUM_QUAT_PARAMS; c++)
        local_right_ls.m_quaternions[it * NUM_QUAT_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    
    vw::TransformPtr local_left_tx = m_left_tx;
    vw::TransformPtr local_right_tx = m_right_tx;
    
    vw::Vector2 error;
    bool success = calcDispBasedError(&local_left_ls, &local_right_ls, 
                                      local_left_tx, local_right_tx,
                                      m_disp, m_P, error);
    
    if (success) {
      residuals[0] = m_weight * error[0];
      residuals[1] = m_weight * error[1];
    } else {
      residuals[0] = m_weight * g_ref_terrain_pix_val;
      residuals[1] = m_weight * g_ref_terrain_pix_val;
    }
    
    // Accept the solution in either case, as it is too late to back out,
    // because that will mess up the bookkeeping.
    return true; 
    
  } catch (...) {
    residuals[0] = m_weight * g_ref_terrain_pix_val;
    residuals[1] = m_weight * g_ref_terrain_pix_val;
    return true; // accept the solution anyway
  }

  return true;
}

bool addRefTerrainReprojectionErr(asp::BaBaseOptions const& opt,
                                  UsgsAstroLsSensorModel *left_ls,
                                  UsgsAstroLsSensorModel *right_ls,
                                  vw::TransformPtr left_tx,
                                  vw::TransformPtr right_tx,
                                  vw::BBox2i const& left_bbox,
                                  vw::BBox2i const& right_bbox,
                                  DispPtr disp,
                                  csm::EcefCoord const& P,
                                  double weight,
                                  ceres::Problem & problem) {

  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  
  // Find all positions and quaternions that can affect the current pixel.
  csm::ImageCoord left_csm_pix = left_ls->groundToImage(P, desired_precision);
  vw::Vector2 left_pix;
  asp::fromCsmPixel(left_pix, left_csm_pix);
  if (!left_bbox.contains(left_pix))
    return false; // the pixel is outside the left image

  int leftBegPosIndex = -1, leftEndPosIndex = -1;
  int leftBegQuatIndex = -1, leftEndQuatIndex = -1;
  calcPosQuatIndexBounds(line_extra, left_ls, left_pix, 
                         // Outputs
                         leftBegPosIndex, leftEndPosIndex, 
                         leftBegQuatIndex, leftEndQuatIndex);

  // Same for the right camera
  csm::ImageCoord right_csm_pix = right_ls->groundToImage(P, desired_precision);
  vw::Vector2 right_pix;
  asp::fromCsmPixel(right_pix, right_csm_pix);
  if (!right_bbox.contains(right_pix))
    return false; // the pixel is outside the right image
    
  int rightBegPosIndex = -1, rightEndPosIndex = -1;
  int rightBegQuatIndex = -1, rightEndQuatIndex = -1;
  calcPosQuatIndexBounds(line_extra, right_ls, right_pix, 
                         // Outputs
                         rightBegPosIndex, rightEndPosIndex, 
                         rightBegQuatIndex, rightEndQuatIndex);
  
  ceres::CostFunction* pixel_cost_function = 
    RefTerrainReprojErr::Create(left_ls, right_ls, left_tx, right_tx, disp, P, weight,
                                leftBegPosIndex, leftEndPosIndex,
                                leftBegQuatIndex, leftEndQuatIndex,
                                rightBegPosIndex, rightEndPosIndex,
                                rightBegQuatIndex, rightEndQuatIndex);
      
  ceres::LossFunction* pixel_loss_function 
    = new ceres::CauchyLoss(opt.reference_terrain_robust_threshold);
  
  // The variable of optimization are the relevant camera positions and quaternions.
  std::vector<double*> vars;
  for (int it = leftBegPosIndex; it < leftEndPosIndex; it++)
    vars.push_back(&left_ls->m_positions[it * NUM_XYZ_PARAMS]);
  for (int it = leftBegQuatIndex; it < leftEndQuatIndex; it++)
    vars.push_back(&left_ls->m_quaternions[it * NUM_QUAT_PARAMS]);

  // Same for the right camera
  for (int it = rightBegPosIndex; it < rightEndPosIndex; it++)
    vars.push_back(&right_ls->m_positions[it * NUM_XYZ_PARAMS]);
  for (int it = rightBegQuatIndex; it < rightEndQuatIndex; it++)
    vars.push_back(&right_ls->m_quaternions[it * NUM_QUAT_PARAMS]);
  
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);
  
  return true;
}

// Read the reference terrain. Ensure enough reference points are loaded that
// project in the camera boxes. Do two attempts.
// TODO(oalexan1): This will need adjustment if there is more than one stereo pair.
void loadReferenceTerrain(asp::BaBaseOptions            const& opt,
                          std::vector<asp::CsmModel*>   const& csm_models,
                          std::vector<int>              const& left_indices,
                          std::vector<int>              const& right_indices,
                          std::vector<vw::TransformPtr> const& left_trans,
                          std::vector<vw::TransformPtr> const& right_trans,
                          std::vector<vw::BBox2i>       const& image_boxes,
                          // Output
                          std::vector<vw::Vector3> & reference_vec) {

  vw::vw_out() << "Loading reference terrain points and then filtering them.\n";
  
  // Set up a GeoReference object using the datum, it may get modified later
  vw::cartography::GeoReference geo;
  geo.set_datum(opt.datum); // We checked for a datum earlier
  // Ensure the read georef lives on the same planet
  bool warn_only = false;
  vw::checkDatumConsistency(opt.datum, geo.datum(), warn_only);
  
  for (size_t i = 0; i < left_indices.size(); i++) {
      
    auto * left_cam = csm_models[left_indices[i]];
    auto * right_cam = csm_models[right_indices[i]];
    auto left_bbox = image_boxes[left_indices[i]];
    auto right_bbox = image_boxes[right_indices[i]];
      
    int num_try_load = 5 * opt.max_num_reference_points;
    for (int attempt = 0; attempt < 2; attempt++) {
   
      reference_vec.clear();
      std::vector<vw::Vector3> local_reference_vec;
      asp::load_csv_or_dem(opt.csv_format_str, opt.csv_srs, opt.reference_terrain,  
                            num_try_load,  
                            // Outputs
                            geo, local_reference_vec);
    
      // TODO(oalexan1): For non-ISIS cameras this could be done in parallel.
      for (size_t data_col = 0; data_col < local_reference_vec.size(); data_col++) {
        vw::Vector3 reference_xyz = local_reference_vec[data_col];
        vw::Vector2 left_pix, right_pix;
        try {
          left_pix = left_cam->point_to_pixel(reference_xyz);
          right_pix = right_cam->point_to_pixel(reference_xyz);
        } catch (...) {
          continue;
        }
        if (!left_bbox.contains(left_pix) || !right_bbox.contains(right_pix))
          continue;
      
        reference_vec.push_back(reference_xyz);
      }
      
      int num_loaded = reference_vec.size();
      double ratio = double(num_loaded) / opt.max_num_reference_points;
      if (ratio >= 1.0)
        break; // loaded enough points

       // Try again
       num_try_load = 1.5 * num_try_load / std::max(ratio, 1e-3); // avoid division by 0
       vw::vw_out() << "Loaded too few points after filtering. Try again.\n";
    
    } // end attempt loop
    
    // If there are too few points, print a warning
    if (reference_vec.size() < opt.max_num_reference_points)
      vw::vw_out(vw::WarningMessage) << "Loaded only " << reference_vec.size()
        << " reference terrain points after filtering by projection into camera.\n";

    // If they are too many, pick a subset
    if (reference_vec.size() > opt.max_num_reference_points) {
      // Deterministic shuffle
      unsigned seed = 0;
      std::mt19937 gen(seed);
      std::shuffle(reference_vec.begin(), reference_vec.end(), gen);
      reference_vec.resize(opt.max_num_reference_points);
    }

    vw::vw_out() << "Read " << reference_vec.size()
      << " reference terrain points (after filtering by projection into camera).\n";
    
  } // end loop through stereo pairs

} // end function loadReferenceTerrain()

// Option --reference-terrain
// TODO(oalexan1): Must add the uncertainty logic.
// TODO(oalexan1): Filter out points outside the box and load more if needed.
void addReferenceTerrainCostFunction(asp::BaBaseOptions            const& opt,
                                     std::vector<asp::CsmModel*>   const& csm_models,
                                     std::vector<int>              const& left_indices,
                                     std::vector<int>              const& right_indices,
                                     std::vector<vw::TransformPtr> const& left_trans,
                                     std::vector<vw::TransformPtr> const& right_trans,
                                     std::vector<std::string>      const& disp_files,
                                     // Outputs
                                     ceres::Problem                 & problem,
                                     std::vector<DispPtr>           & disp_vec,
                                     vw::ImageView<float>           & mapproj_dem,
                                     std::vector<double>            & weight_per_residual,
                                     std::vector<vw::Vector3>       & reference_vec,
                                     std::vector<std::vector<int>>  & ref_indices) {

  // For now, only one stereo pair is supported, so size of left_indices must be 1
  if (left_indices.size() != 1)
    vw::vw_throw(vw::ArgumentErr() << "Expecting only one stereo pair.\n");
    
  // Read the image boxes
  std::vector<vw::BBox2i> image_boxes;
  for (size_t icam = 0; icam < opt.image_files.size(); icam++) {
    vw::DiskImageView<float> img(opt.image_files[icam]);
    vw::BBox2i bbox = vw::bounding_box(img);
    image_boxes.push_back(bbox);
  }
  
  // Read the reference terrain. Ensure enough reference points are loaded that
  // project in the camera boxes. 
  loadReferenceTerrain(opt, csm_models, left_indices, right_indices,
                      left_trans, right_trans, image_boxes, 
                      reference_vec); // output
 
  // Load the disparity files. Keep a pointer to DiskImageView to avoid loading
  // them fully in memory, as they can be huge.
  disp_vec.resize(disp_files.size());
  for (size_t i = 0; i < disp_files.size(); i++) {
    vw::vw_out() << "Reading disparity file: " << disp_files[i] << "\n"; 
    disp_vec[i].reset(new vw::DiskImageView<vw::PixelMask<vw::Vector2f>>(disp_files[i]));
  }
    
  // Record the indices of the residuals of the reference points  
  ref_indices.clear();
  ref_indices.resize(reference_vec.size());
    
  int num_kept_ref_points = 0;
  vw::vw_out() << "Setting up the error to the reference terrain.\n";
  
  // For Map2CamTrans, must turn off caching to ensure thread safety, and read
  // the DEM in memory.
  for (size_t i = 0; i < left_indices.size(); i++) {
    
    auto & left_tx = left_trans[i];
    auto & right_tx = right_trans[i];

    vw::cartography::Map2CamTrans* left_map2cam = 
      dynamic_cast<vw::cartography::Map2CamTrans*>(left_tx.get());
    vw::cartography::Map2CamTrans* right_map2cam = 
      dynamic_cast<vw::cartography::Map2CamTrans*>(right_tx.get());
    
    // Either both must be null or both must be non-null.
    if ((left_map2cam == NULL) != (right_map2cam == NULL))
      vw::vw_throw(vw::ArgumentErr() << "Both images in a stereo pair must be "
                   << "mapprojected, or neither of them.\n");
        
    if (left_map2cam == NULL || right_map2cam == NULL)
      continue; // nothing to do
    
    // Both must be projected onto the same DEM
    if (left_map2cam->m_dem_file != right_map2cam->m_dem_file)
      vw::vw_throw(vw::ArgumentErr() << "The two images in a stereo pair must be "
                   << "projected onto the same DEM.\n");
    
    // Print a warning if the left_map2cam->m_dem width and height are no less than 
    // 5000 pixels, as that would be slow to load.
    int k = 5000;
    if (left_map2cam->m_dem.cols() >= k || left_map2cam->m_dem.rows() >= k)
      vw::vw_out(vw::WarningMessage) 
        << "The mapprojection DEM " << left_map2cam->m_dem_file 
        << " is very large, it may take a long time to load. Dimensions: "
        << left_map2cam->m_dem.cols() << " x " << left_map2cam->m_dem.rows() 
        << " pixels.\n";

    // Load the DEM in memory and set it for each transform. This is a fix
    // for very slow performance. Note that the disparity is still loaded
    // on demand only, as that can be large.
    mapproj_dem = copy(left_map2cam->m_dem);
    
    // TODO(oalexan1): This must be a member function. It uses very detailed
    // info from the class.
    left_map2cam->set_use_cache(false);
    left_map2cam->m_masked_dem = vw::create_mask(mapproj_dem, 
                                                 left_map2cam->m_nodata);
    left_map2cam->m_interp_dem = vw::interpolate(left_map2cam->m_masked_dem, 
                                                  vw::BicubicInterpolation(), 
                                                  vw::ZeroEdgeExtension());

    right_map2cam->set_use_cache(false);
    right_map2cam->m_masked_dem = vw::create_mask(mapproj_dem,
                                                  right_map2cam->m_nodata);
    right_map2cam->m_interp_dem = vw::interpolate(right_map2cam->m_masked_dem, 
                                                  vw::BicubicInterpolation(), 
                                                  vw::ZeroEdgeExtension());
  } // end loop through stereo pairs
      
  // Need a progress bar as this can be slow
  vw::TerminalProgressCallback tpc("asp", "Filter ref points with disparity: --> ");
  // Find the spacing for progress. Avoid int32 overflow.
  double total = double(reference_vec.size()) * double(left_indices.size()); 
  double hundred = 100.0;
  long long spacing = std::max(round(total / hundred), 1.0);
  long long data_count = 0;

  // Loop through the reference points
  for (size_t data_col = 0; data_col < reference_vec.size(); data_col++) {

    vw::Vector3 reference_xyz = reference_vec[data_col];

    // Iterate over left indices
    for (size_t i = 0; i < left_indices.size(); i++) {
      
      data_count++;
      if (data_count % spacing == 0)
        tpc.report_incremental_progress(1.0/hundred);
        
      auto & left_tx = left_trans[i];
      auto & right_tx = right_trans[i];
      int left_index = left_indices[i];
      int right_index = right_indices[i];
      vw::BBox2i left_bbox = image_boxes[left_index];
      vw::BBox2i right_bbox = image_boxes[right_index];
      
      // Get the underlying linescan models      
      csm::RasterGM * left_csm = csm_models[left_index]->m_gm_model.get();
      UsgsAstroLsSensorModel * left_ls 
        = dynamic_cast<UsgsAstroLsSensorModel*>(left_csm);
      csm::RasterGM * right_csm = csm_models[right_index]->m_gm_model.get();
      UsgsAstroLsSensorModel * right_ls 
        = dynamic_cast<UsgsAstroLsSensorModel*>(right_csm);
      if (left_ls == NULL || right_ls == NULL)
        vw::vw_throw(vw::ArgumentErr() << "The option --reference-terrain works only "
                      << "with linescan cameras.\n");

      // Convert to CSM points
      csm::EcefCoord P(reference_xyz[0], reference_xyz[1], reference_xyz[2]);
      
      // If the error cannot be measured even with nominal values of the parameters,
      // there is no point in adding the error.
      vw::Vector2 error;
      bool success = calcDispBasedError(left_ls, right_ls, left_tx, right_tx, disp_vec[i], P, 
                                        error); // output
      
      if (!success) 
        continue; // skip this point
      
      // Find the weight to use with the camera constraint
      double gsd = 0.0;
      try {
        vw::Vector2 pix_obs = csm_models[left_index]->point_to_pixel(reference_xyz);
        gsd = vw::camera::estimatedGSD(csm_models[left_index], image_boxes[left_index],
                                        pix_obs, reference_xyz);
      } catch (...) {
        continue;
      }
      if (gsd <= 0.0) 
        continue; 

      // Bigger uncertainty means a smaller weight for the residual. Also 
      // convert from meters to pixels.
      double weight = gsd / opt.reference_terrain_uncertainty;
      
      // Add the error cost function for this point
      success = addRefTerrainReprojectionErr(opt, left_ls, right_ls, 
                                             left_tx, right_tx, 
                                             left_bbox, right_bbox, 
                                             disp_vec[i], P, weight, problem);
      if (!success) 
        continue; // skip this point
      
      // Record where the residuals for this point are stored
      ref_indices[data_col].push_back(weight_per_residual.size());
      
      // Two residuals were added. Save the corresponding weights. The saved residuals
      // will be divided back by the weight.
      for (int c = 0; c < asp::PIXEL_SIZE; c++)
        weight_per_residual.push_back(weight);

    } // end loop through stereo pairs
    
    // See if this point is kept
    if (ref_indices[data_col].size() > 0)
      num_kept_ref_points++;
  }
  
  tpc.report_finished();
  
  vw::vw_out() << "Read " << num_kept_ref_points 
    << " reference terrain points (after filtering by disparity).\n";
  
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
    median_curvature[c] = vw::math::destructive_median<double>(vals);
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
  double ans = val * (exp(abs(val)) - 1.0);
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
  // quaternion indicies are in the [begQuatIndex, endQuatIndex] range, so
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
