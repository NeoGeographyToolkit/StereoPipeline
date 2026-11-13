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

// Cost functions used in solving for jitter. These need access to the camera models,
// so they are stored in the Camera folder.

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveCostFunsInternal.h>
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

// An error function minimizing the error of projecting an xyz point
// into a given CSM linescan camera pixel. The variables of optimization are a
// portion of the position and quaternion variables affected by this, and the 
// triangulation point.
struct LsPixelReprojErr {
  LsPixelReprojErr(vw::Vector2 const& observation, double weight,
                   UsgsAstroLsSensorModel* ls_model,
                   int begQuatIndex, int endQuatIndex,
                   int begPosIndex, int endPosIndex,
                   double lineBand):
    m_observation(observation), m_weight(weight),
    m_begQuatIndex(begQuatIndex), m_endQuatIndex(endQuatIndex),
    m_begPosIndex(begPosIndex),   m_endPosIndex(endPosIndex),
    m_ls_model(ls_model), m_lineBand(lineBand) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& observation, double weight,
                                     UsgsAstroLsSensorModel* ls_model,
                                     int begQuatIndex, int endQuatIndex,
                                     int begPosIndex, int endPosIndex,
                                     double lineBand) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>
      (new LsPixelReprojErr(observation, weight, ls_model,
                                  begQuatIndex, endQuatIndex,
                                  begPosIndex, endPosIndex, lineBand));

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
  // Half-width (in lines) of the band the pose window covers. The local fast path
  // falls back to the full model if a projection leaves this band.
  double m_lineBand;
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

// Default path: copy the whole model, set the window from the parameters, project.
// Correct, but deep-copies the full pose arrays on every call (the main cost).
static csm::ImageCoord fullCopyLsProject(UsgsAstroLsSensorModel const* src,
                                         double const * const * parameters,
                                         int begQuatIndex, int endQuatIndex,
                                         int begPosIndex, int endPosIndex,
                                         double desired_precision) {
  UsgsAstroLsSensorModel cam = *src;
  int shift = 0;
  csm::EcefCoord P;
  updateLsModelTriPt(parameters, begQuatIndex, endQuatIndex,
                     begPosIndex, endPosIndex, shift, cam, P);
  return cam.groundToImage(P, desired_precision);
}

// Extra samples kept on each side of the window so the window-edge Lagrange
// interpolation uses the same order (up to 8) as the full model. Four is the true
// minimum (the Lagrange half-order); eight is a conservative choice with headroom
// for the groundToImage line search. Even at eight this trimming already gives the
// full ~12x speedup of the Jacobian evaluation, so dropping to four (with more
// testing) is a minor extra win, not a necessity.
static const int LOCAL_LINESCAN_PAD = 8;

// Reshape a cached local model to one residual: keep the small fields (optics,
// line-to-time map, etc.) but trim the position/velocity/quaternion arrays to the
// subset of samples this observation can touch (the window plus a small margin).
// The window comes from the current parameters, the margin from the current model,
// so nothing is stale. The caller owns the one-time full copy. P is an output.
//
// Not precisely identical to the full model: trimming shifts the interpolation
// origin, so (time - t0)/dt rounds slightly differently, a negligible ~1e-9 px.
static void reshapeLocalLsModel(UsgsAstroLsSensorModel const* src,
                                double const * const * parameters,
                                int begQuatIndex, int endQuatIndex,
                                int begPosIndex, int endPosIndex,
                                int pad,
                                UsgsAstroLsSensorModel      & local, // cached, small fields set
                                csm::EcefCoord              & P) {

  int numPos  = src->m_numPositions  / NUM_XYZ_PARAMS;
  int numQuat = src->m_numQuaternions / NUM_QUAT_PARAMS;

  // Padded sample ranges, clamped to the model. The clamping matches the full
  // model near the image ends, so the Lagrange edge-order drop is reproduced.
  int pLo = std::max(0,       begPosIndex  - pad);
  int pHi = std::min(numPos,  endPosIndex  + pad);
  int qLo = std::max(0,       begQuatIndex - pad);
  int qHi = std::min(numQuat, endQuatIndex + pad);

  // Trim positions to [pLo, pHi), re-indexing the ephem origin. Velocity is left
  // zero: no camera operation here uses it (groundToImage with no adjustments
  // never interpolates velocity).
  int nP = pHi - pLo;
  local.m_numPositions = nP * NUM_XYZ_PARAMS;
  local.m_t0Ephem      = src->m_t0Ephem + pLo * src->m_dtEphem;
  local.m_positions.resize(nP * NUM_XYZ_PARAMS);
  local.m_velocities.assign(nP * NUM_XYZ_PARAMS, 0.0);
  for (int pi = pLo; pi < pHi; pi++) {
    int b = pi - pLo;
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      local.m_positions[NUM_XYZ_PARAMS * b + c] = src->m_positions[NUM_XYZ_PARAMS * pi + c];
  }

  // Trim quaternions to [qLo, qHi), re-indexing the quaternion origin.
  int nQ = qHi - qLo;
  local.m_numQuaternions = nQ * NUM_QUAT_PARAMS;
  local.m_t0Quat         = src->m_t0Quat + qLo * src->m_dtQuat;
  local.m_quaternions.resize(nQ * NUM_QUAT_PARAMS);
  for (int qi = qLo; qi < qHi; qi++) {
    int b = qi - qLo;
    for (int c = 0; c < NUM_QUAT_PARAMS; c++)
      local.m_quaternions[NUM_QUAT_PARAMS * b + c] = src->m_quaternions[NUM_QUAT_PARAMS * qi + c];
  }

  // Overwrite the optimization window with the current parameters, re-indexed to
  // the local arrays, then recover the triangulated point. The parameter block
  // order mirrors updateLsModelTriPt: quaternions, then positions, then xyz.
  int shift = 0;
  for (int qi = begQuatIndex; qi < endQuatIndex; qi++) {
    int b = qi - qLo;
    for (int c = 0; c < NUM_QUAT_PARAMS; c++)
      local.m_quaternions[NUM_QUAT_PARAMS * b + c] = parameters[qi + shift - begQuatIndex][c];
  }
  shift += (endQuatIndex - begQuatIndex);
  for (int pi = begPosIndex; pi < endPosIndex; pi++) {
    int b = pi - pLo;
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      local.m_positions[NUM_XYZ_PARAMS * b + c] = parameters[pi + shift - begPosIndex][c];
  }
  shift += (endPosIndex - begPosIndex);
  P.x = parameters[shift][0];
  P.y = parameters[shift][1];
  P.z = parameters[shift][2];
}

// See the documentation higher up in the file.
bool LsPixelReprojErr::operator()(double const * const * parameters,
                                  double * residuals) const {

  try {
    // Project in the camera with high precision. Do not use here
    // anything lower than 1e-8, as the linescan model will then
    // return junk.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;

    // One cached local model per camera per thread: the full copy happens once
    // per camera (to grab the small fields), then the pose arrays are reshaped
    // from current data each call. The pointer is a stable key since the camera
    // models live for the whole solve.
    static thread_local
      std::unordered_map<UsgsAstroLsSensorModel const*, UsgsAstroLsSensorModel> localCache;
    auto it = localCache.find(m_ls_model);
    if (it == localCache.end())
      it = localCache.emplace(m_ls_model, *m_ls_model).first; // one-time full copy
    UsgsAstroLsSensorModel& local = it->second;
    csm::EcefCoord P;
    reshapeLocalLsModel(m_ls_model, parameters,
                        m_begQuatIndex, m_endQuatIndex,
                        m_begPosIndex, m_endPosIndex,
                        LOCAL_LINESCAN_PAD, local, P);
    csm::ImageCoord imagePt = local.groundToImage(P, desired_precision);
    // Fall back to the full model if the projection left the window band, where
    // the trimmed local model lacks the samples the full model would use.
    if (std::abs(imagePt.line - m_observation[1]) > m_lineBand)
      imagePt = fullCopyLsProject(m_ls_model, parameters,
                                  m_begQuatIndex, m_endQuatIndex,
                                  m_begPosIndex, m_endPosIndex, desired_precision);

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
  // This can happen with wildly out-of-range anchor points.
  if (begIndex >= endIndex)
    vw::vw_throw(vw::ArgumentErr() << "Out-of-range interpolation indices.\n");

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
                             begPosIndex, endPosIndex, line_extra);
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

// Calculate a set of anchor points uniformly distributed over the image.
// Will use num_anchor_points_extra_lines. We append to obs_vec, which
// already holds the reprojection observations for the match points.
void calcAnchorPoints(asp::BaBaseOptions              const & opt,
                      int                                     num_anchor_points_per_image,
                      int                                     num_anchor_points_per_tile,
                      int                                     num_anchor_points_extra_lines,
                      int                                     max_anchor_points_total,
                      double                                  anchor_dem_uncertainty,
                      double                                  anchor_weight,
                      std::string                     const & anchor_weight_image_file,
                      vw::ImageViewRef<vw::PixelMask<double>> interp_anchor_dem,
                      vw::GeoReference   const & anchor_georef,
                      std::vector<asp::CsmModel*>     const & csm_models,
                      // Append to these, they already have entries
                      std::vector<std::vector<asp::JitterObs>> & obs_vec,
                      std::vector<double> & orig_tri_points_vec,
                      std::vector<double> & tri_points_vec) {

  vw::vw_out() << "Calculating anchor points.\n";
  vw::Stopwatch sw;
  sw.start();

  if (num_anchor_points_per_image <= 0 && num_anchor_points_per_tile <= 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive number of anchor points.\n");

  bool warning_printed = false;

  // If to use an anchor weight image
  bool have_anchor_weight_image = (!anchor_weight_image_file.empty());
  vw::ImageViewRef<vw::PixelMask<float>> anchor_weight_image;
  float anchor_weight_image_nodata = -std::numeric_limits<float>::max();
  vw::GeoReference anchor_weight_image_georef;
  if (have_anchor_weight_image)
    vw::readGeorefImage(anchor_weight_image_file,
      anchor_weight_image_nodata, anchor_weight_image_georef, anchor_weight_image);

  int num_cams = opt.camera_models.size();

  // First collect all candidate anchor points, uniformly distributed. They are
  // appended to the output structures later, after an optional random pruning to
  // max_anchor_points_total. This separates creation from pruning.
  std::vector<std::vector<vw::Vector2>> anchor_pix(num_cams);
  std::vector<std::vector<double>>      anchor_pix_weight(num_cams); // final pixel weight
  std::vector<std::vector<vw::Vector3>> anchor_xyz(num_cams);

  // Convert the anchor dem uncertainty (1 sigma, in meters on the ground) to a
  // pixel uncertainty by dividing by GSD.
  bool use_anchor_uncertainty = (anchor_dem_uncertainty > 0);

  for (int icam = 0; icam < num_cams; icam++) {

    vw::Vector2 dims = vw::file_image_size(opt.image_files[icam]);
    int numLines   = dims[1];
    int numSamples = dims[0];
    int extra = num_anchor_points_extra_lines;
    vw::BBox2i image_box(0, 0, numSamples, numLines);

    // Per-anchor-point buffers for this image. The final pixel weight needs the
    // per-image median GSD, which is known only after all points are found.
    std::vector<double> anchor_img_mult; // the weight-image multiplier per point
    std::vector<double> anchor_gsd;      // the ground sample distance per point

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (ls_model == NULL)
       extra = 0; // extra lines are only for linescan

    // Find how much image area will be taken by each anchor point
    // Convert to double early on to avoid integer overflow
    double area = double(numSamples) * double(numLines + 2 * extra);
    double area_per_point = 0.0;
    if (num_anchor_points_per_image > 0)
      area_per_point = area / double(num_anchor_points_per_image);
    else
      area_per_point = 1024.0 * 1024.0 / double(num_anchor_points_per_tile);

    double bin_len = sqrt(area_per_point);
    bin_len = std::max(bin_len, 1.0);
    int lenx = ceil(double(numSamples) / bin_len); lenx = std::max(1, lenx);
    int leny = ceil(double(numLines + 2 * extra) / bin_len); leny = std::max(1, leny);

    vw::vw_out() << "Image file: " << opt.image_files[icam] << "\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0/lenx;
    tpc.report_progress(0);

    int numAnchorPoints = 0;
    for (int binx = 0; binx <= lenx; binx++) {

      tpc.report_incremental_progress(inc_amount);
      double posx = binx * bin_len;

      for (int biny = 0; biny <= leny; biny++) {
        double posy = biny * bin_len - extra;

        if (posx > numSamples - 1 || posy < -extra || posy > numLines - 1 + extra)
          continue;

        vw::Vector2 pix(posx, posy);
        vw::Vector3 xyz_guess(0, 0, 0);

        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double height_error_tol = 0.001; // 1 mm should be enough

        vw::Vector3 anchor_dem_xyz = vw::camera_pixel_to_dem_xyz
          (opt.camera_models[icam]->camera_center(pix),
           opt.camera_models[icam]->pixel_to_vector(pix),
           vw::pixel_cast<vw::PixelMask<float>>(interp_anchor_dem), anchor_georef,
           treat_nodata_as_zero, has_intersection,
           height_error_tol, xyz_guess);

        if (!has_intersection || anchor_dem_xyz == vw::Vector3())
          continue;

        vw::Vector2 pix_out;
        try {
          pix_out = opt.camera_models[icam]->point_to_pixel(anchor_dem_xyz);
        } catch (...) {
          continue;
        }

        if (norm_2(pix - pix_out) > 10 * height_error_tol)
          continue; // this is likely a bad point

        // If we have a weight image, use it to multiply the weight
        double anchor_weight_from_image = 1.0;
        if (have_anchor_weight_image) {
          vw::PixelMask<float> img_wt
            = vw::closestPixelVal(anchor_weight_image,
                                  anchor_weight_image_georef,
                                  anchor_dem_xyz);

          // Skip bad weights
          if (!vw::is_valid(img_wt) || std::isnan(img_wt.child()) || img_wt.child() <= 0.0)
            continue;

          anchor_weight_from_image = img_wt.child();
        }

        if (ls_model != NULL) {
          // Anchor points must not be outside the range of tabulated positions and orientations
          csm::ImageCoord imagePt;
          asp::toCsmPixel(pix, imagePt);
          double time    = ls_model->getImageTime(imagePt);
          int numPos     = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
          double posT0   = ls_model->m_t0Ephem;
          double posDt   = ls_model->m_dtEphem;
          int pos_index  = static_cast<int>((time - posT0) / posDt);
          int numQuat    = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
          double quatT0  = ls_model->m_t0Quat;
          double quatDt  = ls_model->m_dtQuat;
          int quat_index = static_cast<int>((time - quatT0) / quatDt);
          if (pos_index < 0  || pos_index >= numPos ||
              quat_index < 0 || quat_index >= numQuat) {
            if (!warning_printed) {
              vw::vw_out(vw::WarningMessage) << "Not placing anchor points outside "
                << "the range of tabulated positions and orientations.\n";
              warning_printed = true;
            }
            continue;
          }
        }

        // The ground sample distance at this anchor pixel, where the ray meets
        // the anchor DEM. Used to convert the ground uncertainty to pixels.
        // Only needed for the uncertainty path, not the legacy weight path.
        double gsd = 0.0;
        if (use_anchor_uncertainty) {
          try {
            gsd = vw::estimatedGSD(opt.camera_models[icam].get(), image_box,
                                   pix, anchor_dem_xyz);
          } catch (...) {
            continue; // cannot convert the uncertainty to pixels without a GSD
          }
          if (gsd <= 0)
            continue;
        }

        anchor_pix[icam].push_back(pix);
        anchor_xyz[icam].push_back(anchor_dem_xyz);
        anchor_img_mult.push_back(anchor_weight_from_image);
        anchor_gsd.push_back(gsd);

        numAnchorPoints++;
      }
    }

    tpc.report_finished();

    // Now that all candidates for this image are found, convert the ground
    // uncertainty to a per-image pixel weight using the median GSD. The median
    // is robust to the few bad GSD estimates at image edges. Without the
    // uncertainty, the legacy pixel weight is used directly.
    double base_wt = anchor_weight; // legacy pixel weight path
    double median_gsd = 0.0;
    if (!anchor_gsd.empty()) {
      std::vector<double> gsd_copy = anchor_gsd;
      median_gsd = vw::median_in_place<double>(gsd_copy);
    }
    if (use_anchor_uncertainty && median_gsd > 0) {
      // Convert the ground uncertainty (meters) to a pixel weight using the
      // per-image median GSD. The pixel sigma is anchor_dem_uncertainty/median_gsd.
      base_wt = median_gsd / anchor_dem_uncertainty; // = 1/sigma_pix
    }

    // Assign the final pixel weight to each candidate of this image.
    anchor_pix_weight[icam].resize(anchor_img_mult.size());
    for (size_t k = 0; k < anchor_img_mult.size(); k++)
      anchor_pix_weight[icam][k] = base_wt * anchor_img_mult[k];

    vw::vw_out() << "Lines and samples: " << numLines << ' ' << numSamples << std::endl;
    vw::vw_out() << "Num anchor points per image: " << numAnchorPoints     << std::endl;
  }

  // Flatten the candidate indices as (camera, local index) pairs, in camera order
  std::vector<std::pair<int, int>> all_anchor;
  for (int icam = 0; icam < num_cams; icam++)
    for (size_t j = 0; j < anchor_pix[icam].size(); j++)
      all_anchor.push_back(std::make_pair(icam, (int)j));

  // Optionally prune the candidates at random to the given total. This is set
  // from --max-anchor-points-to-tri-points-ratio.
  std::vector<char> keep(all_anchor.size(), 1);
  if (max_anchor_points_total >= 0 &&
      (int)all_anchor.size() > max_anchor_points_total) {
    std::vector<int> subset;
    vw::pick_random_indices_in_range(all_anchor.size(),
                                     max_anchor_points_total, subset);
    std::fill(keep.begin(), keep.end(), 0);
    for (size_t it = 0; it < subset.size(); it++)
      keep[subset[it]] = 1;
    vw::vw_out() << "Reducing the number of anchor points from " << all_anchor.size()
             << " to " << max_anchor_points_total
             << ", by selecting a random subset.\n";
  }

  // Append the kept candidates to obs_vec, in camera order, so the xyz_index
  // bookkeeping stays consistent.
  int totalAnchorPoints = 0;
  for (size_t k = 0; k < all_anchor.size(); k++) {
    if (!keep[k])
      continue;
    int icam = all_anchor[k].first;
    int j    = all_anchor[k].second;

    // The current number of points in tri_points_vec is the index of the next point
    bool is_anchor = true;
    int xyz_index = tri_points_vec.size() / 3;
    obs_vec[icam].push_back(asp::JitterObs(anchor_pix[icam][j], anchor_pix_weight[icam][j],
                                           is_anchor, xyz_index));

    // Append every coordinate of the anchor xyz to tri_points_vec
    vw::Vector3 xyz = anchor_xyz[icam][j];
    for (int it = 0; it < 3; it++) {
      orig_tri_points_vec.push_back(xyz[it]);
      tri_points_vec.push_back(xyz[it]);
    }

    totalAnchorPoints++;
  }

  sw.stop();
  vw::vw_out() << "Total number of anchor points: " << totalAnchorPoints << "\n";
  vw::vw_out() << "Elapsed time in calculating anchor points: " << sw.elapsed_seconds()
               << " seconds.\n";
}

// Add reprojection errors. Collect data that will be used to add camera
// constraints that scale with the number of reprojection errors and GSD.
void addReprojCamErrs(asp::BaBaseOptions                    const & opt,
                      asp::CRN                             const & crn,
                      std::vector<std::vector<asp::JitterObs>> const & obs_vec,
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
                      std::vector<asp::JitterResidualMeta>    & residual_meta,
                      std::vector<std::vector<double>>        & weight_per_cam,
                      std::vector<std::vector<double>>        & count_per_cam,
                      std::vector<double>                     & ref_to_curr_sensor_vec,
                      ceres::Problem                          & problem) {

  // Do here two passes, first for non-anchor points and then for anchor ones.
  // Record per-residual metadata in add order, so the residual writer need not
  // re-derive this traversal.
  residual_meta.clear();
  weight_per_cam.resize(2);
  count_per_cam.resize(2);
  for (int pass = 0; pass < 2; pass++) {

     weight_per_cam[pass].resize((int)crn.size(), 0.0);
     count_per_cam[pass].resize((int)crn.size(), 0.0);

    for (int icam = 0; icam < (int)crn.size(); icam++) {

      vw::DiskImageView<float> img(opt.image_files[icam]);
      vw::BBox2 image_box = bounding_box(img);
      std::vector<double> this_cam_weights;

      for (size_t ipix = 0; ipix < obs_vec[icam].size(); ipix++) {

        asp::JitterObs const& obs = obs_vec[icam][ipix];
        vw::Vector2 pix_obs = obs.pixel;
        double * tri_point  = &tri_points_vec[3 * obs.xyz_index];
        double pix_wt       = obs.weight;
        bool isAnchor       = obs.is_anchor;

        // Pass 0 is without anchor points, while pass 1 uses them
        if ((int)isAnchor != pass)
          continue;

        csm::RasterGM * csm = csm_models[icam]->m_gm_model.get();
        UsgsAstroLsSensorModel * ls_model
          = dynamic_cast<UsgsAstroLsSensorModel*>(csm);
        UsgsAstroFrameSensorModel * frame_model
          = dynamic_cast<UsgsAstroFrameSensorModel*>(csm);

        // Skip points whose pixel falls outside the camera ephemeris range.
        // This can happen with --num-anchor-points-extra-lines.
        try {

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

        } catch (...) { continue; } // skipped residuals record nothing, stay aligned

        // Two residuals were added. Save the corresponding weights.
        for (int c = 0; c < asp::PIXEL_SIZE; c++)
          weight_per_residual.push_back(pix_wt);

        // Record the metadata for this residual pair, in add order. Points
        // skipped by the catch above push nothing, so this stays aligned with
        // the residuals that were actually added.
        residual_meta.push_back(asp::JitterResidualMeta(icam, obs.xyz_index,
                                                        obs.is_anchor, pix_wt));

        // Anchor points are fixed by definition. They try to prevent
        // the cameras from moving too much from original poses.
        if (isAnchor)
          problem.SetParameterBlockConstant(tri_point);

        // Find the weight to use with the camera constraint
        vw::Vector3 xyz_obs(tri_point[0], tri_point[1], tri_point[2]);
        double gsd = 0.0;
        try {
          gsd = vw::estimatedGSD(opt.camera_models[icam].get(), image_box,
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
        weight_per_cam[pass][icam] = vw::median_in_place(this_cam_weights);
      else
        weight_per_cam[pass][icam] = 0.0;
    } // end iteration through cameras
  } // end iteration through passes

  return;
}

} // end namespace asp
