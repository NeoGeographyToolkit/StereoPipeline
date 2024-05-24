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
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/SatSimBase.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveRigCostFuns.h>

#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Camera/CameraImage.h>

namespace asp {

// An error function minimizing the error of projecting an xyz point into a
// given CSM frame camera pixel that is on a rig with a linescan camera. The
// variables of optimization are a portion of the position and quaternion
// variables affected by this, the triangulation point, and the transform from
// the ref linescan sensor to the current frame sensor.
struct RigLsFramePixelReprojErr {
  RigLsFramePixelReprojErr(vw::Vector2 const& frame_pix, double weight,
                           asp::RigCamInfo const& rig_cam_info,
                           UsgsAstroLsSensorModel* ref_ls_model,
                           UsgsAstroFrameSensorModel * curr_frame_model,
                           UsgsAstroLsSensorModel * curr_ls_model,
                           int begRefQuatIndex, int endRefQuatIndex, 
                           int begRefPosIndex, int endRefPosIndex,
                           int begCurrQuatIndex, int endCurrQuatIndex,
                           int begCurrPosIndex, int endCurrPosIndex):
    m_frame_pix(frame_pix), m_weight(weight),
    m_rig_cam_info(rig_cam_info),
    m_begRefQuatIndex(begRefQuatIndex), m_endRefQuatIndex(endRefQuatIndex),
    m_begRefPosIndex(begRefPosIndex),   m_endRefPosIndex(endRefPosIndex),
    m_begCurrQuatIndex(begCurrQuatIndex), m_endCurrQuatIndex(endCurrQuatIndex),
    m_begCurrPosIndex(begCurrPosIndex),   m_endCurrPosIndex(endCurrPosIndex),
    m_ref_ls_model(ref_ls_model), m_curr_frame_model(curr_frame_model),
    m_curr_ls_model(curr_ls_model) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const; 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& frame_pix, double weight,
                                     asp::RigCamInfo const& rig_cam_info,
                                     UsgsAstroLsSensorModel* ref_ls_model,
                                     UsgsAstroFrameSensorModel* curr_frame_model,
                                     UsgsAstroLsSensorModel* curr_ls_model,
                                     int begRefQuatIndex, int endRefQuatIndex,
                                     int begRefPosIndex, int endRefPosIndex,
                                     int begCurrQuatIndex, int endCurrQuatIndex,
                                     int begCurrPosIndex, int endCurrPosIndex) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<RigLsFramePixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RigLsFramePixelReprojErr>
      (new RigLsFramePixelReprojErr(frame_pix, weight, rig_cam_info, ref_ls_model,
                                    curr_frame_model, curr_ls_model,
                                    begRefQuatIndex, endRefQuatIndex,
                                    begRefPosIndex, endRefPosIndex,
                                    begCurrQuatIndex, endCurrQuatIndex,
                                    begCurrPosIndex, endCurrPosIndex));

    // The residual size is always the same.
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each quaternion and each position
    for (int it = begRefQuatIndex; it < endRefQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    for (int it = begRefPosIndex; it < endRefPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);

    // Add a parameter block for the xyz point
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    
    // Add a parameter block for the ref to curr sensor rig transform
    cost_function->AddParameterBlock(rig::NUM_RIGID_PARAMS);
    
    return cost_function;
  }

private:
  vw::Vector2 m_frame_pix; // The frame camera pixel 
  double m_weight;
  asp::RigCamInfo m_rig_cam_info;
  UsgsAstroLsSensorModel* m_ref_ls_model;
  UsgsAstroFrameSensorModel* m_curr_frame_model;
  UsgsAstroLsSensorModel* m_curr_ls_model;
  int m_begRefQuatIndex, m_endRefQuatIndex;
  int m_begRefPosIndex, m_endRefPosIndex;
  int m_begCurrQuatIndex, m_endCurrQuatIndex;
  int m_begCurrPosIndex, m_endCurrPosIndex;
  
}; // End class RigLsFramePixelReprojErr

// The implementation of operator() for RigLsFramePixelReprojErr
bool RigLsFramePixelReprojErr::operator()(double const * const * parameters, 
                                  double * residuals) const {

  try {
    
    // Make a copy of the model, as we will update quaternion and position
    // values that are being modified now. This may be expensive.
    // Update the shift too.
    UsgsAstroLsSensorModel ref_ls_cam = *m_ref_ls_model;
    int shift = 0;
    csm::EcefCoord P;
    updateLsModelTriPt(parameters, m_begRefQuatIndex, m_endRefQuatIndex,
                       m_begRefPosIndex, m_endRefPosIndex, shift, ref_ls_cam, P);
    
    // Move forward in the array of parameters, then recover the ref to curr transform
    shift += 1;
    double const* ref_to_curr_trans = parameters[shift];
    
    // Current camera to world transform based on the ref cam and the rig
    std::vector<double> cam2world_vec(rig::NUM_RIGID_PARAMS);
    asp::linescanToCurrSensorTrans(ref_ls_cam, m_rig_cam_info, ref_to_curr_trans,
                                   &cam2world_vec[0]); // output
    
    // Make a copy of the frame camera and set the latest position and orientation
    UsgsAstroFrameSensorModel frame_cam = *m_curr_frame_model;
    for (int coord = 0; coord < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; coord++)
      frame_cam.setParameterValue(coord, cam2world_vec[coord]);
    
    // Project in the camera with high precision. Do not use here anything lower
    // than 1e-8, as the CSM model can return junk. Convert to ASP pixel.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
    csm::ImageCoord imagePt = frame_cam.groundToImage(P, desired_precision);
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);
    
    // Compute the residuals  
    residuals[0] = m_weight*(pix[0] - m_frame_pix[0]);
    residuals[1] = m_weight*(pix[1] - m_frame_pix[1]);
    
  } catch (std::exception const& e) {
    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return true; // accept the solution anyway
  }

  return true;
}

// Reprojection error with ls ref sensor and frame curr sensor
void addRigLsFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                                  asp::RigCamInfo     const & rig_cam_info,
                                  vw::Vector2         const & frame_pix,
                                  double                      weight,
                                  UsgsAstroLsSensorModel    * ref_ls_model,
                                  UsgsAstroFrameSensorModel * curr_frame_model,
                                  double                    * ref_to_curr_trans,
                                  double                    * tri_point,
                                  ceres::Problem            & problem) {

  // The time when the frame camera pixel was observed
  double frame_time = rig_cam_info.beg_pose_time;
  if (frame_time != rig_cam_info.end_pose_time)
   vw::vw_throw(vw::ArgumentErr() 
                << "For a frame sensor beg and end pose time must be same.\n");
  
  // The solver needs to see beyond the current time as later the poses will
  // change relative to the observations. Here using linescan pixels,
  // not the frame pixel in the input.
  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(vw::Vector2(0.0, 0.0), imagePt1);
  asp::toCsmPixel(vw::Vector2(0.0, line_extra), imagePt2);
  double ans1 = ref_ls_model->getImageTime(imagePt1);
  double ans2 = ref_ls_model->getImageTime(imagePt2);
  double delta = std::abs(ans2 - ans1);
  double time1 = frame_time - delta;
  double time2 = frame_time + delta;

  // Find the range of indices that can affect the current pixel
  int numQuat       = ref_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double quatT0     = ref_ls_model->m_t0Quat;
  double quatDt     = ref_ls_model->m_dtQuat;
  int begRefQuatIndex = -1, endRefQuatIndex = -1;
  calcIndexBounds(time1, time2, quatT0, quatDt, numQuat, 
                  begRefQuatIndex, endRefQuatIndex); // outputs

  // Same for positions
  int numPos       = ref_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double posT0     = ref_ls_model->m_t0Ephem;
  double posDt     = ref_ls_model->m_dtEphem;
  int begRefPosIndex = -1, endRefPosIndex = -1;
  calcIndexBounds(time1, time2, posT0, posDt, numPos, 
                  begRefPosIndex, endRefPosIndex); // outputs
  
  // Quantities applicable here
  UsgsAstroLsSensorModel * curr_ls_model = NULL; 
  int begCurrQuatIndex = -1, endCurrQuatIndex = -1;
  int begCurrPosIndex = -1, endCurrPosIndex = -1;
  
  ceres::CostFunction* pixel_cost_function =
    RigLsFramePixelReprojErr::Create(frame_pix, weight, rig_cam_info, ref_ls_model,
                                     curr_frame_model, curr_ls_model, 
                                     begRefQuatIndex, endRefQuatIndex,
                                     begRefPosIndex, endRefPosIndex,
                                     begCurrQuatIndex, endCurrQuatIndex,
                                     begCurrPosIndex, endCurrPosIndex);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera quaternions and positions stored in the
  // camera models, the triangulated point, and the rig transform.
  std::vector<double*> vars;
  for (int it = begRefQuatIndex; it < endRefQuatIndex; it++)
    vars.push_back(&ref_ls_model->m_quaternions[it * NUM_QUAT_PARAMS]);
  for (int it = begRefPosIndex; it < endRefPosIndex; it++)
    vars.push_back(&ref_ls_model->m_positions[it * NUM_XYZ_PARAMS]);
  vars.push_back(tri_point);
  vars.push_back(ref_to_curr_trans); // transform from ref to curr sensor on the rig
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);
}

// Reprojection error with ls ref sensor and ls curr sensor
void addRigLsLsReprojectionErr(asp::BaBaseOptions  const & opt,
                               asp::RigCamInfo     const & rig_cam_info,
                               vw::Vector2         const & curr_observation,
                               double                      weight,
                               UsgsAstroLsSensorModel    * ref_ls_model,
                               UsgsAstroLsSensorModel    * curr_ls_model,
                               double                    * ref_to_curr_trans,
                               double                    * tri_point,
                               ceres::Problem            & problem) {

  // Find the positions and orientations in the current linescan model
  // that can affect the given pixel. Then, expand on this, as an even 
  // bigger range in the reference sensor will affect these.
   
  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(curr_observation - vw::Vector2(0.0, line_extra), imagePt1);
  asp::toCsmPixel(curr_observation + vw::Vector2(0.0, line_extra), imagePt2);
  double time1 = curr_ls_model->getImageTime(imagePt1);
  double time2 = curr_ls_model->getImageTime(imagePt2);

  std::cout << "--times are " << time1 << ' ' << time2 << std::endl;
  
  // Find the range of indices that can affect the current pixel
  int numQuat       = curr_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double quatT0     = curr_ls_model->m_t0Quat;
  double quatDt     = curr_ls_model->m_dtQuat;
  int begRefQuatIndex = -1, endRefQuatIndex = -1;
  calcIndexBounds(time1, time2, quatT0, quatDt, numQuat, 
                  begRefQuatIndex, endRefQuatIndex); // outputs

  // Same for positions
  int numPos       = curr_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double posT0     = curr_ls_model->m_t0Ephem;
  double posDt     = curr_ls_model->m_dtEphem;
  int begRefPosIndex = -1, endRefPosIndex = -1;
  calcIndexBounds(time1, time2, posT0, posDt, numPos, 
                  begRefPosIndex, endRefPosIndex); // outputs
  
  std::cout << "--beg and end quat index: " << begRefQuatIndex << ' ' << endRefQuatIndex << std::endl;
  std::cout << "--beg and end pos index: " << begRefPosIndex << ' ' << endRefPosIndex << std::endl;
  
  // This should not be strictly necessary, but an investigation to validate
  // this is not yet done.
  begRefQuatIndex--; endRefQuatIndex++;
  begRefPosIndex--; endRefPosIndex++;
   
  // Expand these to see the effect of the reference sensor
  time1 = std::min(quatT0 + begRefQuatIndex * quatDt, posT0 + begRefPosIndex * posDt);
  time2 = std::max(quatT0 + endRefQuatIndex * quatDt, posT0 + endRefPosIndex * posDt);
  
  std::cout << "--2 times are " << time1 << ' ' << time2 << std::endl;
  
  // Ref quaternions
  int ref_numQuat   = ref_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double ref_quatT0 = ref_ls_model->m_t0Quat;
  double ref_quatDt = ref_ls_model->m_dtQuat;
  calcIndexBounds(time1, time2, ref_quatT0, ref_quatDt, ref_numQuat, 
                  begRefQuatIndex, endRefQuatIndex); // outputs
  
  // Ref positions
  int ref_numPos   = ref_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double ref_posT0 = ref_ls_model->m_t0Ephem;
  double ref_posDt = ref_ls_model->m_dtEphem;
  calcIndexBounds(time1, time2, ref_posT0, ref_posDt, ref_numPos, 
                  begRefPosIndex, endRefPosIndex); // outputs
  
  std::cout << "--ref beg and end quat index: " << begRefQuatIndex << ' ' << endRefQuatIndex << std::endl;
  std::cout << "--ref beg and end pos index: " << begRefPosIndex << ' ' << endRefPosIndex << std::endl;
   
  std::cout << "--now in addRigLsLsReprojectionErr--\n";
  exit(0);
  
  
}

} // end namespace asp
