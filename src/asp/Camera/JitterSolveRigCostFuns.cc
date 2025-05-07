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

#include <asp/asp_config.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/SatSimBase.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/cost_function.h>
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveRigCostFuns.h>
#include <asp/Rig/basic_algs.h>

#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Camera/CameraImage.h>

namespace asp {

// An error function minimizing the error of projecting an xyz point into a
// given CSM frame camera pixel that is on a rig with a linescan camera ref
// sensor. The variables of optimization are a portion of the position and
// quaternion variables affected by this, the triangulation point, and the
// transform from the ref linescan sensor to the current linescan/frame sensor.
struct RigLsPixelReprojErr {
  RigLsPixelReprojErr(vw::Vector2 const& curr_pix, double weight,
                      asp::RigCamInfo const& rig_cam_info,
                      UsgsAstroLsSensorModel* ref_ls_model,
                      UsgsAstroFrameSensorModel * curr_frame_model,
                      UsgsAstroLsSensorModel * curr_ls_model,
                      int begRefQuatIndex, int endRefQuatIndex, 
                      int begRefPosIndex, int endRefPosIndex,
                      int begCurrQuatIndex, int endCurrQuatIndex,
                      int begCurrPosIndex, int endCurrPosIndex):
    m_curr_pix(curr_pix), m_weight(weight),
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
  static ceres::CostFunction* Create(vw::Vector2 const& curr_pix, double weight,
                                     asp::RigCamInfo const& rig_cam_info,
                                     UsgsAstroLsSensorModel* ref_ls_model,
                                     UsgsAstroFrameSensorModel* curr_frame_model,
                                     UsgsAstroLsSensorModel* curr_ls_model,
                                     int begRefQuatIndex, int endRefQuatIndex,
                                     int begRefPosIndex, int endRefPosIndex,
                                     int begCurrQuatIndex, int endCurrQuatIndex,
                                     int begCurrPosIndex, int endCurrPosIndex) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<RigLsPixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RigLsPixelReprojErr>
      (new RigLsPixelReprojErr(curr_pix, weight, rig_cam_info, ref_ls_model,
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
  vw::Vector2 m_curr_pix; // The pixel on the current camera (rather than ref camera)
  double m_weight;
  asp::RigCamInfo m_rig_cam_info;
  UsgsAstroLsSensorModel* m_ref_ls_model;
  UsgsAstroFrameSensorModel* m_curr_frame_model;
  UsgsAstroLsSensorModel* m_curr_ls_model;
  int m_begRefQuatIndex, m_endRefQuatIndex;
  int m_begRefPosIndex, m_endRefPosIndex;
  int m_begCurrQuatIndex, m_endCurrQuatIndex;
  int m_begCurrPosIndex, m_endCurrPosIndex;
  
}; // End class RigLsPixelReprojErr

// The implementation of operator() for RigLsPixelReprojErr
bool RigLsPixelReprojErr::operator()(double const * const * parameters, 
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
    
    std::vector<double> cam2world_vec(rig::NUM_RIGID_PARAMS);
    
    // Project in the camera with high precision. Do not use here anything lower
    // than 1e-8, as the CSM model can return junk. Convert to ASP pixel.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
    csm::ImageCoord imagePt;
    if (m_curr_frame_model != NULL) {
      // Current camera to world transform based on the ref cam and the rig
      asp::linescanToCurrSensorTrans(ref_ls_cam, m_rig_cam_info.beg_pose_time, 
                                     ref_to_curr_trans,
                                     &cam2world_vec[0]); // output
    
      // Make a copy of the frame camera and set the latest position and orientation
      UsgsAstroFrameSensorModel curr_frame_cam = *m_curr_frame_model;
      for (int coord = 0; coord < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; coord++)
        curr_frame_cam.setParameterValue(coord, cam2world_vec[coord]);
      
      // Project the point into the frame camera
      imagePt = curr_frame_cam.groundToImage(P, desired_precision);
      
    } else if (m_curr_ls_model != NULL) {
      // Make a copy of the current linescan camera
      UsgsAstroLsSensorModel curr_ls_cam = *m_curr_ls_model;

      // Update the relevant quaternions in the local copy by interpolating
      // in the ref camera model and applying the rig transform.
      asp::updateLinescanWithRig(ref_ls_cam, ref_to_curr_trans, 
                                 curr_ls_cam, // update this
                                 m_begCurrQuatIndex, m_endCurrQuatIndex,
                                 m_begCurrPosIndex, m_endCurrPosIndex);

      // Project the point into the frame camera
      imagePt = curr_ls_cam.groundToImage(P, desired_precision);
      
    } else {
      vw::vw_throw(vw::ArgumentErr() << "Expecting either a frame or a linescan model.\n");
    }
    
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);
    
    // Compute the residuals  
    residuals[0] = m_weight*(pix[0] - m_curr_pix[0]);
    residuals[1] = m_weight*(pix[1] - m_curr_pix[1]);
    
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
                                  vw::Vector2         const & curr_pix,
                                  double                      weight,
                                  UsgsAstroLsSensorModel    * ref_ls_model,
                                  UsgsAstroFrameSensorModel * curr_frame_model,
                                  double                    * ref_to_curr_trans,
                                  double                    * tri_point,
                                  bool                        fix_rig_translations,
                                  bool                        fix_rig_rotations,
                                  ceres::SubsetManifold     * constant_transform_manifold,
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
  int numRefQuat   = ref_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double refQuatT0 = ref_ls_model->m_t0Quat;
  double refQuatDt = ref_ls_model->m_dtQuat;
  int begRefQuatIndex = -1, endRefQuatIndex = -1;
  calcIndexBounds(time1, time2, refQuatT0, refQuatDt, numRefQuat, 
                  begRefQuatIndex, endRefQuatIndex); // outputs

  // Same for positions
  int numRefPos   = ref_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double refPosT0 = ref_ls_model->m_t0Ephem;
  double refPosDt = ref_ls_model->m_dtEphem;
  int begRefPosIndex = -1, endRefPosIndex = -1;
  calcIndexBounds(time1, time2, refPosT0, refPosDt, numRefPos, 
                  begRefPosIndex, endRefPosIndex); // outputs
  
  // This should not be strictly necessary, but an investigation to validate
  // this is not yet done. Expand, just in case.
  begRefQuatIndex--; endRefQuatIndex++;
  begRefPosIndex--; endRefPosIndex++;
  // Keep in bounds
  begRefQuatIndex = std::max(0, begRefQuatIndex);
  begRefPosIndex = std::max(0, begRefPosIndex);
  endRefQuatIndex = std::min(endRefQuatIndex, numRefQuat);
  endRefPosIndex = std::min(endRefPosIndex, numRefPos);
  
  // Quantities not applicable here
  UsgsAstroLsSensorModel * curr_ls_model = NULL; 
  int begCurrQuatIndex = -1, endCurrQuatIndex = -1;
  int begCurrPosIndex = -1, endCurrPosIndex = -1;
  
  ceres::CostFunction* pixel_cost_function =
    RigLsPixelReprojErr::Create(curr_pix, weight, rig_cam_info, ref_ls_model,
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
  
  if (fix_rig_translations || fix_rig_rotations) 
    problem.SetManifold(ref_to_curr_trans, constant_transform_manifold);
}

// Reprojection error with ref ls ref sensor and curr ls sensor
// TODO(oalexan1): Find ways of integrating with the frame
void addRigLsLsReprojectionErr(asp::BaBaseOptions  const & opt,
                               asp::RigCamInfo     const & rig_cam_info,
                               vw::Vector2         const & curr_pix,
                               double                      weight,
                               UsgsAstroLsSensorModel    * ref_ls_model,
                               UsgsAstroLsSensorModel    * curr_ls_model,
                               double                    * ref_to_curr_trans,
                               double                    * tri_point,
                               bool                        fix_rig_translations,
                               bool                        fix_rig_rotations,
                               ceres::SubsetManifold     * constant_transform_manifold,
                               ceres::Problem            & problem) {

  // Find the positions and orientations in the current linescan model
  // that can affect the given pixel. Then, expand on this, as an even 
  // bigger range in the reference sensor will affect these.

  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(curr_pix - vw::Vector2(0.0, line_extra), imagePt1);
  asp::toCsmPixel(curr_pix + vw::Vector2(0.0, line_extra), imagePt2);
  double time1 = curr_ls_model->getImageTime(imagePt1);
  double time2 = curr_ls_model->getImageTime(imagePt2);
  
  // Find the range of indices that can affect the current pixel
  int numCurrQuat       = curr_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double currQuatT0     = curr_ls_model->m_t0Quat;
  double currQuatDt     = curr_ls_model->m_dtQuat;
  int begCurrQuatIndex = -1, endCurrQuatIndex = -1;
  calcIndexBounds(time1, time2, currQuatT0, currQuatDt, numCurrQuat, 
                  begCurrQuatIndex, endCurrQuatIndex); // outputs

  // Same for positions
  int numCurrPos       = curr_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double currPosT0     = curr_ls_model->m_t0Ephem;
  double currPosDt     = curr_ls_model->m_dtEphem;
  int begCurrPosIndex = -1, endCurrPosIndex = -1;
  calcIndexBounds(time1, time2, currPosT0, currPosDt, numCurrPos, 
                  begCurrPosIndex, endCurrPosIndex); // outputs

  // This should not be strictly necessary, but an investigation to validate
  // this is not yet done. Expand, just in case.
  begCurrQuatIndex--; endCurrQuatIndex++;
  begCurrPosIndex--; endCurrPosIndex++;
  // Keep in bounds
  begCurrQuatIndex = std::max(0, begCurrQuatIndex);
  begCurrPosIndex = std::max(0, begCurrPosIndex);
  endCurrQuatIndex = std::min(endCurrQuatIndex, numCurrQuat);
  endCurrPosIndex = std::min(endCurrPosIndex, numCurrPos);
 
  // Expand these to see the effect of the reference sensor
  time1 = std::min(currQuatT0 + begCurrQuatIndex * currQuatDt, 
                   currPosT0 + begCurrPosIndex * currPosDt);
  time2 = std::max(currQuatT0 + endCurrQuatIndex * currQuatDt, 
                   currPosT0 + endCurrPosIndex * currPosDt);
  
  // Ref quaternions
  int numRefQuat   = ref_ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double refQuatT0 = ref_ls_model->m_t0Quat;
  double refQuatDt = ref_ls_model->m_dtQuat;
  int begRefQuatIndex = -1, endRefQuatIndex = -1;
  calcIndexBounds(time1, time2, refQuatT0, refQuatDt, numRefQuat, 
                  begRefQuatIndex, endRefQuatIndex); // outputs
  
  // Ref positions
  int numRefPos   = ref_ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double refPosT0 = ref_ls_model->m_t0Ephem;
  double refPosDt = ref_ls_model->m_dtEphem;
  int begRefPosIndex = -1, endRefPosIndex = -1;
  calcIndexBounds(time1, time2, refPosT0, refPosDt, numRefPos, 
                  begRefPosIndex, endRefPosIndex); // outputs
  
  // Quantities not applicable here
  UsgsAstroFrameSensorModel * curr_frame_model = NULL; 
  
  ceres::CostFunction* pixel_cost_function =
    RigLsPixelReprojErr::Create(curr_pix, weight, rig_cam_info, ref_ls_model,
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
  
  if (fix_rig_translations || fix_rig_rotations) 
    problem.SetManifold(ref_to_curr_trans, constant_transform_manifold);
}

// An error function minimizing the error of projecting an xyz point into a
// given CSM frame camera pixel that is on a rig with a frame camera ref
// sensor. The variables of optimization are the ref cam bracketing 
// positions and orientations, the triangulation point, and the
// transform from the ref frame sensor to the current frame sensor.
struct RigFramePixelReprojErr {
  RigFramePixelReprojErr(vw::Vector2 const& curr_pix, double weight,
                           asp::RigCamInfo const& rig_cam_info,
                           UsgsAstroFrameSensorModel * curr_frame_model,
                           double beg_ref_time, double end_ref_time):
    m_curr_pix(curr_pix), m_weight(weight),
    m_rig_cam_info(rig_cam_info),
    m_curr_frame_model(curr_frame_model),
    m_beg_ref_time(beg_ref_time), m_end_ref_time(end_ref_time) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const; 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& curr_pix, double weight,
                                     asp::RigCamInfo const& rig_cam_info,
                                     UsgsAstroFrameSensorModel* curr_frame_model,
                                     double beg_ref_time, double end_ref_time) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<RigFramePixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RigFramePixelReprojErr>
      (new RigFramePixelReprojErr(curr_pix, weight, rig_cam_info, 
                                  curr_frame_model, beg_ref_time, end_ref_time));

    // Add a parameter block for beg and end positions
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    
    // Add a parameter block for beg and end quaternions
    cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
        
    // Add a parameter block for the ref to curr sensor rig transform
    cost_function->AddParameterBlock(rig::NUM_RIGID_PARAMS);
    
    // Add a parameter block for the xyz point
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);

    // The residual size the pixel size
    cost_function->SetNumResiduals(PIXEL_SIZE);
    
    return cost_function;
  }

private:
  vw::Vector2 m_curr_pix; // The pixel on the current camera (rather than ref camera)
  double m_weight;
  asp::RigCamInfo m_rig_cam_info;
  UsgsAstroFrameSensorModel* m_curr_frame_model;
  double m_beg_ref_time, m_end_ref_time;
}; // End class RigFramePixelReprojErr

// The implementation of operator() for RigFramePixelReprojErr
bool RigFramePixelReprojErr::operator()(double const * const * parameters, 
                                        double * residuals) const {

  try {

    double const* beg_frame_xyz_arr  = parameters[0];
    double const* end_frame_xyz_arr  = parameters[1];
    double const* beg_frame_quat_arr = parameters[2];
    double const* end_frame_quat_arr = parameters[3];
    double const* ref_to_curr_trans  = parameters[4];
    double const* tri_point          = parameters[5];

    double frame_time = m_rig_cam_info.beg_pose_time;

    // Must concatenate the position and orientation arrays
    double beg_frame_arr[rig::NUM_RIGID_PARAMS];
    double end_frame_arr[rig::NUM_RIGID_PARAMS];
    for (int i = 0; i < NUM_XYZ_PARAMS; i++) {
      beg_frame_arr[i] = beg_frame_xyz_arr[i];
      end_frame_arr[i] = end_frame_xyz_arr[i];
    }
    for (int i = 0; i < NUM_QUAT_PARAMS; i++) {
      beg_frame_arr[NUM_XYZ_PARAMS + i] = beg_frame_quat_arr[i];
      end_frame_arr[NUM_XYZ_PARAMS + i] = end_frame_quat_arr[i];
    }
    
    // Find the interpolated current cam2world transform
    double cam2world_arr[rig::NUM_RIGID_PARAMS];
    asp::interpCurrPose(m_beg_ref_time, m_end_ref_time, frame_time, 
                        beg_frame_arr, end_frame_arr, ref_to_curr_trans,
                        cam2world_arr);
    
    // Make a copy of the model, and set the latest position and orientation. 
    UsgsAstroFrameSensorModel curr_frame_cam = *m_curr_frame_model;
    for (int coord = 0; coord < rig::NUM_RIGID_PARAMS; coord++)
      curr_frame_cam.setParameterValue(coord, cam2world_arr[coord]);

    // Convert the 3D point to a CSM object
    csm::EcefCoord P;
    P.x = tri_point[0];
    P.y = tri_point[1];
    P.z = tri_point[2];
      
    // Project in the camera with high precision. Do not use here anything lower
    // than 1e-8, as the CSM model can return junk.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
    csm::ImageCoord imagePt = curr_frame_cam.groundToImage(P, desired_precision);
   
    // Convert to ASP pixel
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);
    
    // Compute the residuals  
    residuals[0] = m_weight*(pix[0] - m_curr_pix[0]);
    residuals[1] = m_weight*(pix[1] - m_curr_pix[1]);
    
  } catch (std::exception const& e) {
    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return true; // accept the solution anyway
  }

  return true;
}

// Reprojection error with ls ref sensor and frame curr sensor
void addRigFrameFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                                     asp::RigCamInfo     const & rig_cam_info,
                                     std::vector<asp::CsmModel*> const& csm_models,
                                     std::map<int, int>  const & cam2group,
                                     TimestampMap        const & timestamp_map,
                                     vw::Vector2         const & curr_pix,
                                     double                      weight,
                                     UsgsAstroFrameSensorModel * curr_frame_model,
                                     std::vector<double>       & frame_params,
                                     double                    * ref_to_curr_trans,
                                     double                    * tri_point,
                                     bool                        fix_rig_translations,
                                     bool                        fix_rig_rotations,
                                     ceres::SubsetManifold     * constant_transform_manifold,
                                     ceres::Problem            & problem) {

  double beg_ref_time = 0.0, end_ref_time = 0.0;
  int beg_ref_index = 0, end_ref_index = 0;
  bool success = timestampBrackets(rig_cam_info, cam2group, timestamp_map, 
                                   // Outputs
                                   beg_ref_time, end_ref_time, 
                                   beg_ref_index, end_ref_index);
  
  if (!success) 
    return;

  double * beg_frame_arr = &frame_params[beg_ref_index * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
  double * end_frame_arr = &frame_params[end_ref_index * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];

  ceres::CostFunction* pixel_cost_function =
    RigFramePixelReprojErr::Create(curr_pix, weight, rig_cam_info, curr_frame_model, 
                                   beg_ref_time, end_ref_time);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are bracketing ref camera poses, stored
  // separately as position and orientation, the rig transform, and the
  // triangulated point.
  std::vector<double*> vars;
  vars.push_back(&beg_frame_arr[0]);
  vars.push_back(&end_frame_arr[0]);
  vars.push_back(&beg_frame_arr[NUM_XYZ_PARAMS]);
  vars.push_back(&end_frame_arr[NUM_XYZ_PARAMS]);
  vars.push_back(ref_to_curr_trans);
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);
  
  if (fix_rig_translations || fix_rig_rotations) 
    problem.SetManifold(ref_to_curr_trans, constant_transform_manifold);
}

// Add the ls or frame camera model reprojection error to the cost function
void addRigLsOrFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                                    int                         icam,
                                    UsgsAstroLsSensorModel    * ref_ls_model,
                                    UsgsAstroFrameSensorModel * ref_frame_model,
                                    UsgsAstroLsSensorModel    * ls_model,
                                    UsgsAstroFrameSensorModel * frame_model,
                                    std::vector<double>       & frame_params,
                                    std::vector<asp::CsmModel*> const& csm_models,
                                    std::map<int, int>          const& cam2group,
                                    TimestampMap                const & timestamp_map,
                                    vw::Vector2                 const & pix_obs,
                                    double                      pix_wt,
                                    double                    * tri_point,
                                    double                    * ref_to_curr_sensor_trans, 
                                    asp::RigCamInfo     const & rig_info,
                                    bool                        fix_rig_translations,
                                    bool                        fix_rig_rotations,
                                    ceres::Problem            & problem) {

  // Prepare for the case of fixed rig translations and/or rotations    
  ceres::SubsetManifold* constant_transform_manifold = nullptr;
  bool no_rig = false;
  rig::setUpFixRigOptions(no_rig, fix_rig_translations, 
                          fix_rig_rotations,
                          constant_transform_manifold);

  if (ref_ls_model != NULL) {
    
    // Sanity check: the current pixel time must be in bounds
    double curr_time = -1.0;
    if (frame_model != NULL) {
      curr_time = rig_info.beg_pose_time;
    } else if (ls_model != NULL) {
      csm::ImageCoord imagePt;
      asp::toCsmPixel(pix_obs, imagePt);
      curr_time = ls_model->getImageTime(imagePt);
    }
    double beg_ref_time = -1.0, end_ref_time = -1.0;
    asp::linescanTimeBounds(ref_ls_model, beg_ref_time, end_ref_time);
    if (curr_time < beg_ref_time || curr_time > end_ref_time)
      vw::vw_throw(vw::ArgumentErr() 
                  << std::setprecision(17)
                  << "Time " << curr_time
                  << " is outside the time range of the reference sensor, which is: "
                  << beg_ref_time << ' ' << end_ref_time << ".\n"
                  << "This may also be due to mixup of image and camera order, or because of "
                  << "anchor points far out of range. Offending pixel: " 
                  << pix_obs << ".\n");
  
    // Ref sensor is linescan
    if (frame_model != NULL)
      addRigLsFrameReprojectionErr(opt, rig_info, pix_obs, pix_wt, ref_ls_model, 
                  frame_model, ref_to_curr_sensor_trans, tri_point, 
                  fix_rig_translations, fix_rig_rotations, 
                  constant_transform_manifold,
                  problem);
    else if (ls_model != NULL)
      addRigLsLsReprojectionErr(opt, rig_info, pix_obs, pix_wt, ref_ls_model, 
                  ls_model, ref_to_curr_sensor_trans, tri_point, 
                  fix_rig_translations, fix_rig_rotations,
                  constant_transform_manifold,
                  problem);
    else 
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
  
  } else if (ref_frame_model != NULL) {

    // The check for time being in bounds will be done when attempting to interpolate
    
    if (frame_model != NULL)
      addRigFrameFrameReprojectionErr(opt, rig_info, 
                                      csm_models, cam2group, timestamp_map, 
                                      pix_obs, pix_wt, 
                                      frame_model, frame_params,
                                      ref_to_curr_sensor_trans, tri_point, 
                                      fix_rig_translations, fix_rig_rotations,
                                      constant_transform_manifold,
                                      problem);
    else if (ls_model != NULL)
      vw::vw_throw(vw::ArgumentErr() << "When the reference sensor is frame, "
                   << "the other sensors must also be frame.\n");
    else 
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
       
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
  }  
}

} // end namespace asp

#endif // ASP_HAVE_PKG_ISISIO

