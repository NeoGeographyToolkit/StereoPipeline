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

// Cost functions that tie the camera poses to external data in jitter solving:
// DEM heights, triangulated points, GCP, camera-position priors, and the
// quaternion-norm term. Stored in the Camera folder, as they use the camera models.

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

/// A ceres cost function. The residual is the difference between the observed
/// 3D point and the current (floating) 3D point, multiplied by given weight.
struct weightedXyzError {
  weightedXyzError(vw::Vector3 const& observation, double weight):
    m_observation(observation), m_weight(weight) {}

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
  static ceres::CostFunction* Create(const double * init_quat, double weight) {
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
  static ceres::CostFunction* Create(const double * init_position, double weight) {
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

// Add the constraint based on DEM
void addDemConstraint(asp::BaBaseOptions       const& opt,
                      std::vector<vw::Vector3> const& dem_xyz_vec,
                      std::set<int>            const& outliers,
                      vw::ControlNetwork   const& cnet,
                      // Outputs
                      std::vector<double>           & tri_points_vec,
                      std::vector<double>           & weight_per_residual, // append
                      ceres::Problem                & problem) {

  double xyz_weight = -1.0, xyz_threshold = -1.0;

  if (!opt.heights_from_dem.empty() || !opt.heights_from_dem_list.empty()) {
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

    if (cnet[ipt].type() == vw::ControlPoint::GroundControlPoint)
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
                      vw::ControlNetwork const& cnet,
                      asp::CRN              const& crn,
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
    if (cnet[ipt].type() == vw::ControlPoint::GroundControlPoint ||
        cnet[ipt].type() == vw::ControlPoint::PointFromDem)
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
                      vw::ControlNetwork      & cnet,
                      std::vector<double>         & tri_points_vec,
                      std::vector<double>         & weight_per_residual, // append
                      ceres::Problem              & problem) {

  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {

    if (cnet[ipt].type() != vw::ControlPoint::GroundControlPoint)
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

    // By default GCP get a non-robust loss, as they are assumed to be accurate.
    // With --gcp-robust-threshold, a robust loss (of type --cost-function) is
    // applied instead. The robust threshold acts on the GCP residual already
    // normalized by the GCP sigma (see XYZError / LLHError).
    ceres::LossFunction* loss_function = NULL;
    if (opt.gcp_robust_threshold > 0)
      loss_function = get_loss_function(opt.cost_function, opt.gcp_robust_threshold);
    else
      loss_function = new ceres::TrivialLoss();
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
                                  asp::CRN                        const& crn,
                                  std::vector<asp::CsmModel*>      const& csm_models,
                                  std::vector<std::vector<double>> const& count_per_cam,
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
                                        weight, opt.datum,
                                        opt.camera_position_uncertainty_power);
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
                                        weight, opt.datum,
                                        opt.camera_position_uncertainty_power);
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
                              asp::CRN                        const& crn,
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
                        asp::CRN                   const& crn,
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

} // end namespace asp
