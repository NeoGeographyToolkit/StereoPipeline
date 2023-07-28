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

/// \file JitterSolveCostFuns.h

// Cost functions used in solving for jitter. These need access to the camera
// models, so they are stored in the Camera folder. The bigger functions defined
// here are implemented in the .cc file.

#ifndef __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__
#define __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__

#include <asp/Camera/CsmModel.h>
#include <asp/Camera/JitterSolveUtils.h>

#include <vw/Cartography/GeoReference.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <string>
#include <map>
#include <vector>

namespace asp {

const double g_big_pixel_value = 1000.0;  // don't make this too big

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

  // The implementation is in the .cc file
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

  // The implementation is in the .cc file
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
  static ceres::CostFunction* Create(vw::Vector3 const& observation, double const& weight){
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

// A Ceres cost function. The residual is the roll and/or yaw component of the camera
// rotation, as measured relative to the initial along-track direction. We assume
// that all positions are along the same segment in projected coordinates, or at
// least that the current position and its nearest neighbors are roughly on
// such a segment. That one is used to measure the roll/yaw from. This is consistent
// with how sat_sim creates the cameras.
struct weightedRollYawError {
  // See the .cc file for the implementation.  
  weightedRollYawError(std::vector<double>       const& positions, 
                   std::vector<double>           const& quaternions,
                   vw::cartography::GeoReference const& georef,
                   int cur_pos, double rollWeight, double yawWeight,
                   bool initial_camera_constraint);

  // Compute the weighted roll/yaw error between the current position and along-track
  // direction. Recall that quaternion = cam2world = sat2World * rollPitchYaw * rotXY.
  // rollPitchYaw is variable and can have jitter. Extract from it roll, pitch,
  // yaw. See the .cc file for the implementation.
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

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__
