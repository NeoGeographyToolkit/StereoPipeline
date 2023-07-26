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

// Cost functions used in solving for jitter. These need access to the camera models,
// so they are stored in the Camera folder.

#ifndef __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__
#define __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__

#include <asp/Camera/CsmModel.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <string>
#include <map>
#include <vector>

namespace asp {

const int NUM_XYZ_PARAMS  = 3;
const int NUM_QUAT_PARAMS = 4;
const int PIXEL_SIZE      = 2;

const double g_big_pixel_value = 1000.0;  // don't make this too big

// An error function minimizing the error of projecting an xyz point
// into a given CSM linescan camera pixel. The variables of optimization are a
// portion of the position and quaternion variables affected by this.
struct LsPixelReprojErr {
  LsPixelReprojErr(vw::Vector2 const& observation, double weight,
                   UsgsAstroLsSensorModel* ls_model,
                   int begQuatIndex, int endQuatIndex, int begPosIndex, int endPosIndex):
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

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_COST_FUNS_H__
