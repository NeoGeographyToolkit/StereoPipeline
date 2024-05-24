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

#ifndef __ASP_CAMERA_JITTER_SOLVE_RIG_COST_FUNS_H__
#define __ASP_CAMERA_JITTER_SOLVE_RIG_COST_FUNS_H__

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <string>
#include <map>
#include <vector>

class UsgsAstroLsSensorModel;
class UsgsAstroFrameSensorModel;

namespace asp {

struct BaBaseOptions;

// Reprojection error with ls ref sensor and frame curr sensor
void addRigLsFrameReprojectionErr(asp::BaBaseOptions  const & opt,
                                  asp::RigCamInfo     const & rig_cam_info,
                                  vw::Vector2         const & frame_observation,
                                  double                      weight,
                                  UsgsAstroLsSensorModel    * ref_ls_model,
                                  UsgsAstroFrameSensorModel * curr_frame_model,
                                  double                    * ref_to_curr_trans,
                                  double                    * tri_point,
                                  ceres::Problem            & problem);

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_RIG_COST_FUNS_H__
