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

/// \file JitterSolveUtils.h

// Low-level functions used in jitter_solve.cc.

#ifndef __ASP_CAMERA_JITTER_SOLVE_UTILS_H__
#define __ASP_CAMERA_JITTER_SOLVE_UTILS_H__

#include <asp/Camera/CsmModel.h>

#include <vw/Cartography/GeoReference.h>

#include <string>
#include <map>
#include <vector>

namespace rig {
  class RigSet;
}

namespace asp {

class RigCamInfo;

// If several images are acquired in quick succession along the same orbit and
// stored in the same list, record this structure by grouping them together.
// Each element in the input vector below is either a standalone image, then it
// is in a group of its own, or it is a list of images, all going to the same
// group. Here we ignore the cameras. Matching cameras to images will be done
// outside of this function.
void readGroupStructure(std::vector<std::string> const & image_lists,
                        std::map<int, int> & cam2group);


// Given a set of integers in increasing order, with each assigned to a group,
// return the index of the current integer in its group.
int indexInGroup(int icam, std::map<int, int> const& cam2group);

// For frame cameras that belong to the same orbital group, collect together
// the initial positions in a single vector, and same for quaternions. Linescan 
// cameras are skipped as their positions/quaternions are already in one vector.
void formPositionQuatVecPerGroup(std::map<int, int> const& cam2group,
               std::vector<asp::CsmModel*> const& csm_models,
               // Outputs 
               std::map<int, std::vector<double>> & orbital_group_positions,
               std::map<int, std::vector<double>> & orbital_group_quaternions);

// For the cameras that are of frame type, copy the initial values of camera position
// and orientation to the vector of variables that we will optimize. Have to keep this 
// vector separate since UsgsAstroFrameSensorModel does not have a way to access the
// underlying array of variables directly.
void initFrameCameraParams(std::vector<asp::CsmModel*> const& csm_models,
  std::vector<double> & frame_params); // output

// Update the cameras given the optimized parameters
void updateCameras(bool have_rig,
                   rig::RigSet                  const& rig,
                   std::vector<asp::RigCamInfo> const& rig_cam_info,
                   std::vector<double>          const& ref_to_curr_sensor_vec,
                   std::vector<asp::CsmModel*>       & csm_models,
                   std::vector<double>               & frame_params);

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_UTILS_H__