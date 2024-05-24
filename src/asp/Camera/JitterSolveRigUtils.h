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

/// \file JitterSolveRigUtils.h

// Functions invoked in jitter_solve.cc that need a rig.

#ifndef __ASP_CAMERA_JITTER_SOLVE_RIG_UTILS_H__
#define __ASP_CAMERA_JITTER_SOLVE_RIG_UTILS_H__

#include <asp/Camera/CsmModel.h>

#include <string>
#include <map>
#include <vector>

class UsgsAstroLsSensorModel;

namespace rig {
  class RigSet;
}

namespace asp {

// Create enum for senor type, which can be frame or linescan
enum RigSensorType {RIG_LINESCAN_SENSOR, RIG_FRAME_SENSOR}; 

// For each camera this will info info that will tie it to the rig
struct RigCamInfo {
  int sensor_id; // The sensor id in the rig set
  RigSensorType sensor_type; // frame or linescan
  
  // The time at the image center for linescan and mid group for frame
  double mid_group_time;
  
  // For linescan: the starting and ending time for positions/orientations.
  // For frame: both are the current pose time.
  double beg_pose_time, end_pose_time;
  
  // The index of the camera in opt.camera_models
  int cam_index;
  // The index of the reference camera in opt.camera_models. The reference
  // camera is assumed to be linescan.
  int ref_cam_index; 
  
  // Declare the constructor
  RigCamInfo();
};

// Book-keeping needed to tie each camera to the rig
void populateRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
                        std::map<int, int> const& orbital_groups,
                        // Outputs
                        std::vector<RigCamInfo> & rig_cam_info,
                        std::vector<double>     & ref_to_curr_sensor_vec);

// Given a linescan camera and the transform from it to the current camera,
// find the current camera to world transform as an array.
void linescanToCurrSensorTrans(const UsgsAstroLsSensorModel & ls_cam,
                               const asp::RigCamInfo & rig_cam_info,
                               double const* ref_to_curr_trans,
                               // Output
                               double * cam2world_arr);

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_RIG_UTILS_H__
