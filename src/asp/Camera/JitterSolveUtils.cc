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

// Low-level functions used in jitter_solve.cc.

#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/JitterSolveUtils.h>
#include <asp/Core/Common.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/JitterSolveRigUtils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/transform_utils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace asp {

// If several images are acquired in quick succession along the same orbit and
// stored in the same list, record this structure by grouping them together.
// Each element in the input vector below is either a standalone image, then it
// is in a group of its own, or it is a list of images, all going to the same
// group. Here we ignore the cameras. Matching cameras to images will be done
// outside of this function.
void readGroupStructure(std::vector<std::string> const & image_lists,
                        std::map<int, int> & cam2group) {

  // Wipe the output
  cam2group.clear();

  int group_count = 0, image_count = 0;
  for (size_t i = 0; i < image_lists.size(); i++) {

    // The case when we have a standalone image
    if (asp::has_image_extension(image_lists[i])) {
      cam2group[image_count] = group_count;
      group_count++;
      image_count++;
      continue;
    }

    // Check if we have a list, ending in .txt. Here we skip over the cameras.
    std::string ext = vw::get_extension(image_lists[i]);
    if (ext != ".txt")
      continue;

    // Read the list
    std::vector<std::string> image_names;
    asp::read_list(image_lists[i], image_names);
    
    // Add a new group, and let all images in the list be in that group
    bool has_images = false;
    for (size_t j = 0; j < image_names.size(); j++) {
      if (!asp::has_image_extension(image_names[j]))
        continue;
      if (!fs::exists(image_names[j])) // additional robustness check
        continue; 

      has_images = true;
      cam2group[image_count] = group_count;
      image_count++;
    }

    if (has_images)
      group_count++;
  }

  return;
} 

// Given a set of integers in increasing order, with each assigned to a group,
// return the index of the current integer in its group.
int indexInGroup(int icam, std::map<int, int> const& cam2group) {

  auto it = cam2group.find(icam);
  if (it == cam2group.end())
    vw::vw_throw(vw::ArgumentErr() << "indexInGroup: Camera not found.\n");

  int group_id = it->second;

  // Now iterate over all the integers in the group having icam,
  // and the index of icam in its group.
  int pos_in_group = -1;
  for (auto it = cam2group.begin(); it != cam2group.end(); it++) {
    
    if (it->second != group_id)
      continue; // not the same group

    pos_in_group++;

    if (it->first == icam)
        return pos_in_group;
  }

  // Throw an error, as we could not find the integer in the group
  vw::vw_throw(vw::ArgumentErr() << "indexInGroup: Could not find camera in group.\n");

  return -1;
}

// For frame cameras that belong to the same orbital group, collect together
// the initial positions in a single vector, and same for quaternions. Linescan 
// cameras are skipped as their positions/quaternions are already in one vector.
void formPositionQuatVecPerGroup(std::map<int, int> const& cam2group,
               std::vector<asp::CsmModel*> const& csm_models,
               // Outputs 
               std::map<int, std::vector<double>> & orbital_group_positions,
               std::map<int, std::vector<double>> & orbital_group_quaternions) {

  // Wipe the outputs
  orbital_group_positions.clear();
  orbital_group_quaternions.clear();

  int num_cams = csm_models.size();
  for (int icam = 0; icam < num_cams; icam++) {
    
    auto it = cam2group.find(icam);
    if (it == cam2group.end())
      vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Failed to find orbital group for camera.\n"); 
    int group_id = it->second;

    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (frame_model == NULL)
      continue; // Skip non-frame cameras
   
    // Append the positions
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        orbital_group_positions[group_id].push_back(frame_model->getParameterValue(c));
    // Append the quaternions
    for (int c = NUM_XYZ_PARAMS; c < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; c++)
        orbital_group_quaternions[group_id].push_back(frame_model->getParameterValue(c));
  }
  
  // TODO(oalexan1): Must normalize the quaternions per group here and ensure
  // the same sign for the dominant coefficient.

  return;
}

// For the cameras that are of frame type, copy the initial values of camera position
// and orientation to the vector of variables that we will optimize. Have to keep this 
// vector separate since UsgsAstroFrameSensorModel does not have a way to access the
// underlying array of variables directly.
void initFrameCameraParams(std::vector<asp::CsmModel*> const& csm_models,
  std::vector<double> & frame_params) { // output

  // Note how we allocate storage for all cameras, for simplicity.
  frame_params.resize((NUM_XYZ_PARAMS + NUM_QUAT_PARAMS) * csm_models.size(), 0.0);

  for (size_t icam = 0; icam < csm_models.size(); icam++) {

    UsgsAstroFrameSensorModel * frame_model
     = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (frame_model == NULL)
      continue;

    // The UsgsAstroFrameSensorModel stores first 3 position parameters, then 4
    // quaternion parameters.
    double * vals = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
    for (size_t i = 0; i < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; i++)
      vals[i] = frame_model->getParameterValue(i); 
  }
}

// Update the cameras given the optimized parameters. Without a rig,
// the linescan cameras are optimized in-place but the frame cameras still
// have external parameters. 
void updateCameras(bool have_rig,
                   rig::RigSet                  const& rig,
                   std::vector<asp::RigCamInfo> const& rig_cam_info,
                   std::vector<double>          const& ref_to_curr_sensor_vec,
                   std::vector<asp::CsmModel*>       & csm_models,
                   std::vector<double>               & frame_params) {
   
  // Sanity check. The allocated space is for all cameras, for simplicity.
  if (frame_params.size() != (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS) * csm_models.size())
    vw::vw_throw(vw::ArgumentErr() << "Invalid number of frame camera parameters.");

  for (size_t icam = 0; icam < csm_models.size(); icam++) {

    UsgsAstroLsSensorModel * ls_model
     = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
     = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

    // Current frame cam parameters. Will be updated for rig. 
    double * frame_arr = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];

    if (have_rig) {
      auto rig_info = rig_cam_info[icam];
      int ref_cam   = rig_info.ref_cam_index;
      int sensor_id = rig_info.sensor_id;
      double const* ref_to_curr_sensor_trans 
                    = &ref_to_curr_sensor_vec[rig::NUM_RIGID_PARAMS * sensor_id];
      
      // For now, the reference sensor must be linescan
      UsgsAstroLsSensorModel * ref_ls_model 
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[ref_cam]->m_gm_model).get());
      if (ref_ls_model == NULL)
        vw::vw_throw(vw::ArgumentErr() << "The rig reference sensor must be linescan.\n");

      if (ls_model != NULL) {
        // Update the ls model
        asp::updateLinescanWithRig(*ref_ls_model, ref_to_curr_sensor_trans,
                                   *ls_model); // update this
      } else if (frame_model != NULL) {
        // Update the frame camera model
        asp::linescanToCurrSensorTrans(*ref_ls_model, rig_info.beg_pose_time, 
                                     ref_to_curr_sensor_trans,
                                     frame_arr); // output
      } else {
        vw::vw_throw(vw::ArgumentErr() << "Unknown camera type.\n");
      }
    }

    if (frame_model != NULL) {
      // Update the frame camera model based on frame_arr. The
      // UsgsAstroFrameSensorModel stores first 3 position parameters, then 4
      // quaternion parameters.
      for (size_t i = 0; i < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; i++)
        frame_model->setParameterValue(i, frame_arr[i]); 
    }
  }

  return;
}

} // end namespace asp
