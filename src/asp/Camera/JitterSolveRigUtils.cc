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

// Functions invoked in jitter_solve.cc that use a rig.

#include <asp/Rig/rig_config.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/JitterSolveRigUtils.h>
#include <asp/Core/BundleAdjustUtils.h>

#include <vw/Core/Log.h>
#include <vw/Math/Functors.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>

namespace asp {

// Find the timestamps for each group of frame cameras
void timestampsPerGroup(std::map<int, int> const& orbital_groups,
               std::vector<asp::CsmModel*> const& csm_models,
               std::vector<RigCamInfo> const& rig_cam_info,
               // Outputs 
               std::map<int, std::vector<double>> & group_timestamps) {

  // Wipe the output
  group_timestamps.clear();

  int num_cams = csm_models.size();
  for (int icam = 0; icam < num_cams; icam++) {
    
    auto it = orbital_groups.find(icam);
    if (it == orbital_groups.end())
      vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Failed to find orbital group for camera.\n"); 
    int group_id = it->second;

    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (frame_model == NULL)
      continue; // Skip non-frame cameras
    group_timestamps[group_id].push_back(rig_cam_info[icam].beg_pose_time);
  }
}

// First pass at collecting info about relationships between cameras and the rig
void populateInitRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
                        std::map<int, int> const& orbital_groups,
                        // Outputs
                        std::vector<RigCamInfo> & rig_cam_info) {

  int num_images = csm_models.size();
  rig_cam_info.resize(num_images);

  // Sanity check
  if (num_images != (int)image_files.size() || num_images != (int)camera_files.size())
    vw::vw_throw(vw::ArgumentErr() 
             << "Expecting the number of cameras to match the number of images.\n");
    
  // Print the image names
  for (int i = 0; i < num_images; i++) 
    vw::vw_out() << "Image: " << image_files[i] << std::endl;
  // Print the camera names
  for (int i = 0; i < num_images; i++)  {
    vw::vw_out() << "Camera: " << camera_files[i] << std::endl;

    auto & rig_info = rig_cam_info[i]; // alias
    
    // Each camera must know where it belongs
    rig_info.cam_index = i;
    
    // Get the underlying linescan model or frame model
    asp::CsmModel * csm_cam = csm_models[i];
    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_cam->m_gm_model).get());
    
    if (frame_model != NULL) {
      
      int cam_type = -1;
      double timestamp = -1.0; 
      rig::findCamTypeAndTimestamp(camera_files[i], rig.cam_names,
                                  cam_type, timestamp);
      rig_info.sensor_type = RIG_FRAME_SENSOR;
      rig_info.sensor_id = cam_type;
      // The mid_group_time will be the mid time for the frame group, to be filled later.
      rig_info.beg_pose_time = timestamp;
      rig_info.end_pose_time = timestamp;
      
    } else if (ls_model != NULL) {

      int cam_type = -1; 
      rig::findCamType(camera_files[i], rig.cam_names, cam_type);
      int numPos     = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
      double beg_pos_time = ls_model->m_t0Ephem;
      double pos_dt   = ls_model->m_dtEphem;
      double end_pos_time = beg_pos_time + (numPos - 1) * pos_dt;
      int numQuat    = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
      double beg_quat_time = ls_model->m_t0Quat;
      double quat_dt   = ls_model->m_dtQuat;
      double end_quat_time = beg_quat_time + (numQuat - 1) * quat_dt;
      double beg_time = std::max(beg_pos_time, beg_quat_time);
      double end_time = std::min(end_pos_time, end_quat_time);

      int numLines = ls_model->m_nLines;
      csm::ImageCoord imagePt;
      asp::toCsmPixel(vw::Vector2(0, numLines/2.0), imagePt);
      rig_info.sensor_type = RIG_LINESCAN_SENSOR;
      rig_info.sensor_id = cam_type;
      // Each pose has its time. The mid group time is a compromise between them.
      rig_info.mid_group_time = ls_model->getImageTime(imagePt);
      rig_info.beg_pose_time = beg_time;
      rig_info.end_pose_time = end_time;
      
    } else {
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
    }
  }
}    

// Each frame camera will have a timestamp. Find the median timestamp for each group
// of such cameras.
void populateFrameGroupMidTimestamp(std::vector<asp::CsmModel*>        const& csm_models,
                                    std::map<int, int>                 const& orbital_groups,
                                    std::map<int, std::vector<double>> const& group_timestamps,
                                    // Outputs
                                    std::vector<RigCamInfo> & rig_cam_info) {

  int num_cams = csm_models.size();
  for (int icam = 0; icam < num_cams; icam++) {
    
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (frame_model == NULL)
      continue; // Skip non-frame cameras
    
    auto it = orbital_groups.find(icam);
    if (it == orbital_groups.end())
      vw::vw_throw(vw::ArgumentErr() 
         << "Failed to find orbital group for camera.\n"); 
    int group_id = it->second;

    auto g_it = group_timestamps.find(group_id);
    if (g_it == group_timestamps.end())
      vw::vw_throw(vw::ArgumentErr() 
         << "Failed to find group in timestamps.\n");
    
    std::vector<double> timestamps = g_it->second;
    double median = vw::math::destructive_median(timestamps);
    rig_cam_info[icam].mid_group_time = median;
  }
}

// For each camera, find the camera closest in time that was acquired with the
// reference sensor on the same rig as the current camera. The complexity of
// this is total number of cameras times the number of cameras acquired with
// a reference sensor. Something more clever did not work, and this may be
// good enough.
void findClosestRefCamera(rig::RigSet const& rig,
                          std::vector<asp::CsmModel*> const& csm_models,
                          std::vector<RigCamInfo> & rig_cam_info) {
  
  // Find the cameras acquired with the reference sensor on a rig
  int num_cams = csm_models.size();
  std::vector<int> ref_sensor_cams;
  for (int icam = 0; icam < num_cams; icam++) {
    auto const& info = rig_cam_info[icam]; // alias
    if (!rig.isRefSensor(info.sensor_id))
      continue;
    ref_sensor_cams.push_back(icam);
  }

  for (int icam = 0; icam < num_cams; icam++) {
   
    auto const& info = rig_cam_info[icam]; // alias
    int sensor_id = info.sensor_id;
    int ref_sensor_id = rig.refSensorId(info.sensor_id);
    
    // Iterate over ref cameras
    double max_time = std::numeric_limits<double>::max();
    for (int ref_cam_index: ref_sensor_cams) {
      
      auto const& ref_info = rig_cam_info[ref_cam_index]; // alias
      int ref_sensor_id2 = rig.refSensorId(ref_info.sensor_id);
      
      // This must be a ref sensor
      if (ref_sensor_id2 != ref_info.sensor_id) 
        vw::vw_throw(vw::ArgumentErr() << "Expecting a ref sensor.\n");
        
      if (ref_sensor_id2 != ref_sensor_id) 
        continue; // Not the same ref sensor
        
      // See if it is closer than the current best
      double dt = fabs(ref_info.mid_group_time - info.mid_group_time);
      if (dt >= max_time) 
        continue;
        
      max_time = dt;
      rig_cam_info[icam].ref_cam_index = ref_info.cam_index;
    } // end iterating over ref sensor cameras
  } // end iterating over cameras
}
                           
// Find the relationship between the cameras relative to the rig
void populateRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
                        std::map<int, int> const& orbital_groups,
                        // Outputs
                        std::vector<RigCamInfo> & rig_cam_info) {
  
  // Print a message, as this can take time
  vw::vw_out() << "Determine the rig relationships for the cameras.\n";
   
  // Initialize the rig cam info after a first pass through the cameras
  populateInitRigCamInfo(rig, image_files, camera_files, csm_models, 
                         orbital_groups, rig_cam_info);

  // Find the range of timestamps for each group of frame cameras
  std::map<int, std::vector<double>> group_timestamps;
  timestampsPerGroup(orbital_groups, csm_models, rig_cam_info, group_timestamps);
  
  // Find the mid time for each frame group as the median of the group timestamps  
  populateFrameGroupMidTimestamp(csm_models, orbital_groups, group_timestamps, 
                                 rig_cam_info);

  // For each camera find the closest camera acquired with the reference sensor
  findClosestRefCamera(rig, csm_models, rig_cam_info);
   
  // Print for each element in rig_cam_info the type, sensor id, and times
  // for (size_t i = 0; i < rig_cam_info.size(); i++) {
  //   auto const& info = rig_cam_info[i];
  //   std::cout << "cam index: " << i << info.cam_index << " type: " << info.sensor_type 
  //     << " id: " << info.sensor_id
  //     << " mid time: " << info.mid_group_time << " beg time: " << info.beg_pose_time
  //     << " end time: " << info.end_pose_time << " ref camera index: " << info.ref_cam_index 
  //     << "\n";
  // }
}

} // end namespace asp
