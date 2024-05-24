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
#include <asp/Rig/transform_utils.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/JitterSolveRigUtils.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/EigenTransformUtils.h>

#include <vw/Core/Log.h>
#include <vw/Math/Functors.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>

namespace asp {

// Constructor for class RigCamInfo. Initialize all fields to NaN to ensure full
// initialization later. That is important, because initialization happens over
// many stages and we want to catch any errors.
RigCamInfo::RigCamInfo() {
  double nan     = std::numeric_limits<double>::quiet_NaN();
  sensor_id      = nan;
  sensor_type    = RIG_LINESCAN_SENSOR;
  mid_group_time = nan;
  beg_pose_time  = nan;
  end_pose_time  = nan;
  cam_index      = nan;
  ref_cam_index  = nan;
}

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
    
  for (int i = 0; i < num_images; i++)  {

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
      
      int sensor_id = -1;
      double timestamp = -1.0; 
      rig::findCamTypeAndTimestamp(camera_files[i], rig.cam_names,
                                  sensor_id, timestamp);
      rig_info.sensor_type = RIG_FRAME_SENSOR;
      rig_info.sensor_id = sensor_id;
      // The mid_group_time will be the mid time for the frame group, to be filled later.
      rig_info.beg_pose_time = timestamp;
      rig_info.end_pose_time = timestamp;
      
    } else if (ls_model != NULL) {

      int sensor_id = -1; 
      rig::findCamType(camera_files[i], rig.cam_names, sensor_id);
      int numPos           = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
      double beg_pos_time  = ls_model->m_t0Ephem;
      double pos_dt        = ls_model->m_dtEphem;
      double end_pos_time  = beg_pos_time + (numPos - 1) * pos_dt;
      int numQuat          = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
      double beg_quat_time = ls_model->m_t0Quat;
      double quat_dt       = ls_model->m_dtQuat;
      double end_quat_time = beg_quat_time + (numQuat - 1) * quat_dt;
      double beg_time      = std::max(beg_pos_time, beg_quat_time);
      double end_time      = std::min(end_pos_time, end_quat_time);

      int numLines = ls_model->m_nLines;
      csm::ImageCoord imagePt;
      asp::toCsmPixel(vw::Vector2(0, numLines/2.0), imagePt);
      rig_info.sensor_type = RIG_LINESCAN_SENSOR;
      rig_info.sensor_id = sensor_id;
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

// Given all camera-to-world transforms, find the median rig transforms.
// This is robust to outliers.
void calcRigTransforms(rig::RigSet const& rig,
                       std::vector<asp::CsmModel*> const& csm_models,
                       std::vector<RigCamInfo> const& rig_cam_info,
                       // Outputs
                       std::vector<double> & ref_to_curr_sensor_vec) {
  
  int num_rig_sensors = rig.cam_names.size();
  ref_to_curr_sensor_vec.resize(rig::NUM_RIGID_PARAMS * num_rig_sensors, 0.0);
  std::map<int, std::vector<Eigen::MatrixXd>> transforms_map;
  
  for (int icam = 0; icam < (int)csm_models.size(); icam++) {

    auto const& rig_info = rig_cam_info[icam]; // alias
    Eigen::Affine3d ref_to_curr;

    // Get the underlying linescan model or frame model
    asp::CsmModel * csm_cam = csm_models[icam];
    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_cam->m_gm_model).get());
    // Find the ref camera
    int ref_cam_index = rig_info.ref_cam_index;
    UsgsAstroLsSensorModel * ref_ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>(csm_models[ref_cam_index]->m_gm_model.get());
    // It must be non-null for now
    if (ref_ls_model == NULL) 
      vw::vw_throw(vw::ArgumentErr() << "Expecting a linescan model as ref sensor.\n");

    int sensor_id = rig_info.sensor_id;
    if (rig.isRefSensor(sensor_id)) {
      // The transform from a reference sensor to itself is the identity
      ref_to_curr.setIdentity();
      transforms_map[sensor_id].push_back(ref_to_curr.matrix());

    } else if (ls_model != NULL) {

      // Iterate over pose samples, and find the transform from the reference
      // sensor to the current sensor at each sample time.
      int numPos          = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
      double beg_pos_time = ls_model->m_t0Ephem;
      double pos_dt       = ls_model->m_dtEphem;
      int ref_icam = rig_info.ref_cam_index;
      for (int i = 0; i < numPos; i++) {
        double time = beg_pos_time + i * pos_dt;
        
        // Ensure we stay in bounds
        if (time < rig_info.beg_pose_time || time < rig_cam_info[ref_icam].beg_pose_time ||
            time > rig_info.end_pose_time || time > rig_cam_info[ref_icam].end_pose_time) 
          continue;
        
        double pos[3], q[4];
        asp::interpPositions(ls_model, time, pos);
        asp::interpQuaternions(ls_model, time, q);
        Eigen::Affine3d cam2world 
          = asp::calcTransform(pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3]);
        
        double ref_pos[3], ref_q[4];  
        asp::interpPositions(ref_ls_model, time, ref_pos);
        asp::interpQuaternions(ref_ls_model, time, ref_q);  
        Eigen::Affine3d ref_cam2world 
           = asp::calcTransform(ref_pos[0], ref_pos[1], ref_pos[2],
                                ref_q[0], ref_q[1], ref_q[2], ref_q[3]);
        
        ref_to_curr = cam2world.inverse() * ref_cam2world;
        transforms_map[sensor_id].push_back(ref_to_curr.matrix());
      }
      
    } else if (frame_model != NULL) {
    
      // This is the precise acquisition time
      double time = rig_info.beg_pose_time;
      
      // Find current position and orientation
      double x, y, z, qx, qy, qz, qw;
      csm_models[icam]->frame_position(x, y, z);
      csm_models[icam]->frame_quaternion(qx, qy, qz, qw);
      
      Eigen::Affine3d cam2world = asp::calcTransform(x, y, z, qx, qy, qz, qw);
      
      double ref_pos[3], ref_q[4];  
      asp::interpPositions(ref_ls_model, time, ref_pos);
      asp::interpQuaternions(ref_ls_model, time, ref_q);  
      
       Eigen::Affine3d ref_cam2world 
        = asp::calcTransform(ref_pos[0], ref_pos[1], ref_pos[2],
                             ref_q[0], ref_q[1], ref_q[2], ref_q[3]);
        
       ref_to_curr = cam2world.inverse() * ref_cam2world;
       transforms_map[sensor_id].push_back(ref_to_curr.matrix());
    }
  }
    
  // Find the median, for robustness. 
  for (auto it = transforms_map.begin(); it != transforms_map.end(); it++) {

    int sensor_id = it->first;
    auto & transforms = it->second;
    if (transforms.empty()) 
        LOG(FATAL) << "No poses were found for rig sensor with id: " << sensor_id << "\n";

    Eigen::Affine3d median_trans;
    median_trans.matrix() = rig::median_matrix(transforms);
    
    // Normalize the linear component of median_trans
    median_trans.linear() /= pow(median_trans.linear().determinant(), 1.0 / 3.0);

    // Pack the median transform into the output vector    
    rig::rigid_transform_to_array(median_trans,
       &ref_to_curr_sensor_vec[rig::NUM_RIGID_PARAMS * sensor_id]);
  }
  
  return;
}
                           
// Find the relationship between the cameras relative to the rig
void populateRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
                        std::map<int, int> const& orbital_groups,
                        // Outputs
                        std::vector<RigCamInfo> & rig_cam_info,
                        std::vector<double>     & ref_to_curr_sensor_vec) {

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

  // For each camera find the closest camera in time acquired with the reference sensor
  findClosestRefCamera(rig, csm_models, rig_cam_info);
  
  // Find the initial guess rig transforms based on all camera-to-world transforms
  calcRigTransforms(rig, csm_models, rig_cam_info, ref_to_curr_sensor_vec);
}

// Given a linescan camera and the transform from it to the current camera,
// find the current camera to world transform as an array.
void linescanToCurrSensorTrans(const UsgsAstroLsSensorModel & ls_cam,
                               const asp::RigCamInfo & rig_cam_info,
                               double const* ref_to_curr_trans,
                               // Output
                               double * cam2world_arr) {

  Eigen::Affine3d ref_to_curr_trans_aff;
  rig::array_to_rigid_transform(ref_to_curr_trans_aff, ref_to_curr_trans);
  
  // The time at which the pixel is seen
  double frame_time = rig_cam_info.beg_pose_time;

  // The transform from the reference camera to the world. We assume
  // the linescan and frame cameras use the same clock.
  double ref_pos[3], ref_q[4];  
  asp::interpPositions(&ls_cam, frame_time, ref_pos);
  asp::interpQuaternions(&ls_cam, frame_time, ref_q);  
  Eigen::Affine3d ref_cam2world 
      = asp::calcTransform(ref_pos[0], ref_pos[1], ref_pos[2],
                            ref_q[0], ref_q[1], ref_q[2], ref_q[3]);
  // Apply the rig constraint
  Eigen::Affine3d cam2world = ref_cam2world * ref_to_curr_trans_aff.inverse();

  // Convert to an array
  rig::rigid_transform_to_array(cam2world, cam2world_arr);
}

} // end namespace asp
