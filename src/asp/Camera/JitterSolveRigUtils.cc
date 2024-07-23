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
#include <asp/Rig/basic_algs.h>

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

// Find the time bounds for a linescan sensor
void linescanTimeBounds(UsgsAstroLsSensorModel const* ls_model, 
                        // Outputs
                        double & beg_time, double & end_time) {

  int numPos           = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double beg_pos_time  = ls_model->m_t0Ephem;
  double pos_dt        = ls_model->m_dtEphem;
  double end_pos_time  = beg_pos_time + (numPos - 1) * pos_dt;
  int numQuat          = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double beg_quat_time = ls_model->m_t0Quat;
  double quat_dt       = ls_model->m_dtQuat;
  double end_quat_time = beg_quat_time + (numQuat - 1) * quat_dt;
  
  beg_time      = std::max(beg_pos_time, beg_quat_time);
  end_time      = std::min(end_pos_time, end_quat_time);
}

// First pass at collecting info about relationships between cameras and the rig
void populateInitRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
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
    
    // Each camera must know where it belongs, and other info
    rig_info.cam_index = i;
    rig_info.image_file = image_files[i];
    rig_info.camera_file = camera_files[i];
    
    // Get the underlying linescan model or frame model
    asp::CsmModel * csm_cam = csm_models[i];
    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_cam->m_gm_model).get());
    
    int sensor_id = -1;
    std::string group; 
    if (frame_model != NULL) {
      
      double timestamp = -1.0; 
      rig::findCamTypeAndTimestamp(camera_files[i], rig.cam_names,
                                   sensor_id, timestamp, group); // outputs
      rig_info.sensor_type = RIG_FRAME_SENSOR;
      rig_info.sensor_id = sensor_id;
      rig_info.rig_group = group;
      
      // The mid_group_time will be the mid time for the frame group, to be filled later.
      rig_info.beg_pose_time = timestamp;
      rig_info.end_pose_time = timestamp;
      
      // Image files must have the same convention as camera files.
      rig::findCamTypeAndTimestamp(image_files[i], rig.cam_names,
                                   sensor_id, timestamp, group); // outputs
      if (sensor_id != rig_info.sensor_id     || 
          timestamp != rig_info.beg_pose_time ||
          group     != rig_info.rig_group)
        vw::vw_throw(vw::ArgumentErr() 
                     << "Mismatch between image and camera name. "
                     << "Image: " << image_files[i] << ", camera: " << camera_files[i] 
                     << "\n");
      
    } else if (ls_model != NULL) {

      rig::findCamTypeAndGroup(camera_files[i], rig.cam_names,
                               sensor_id, group); // outputs

      // Each pose has a timestamp. The mid group time is a compromise between 
      // the timestamps of the poses in each group.
      int numLines = ls_model->m_nLines;
      csm::ImageCoord imagePt;
      asp::toCsmPixel(vw::Vector2(0, numLines/2.0), imagePt);
      rig_info.sensor_type = RIG_LINESCAN_SENSOR;
      rig_info.sensor_id = sensor_id;
      rig_info.rig_group = group;
      rig_info.mid_group_time = ls_model->getImageTime(imagePt);

      // Find the time bounds for the linescan sensor
      linescanTimeBounds(ls_model, rig_info.beg_pose_time, rig_info.end_pose_time);
      
      // Image files must have the same convention as camera files.
      rig::findCamTypeAndGroup(image_files[i], rig.cam_names, 
                               sensor_id, group); // outputs
      if (sensor_id != rig_info.sensor_id ||
          group     != rig_info.rig_group)
        vw::vw_throw(vw::ArgumentErr() 
                     << "Mismatch between image and camera name. "
                     << "Image: " << image_files[i] << ", camera: " << camera_files[i] 
                     << "\n");
      
    } else {
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
    }
  }
}    

// For each group of frame cameras, find the map from each timestamp to the index
// of the camera in the full array of cameras.
void populateTimestampMap(std::map<int, int> const& cam2group,
                          std::vector<RigCamInfo> const& rig_cam_info,
                          TimestampMap & timestamp_map) {
  
  timestamp_map.clear();
  for (int icam = 0; icam < (int)rig_cam_info.size(); icam++) {
     auto const& info = rig_cam_info[icam]; // alias
     
     if (info.sensor_type != RIG_FRAME_SENSOR)
        continue;
     
     int group = rig::mapVal(cam2group, icam);
     timestamp_map[group][info.beg_pose_time] = icam;
  }
}

// Each frame camera will have a timestamp. Find the median timestamp for each group
// of such cameras.
void populateFrameGroupMidTimestamp(std::vector<asp::CsmModel*>          const& csm_models,
                                    std::map<int, int>                   const& cam2group,
                                    TimestampMap const& timestamp_map,
                                    // Outputs
                                    std::vector<RigCamInfo> & rig_cam_info) {

  int num_cams = csm_models.size();
  for (int icam = 0; icam < num_cams; icam++) {
    
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (frame_model == NULL)
      continue; // Skip non-frame cameras
    
    int group_id = rig::mapVal(cam2group, icam);

    auto g_it = timestamp_map.find(group_id);
    if (g_it == timestamp_map.end())
      vw::vw_throw(vw::ArgumentErr() 
         << "Failed to find group in timestamps.\n");
    
    // Put the keys in a vector
    auto map = g_it->second;
    std::vector<double> timestamps;
    for (auto const& p: map) 
      timestamps.push_back(p.first);

    // Find the median    
    double median = vw::math::destructive_median(timestamps);
    rig_cam_info[icam].mid_group_time = median;
  }
}

// For each camera, find the camera closest in time that was acquired with the
// reference sensor on the same rig as the current camera. The complexity of
// this is total number of cameras times the number of cameras acquired with
// a reference sensor. Something more clever did not work, and this may be
// good enough.

// TODO(oalexan1): Decrease the complexity of this function. Use binary search.
// Also process each rig group separately.
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
      
      // The rig group must be non-empty  
      if (info.rig_group.empty() || ref_info.rig_group.empty())
        vw::vw_throw(vw::ArgumentErr() << "Empty rig group.\n");

      // Both must be in the same rig group
      if (ref_info.rig_group != info.rig_group)
        continue;
          
      int ref_sensor_id2 = rig.refSensorId(ref_info.sensor_id);
      
      // This must be a ref sensor
      if (ref_sensor_id2 != ref_info.sensor_id) 
        vw::vw_throw(vw::ArgumentErr() << "Expecting a ref sensor.\n");
        
      if (ref_sensor_id2 != ref_sensor_id) 
        continue; // Not the same ref sensor
        
      // See if it is closer than the current best
      double dt = std::abs(ref_info.mid_group_time - info.mid_group_time);
      if (dt >= max_time) 
        continue;
        
      max_time = dt;
      rig_cam_info[icam].ref_cam_index = ref_info.cam_index;
      
    } // end iterating over ref sensor cameras
  } // end iterating over cameras
  
}

// Find the times and indices bracketing a given time
bool timestampBrackets(double time, 
                  std::map<double, int> const& timestamps,
                  // Outputs
                  double & time1, double & time2,
                  int & index1, int & index2) {

  // Initialize the outputs to something
  time1 = 0.0; time2 = 0.0;
  index1 = 0; index2 = 0;
  
  double first_time = timestamps.begin()->first;
  double last_time  = timestamps.rbegin()->first;

  // This is a very important check. We cannot extrapolate in time.
  // TODO(oalexan1): Offending cameras better be filtered out before this point.
  if (time < first_time || time > last_time)
    LOG(FATAL) << "Found a time outsie the range of reference camera timestamps.\n";

  // Find the time no earlier than time
  auto it = timestamps.lower_bound(time);
  if (it == timestamps.end()) {
    return false; // out of bounds
  }
  time2 = it->first;
  index2 = it->second;

  // Must have index1 < index2, as later we need two independent poses
  // at end points when we optimize them.
  if (it == timestamps.begin()) {
    if (timestamps.size() == 1)
      LOG(FATAL) << "Too few timestamps.\n";
    
    time1 = it->first;
    index1 = it->second;
    it++;
    time2 = it->first;
    index2 = it->second;  
  } else {
    it--;
    time1 = it->first;
    index1 = it->second;
  }

  // Very important checks
  if (index1 == index2)
    LOG(FATAL) << "Bookkeeping failure in timestamps: equal indices.\n";
  if (time1 == time2)
    LOG(FATAL) << "Bookkeeping failure in timestamps: equal times.\n";
  if (time < time1 || time > time2)
    LOG(FATAL) << "Bookkeeping failure in timestamps: time out of bounds.\n";
    
  return true;
}

// Find the timestamps bracketing the camera with the given rig_cam_info.
// This is a wrapper around the above function.
bool timestampBrackets(asp::RigCamInfo     const & rig_cam_info,
                        std::map<int, int> const & cam2group,
                        TimestampMap       const & timestamp_map,
                        // Outputs
                        double & beg_ref_time, double & end_ref_time, 
                        int & beg_ref_index, int & end_ref_index) {

  // Initialize the outputs
  beg_ref_time = -1.0; end_ref_time = -1.0;
  beg_ref_index = -1;  end_ref_index = -1;
  
  double frame_time = rig_cam_info.beg_pose_time;
  if (frame_time != rig_cam_info.end_pose_time)
   vw::vw_throw(vw::ArgumentErr() 
                << "For a frame sensor beg and end pose time must be same.\n");
   
  int icam = rig_cam_info.cam_index;
  int ref_icam = rig_cam_info.ref_cam_index;
  int ref_group = rig::mapVal(cam2group, ref_icam);
  auto const& ref_timestamps = rig::mapVal(timestamp_map, ref_group);
  bool success = timestampBrackets(frame_time, ref_timestamps, 
                                   // Outputs
                                   beg_ref_time, end_ref_time, 
                                   beg_ref_index, end_ref_index);
  return success;
}

// Given two poses, each as x, y, z, qx, qy, qz, qw, linearly interpolate between them.
void interpPose(double time1, double time2, 
                double const* pose1, double const* pose2, double time, 
                // Output
                double* pose) {

  double alpha = (time - time1)/(time2 - time1);
  if (time1 == time2) 
    alpha = 0.0; // Corner case

  for (int i = 0; i < NUM_XYZ_PARAMS + NUM_QUAT_PARAMS; i++) 
    pose[i] = pose1[i] + alpha*(pose2[i] - pose1[i]);
    
  // Normalize the quaternion
  double * q = pose + NUM_XYZ_PARAMS;
  double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  for (int i = 0; i < NUM_QUAT_PARAMS; i++) 
    q[i] /= norm;
}

// Given reference poses at time1 and time2, a time in between, and the transform
// from the rig from the reference to the current sensor, find the current sensor
// pose (camera-to-world) at the given time.
void interpCurrPose(double time1, double time2, double time, 
                    double const* pose1, double const* pose2, 
                    double const* ref_to_curr_trans,
                    // Output
                    double * cam2world_arr) {

  // Interpolate the reference poses, and convert to a transform
  double r[NUM_XYZ_PARAMS + NUM_QUAT_PARAMS];
  interpPose(time1, time2, pose1, pose2, time, r);
  Eigen::Affine3d ref_cam2world = asp::calcTransform(r[0], r[1], r[2], 
                                                     r[3], r[4], r[5], r[6]);
  
  // Convert the reference to current transform array to a transform
  Eigen::Affine3d ref_to_curr_trans_aff;
  rig::array_to_rigid_transform(ref_to_curr_trans_aff, ref_to_curr_trans);

  // Apply the rig constraint
  Eigen::Affine3d cam2world = ref_cam2world * ref_to_curr_trans_aff.inverse();

  // Convert to an array
  rig::rigid_transform_to_array(cam2world, cam2world_arr);
}    

// Given a map from timestamps to frame camera indices in csm_models, do linear
// interpolation in time. Return false if out of bounds.
// TODO(oalexan1): Must allow some slack at end points, so do a little extrapolation.
bool interpFramePose(std::vector<asp::CsmModel*> const& csm_models,
                     std::map<double, int> const& timestamps,
                     double time, 
                     // Output
                     Eigen::Affine3d & cam2world) {
  
  double time1 = 0.0, time2 = 0.0;
  int index1 = 0, index2 = 0;
  bool success = timestampBrackets(time, timestamps, time1, time2, index1, index2);
  if (!success) 
    return false;

  // Position and orientation at time1
  double a[NUM_XYZ_PARAMS + NUM_QUAT_PARAMS];
  csm_models[index1]->frame_position(a[0], a[1], a[2]);
  csm_models[index1]->frame_quaternion(a[3], a[4], a[5], a[6]);
  
  // Position and orientation at time2
  double b[NUM_XYZ_PARAMS + NUM_QUAT_PARAMS];
  csm_models[index2]->frame_position(b[0], b[1], b[2]);
  csm_models[index2]->frame_quaternion(b[3], b[4], b[5], b[6]);
  
  double c[NUM_XYZ_PARAMS + NUM_QUAT_PARAMS];
  interpPose(time1, time2, a, b, time, c);

  // Convert to a transform      
  cam2world = asp::calcTransform(c[0], c[1], c[2], c[3], c[4], c[5], c[6]);
  
  return true;
}

bool calcInterpRefCamToWorld(std::vector<asp::CsmModel*> const& csm_models,
                             std::vector<RigCamInfo> const& rig_cam_info, // all cam info
                             RigCamInfo const& rig_info, // current cam info
                             UsgsAstroFrameSensorModel * ref_frame_model,
                             UsgsAstroLsSensorModel * ref_ls_model,
                             std::map<int, int> const& cam2group,
                             TimestampMap const& timestamp_map,
                             int ref_icam, double time,
                             // Output
                             Eigen::Affine3d & ref_cam2world) {

  if (ref_frame_model != NULL) {

    // Find the interpolated ref cam at the current time.
    int ref_group = rig::mapVal(cam2group, ref_icam);
    std::map<double, int> const& ref_timestamps 
      = rig::mapVal(timestamp_map, ref_group);
    bool success = interpFramePose(csm_models, ref_timestamps, time, ref_cam2world);
    if (!success) 
      return false;
      
  } else if (ref_ls_model != NULL) {
  
    // Ensure we stay in bounds
    if (time < rig_info.beg_pose_time || time < rig_cam_info[ref_icam].beg_pose_time ||
        time > rig_info.end_pose_time || time > rig_cam_info[ref_icam].end_pose_time) 
      return false;
  
    double ref_pos[3], ref_q[4];  
    asp::interpPositions(ref_ls_model, time, ref_pos);
    asp::interpQuaternions(ref_ls_model, time, ref_q);  
    ref_cam2world = asp::calcTransform(ref_pos[0], ref_pos[1], ref_pos[2],
                                       ref_q[0], ref_q[1], ref_q[2], ref_q[3]);
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
  }

  return true;
}

// Given all camera-to-world transforms, find the median rig transforms.
// This is robust to outliers. Must handle both frame and linescan cameras
// for both the ref and curr sensor, which is 4 cases.
void calcRigTransforms(rig::RigSet const& rig,
                       std::vector<asp::CsmModel*> const& csm_models,
                       std::vector<RigCamInfo> const& rig_cam_info,
                       std::map<int, int> const& cam2group,
                       TimestampMap const& timestamp_map,
                       bool use_initial_rig_transforms,
                       // Outputs
                       std::vector<double> & ref_to_curr_sensor_vec) {
  
  int num_rig_sensors = rig.cam_names.size();
  ref_to_curr_sensor_vec.resize(rig::NUM_RIGID_PARAMS * num_rig_sensors, 0.0);

  if (use_initial_rig_transforms) {
     // Pack the initial rig transforms into the output vector 
     for (int sensor_id = 0; sensor_id < num_rig_sensors; sensor_id++) 
       rig::rigid_transform_to_array(rig.ref_to_cam_trans[sensor_id],
         &ref_to_curr_sensor_vec[rig::NUM_RIGID_PARAMS * sensor_id]);
    return;
  }

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
    UsgsAstroFrameSensorModel * ref_frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>(csm_models[ref_cam_index]->m_gm_model.get());

    int sensor_id = rig_info.sensor_id;
    if (rig.isRefSensor(sensor_id)) {
      // The transform from a reference sensor to itself is the identity
      ref_to_curr.setIdentity();
      transforms_map[sensor_id].push_back(ref_to_curr.matrix());
      continue;
    }

    int ref_icam = rig_info.ref_cam_index;
    
    // Assume the ref sensor is a linescan sensor
    if (ls_model != NULL) {

      // Iterate over pose samples, and find the transform from the reference
      // sensor to the current sensor at each sample time.
      int numPos          = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
      double beg_pos_time = ls_model->m_t0Ephem;
      double pos_dt       = ls_model->m_dtEphem;
      for (int i = 0; i < numPos; i++) {
        double time = beg_pos_time + i * pos_dt;

        // Current camera position and orientation        
        double pos[3], q[4];
        asp::interpPositions(ls_model, time, pos);
        asp::interpQuaternions(ls_model, time, q);
        Eigen::Affine3d cam2world 
          = asp::calcTransform(pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3]);
        
        // Ref camera position and orientation
        Eigen::Affine3d ref_cam2world;
        bool success = calcInterpRefCamToWorld(csm_models, rig_cam_info, rig_info,
                                               ref_frame_model, ref_ls_model,
                                               cam2group, timestamp_map, ref_icam, time,
                                               // Output
                                               ref_cam2world);
        if (!success) 
          continue;
        
        ref_to_curr = cam2world.inverse() * ref_cam2world;
        transforms_map[sensor_id].push_back(ref_to_curr.matrix());
      }
      
    } else if (frame_model != NULL) {
    
      // This is the precise acquisition time
      double time = rig_info.beg_pose_time;
      
      // Find current frame cam position and orientation
      double x, y, z, qx, qy, qz, qw;
      csm_models[icam]->frame_position(x, y, z);
      csm_models[icam]->frame_quaternion(qx, qy, qz, qw);
      Eigen::Affine3d cam2world = asp::calcTransform(x, y, z, qx, qy, qz, qw);
      
      // Find reference linescan cam position and orientation
      Eigen::Affine3d ref_cam2world;
      bool success = calcInterpRefCamToWorld(csm_models, rig_cam_info, rig_info,
                                             ref_frame_model, ref_ls_model,
                                             cam2group, timestamp_map, ref_icam, time,
                                             // Output
                                             ref_cam2world);
      if (!success) 
        continue;
        
      ref_to_curr = cam2world.inverse() * ref_cam2world;
      transforms_map[sensor_id].push_back(ref_to_curr.matrix());
    }
    
  } // End loop through cameras
  
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

// Update the rig with the optimized transforms
void updateRig(std::vector<double> const& ref_to_curr_sensor_vec,
               rig::RigSet & rig) {

  int num_rig_sensors = rig.cam_names.size();
  for (int sensor_id = 0; sensor_id < num_rig_sensors; sensor_id++) {
      rig::array_to_rigid_transform
        (rig.ref_to_cam_trans[sensor_id],
         &ref_to_curr_sensor_vec[rig::NUM_RIGID_PARAMS * sensor_id]);
  }
}

// Find the relationship between the cameras relative to the rig
void populateRigCamInfo(rig::RigSet const& rig,
                        std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::vector<asp::CsmModel*> const& csm_models,
                        std::map<int, int> const& cam2group,
                        bool use_initial_rig_transforms,
                        // Outputs
                        std::vector<RigCamInfo> & rig_cam_info,
                        std::vector<double>     & ref_to_curr_sensor_vec,
                        TimestampMap & timestamp_map) {

  // Print a message, as this can take time
  vw::vw_out() << "Determining the rig relationships between the cameras.\n";
   
  // Initialize the rig cam info after a first pass through the cameras
  populateInitRigCamInfo(rig, image_files, camera_files, csm_models, 
                         rig_cam_info);

  // Find a map that from each group to timestamps and indicies in that group
  populateTimestampMap(cam2group, rig_cam_info, timestamp_map);
  
  // Find the mid time for each frame group as the median of the group timestamps  
  populateFrameGroupMidTimestamp(csm_models, cam2group, timestamp_map,
                                 rig_cam_info);

  // For each camera find the closest camera within the rig and group
  // that is acquired with the reference sensor
  findClosestRefCamera(rig, csm_models, rig_cam_info);
  
  // Find the initial guess rig transforms based on all camera-to-world transforms
  calcRigTransforms(rig, csm_models, rig_cam_info, cam2group, timestamp_map,
                    use_initial_rig_transforms, ref_to_curr_sensor_vec);
}

// Given a reference linescan camera and the transform from it to the current
// camera, find the current camera to world transform as an array.
void linescanToCurrSensorTrans(UsgsAstroLsSensorModel const& ref_ls_cam,
                               double curr_time,
                               double const* ref_to_curr_trans,
                               // Output
                               double * cam2world_arr) {

  Eigen::Affine3d ref_to_curr_trans_aff;
  rig::array_to_rigid_transform(ref_to_curr_trans_aff, ref_to_curr_trans);
  
  // The transform from the reference camera to the world. We assume
  // the linescan and frame cameras use the same clock.
  double ref_pos[3], ref_q[4];  
  asp::interpPositions(&ref_ls_cam, curr_time, ref_pos);
  asp::interpQuaternions(&ref_ls_cam, curr_time, ref_q);  
  Eigen::Affine3d ref_cam2world 
      = asp::calcTransform(ref_pos[0], ref_pos[1], ref_pos[2],
                            ref_q[0], ref_q[1], ref_q[2], ref_q[3]);
  // Apply the rig constraint
  Eigen::Affine3d cam2world = ref_cam2world * ref_to_curr_trans_aff.inverse();

  // Convert to an array
  rig::rigid_transform_to_array(cam2world, cam2world_arr);
}

// Given a frame linescan camera and the transform from it to the current
// camera, find the current camera to world transform as an array.
void frameToCurrSensorTrans(std::vector<double>       const& frame_params,
                            asp::RigCamInfo           const& rig_cam_info,
                            std::map<int, int>        const& cam2group,
                            TimestampMap              const& timestamp_map,
                            double                    const* ref_to_curr_trans,
                            // Output
                            double                         * cam2world_arr) {

  double beg_ref_time = 0.0, end_ref_time = 0.0;
  int beg_ref_index = 0, end_ref_index = 0;
  bool success = timestampBrackets(rig_cam_info, cam2group, timestamp_map, 
                                  // Outputs
                                  beg_ref_time, end_ref_time, 
                                  beg_ref_index, end_ref_index);
  if (!success) 
    vw::vw_throw(vw::ArgumentErr() << "Failed to find the bracketing times.\n");

  // The pointers to the bracketing frame camera parameters
  double const* beg_frame_arr = &frame_params[beg_ref_index * rig::NUM_RIGID_PARAMS];
  double const* end_frame_arr = &frame_params[end_ref_index * rig::NUM_RIGID_PARAMS];

  // Find the interpolated current cam2world transform
  double frame_time = rig_cam_info.beg_pose_time;
  asp::interpCurrPose(beg_ref_time, end_ref_time, frame_time, 
                      beg_frame_arr, end_frame_arr, ref_to_curr_trans,
                      cam2world_arr);
}

// Given a reference linescan camera and the transform from it to the current
// linescan camera, update the the current camera poses within the given range.
void updateLinescanWithRig(UsgsAstroLsSensorModel const& ref_ls_cam,
                           double const* ref_to_curr_trans,
                           UsgsAstroLsSensorModel & curr_ls_cam, // update this
                           // Range of quat and position indices to update.
                           // The default is to update all.
                           int beg_quat_index, int end_quat_index,
                           int beg_pos_index, int end_pos_index) {

  // See if have to update the whole range of quats and positions
  if (beg_quat_index < 0 || end_quat_index < 0) {
    beg_quat_index = 0;
    end_quat_index = curr_ls_cam.m_quaternions.size() / NUM_QUAT_PARAMS;
  }
  if (beg_pos_index < 0 || end_pos_index < 0) {
    beg_pos_index = 0;
    end_pos_index = curr_ls_cam.m_positions.size() / NUM_XYZ_PARAMS;
  }
  
  std::vector<double> cam2world_vec(rig::NUM_RIGID_PARAMS);
  double currQuatT0 = curr_ls_cam.m_t0Quat;
  double currQuatDt = curr_ls_cam.m_dtQuat;
  double currPosT0  = curr_ls_cam.m_t0Ephem;
  double currPosDt  = curr_ls_cam.m_dtEphem;
  
  // Update the quaternions
  for (int qi = beg_quat_index; qi < end_quat_index; qi++) {
    double t = currQuatT0 + qi * currQuatDt;
    asp::linescanToCurrSensorTrans(ref_ls_cam, t, ref_to_curr_trans,
                                   &cam2world_vec[0]); // output
    for (int coord = 0; coord < NUM_QUAT_PARAMS; coord++)
      curr_ls_cam.m_quaternions[qi * NUM_QUAT_PARAMS + coord] 
        = cam2world_vec[coord + NUM_XYZ_PARAMS];
  }

  // Update the positions
  for (int pi = beg_pos_index; pi < end_pos_index; pi++) {
    double t = currPosT0 + pi * currPosDt;
    asp::linescanToCurrSensorTrans(ref_ls_cam, t,
                                  ref_to_curr_trans,
                                  &cam2world_vec[0]); // output
    for (int coord = 0; coord < NUM_XYZ_PARAMS; coord++)
      curr_ls_cam.m_positions[pi * NUM_XYZ_PARAMS + coord] 
      = cam2world_vec[coord];
  }  
}

} // end namespace asp
