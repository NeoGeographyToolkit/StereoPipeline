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

/// \file SatSim.cc
/// Top-level orchestration and rig coordination for sat_sim.

#include <asp/Camera/SatSim.h>
#include <asp/Camera/SatSimTrajectory.h>
#include <asp/Camera/SatSimSensor.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Rig/RigConfig.h>

#include <vw/Core/Log.h>

using namespace vw;

namespace asp {

std::string camPrefix(SatSimOptions const& opt, int iFrame, double timestamp, bool isRef,
                      bool isFrame, std::string const& suffix) {

  std::string ref = ""; 
  if (isRef) 
    ref = "-ref";
  
  // Prepare the timestamp string. If modeling time, will do sprintf with 7
  // digits before dot and 9 after (with the dot, there will be 17 characters in
  // total). This is to ensure that the time is unique. Use fixed precision. Use
  // leading zeros to ensure that the string is always the same length and will
  // be sorted correctly.
  char time[256];
  if (!isFrame) {
    time[0] = '\0'; // empty string
  } else {
    if (!opt.model_time) 
      snprintf(time, sizeof(time), "-%d", 10000 + iFrame); // 5 characters, starts with 1
    else
      snprintf(time, sizeof(time), "-%017.9f", timestamp); // 17 characters, 9 after dot
  }
  
  return opt.out_prefix + time + suffix + ref;
}

// A function to read Pinhole cameras from disk

void applyRigTransform(Eigen::Affine3d const & ref_to_sensor,
                       vw::Vector3 & ctr, vw::Matrix3x3 & cam2world) {

  // Create 4x4 transform matrix
  Eigen::Matrix<double, 4, 4> cam2world4x4 = Eigen::Matrix<double, 4, 4>::Identity();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      cam2world4x4(r, c) = cam2world(r, c);
    }
  }
  for (int r = 0; r < 3; r++) 
    cam2world4x4(r, 3) = ctr[r];

  // Apply the rig transform 
  cam2world4x4 = cam2world4x4 * ref_to_sensor.matrix().inverse(); 
  
  // Extract the rotation and translation
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      cam2world(r, c) = cam2world4x4(r, c);
    }
    ctr[r] = cam2world4x4(r, 3);
  }
  
}

// A function to create and save Pinhole cameras. Assume no distortion, and pixel
// pitch = 1.

void genCamerasImages(float ortho_nodata_val,
            bool have_rig,
            int rig_sensor_index,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::GeoReference const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            SatSimOptions                      & opt,
            rig::RigSet                        & rig,
            vw::GeoReference const& dem_georef,
            std::string                   const& suffix) {

  std::vector<vw::Vector3> positions;
  std::vector<vw::Matrix3x3> cam2world, cam2world_no_jitter, ref_cam2world;
  std::vector<double> cam_times;
  int first_pos = 0; // used with linescan poses, which start before first image line
  double orbit_len = 0.0, first_line_time = 0.0; // will change
  
  // Compute the camera poses. Some logic here is different for linescan and frame
  // cameras, that's why this logic is called individually for each rig sensor,
  // even though if all sensors are of the same time it would be enough to call
  // it once.
  asp::genCamPoses(opt, dem_georef, dem, height_guess, have_rig,
                   // Outputs
                   first_pos, first_line_time, orbit_len, positions, cam2world, 
                   cam2world_no_jitter, ref_cam2world, cam_times, rig);

  // In genCamPoses() the rig may have been updated. Fetch the latest.
  Eigen::Affine3d ref2sensor = Eigen::Affine3d::Identity();
  if (have_rig)
    ref2sensor = rig.ref_to_cam_trans[rig_sensor_index];

  // Sequence of camera names and cameras for one sensor
  std::vector<std::string> cam_names; 
  std::vector<vw::CamPtr> cams;
  // The suffix is needed to distinguish the cameras and images for each rig sensor
  if (opt.sensor_type == "pinhole")
    asp::genPinholeCameras(opt, dem_georef, positions, cam2world, ref_cam2world,
                           cam_times, have_rig, ref2sensor, suffix, cam_names, cams);
  else
    asp::genLinescanCameras(first_line_time, orbit_len, dem_georef, dem, first_pos, 
                            positions, cam2world, cam2world_no_jitter, ref_cam2world, 
                            cam_times, height_guess, have_rig, ref2sensor, suffix, 
                            opt, cam_names, cams);

  bool external_cameras = false;
  if (!opt.no_images)
    asp::genImages(opt, external_cameras, cam_names, cams, suffix, dem_georef, dem, 
        height_guess, ortho_georef, ortho, ortho_nodata_val);

}

// Generate the cameras and images for a rig
void genRigCamerasImages(SatSimOptions          & opt,
            rig::RigSet                         & rig,
            vw::GeoReference  const& dem_georef,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::GeoReference  const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            float ortho_nodata_val) {

  // Sanity checks
  if (opt.rig_config == "")            
    vw::vw_throw(vw::ArgumentErr() << "The rig configuration file must be set.\n");
  
  // Handle sensor types
  std::vector<std::string> sensor_types;
  handleSensorType(rig.cam_names.size(), opt.sensor_type, sensor_types);
  
  std::vector<std::string> sensor_names;
  if (opt.sensor_name == "all")
    sensor_names = rig.cam_names; 
  else
    boost::split(sensor_names, opt.sensor_name, boost::is_any_of(","));
  
  // Map from each sensor in rig.cam_names to index
  std::map<std::string, int> sensor_name2index;
  for (int i = 0; i < int(rig.cam_names.size()); i++)
    sensor_name2index[rig.cam_names[i]] = i;
  
  // Iterate over sensor_names. Check if it is sensor_name2index
  for (size_t sensor_it = 0; sensor_it < sensor_names.size(); sensor_it++) {
    auto it = sensor_name2index.find(sensor_names[sensor_it]);
    if (it == sensor_name2index.end())
      vw::vw_throw(vw::ArgumentErr() << "Sensor: " << sensor_names[sensor_it] 
        << " not found in the rig.\n");
    
    // Pass the intrinsics to a local copy of the options
    SatSimOptions local_opt     = opt;
    int sensor_index            = it->second;
    auto params                 = rig.cam_params[sensor_index];
    local_opt.focal_length      = params.GetFocalLength();
    local_opt.optical_center[0] = params.GetOpticalOffset()[0];
    local_opt.optical_center[1] = params.GetOpticalOffset()[1];
    local_opt.image_size[0]     = params.GetDistortedSize()[0];
    local_opt.image_size[1]     = params.GetDistortedSize()[1];
    local_opt.sensor_type       = sensor_types[sensor_index];
    
    // The transform from the reference sensor to the current sensor
    Eigen::Affine3d ref2sensor = rig.ref_to_cam_trans[sensor_index];
    std::string suffix = "-" + rig.cam_names[sensor_index];
    bool have_rig = true;
    genCamerasImages(ortho_nodata_val, have_rig, sensor_index, dem, height_guess, 
                     ortho_georef, ortho, local_opt, rig, dem_georef, suffix); 
    
  }              
}

// Modify the rig that was used to produce the images to make the sensor transforms
// relative to the first sensor.or. 
void writeRelRig(std::string const& out_prefix, rig::RigSet const& rig) {
  
  rig::RigSet ref_rig = rig;
  for (size_t i = 0; i < rig.ref_to_cam_trans.size(); i++)
    ref_rig.ref_to_cam_trans[i] 
        = rig.ref_to_cam_trans[i] * rig.ref_to_cam_trans[0].inverse();
  
  // Ensure the first transform is identity. Now it may not be precisely it
  // because of numerical errors.
  ref_rig.ref_to_cam_trans[0] = Eigen::Affine3d::Identity();
  
  std::string ref_rig_config = out_prefix + "-rig_config.txt";
  bool have_rig = true;
  rig::writeRigConfig(ref_rig_config, have_rig, ref_rig);
  
}

} // end namespace asp
