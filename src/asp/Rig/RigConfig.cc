/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <asp/Rig/RigConfig.h>
#include <asp/Rig/TransformUtils.h>
#include <asp/Rig/SystemUtils.h>

#include <vw/Core/Log.h>
#include <vw/Core/Exception.h>

#include <opencv2/calib3d.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <set>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

namespace rig {

void RigSet::validate() const {
  if (cam_set.empty()) 
    vw::vw_throw(vw::IOErr() << "Found an empty set of rigs.\n");
  
  size_t num_cams = 0;
  std::set<std::string> all_cams; // checks for duplicates
  for (size_t rig_it = 0; rig_it < cam_set.size(); rig_it++) {
    if (cam_set[rig_it].empty()) 
      vw::vw_throw(vw::IOErr() << "Found a rig with no cams.\n");
    
    num_cams += cam_set[rig_it].size();
    for (size_t cam_it = 0; cam_it < cam_set[rig_it].size(); cam_it++)
      all_cams.insert(cam_set[rig_it][cam_it]);
  }
  
  if (num_cams != all_cams.size() || num_cams != cam_names.size())
    vw::vw_throw(vw::IOErr()
                 << "Found a duplicate sensor name in the rig set.\n");
  
  if (num_cams != ref_to_cam_trans.size()) 
    vw::vw_throw(vw::IOErr() 
                 << "Number of sensors is not equal to number of ref-to-cam transforms.\n");
  
  if (num_cams != depth_to_image.size()) 
    vw::vw_throw(vw::IOErr()
                  << "Number of sensors is not equal to number of depth-to-image "
                  << "transforms.\n");
  
  if (num_cams != ref_to_cam_timestamp_offsets.size()) 
    vw::vw_throw(vw::IOErr() 
                 << "Number of sensors is not equal to number of ref-to-cam "
                 << "timestamp offsets.\n");
  
  if (num_cams != cam_params.size()) 
    vw::vw_throw(vw::IOErr() 
                 << "Number of sensors is not equal to number of camera models.\n");
  
  for (size_t cam_it = 0; cam_it < cam_names.size(); cam_it++) {
    if (isRefSensor(cam_names[cam_it]) && ref_to_cam_timestamp_offsets[cam_it] != 0) 
      vw::vw_throw(vw::IOErr() << "The timestamp offsets for the reference sensors must be always 0.\n");
  }
  
  for (size_t i = 0; i < this->cam_params.size(); i++) {
    auto const& params = this->cam_params[i];
    if (params.GetDistortedSize()[0] <= 1 || params.GetDistortedSize()[1] <= 1)
        vw::vw_throw(vw::IOErr() << "The image size must be at least 2 x 2.\n");
    if (params.GetFocalLength() == 0)
       vw::vw_throw(vw::IOErr() << "The focal length must be non-zero.\n");
  }

  for (size_t cam_it = 0; cam_it < cam_names.size(); cam_it++) {
    if (isRefSensor(cam_names[cam_it]) &&
        ref_to_cam_trans[cam_it].matrix() != Eigen::Affine3d::Identity().matrix())
      vw::vw_throw(vw::IOErr() 
        << "The transform from the reference sensor to itself must be the identity.\n");
  }

}
  
// We assume the first encountered sensor is the ref sensor. This is enforced
// on reading the rig.
bool RigSet::isRefSensor(std::string const& cam_name) const {
  for (size_t rig_it = 0; rig_it < cam_set.size(); rig_it++) 
    if (cam_set[rig_it][0] == cam_name) 
      return true;
  return false;
}

// A ref sensor is the first sensor on each rig
bool RigSet::isRefSensor(int cam_id) const {
  return isRefSensor(cam_names[cam_id]);
}

// Return the id of the rig given the index of the sensor
// in cam_names.
int RigSet::rigId(int cam_id) const {
  if (cam_id < 0 || cam_id >= cam_names.size()) 
    vw::vw_throw(vw::IOErr() << "Out of bounds sensor id.\n");
  
  std::string cam_name = cam_names[cam_id];
  
  for (size_t rig_it = 0; rig_it < cam_set.size(); rig_it++) {
    for (size_t cam_it = 0; cam_it < cam_set[rig_it].size(); cam_it++) {
      if (cam_set[rig_it][cam_it] == cam_name) {
        return rig_it;
      }
    }
  }

  // Should not arrive here
  vw::vw_throw(vw::IOErr() 
               << "Could not look up in the rig the sensor: " << cam_name << "\n");
  return -1;
}

// The name of the ref sensor for the rig having the given sensor id. We assume
// the first encountered sensor is the ref sensor. This is enforced on reading
// the rig.
std::string RigSet::refSensor(int cam_id) const {
  return cam_set[rigId(cam_id)][0];
}
  
// Index in the list of sensors of the sensor with given name
int RigSet::sensorIndex(std::string const& sensor_name) const {
  auto it = std::find(cam_names.begin(), cam_names.end(), sensor_name);
  if (it == cam_names.end()) 
    vw::vw_throw(vw::IOErr()
                 << "Could not find sensor in rig. That is unexpected. Offending sensor: "
                 << sensor_name << ".\n");
    
  return it - cam_names.begin();
}

// Given the id of a sensor, find the id of the ref sensor for the rig
// having this sensor
int RigSet::refSensorId(int cam_id) const {
  return sensorIndex(refSensor(cam_id));
}
  
// Create a rig set having a single rig  
RigSet RigSet::subRig(int rig_id) const {

  if (rig_id < 0 || rig_id >= cam_set.size()) 
    vw::vw_throw(vw::IOErr() << "Out of range in rig set.\n");

  RigSet sub_rig;
  sub_rig.cam_set.push_back(cam_set[rig_id]);

  // Add the relevant portion of each rig member
  for (size_t subrig_it = 0; subrig_it < cam_set[rig_id].size(); subrig_it++) {
    
    std::string sensor_name = cam_set[rig_id][subrig_it];
    int rig_index = sensorIndex(sensor_name);

    sub_rig.cam_names.push_back(cam_names[rig_index]);
    sub_rig.ref_to_cam_trans.push_back(ref_to_cam_trans[rig_index]);
    sub_rig.depth_to_image.push_back(depth_to_image[rig_index]);
    sub_rig.ref_to_cam_timestamp_offsets.push_back(ref_to_cam_timestamp_offsets[rig_index]);
    sub_rig.cam_params.push_back(cam_params[rig_index]);
  }
  sub_rig.validate();

  return sub_rig;
}
  
// Save the optimized rig configuration
void writeRigConfig(std::string const& rig_config, bool model_rig, RigSet const& R) {

  R.validate();
  
  // Ensure that the output directory exists
  std::string out_dir = fs::path(rig_config).parent_path().string();
  rig::createDir(out_dir);
  
  vw::vw_out() << "Writing: " << rig_config << "\n";
  std::ofstream f;
  f.open(rig_config.c_str(), std::ios::binary | std::ios::out);
  if (!f.is_open()) 
    vw::vw_throw(vw::IOErr() << "Cannot open file for writing: " << rig_config << "\n");
  f.precision(17);

  for (size_t cam_type = 0; cam_type < R.cam_params.size(); cam_type++) {

    if (R.isRefSensor(R.cam_names[cam_type])) {
      if (cam_type > 0)
        f << "\n"; // add an empty line for clarity
      
      f << "ref_sensor_name: " << R.cam_names[cam_type] << "\n";
    }
    
    f << "\n";
    f << "sensor_name: "  << R.cam_names[cam_type] << "\n";
    f << "focal_length: " << R.cam_params[cam_type].GetFocalLength() << "\n";

    Eigen::Vector2d c = R.cam_params[cam_type].GetOpticalOffset();
    f << "optical_center: " << c[0] << " " << c[1] << "\n";

    Eigen::VectorXd D = R.cam_params[cam_type].GetDistortion();

    f << "distortion_coeffs: ";
    for (int d = 0; d < D.size(); d++) {
      f << D[d];
      if (d + 1 < D.size()) f << " ";
    }
    f << "\n";

    auto dist_type = R.cam_params[cam_type].m_distortion_type;
    if (D.size() == 0 && dist_type == rig::NO_DISTORTION)
      f << "distortion_type: " << rig::NO_DISTORTION_STR << "\n";
    else if (D.size() == 1 && dist_type == rig::FOV_DISTORTION)
      f << "distortion_type: " << rig::FOV_DISTORTION_STR << "\n";
    // Both fisheye and radtan distortion can have 4 coefficients  
    else if (D.size() == 4 && dist_type == rig::FISHEYE_DISTORTION)
      f << "distortion_type: " << rig::FISHEYE_DISTORTION_STR << "\n";
    else if (D.size() >= 4 && D.size() <= 5 && dist_type == rig::RADTAN_DISTORTION)
      f << "distortion_type: " << rig::RADTAN_DISTORTION_STR << "\n";
    else if (D.size() > 5 && dist_type == rig::RPC_DISTORTION)
      f << "distortion_type: " << rig::RPC_DISTORTION_STR << "\n";
    else
      vw::vw_throw(vw::IOErr() 
                   << "Expecting 0, 1, 4, 5, or more distortion coefficients. Got: "
                   << D.size() << ".\n");

    Eigen::Vector2i image_size = R.cam_params[cam_type].GetDistortedSize();
    f << "image_size: " << image_size[0] << ' ' << image_size[1] << "\n";

    Eigen::Vector2i distorted_crop_size = R.cam_params[cam_type].GetDistortedCropSize();
    f << "distorted_crop_size: " << distorted_crop_size[0] << ' ' << distorted_crop_size[1] << "\n";

    Eigen::Vector2i undist_size = R.cam_params[cam_type].GetUndistortedSize();
    f << "undistorted_image_size: " << undist_size[0] << ' ' << undist_size[1] << "\n";

    Eigen::Affine3d T;
    if (model_rig)
      T = R.ref_to_cam_trans[cam_type];
    else
      T = Eigen::Affine3d::Identity(); // write something valid

    f << "ref_to_sensor_transform: " << rig::affineToStr(T) << "\n";

    f << "depth_to_image_transform: " << rig::affineToStr(R.depth_to_image[cam_type]) << "\n";

    f << "ref_to_sensor_timestamp_offset: " << R.ref_to_cam_timestamp_offsets[cam_type] << "\n";
  }

  f.close();
}

// Read real values after given tag. Ignore comments, so any line starting
// with #, and empty lines. If desired_num_vals >=0, validate that we
// read the desired number.
void readConfigVals(std::ifstream & f, std::string const& tag, int desired_num_vals,
                    Eigen::VectorXd & vals) {

  // Clear the output
  vals.resize(0);

  std::vector<double> local_vals;  // std::vector has push_back()
  std::string line;
  while (getline(f, line)) {

    // Remove everything after any point sign
    bool have_comment = (line.find('#') != line.npos);
    if (have_comment) {
      std::string new_line;
      for (size_t c = 0; c < line.size(); c++) {
        if (line[c] == '#') 
          break; // got to the pound sign
        
        new_line += line[c];
      }

      line = new_line;
    }
    
    // Here must remove anything after the pound sign
    
    if (line.empty() || line[0] == '#') continue;
    
    // Remove commas that occasionally appear in the file
    for (size_t c = 0; c < line.size(); c++) {
      if (line[c] == ',') 
        line[c] = ' ';
    }

    std::istringstream iss(line);
    std::string token;
    iss >> token;
    std::string val; 
    while (iss >> val)
      local_vals.push_back(atof(val.c_str()));

    if (token == "") 
      continue; // likely just whitespace is present on the line
    
    if (token != tag) 
      vw::vw_throw(vw::IOErr() << "Could not read value for: " << tag << "\n");

    // Copy to Eigen::VectorXd
    vals.resize(local_vals.size());
    for (int it = 0; it < vals.size(); it++) vals[it] = local_vals[it];

    if (desired_num_vals >= 0 && vals.size() != desired_num_vals)
      vw::vw_throw(vw::IOErr() << "Read an incorrect number of values for: " << tag << "\n");

    return;
  }

  vw::vw_throw(vw::IOErr() << "Could not read value for: " << tag << "\n");
}
  
// Read strings separated by spaces after given tag. Ignore comments, so any line starting
// with #, and empty lines. If desired_num_vals >=0, validate that we
// read the desired number.
void readConfigVals(std::ifstream & f, std::string const& tag, int desired_num_vals,
                    std::vector<std::string> & vals) {

  // Clear the output
  vals.resize(0);

  std::string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    std::string token;
    iss >> token;
    std::string val;
    while (iss >> val) {
      vals.push_back(val);
    }

    if (token != tag) 
      vw::vw_throw(vw::IOErr() << "Could not read value for: " << tag << "\n");

    if (desired_num_vals >= 0 && vals.size() != desired_num_vals)
      vw::vw_throw(vw::IOErr() << "Read an incorrect number of values for: " << tag << "\n");

    return;
  }

  vw::vw_throw(vw::IOErr() << "Could not read value for: " << tag << "\n");
}

// Read a rig configuration. Check if the transforms among the sensors
// on the rig is not 0, in that case will use it.
void readRigConfig(std::string const& rig_config, bool have_rig_transforms, RigSet & R) {
  
  // To check for duplicate sensors
  std::set<std::string> ref_sensors, sensors;
  
  try {
    // Initialize the outputs
    R = RigSet();

    // Open the file
    vw::vw_out() << "Reading: " << rig_config << "\n";
    
    std::ifstream f;
    f.open(rig_config.c_str(), std::ios::in);
    if (!f.is_open()) 
      vw::vw_throw(vw::IOErr() << "Cannot open file: " << rig_config << "\n");

    int ref_sensor_count = 0;
    Eigen::VectorXd vals;
    std::vector<std::string> str_vals;
    
    // It is assumed in several places that the first encountered
    // sensor is the ref sensor.
    
    // Read each sensor
    bool is_ref_sensor = false; 
    std::string ref_sensor_name;
    while (1) {
      int curr_pos = f.tellg(); // where we are in the file
      // Read the reference sensor
      try {
        readConfigVals(f, "ref_sensor_name:", 1, str_vals);
        ref_sensor_name = str_vals[0];
        ref_sensor_count++; // found a ref sensor
        R.cam_set.resize(ref_sensor_count);
        is_ref_sensor = true;
        
       // Check for duplicate ref sensor
        if (ref_sensors.find(ref_sensor_name) != ref_sensors.end())
          vw::vw_throw(vw::IOErr() << "Found a duplicate reference sensor: " << ref_sensor_name << "\n");
        ref_sensors.insert(ref_sensor_name);
      } catch (...) {
        // No luck, go back to the line we tried to read, and continue reading other fields
        f.seekg(curr_pos, std::ios::beg);
      }
      
      try {
        readConfigVals(f, "sensor_name:", 1, str_vals);
      } catch (...) {
        // Likely no more sensors
        break;
      }
      
      std::string sensor_name = str_vals[0];
      
      // The first sensor must be the ref sensor
      if (is_ref_sensor && sensor_name != ref_sensor_name) 
        vw::vw_throw(vw::IOErr() 
                     << "The first sensor in each rig must be the reference sensor. "
                     << "Check the rig configuration file.\n");
        
      is_ref_sensor = false; // reset  
      
      if (ref_sensor_name == "")
        vw::vw_throw(vw::IOErr() 
                     << "The reference sensor name must be the first entry in "
                     << "the rig configuration file.\n");

      // Check for duplicate sensor
      if (sensors.find(sensor_name) != sensors.end())
        vw::vw_throw(vw::IOErr() << "Found a duplicate sensor: " << sensor_name << "\n");
      sensors.insert(sensor_name);

      // It is convenient to store each sensor in cam_set, which has the rig set structure,
      // and in R.cam_names, which is enough for almost all processing.
      R.cam_set.back().push_back(sensor_name);
      R.cam_names.push_back(sensor_name);
      
      readConfigVals(f, "focal_length:", 1, vals);
      Eigen::Vector2d focal_length(vals[0], vals[0]);

      readConfigVals(f, "optical_center:", 2, vals);
      Eigen::Vector2d optical_center(vals[0], vals[1]);

      // Read distortion
      
      rig::DistortionType dist_type = rig::NO_DISTORTION;
      
      readConfigVals(f, "distortion_coeffs:", -1, vals);
      if (vals.size() != 0 && vals.size() != 1 && vals.size() != 4 && vals.size() < 5)
        vw::vw_throw(vw::IOErr()
                     << "Expecting 0, 1, 4, 5, or more distortion coefficients.\n");
      Eigen::VectorXd distortion = vals;

      readConfigVals(f, "distortion_type:", 1, str_vals);
      if (distortion.size() == 0 && str_vals[0] != rig::NO_DISTORTION_STR)
        vw::vw_throw(vw::IOErr() 
                     << "When there are no distortion coefficients, distortion type must be: "
                     << rig::NO_DISTORTION_STR << "\n");
      
      // For backward compatibility, accept rig::FISHEYE_DISTORTION_STR with 1 distortion coefficient, but use the FOV model
      if (distortion.size() == 1 && str_vals[0] == rig::FISHEYE_DISTORTION_STR)
        str_vals[0] = rig::FOV_DISTORTION_STR;
      
      // Validation 
      if (distortion.size() == 1 && str_vals[0] != rig::FOV_DISTORTION_STR)
          vw::vw_throw(vw::IOErr() 
                       << "When there is 1 distortion coefficient, distortion type must be: "
                       << rig::FOV_DISTORTION_STR << "\n");
      if (distortion.size() == 4 &&
          str_vals[0] != rig::FISHEYE_DISTORTION_STR &&
          str_vals[0] != rig::RADTAN_DISTORTION_STR)
        vw::vw_throw(vw::IOErr() 
                     << "When there are 4 distortion coefficients, distortion type "
                     << "must be: " << rig::FISHEYE_DISTORTION_STR << " or "
                     << rig::RADTAN_DISTORTION_STR << "\n");
      if (distortion.size() == 5 &&
          str_vals[0] != rig::RADTAN_DISTORTION_STR)
        vw::vw_throw(vw::IOErr() 
                     << "When there are 5 distortion coefficient, distortion type must be: "
                     << rig::RADTAN_DISTORTION_STR << "\n");
      if ((distortion.size() > 5) &&
          str_vals[0] != rig::RPC_DISTORTION_STR)
        vw::vw_throw(vw::IOErr() 
                     << "When there are more than 5 distortion coefficients, distortion "
                     << "type must be: " << rig::RPC_DISTORTION_STR << "\n");

      // Set distortion type based on str_vals[0]
      if (str_vals[0] == rig::NO_DISTORTION_STR) 
        dist_type = rig::NO_DISTORTION;
      else if (str_vals[0] == rig::FOV_DISTORTION_STR) 
        dist_type = rig::FOV_DISTORTION;
      else if (str_vals[0] == rig::FISHEYE_DISTORTION_STR) 
        dist_type = rig::FISHEYE_DISTORTION;
      else if (str_vals[0] == rig::RADTAN_DISTORTION_STR)
        dist_type = rig::RADTAN_DISTORTION;
      else if (str_vals[0] == rig::RPC_DISTORTION_STR)
        dist_type = rig::RPC_DISTORTION;
      else
        vw::vw_throw(vw::IOErr() << "Unknown distortion type: " << str_vals[0] << "\n");
      
      readConfigVals(f, "image_size:", 2, vals);
      Eigen::Vector2i image_size(vals[0], vals[1]);

      readConfigVals(f, "distorted_crop_size:", 2, vals);
      Eigen::Vector2i distorted_crop_size(vals[0], vals[1]);

      readConfigVals(f, "undistorted_image_size:", 2, vals);
      Eigen::Vector2i undistorted_image_size(vals[0], vals[1]);

      rig::CameraParameters params(image_size, focal_length, optical_center, 
                                      distortion, dist_type);
      
      params.SetDistortedCropSize(distorted_crop_size);
      params.SetUndistortedSize(undistorted_image_size);
      R.cam_params.push_back(params);

      readConfigVals(f, "ref_to_sensor_transform:", 12, vals);
      R.ref_to_cam_trans.push_back(vecToAffine(vals));

      // Sanity check
      if (have_rig_transforms) {
        if (R.ref_to_cam_trans.back().matrix() == 0 * R.ref_to_cam_trans.back().matrix())
          vw::vw_throw(vw::IOErr() 
                       << "Failed to read valid transforms between the sensors on the rig\n");
          
        // Put a warning if the transform is the identity matrix if the sensor
        // is not the ref one
        if (sensor_name != ref_sensor_name && 
            R.ref_to_cam_trans.back().matrix() == Eigen::Affine3d::Identity().matrix()) 
          vw::vw_out(vw::WarningMessage) 
            << "A non-reference sensor on the rig has the identity as the "
            << "sensor transform. Sensor name: " << sensor_name << "\n";
          
        // The determinant of the transform must be 1.
        double scale = pow(R.ref_to_cam_trans.back().linear().determinant(), 1.0 / 3.0);
        if (std::abs(scale - 1.0) > 1e-6)
          vw::vw_throw(vw::IOErr() 
                       << "The determinant of the ref-to-sensor transform must be 1.\n");
      }

      readConfigVals(f, "depth_to_image_transform:", 12, vals);
      R.depth_to_image.push_back(vecToAffine(vals));

      readConfigVals(f, "ref_to_sensor_timestamp_offset:", 1, vals);
      double timestamp_offset = vals[0];
      R.ref_to_cam_timestamp_offsets.push_back(timestamp_offset);
    }
    
  } catch(std::exception const& e) {
    vw::vw_throw(vw::IOErr() << e.what() << "\n");
  }

  R.validate();

  return;
}
  
}  // end namespace rig
