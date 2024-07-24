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

#include <Rig/camera_image.h>
#include <Rig/rig_config.h>
#include <Rig/basic_algs.h>
#include <Rig/image_lookup.h>
#include <Rig/RigCameraParams.h>
#include <Rig/nvm.h>
#include <Rig/interpolation_utils.h>
#include <Rig/transform_utils.h>

#include <glog/logging.h>
#include <vw/Core/Log.h>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

namespace rig {
  
// Sort by timestamps adjusted to be relative to the ref camera clock.
// Additionally sort by image name, so that the order is deterministic.
bool timestampLess(cameraImage i, cameraImage j) {
  return (i.ref_timestamp < j.ref_timestamp) || 
    (i.ref_timestamp == j.ref_timestamp && i.image_name < j.image_name);
}

// The images from the bag may need to be resized to be the same
// size as in the calibration file. Sometimes the full-res images
// can be so blurry that interest point matching fails, hence the
// resizing.
// Similar logic to deal with differences between image size and calibrated size
// is used further down this code.
void adjustImageSize(camera::CameraParameters const& cam_params, cv::Mat & image) {
  int64_t raw_image_cols = image.cols;
  int64_t raw_image_rows = image.rows;
  int64_t calib_image_cols = cam_params.GetDistortedSize()[0];
  int64_t calib_image_rows = cam_params.GetDistortedSize()[1];
  int64_t factor = raw_image_cols / calib_image_cols;

  // If the raw image has size 0, skip, but give a warning. This happens
  // when the image is not found. If prior interest point matches exist,
  // the workflow can still continue.
  if (raw_image_cols == 0 || raw_image_rows == 0) {
    LOG(WARNING) << "Image has size 0, skipping.";
    return;
  }
  
  if ((raw_image_cols != calib_image_cols * factor) ||
      (raw_image_rows != calib_image_rows * factor)) {
    LOG(FATAL) << "Image width and height are: "
               << raw_image_cols << ' ' << raw_image_rows << "\n"
               << "Calibrated image width and height are: "
               << calib_image_cols << ' ' << calib_image_rows << "\n"
               << "These must be equal up to an integer factor.\n";
  }

  if (factor != 1) {
    // TODO(oalexan1): This kind of resizing may be creating aliased images.
    cv::Mat local_image;
    cv::resize(image, local_image, cv::Size(), 1.0/factor, 1.0/factor, cv::INTER_AREA);
    local_image.copyTo(image);
  }

  // Check
  if (image.cols != calib_image_cols || image.rows != calib_image_rows)
    LOG(FATAL) << "The images have the wrong size.";
}

// Find cam type based on cam name
void camTypeFromName(std::string const& cam_name,
                     std::vector<std::string> const& cam_names,
                     int& cam_type) {
  cam_type = 0; // initialize
  for (size_t cam_it = 0; cam_it < cam_names.size(); cam_it++) {
    if (cam_names[cam_it] == cam_name) {
      cam_type = cam_it;
      return;
    }
  }

  throw std::string("Could not determine the sensor type for: " + cam_name);
}

// Given a file with name 
// <dir><text><digits>.<digits><text>ref_cam<text>.jpg
// or 
// <dir>/<cam name>/<digits>.<digits>.jpg
// find the cam type
// Return the index in the basename where the cam name starts or std::string::npos
// if not found.
size_t findCamType(std::string const& image_file,
                   std::vector<std::string> const& cam_names,
                   // Output
                   int & cam_type) {
  // Initialize the output
  cam_type = 0;
  
  std::string basename = fs::path(image_file).filename().string();

  // Try to find the cam name from the basename
  // Convention: my_images/<text>10004.6<text>ref_cam<text>.jpg,
  // where ref_cam is the cam name.
  bool found_cam_name = false;
  std::string cam_name; 
  size_t cam_name_pos = std::string::npos;
  for (size_t cam_it = 0; cam_it < cam_names.size(); cam_it++) {
    auto pos_it = basename.find(cam_names[cam_it]);
    if (pos_it != std::string::npos) {
      cam_name = cam_names[cam_it];
      found_cam_name = true;
      cam_name_pos = pos_it;
      break;
    }
  }

  // Infer cam name based on name of parent directory
  if (!found_cam_name) {
    try {
      // The cam name is the subdir having the images
      cam_name = rig::parentSubdir(image_file);
      found_cam_name = true;
    } catch (std::string const& e) {}
  }

  // Find the sensor id (type)
  camTypeFromName(cam_name, cam_names, cam_type);
  
  // If no luck, cannot continue. In this case the user is supposed to provide
  // --image_sensor_list.
  if (!found_cam_name)
    LOG(FATAL) << "Could not determine the sensor type for: " << image_file
               << ". Check your rig configuration, or provide --image_sensor_list.\n"; 
               
  return cam_name_pos;            
}

// Must keep only alphanumeric characters in the s
void removeNonAlnum(std::string & s) {
  for (size_t it = 0; it < s.size(); it++) {
    if ((s[it] < '0' || s[it] > '9') &&
        (s[it] < 'a' || s[it] > 'z') &&
        (s[it] < 'A' || s[it] > 'Z')) {
      s = s.substr(0, it) + s.substr(it+1);
      it--;
    }
  }
}

// Given a file with name 
// <dir>/<text><digits>.<digits><text>ref_cam<text>.jpg
// or 
// <dir>/<cam name>/<digits>.<digits>.jpg
// find the cam name and the timestamp. 
// Can also handle:
// <dir>/<group><separator><digits>.<digits><text>ref_cam<text>.jpg
// when it parses the group.
void findCamTypeAndTimestamp(std::string const& image_file,
                             std::vector<std::string> const& cam_names,
                             // Outputs
                             int    & cam_type,
                             double & timestamp,
                             std::string & group) {

  // Initialize the outputs
  cam_type  = -1;
  timestamp = 0.0;
  group     = "";
  
  size_t cam_name_pos = rig::findCamType(image_file, cam_names, cam_type);
  std::string basename = fs::path(image_file).filename().string();
  
  // Eliminate the text starting with the sensor name, if we have
  // the sensor name as part of the image name.
  if (cam_name_pos != std::string::npos)
    basename = basename.substr(0, cam_name_pos);

  // Eliminate all non-digits at the end of the basename
  for (size_t it = basename.size(); it > 0; it--) {
    if (basename[it-1] < '0' || basename[it-1] > '9') {
      basename = basename.substr(0, it-1);
    } else {
      break;
    }
  }
  
  // Search backward until finding a non-digit or a second dot.
  // The entry before that is the group, and after is the timestamp.
  int num_dots = 0;
  for (size_t it = basename.size(); it > 0; it--) {
    if (basename[it-1] == '.')
      num_dots++;
      
    if (num_dots >= 2) {
      basename = basename.substr(it);
      break;
    }
  
    if (basename[it-1] != '.' && (basename[it-1] < '0' || basename[it-1] > '9')) {
      group = basename.substr(0, it);
      basename = basename.substr(it);
      break;
    }
  }  
  
  removeNonAlnum(group);

  // Read the last sequence of digits, followed potentially by a dot and more 
  // digits. Remove anything after <digits>.<digits>.
  bool have_dot = false;
  std::string timestamp_str;
  bool found_digits = false;
  for (size_t it = 0; it < basename.size(); it++) {

    if (!found_digits && (basename[it] < '0' || basename[it] > '9')) 
      continue; // Not a digit yet, keep going
    
    found_digits = true;
      
    if (basename[it] == '.') {
      if (have_dot) 
        break; // We have seen a dot already, ignore the rest
      have_dot = true;
      timestamp_str += basename[it];
      continue;
    }

    if (basename[it] < '0' || basename[it] > '9') 
      break; // Not a digit, ignore the rest
    
    timestamp_str += basename[it];
  }

  if (timestamp_str.empty())
    	throw (std::string("Image name (without directory) must have digits as part of ")
              + std::string("their name, which will be converted to a timestamp."));

  // Having the timestamp extracted from the image name is convenient though it
  // requires some care. This is well-documented.
  timestamp = atof(timestamp_str.c_str());
}

// Given a file with name 
// <dir>/<group><separator><separator>ref_cam<text>.ext
// parse the group and the cam index.
void findCamTypeAndGroup(std::string const& image_file,
                        std::vector<std::string> const& cam_names,
                        // Outputs
                        int         & cam_type,
                        std::string & group) {

  // Initialize the outputs
  cam_type  = -1;
  group     = "";
  
  size_t cam_name_pos = rig::findCamType(image_file, cam_names, cam_type);
  std::string basename = fs::path(image_file).filename().string();
  
  // Eliminate the text starting with the sensor name, if we have
  // the sensor name as part of the image name.
  if (cam_name_pos != std::string::npos)
    group = basename.substr(0, cam_name_pos);

  removeNonAlnum(group);
  
  // The group must be non-empty
  if (group.empty())
    LOG(FATAL) << "Could not parse the group from: " << image_file << "\n";
}

// Parse a file each line of which contains a filename, a sensor name, and a timestamp.
// Then reorder the data based on input file names. 
bool parseImageSensorList(std::string const& image_sensor_list,
                          std::vector<std::string> const& image_files,
                          std::vector<std::string> const& cam_names,
                          bool flexible_strategy,
                          // Outputs
                          std::vector<int> & cam_types,
                          std::vector<double> & timestamps) {

  // Wipe the outputs
  cam_types.clear(); cam_types.resize(image_files.size());
  timestamps.clear(); timestamps.resize(image_files.size());
     
  // Open the file
  std::ifstream f(image_sensor_list.c_str());
  if (!f.is_open()) {
    if (flexible_strategy)
      return false;
    else
      LOG(FATAL) << "Cannot open file for reading: " << image_sensor_list << "\n";
  }

  vw::vw_out() << "Reading: " << image_sensor_list << "\n";
  
  // Go from cam name to cam type
  std::map<std::string, int> cam_name_to_type;
  for (size_t it = 0; it < cam_names.size(); it++)
    cam_name_to_type[cam_names[it]] = it;
    
  // Must put the data to read in maps, as they may not be in the same
  // order as in the input image_files that we must respect.
  std::map<std::string, int> image_to_cam_type;
  std::map<std::string, double> image_to_timestamp;
  std::string line;
  while (getline(f, line)) {
    
    // if line starts with comment or has only white spaces, skip it
    if (line.empty() || line[0] == '#') continue;
    if (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos) continue;
     
    std::string image_file;
    double timestamp = 0.0;
    std::string cam_name;
    std::istringstream iss(line);
    if (!(iss >> image_file >> cam_name >> timestamp)) {
      if (flexible_strategy)
        return false;
      else 
       LOG(FATAL) << "Cannot parse: " << image_sensor_list << "\n";
    }
    
    // Must not have duplicate image_file
    if (image_to_cam_type.find(image_file) != image_to_cam_type.end())
      LOG(FATAL) << "Duplicate image file: " << image_file << " in: "
                 << image_sensor_list << "\n";
    if (image_to_timestamp.find(image_file) != image_to_timestamp.end())
      LOG(FATAL) << "Duplicate image file: " << image_file << " in: "
                 << image_sensor_list << "\n";
    
    // Look up the sensor name    
    auto it = cam_name_to_type.find(cam_name);
    if (it == cam_name_to_type.end())
      LOG(FATAL) << "Cannot find sensor name: " << cam_name << "\n";
    image_to_cam_type[image_file] = it->second;
    image_to_timestamp[image_file] = timestamp;
  }
  
  // Must now add them in the order of image_files
  for (size_t img_it = 0; img_it < image_files.size(); img_it++) {
    auto type_it = image_to_cam_type.find(image_files[img_it]);
    auto time_it = image_to_timestamp.find(image_files[img_it]);
    if (type_it == image_to_cam_type.end() || time_it == image_to_timestamp.end())
      LOG(FATAL) << "Cannot find image file: " << image_files[img_it] 
                 << " in list: " << image_sensor_list << "\n";
    cam_types[img_it] = type_it->second;
    timestamps[img_it] = time_it->second;
  }
    
  return true;
}

// For each image, find its sensor name and timestamp. The info can be in a list or
// from the file or directory structure. If flexible_strategy is true, then 
// can try from list first, and if that fails, then from file/directory structure.
void readImageSensorTimestamp(std::string const& image_sensor_list, 
                              std::vector<std::string> const& image_files,
                              std::vector<std::string> const& cam_names,
                              bool flexible_strategy,
                              // Outputs
                              std::vector<int> & cam_types,
                              std::vector<double> & timestamps) {

  // Parse the image sensor list if it is not empty
  bool success = false;
  if (image_sensor_list != "") {
    bool success = parseImageSensorList(image_sensor_list, image_files, cam_names,
                                        flexible_strategy, 
                                        cam_types, timestamps); // outputs
    if (success)
      return;
    if (!success && !flexible_strategy)
      LOG(FATAL) << "Cannot parse the image sensor list: " << image_sensor_list << "\n";
  }
  
  // Clear the outputs
  cam_types.clear(); cam_types.resize(image_files.size());
  timestamps.clear(); timestamps.resize(image_files.size());
  
  for (size_t it = 0; it < image_files.size(); it++) {
    int cam_type = 0;
    double timestamp = 0.0;
    try {
      std::string group; // not used 
      findCamTypeAndTimestamp(image_files[it], cam_names,  
                              cam_type, timestamp, group); // outputs
    } catch (std::exception const& e) {
        LOG(FATAL) << "Could not infer sensor type and image timestamp. See the naming "
                   << "convention, and check your images. Detailed message:\n" << e.what();
    }
    
    cam_types[it] = cam_type;
    timestamps[it] = timestamp;
  }
}

// Find an image at the given timestamp or right after it. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable start_pos that we update as we go.
// TODO(oalexan1): Wipe this!
bool lookupImage(// Inputs
                 double desired_time, std::vector<ImageMessage> const& msgs,
                 // Outputs
                 cv::Mat& image, std::string & image_name,
                 int& start_pos, double& found_time) {
  // Initialize the outputs. Note that start_pos is passed in from outside.
  image = cv::Mat();
  image_name = "";
  found_time = -1.0;

  int num_msgs = msgs.size();
  double prev_image_time = -1.0;

  for (int local_pos = start_pos; local_pos < num_msgs; local_pos++) {
    start_pos = local_pos;  // save this for exporting

    found_time = msgs[local_pos].timestamp;

    // Sanity check: We must always travel forward in time
    if (found_time < prev_image_time) {
      LOG(FATAL) << "Found images not in chronological order.\n"
                 << std::fixed << std::setprecision(17)
                 << "Times in wrong order: " << prev_image_time << ' ' << found_time << ".\n";
      continue;
    }

    prev_image_time = found_time;

    if (found_time >= desired_time) {
      // Found the desired data. Do a deep copy, to not depend on the
      // original structure.
      msgs[local_pos].image.copyTo(image);
      image_name = msgs[local_pos].name;
      return true;
    }
  }
  return false;
}

  
// Find an image at the given timestamp or right after it. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable start_pos that we update as we go.
bool lookupImage(// Inputs
                 double desired_time, MsgMap const& msgs,
                 // Outputs
                 cv::Mat& image, std::string & image_name,
                 MsgMapIter& start_pos, double& found_time) {
  // Initialize the outputs. Note that start_pos is passed in from outside.
  image = cv::Mat();
  image_name = "";
  found_time = -1.0;

   int num_msgs = msgs.size();
   double prev_image_time = -1.0;

   for (auto local_pos = start_pos; local_pos != msgs.end(); local_pos++) {
     start_pos = local_pos;  // save this for exporting

     rig::ImageMessage const& imgMsg = local_pos->second; // alias
     found_time = imgMsg.timestamp;

     // Sanity check: We must always travel forward in time
     if (found_time < prev_image_time) {
       LOG(FATAL) << "Found images not in chronological order.\n"
                  << std::fixed << std::setprecision(17)
                  << "Times in wrong order: " << prev_image_time << ' ' << found_time << ".\n";
       continue;
     }

     prev_image_time = found_time;

     if (found_time >= desired_time) {
       // Found the desired data. Do a deep copy, to not depend on the
       // original structure.
       imgMsg.image.copyTo(image);
       image_name = imgMsg.name;
       return true;
     }
   }
  return false;
}

// A function to extract poses, filenames, and timestamps from read data.
void lookupFilesPoses(// Inputs
                      rig::RigSet const& R,
                      std::vector<std::map<double, rig::ImageMessage>> const& image_maps,
                      std::vector<std::map<double, rig::ImageMessage>> const& depth_maps,
                      // Outputs
                      std::vector<double>& ref_timestamps,
                      std::vector<Eigen::Affine3d> & world_to_ref) {

  // Sanity checks
  if (image_maps.size() != depth_maps.size() || image_maps.size() != R.cam_names.size())
    LOG(FATAL) << "Bookkeeping failure in lookupFilesPoses()\n";
  
  // Wipe the outputs
  ref_timestamps.clear();
  world_to_ref.clear();

  int num_cams = R.cam_names.size();
  for (size_t cam_type = 0; cam_type < image_maps.size(); cam_type++) {

    auto const& image_map = image_maps[cam_type];

    for (auto it = image_map.begin(); it != image_map.end(); it++) {
      // Collect the ref cam timestamps, world_to_ref, in chronological order
      if (R.isRefSensor(R.cam_names[cam_type])) {
        world_to_ref.push_back(it->second.world_to_cam);
        ref_timestamps.push_back(it->second.timestamp);
      }
    }
  }
}

// Read an image with 3 floats per pixel. OpenCV's imread() cannot do that.
void readXyzImage(std::string const& filename, cv::Mat & img) {
  std::ifstream f;
  f.open(filename.c_str(), std::ios::binary | std::ios::in);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for reading: " << filename << "\n";

  int rows, cols, channels;
  // TODO(oalexan1): Replace below with int32_t and check that it is same thing.
  f.read((char*)(&rows), sizeof(rows));         // NOLINT
  f.read((char*)(&cols), sizeof(cols));         // NOLINT
  f.read((char*)(&channels), sizeof(channels)); // NOLINT

  img = cv::Mat::zeros(rows, cols, CV_32FC3);

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      cv::Vec3f P;
      // TODO(oalexan1): See if using reinterpret_cast<char*> does the same
      // thing.
      for (int c = 0; c < channels; c++)
        f.read((char*)(&P[c]), sizeof(P[c])); // NOLINT
      img.at<cv::Vec3f>(row, col) = P;
    }
  }

  return;
}

void readImageEntry(// Inputs
                    std::string const& image_file,
                    Eigen::Affine3d const& world_to_cam,
                    std::vector<std::string> const& cam_names,
                    int cam_type,
                    double timestamp,
                    // Outputs
                    std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                    std::vector<std::map<double, rig::ImageMessage>> & depth_maps) {
  
  // Aliases
  std::map<double, ImageMessage> & image_map = image_maps[cam_type];
  std::map<double, ImageMessage> & depth_map = depth_maps[cam_type];

  if (image_map.find(timestamp) != image_map.end())
    vw::vw_out(vw::WarningMessage) 
      << "Duplicate timestamp " << std::setprecision(17) << timestamp
      << " for sensor id " << cam_type << "\n";
  
  // Read the image as grayscale, in order for feature matching to work
  // For texturing, texrecon should use the original color images.
  //vw::vw_out() << "Reading: " << image_file << std::endl;
  image_map[timestamp].image        = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  image_map[timestamp].name         = image_file;
  image_map[timestamp].timestamp    = timestamp;
  image_map[timestamp].world_to_cam = world_to_cam;

  // Sanity check
  if (depth_map.find(timestamp) != depth_map.end())
    LOG(WARNING) << "Duplicate timestamp " << std::setprecision(17) << timestamp
                 << " for sensor id " << cam_type << "\n";

  // Read the depth data, if present
  std::string depth_file = fs::path(image_file).replace_extension(".pc").string();
  if (fs::exists(depth_file)) {
    //vw::vw_out() << "Reading: " << depth_file << std::endl;
    rig::readXyzImage(depth_file, depth_map[timestamp].image);
    depth_map[timestamp].name      = depth_file;
    depth_map[timestamp].timestamp = timestamp;
  }
}

// Add poses for the extra desired images based on interpolation, extrapolation,
// and/or the rig transform.
void calcExtraPoses(std::string const& extra_list, bool use_initial_rig_transforms,
                    double bracket_len, bool nearest_neighbor_interp,
                    rig::RigSet const& R,
                    // Append here
                    std::vector<std::string>     & cid_to_filename,
                    std::vector<int>             & cam_types,
                    std::vector<double>          & timestamps,
                    std::vector<Eigen::Affine3d> & cid_to_cam_t_global) {

  // Put the existing poses in a map
  std::map<int, std::map<double, Eigen::Affine3d>> existing_world_to_cam;
  std::set<std::string> existing_images;

  for (size_t image_it = 0; image_it < cid_to_filename.size(); image_it++) {
    auto const& image_file = cid_to_filename[image_it];
    existing_images.insert(image_file); 
    int cam_type = cam_types[image_it];
    double timestamp = timestamps[image_it];
    Eigen::Affine3d world2cam = cid_to_cam_t_global[image_it];
    existing_world_to_cam[cam_type][timestamp] = world2cam;

    if (use_initial_rig_transforms) {
      // Use the rig constraint to find the poses for the other sensors on the rig
      // First go to the ref sensor
      double ref_timestamp = timestamp - R.ref_to_cam_timestamp_offsets[cam_type];

      // Careful here with transform directions and order
      Eigen::Affine3d cam_to_ref = R.ref_to_cam_trans[cam_type].inverse();
      Eigen::Affine3d world_to_ref = cam_to_ref * world2cam;

      // Now do all the sensors on that rig. Note how we do the reverse of the above
      // timestamp and camera operations, but not just for the given cam_type,
      // but for any sensor on the rig.
      for (size_t sensor_it = 0; sensor_it < R.ref_to_cam_trans.size(); sensor_it++) {

        if (R.rigId(sensor_it) != R.rigId(cam_type)) 
          continue; // stay within the current rig
        
        // Initialize the map if needed
        if (existing_world_to_cam.find(sensor_it) == existing_world_to_cam.end())
          existing_world_to_cam[sensor_it] = std::map<double, Eigen::Affine3d>();

        // Add an entry, unless one already exists
        std::map<double, Eigen::Affine3d> & map = existing_world_to_cam[sensor_it]; // alias
        // TODO(oalexan1): Any issues with numerical precision of timestamps?
        double curr_timestamp = ref_timestamp + R.ref_to_cam_timestamp_offsets[sensor_it];
        if (map.find(curr_timestamp) == map.end())
          existing_world_to_cam[sensor_it][curr_timestamp]
            = R.ref_to_cam_trans[sensor_it] * world_to_ref;
      }
    }
  }
  
  // Read the extra image names. Ignore the ones already existing.
  std::ifstream f(extra_list.c_str());
  std::vector<std::string> extra_images;
  if (!f.is_open())
    LOG(FATAL) << "Cannot open file for reading: " << extra_list << "\n";
  std::string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::string image_file;
    std::istringstream iss(line);
    if (!(iss >> image_file))
      LOG(FATAL) << "Cannot parse the image file in: " << extra_list << "\n";
    if (existing_images.find(image_file) != existing_images.end()) 
      continue; // this image already exists
    extra_images.push_back(image_file);
  }
  
  // Infer the timestamp and sensor type for the extra images
  std::vector<int> extra_cam_types;
  std::vector<double> extra_timestamps;
  bool flexible_strategy = true; // can handle with and without separate attributes
  readImageSensorTimestamp(extra_list, extra_images, R.cam_names, 
                           flexible_strategy,
                           extra_cam_types, extra_timestamps); // outputs
  
  // Save the new images in a map, to ensure they are sorted.
  // Also need maps for cam types and timestamps, to be able to associate
  // image names with these
  std::map<int, std::map<double, std::string>> extra_map; 
  std::map<std::string, int> extra_cam_types_map;
  std::map<std::string, double> extra_timestamps_map;
  for (size_t image_it = 0; image_it < extra_images.size(); image_it++) {
    std::string image_file = extra_images[image_it]; 
    int cam_type = extra_cam_types[image_it];
    double curr_timestamp = extra_timestamps[image_it];
    extra_map[cam_type][curr_timestamp] = image_file;
    extra_cam_types_map[image_file] = cam_type;
    extra_timestamps_map[image_file] = curr_timestamp;
  }

  // Iterate over each sensor type and interpolate or extrapolate into existing data
  for (auto sensor_it = extra_map.begin(); sensor_it != extra_map.end(); sensor_it++) {

    int cam_type = sensor_it->first;
    std::map<double, std::string> & target_map = sensor_it->second; // alias
    
    // Look up existing poses to be used for interpolation/extrapolation
    auto & input_map = existing_world_to_cam[cam_type]; // alias
    if (input_map.empty()) {
      std::string msg = std::string("Cannot find camera poses for sensor: ")
        + R.cam_names[cam_type] + " as the data is insufficient.\n";
      if (!use_initial_rig_transforms) 
        msg += std::string("If the rig configuration file has an initial rig, consider ")
          + "using the option --use_initial_rig_transforms.\n";
      vw::vw_out() << msg;
      continue;
    }

    std::vector<std::string> found_images;
    std::vector<Eigen::Affine3d> found_poses;
    interpOrExtrap(input_map, target_map, bracket_len, nearest_neighbor_interp,
                   found_images, found_poses); // outputs

    for (size_t found_it = 0; found_it < found_images.size(); found_it++) {
      cid_to_filename.push_back(found_images[found_it]);
      cid_to_cam_t_global.push_back(found_poses[found_it]);
      
      // Add the cam type and timestamp
      auto type_it = extra_cam_types_map.find(found_images[found_it]);
      if (type_it == extra_cam_types_map.end())
        LOG(FATAL) << "Cannot find cam type for image: " << found_images[found_it] << "\n";
      cam_types.push_back(type_it->second);
      auto time_it = extra_timestamps_map.find(found_images[found_it]);
      if (time_it == extra_timestamps_map.end())
        LOG(FATAL) << "Cannot find timestamp for image: " << found_images[found_it] << "\n";
      timestamps.push_back(time_it->second);
    }
  }
}

void readCameraPoses(// Inputs
                     std::string const& camera_poses_file,
                     // Outputs
                     nvmData & nvm) {
  
  // Clear the outputs
  nvm = nvmData();

  // Open the file
  vw::vw_out() << "Reading: " << camera_poses_file << std::endl;
  std::ifstream f(camera_poses_file.c_str());
  if (!f.is_open())
    LOG(FATAL) << "Cannot open file for reading: " << camera_poses_file << "\n";
  
  std::string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    
    std::string image_file;
    std::istringstream iss(line);
    if (!(iss >> image_file))
      LOG(FATAL) << "Cannot parse the image file in: "
                 << camera_poses_file << "\n";
    
    // Read the camera to world transform
    Eigen::VectorXd vals(12);
    double val = -1.0;
    int count = 0;
    while (iss >> val) {
      if (count >= 12) break;
      vals[count] = val;
      count++;
    }
    
    if (count != 12)
      LOG(FATAL) << "Expecting 12 values for the transform on line:\n" << line << "\n";
    
    Eigen::Affine3d world2cam = vecToAffine(vals);
    nvm.cid_to_cam_t_global.push_back(world2cam);
    nvm.cid_to_filename.push_back(image_file);
  }
}

// TODO(oalexan1): Move this to fileio.cc.  
// Read camera information and images from a list or from an NVM file.
// Can interpolate/extrapolate poses for data from an extra list.
// Only later we will consider if the features are shifted or not in the nvm.
void readListOrNvm(// Inputs
                   std::string const& camera_poses_list,
                   std::string const& nvm_file,
                   std::string const& image_sensor_list,
                   std::string const& extra_list,
                   bool use_initial_rig_transforms,
                   double bracket_len, bool nearest_neighbor_interp,
                   bool read_nvm_no_shift,
                   rig::RigSet const& R,
                   // Outputs
                   nvmData & nvm,
                   std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                   std::vector<std::map<double, rig::ImageMessage>> & depth_maps) {

  // Wipe the outputs
  image_maps.clear();
  depth_maps.clear();
  image_maps.resize(R.cam_names.size());
  depth_maps.resize(R.cam_names.size());
  
  if (int(camera_poses_list.empty()) + int(nvm_file.empty()) != 1)
    LOG(FATAL) << "Must specify precisely one of --camera-poses or --nvm.\n";

  if (camera_poses_list != "") {
    rig::readCameraPoses(// Inputs
                               camera_poses_list,  
                               // Outputs
                               nvm);
  } else {
    rig::ReadNvm(nvm_file, 
                 nvm.cid_to_keypoint_map,  
                 nvm.cid_to_filename,  
                 nvm.pid_to_cid_fid,  
                 nvm.pid_to_xyz,  
                 nvm.cid_to_cam_t_global);
    if (!read_nvm_no_shift) {
      std::string offsets_file = rig::offsetsFilename(nvm_file);
      rig::readNvmOffsets(offsets_file, nvm.optical_centers); 
      // Must have as many offsets as images
      if (nvm.optical_centers.size() != nvm.cid_to_filename.size())
        LOG(FATAL) << "Expecting as many optical centers as images.\n";
    }
  }
  
  // Infer the timestamp and sensor type from list or directory structure
  std::vector<int> cam_types;
  std::vector<double> timestamps;
  bool flexible_strategy = false;
  readImageSensorTimestamp(image_sensor_list, nvm.cid_to_filename, R.cam_names, 
                           flexible_strategy,
                           cam_types, timestamps); // outputs
  
  // Extra poses need be be added right after reading the original ones,
  // to ensure the same book-keeping is done for all of them. The extra
  // entries do not mess up the bookkeeping of pid_to_cid_fid, etc,
  // if their cid is larger than the ones read from NVM.
  if (extra_list != "")
    calcExtraPoses(extra_list, use_initial_rig_transforms, bracket_len,
                   nearest_neighbor_interp, R,
                   // Append here
                   nvm.cid_to_filename, cam_types, timestamps, nvm.cid_to_cam_t_global);

  vw::vw_out() << "Reading the images.\n";
  for (size_t it = 0; it < nvm.cid_to_filename.size(); it++) {
    // Aliases
    auto const& image_file = nvm.cid_to_filename[it];
    auto const& world2cam = nvm.cid_to_cam_t_global[it];
    readImageEntry(image_file, world2cam, R.cam_names,  
                   cam_types[it], timestamps[it],
                   // Outputs
                   image_maps, depth_maps);
  }

  return;
}
  
// Look up each ref cam image by timestamp, with the rig assumption. In between
// any two ref cam timestamps, which are no further from each other than the
// bracket length, look up one or more images of each of the other camera types
// in the rig. If more than one choice, and bracket_single_image is true, choose
// the one closest to the midpoint of the two bracketing ref cam timestamps.
// This way there's more wiggle room later if one attempts to modify the
// timestamp offset.
// TODO(oalexan1): This is messy code but was developed with much testing,
// and it works. If it is to be cleaned up, it should be done with care.
// Especially the cases when the other cameras have exactly the same timestamp
// as the ref cam are tricky, and it gets even trickier at the last timestamp.
// Also need to consider the case when timestamp_offsets_max_change is true.
void lookupImagesAndBrackets(// Inputs
                             double bracket_len,
                             double timestamp_offsets_max_change,
                             bool bracket_single_image,
                             rig::RigSet   const& R,
                             std::vector<double> const& ref_timestamps,
                             std::vector<MsgMap> const& image_data,
                             std::vector<MsgMap> const& depth_data,
                             // Outputs
                             std::vector<rig::cameraImage>& cams,
                             std::vector<double>& min_timestamp_offset,
                             std::vector<double>& max_timestamp_offset) {

  vw::vw_out() << "Looking up the images and bracketing the timestamps." << std::endl;

  int num_ref_cams = ref_timestamps.size();
  int num_cam_types = R.cam_names.size();

  // Sanity checks
  if (R.cam_names.size() != image_data.size()) 
    LOG(FATAL) << "Expecting as many sensors as image datasets for them.\n";
  if (R.cam_names.size() != depth_data.size()) 
    LOG(FATAL) << "Expecting as many sensors as depth datasets for them.\n";
    
  // Initialize the outputs
  cams.clear();
  min_timestamp_offset.resize(num_cam_types, -1.0e+100);
  max_timestamp_offset.resize(num_cam_types,  1.0e+100);

  // A lot of care is needed with positions. This remembers how we travel in time
  // for each camera type so we have fewer messages to search.
  // But if a mistake is done below it will mess up this bookkeeping.
  std::vector<MsgMapIter> image_start_positions(num_cam_types);
  std::vector<MsgMapIter> depth_start_positions(num_cam_types);
  for (int cam_it = 0; cam_it < num_cam_types; cam_it++) {
    image_start_positions[cam_it] = image_data[cam_it].begin();
    depth_start_positions[cam_it] = depth_data[cam_it].begin();
  }

  double big = std::numeric_limits<double>::max();
  
  // Populate the data for each camera image
  for (int beg_ref_it = 0; beg_ref_it < num_ref_cams; beg_ref_it++) {

    // For when we have last ref timestamp and last other cam timestamp and they are equal
    int end_ref_it = beg_ref_it + 1;
    bool last_timestamp = (end_ref_it == num_ref_cams);
    if (last_timestamp) end_ref_it = beg_ref_it;

    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      std::vector<rig::cameraImage> local_cams;
      bool success = false;

      // The ref cam does not need bracketing, but the others need to be bracketed
      // by ref cam, so there are two cases to consider.

      if (R.isRefSensor(R.cam_names[cam_type])) {
        // Case of ref sensor
        rig::cameraImage cam;
        cam.camera_type   = cam_type;
        cam.timestamp     = ref_timestamps[beg_ref_it];
        cam.ref_timestamp = cam.timestamp;  // the time offset is 0 between ref and itself
        cam.beg_ref_index = beg_ref_it;
        cam.end_ref_index = beg_ref_it;  // same index for beg and end

        // Start looking up the image timestamp from this position. Some care
        // is needed here as we advance in time in image_start_positions[cam_type].
        double found_time = -1.0;
        // This has to succeed since this timestamp came from an existing image
        bool have_lookup =  
          rig::lookupImage(cam.timestamp, image_data[cam_type],
                           // Outputs
                           cam.image, cam.image_name, 
                           image_start_positions[cam_type], // this will move forward
                           found_time);
        
        if (!have_lookup)
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";

        // The exact time is expected
        if (found_time != cam.timestamp)
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";

        success = true;
        local_cams.push_back(cam);

      } else {
        // Case of not ref sensor
        // Need care here since sometimes ref_cam and current cam can have
        // exactly the same timestamp, so then bracketing should succeed.

        // Convert the bracketing timestamps to current cam's time
        double ref_to_cam_offset = R.ref_to_cam_timestamp_offsets[cam_type];
        double beg_timestamp     = ref_timestamps[beg_ref_it] + ref_to_cam_offset;
        double end_timestamp     = ref_timestamps[end_ref_it] + ref_to_cam_offset;
        if (end_timestamp == beg_timestamp && last_timestamp)  // necessary adjustment
          end_timestamp = std::nextafter(end_timestamp, big); 

        if (end_timestamp <= beg_timestamp)
          LOG(FATAL) << "Ref timestamps must be in strictly increasing order.\n";

        // Find the image timestamp closest to the midpoint of the brackets. This will give
        // more room to vary the timestamp later.
        double mid_timestamp = (beg_timestamp + end_timestamp)/2.0;

        // Search forward in time from image_start_positions[cam_type].
        // We will update that too later. One has to be very careful
        // with it so it does not go too far forward in time
        // so that at the next iteration we are passed what we
        // search for.
        MsgMapIter start_pos = image_start_positions[cam_type]; // care here
        double curr_timestamp = beg_timestamp;                  // start here
        double found_time = -1.0;
        std::vector<double> found_times;
        std::vector<cv::Mat> found_images;
        std::vector<std::string> found_image_names;
        while (1) {
          
          // Stop when we are past the end of the bracket
          if (found_time > end_timestamp) break;

          cv::Mat image;
          std::string image_name;
          bool have_lookup =
            rig::lookupImage(curr_timestamp, image_data[cam_type],
                             // Outputs
                             image, image_name,
                             // care here, start_pos moves forward
                             start_pos,
                             // found_time will be updated now
                             found_time);

          // Need not succeed, but then there's no need to go on as we
          // are at the end
          if (!have_lookup)
            break; 

          // Check if the found time is in the bracket. Note how we allow
          // found_time == beg_timestamp if there's no other choice.
          bool is_in_bracket = (beg_timestamp <= found_time && found_time < end_timestamp);
          double curr_dist = std::abs(found_time - mid_timestamp);

          // Must respect the bracket length, unless best time equals beg time,
          // as then the bracketing is not going to be used.
          bool fail_bracket_len = ((found_time > beg_timestamp && 
                                    end_timestamp - beg_timestamp > bracket_len));

          if (is_in_bracket && !fail_bracket_len) {
            // Update the start position for the future only if this is a good
            // solution. Otherwise we may have moved too far.
            image_start_positions[cam_type] = start_pos;
            
            // Record the found image
            found_images.push_back(cv::Mat());
            image.copyTo(found_images.back());
            found_times.push_back(found_time);
            found_image_names.push_back(image_name);
          }

          // Go forward in time. We count on the fact that
          // lookupImage() looks forward from given guess.
          // Careful here with the api of std::nextafter().
          curr_timestamp = std::nextafter(found_time, big);
        } // end while loop
        
        if (bracket_single_image) {
          // Must pick only one image, which is closest to the midpoint 
          cv::Mat best_image;
          std::string best_image_name;
          double best_dist = 1.0e+100;
          double best_time = -1.0;
          best_dist = 1.0e+100;
          // loop over found images to find the closest one to the midpoint. Save 
          // that as best image, and update best_time and best_image_name, and best_dist.
          for (size_t i = 0; i < found_times.size(); i++) {
            double curr_dist = std::abs(found_times[i] - mid_timestamp);
            if (curr_dist < best_dist) {
              best_dist = curr_dist;
              best_time = found_times[i];
              found_images[i].copyTo(best_image);
              best_image_name = found_image_names[i];
            }
          }

          if (best_time < 0.0) continue;  // bracketing failed
          
          // Make the vector of found images contain only the best image
          found_images.resize(1);
          found_times.resize(1);
          found_image_names.resize(1);
          best_image.copyTo(found_images[0]);
          found_times[0] = best_time;
          found_image_names[0] = best_image_name;
        }

        // Iterate ove found_times and add to local_cams
        for (size_t i = 0; i < found_times.size(); i++) {
          rig::cameraImage cam;
          cam.camera_type   = cam_type;
          cam.timestamp     = found_times[i];
          cam.ref_timestamp = found_times[i] - ref_to_cam_offset;
          cam.beg_ref_index = beg_ref_it;
          cam.end_ref_index = end_ref_it;
          found_images[i].copyTo(cam.image);
          cam.image_name = found_image_names[i];
          local_cams.push_back(cam);
        }
          
        success = (local_cams.size() > 0);
      } // end case of not ref sensor

      if (!success) continue;
      
      // Iterate over the local_cams and update the timestamp offset bounds
      for (size_t i = 0; i < local_cams.size(); i++) {
        auto & cam = local_cams[i];

        if (!R.isRefSensor(R.cam_names[cam_type])) { // Not a ref sensor
          double ref_to_cam_offset = R.ref_to_cam_timestamp_offsets[cam_type];

          // Assuming the option --bracket_single_image is used, 
          // cam.timestamp was chosen as centrally as possible so that
          // ref_timestamps[beg_ref_it] + ref_to_cam_offset <= cam.timestamp
          // and
          // cam.timestamp <= ref_timestamps[end_ref_it] + ref_to_cam_offset
          // Find the range of potential future values of ref_to_cam_offset so that
          // cam.timestamp still respects these bounds.
          min_timestamp_offset[cam_type]
            = std::max(min_timestamp_offset[cam_type],
                      cam.timestamp - ref_timestamps[end_ref_it]);
          max_timestamp_offset[cam_type]
            = std::min(max_timestamp_offset[cam_type],
                      cam.timestamp - ref_timestamps[beg_ref_it]);
        }

        // Look up the closest depth in time (either before or after cam.timestamp)
        // This need not succeed.
        cam.cloud_timestamp = -1.0;  // will change
        if (!depth_data.empty()) 
          rig::lookupImage(cam.timestamp,  // start looking from this time forward
                           depth_data[cam_type],
                           // Outputs
                           cam.depth_cloud, cam.depth_name, 
                           depth_start_positions[cam_type],  // this will move forward
                           cam.cloud_timestamp);             // found time
        
        cams.push_back(cam);
      } // end loop over local_cams
    }  // end loop over camera types
  }    // end loop over ref images

  // Adjust for timestamp_offsets_max_change. Printing the bounds is useful if
  // the timestamps can be allowed to change. Turn off printing this for now, as
  // it is rather confusing. 
  // vw::vw_out() << "If optimizing the timestamp offset, its bounds must be, per sensor:\n";
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    if (R.isRefSensor(R.cam_names[cam_type]))
      continue;  // bounds don't make sense here
    min_timestamp_offset[cam_type] = std::max(min_timestamp_offset[cam_type],
                                              R.ref_to_cam_timestamp_offsets[cam_type]
                                              - timestamp_offsets_max_change);
    max_timestamp_offset[cam_type] = std::min(max_timestamp_offset[cam_type],
                                              R.ref_to_cam_timestamp_offsets[cam_type]
                                              + timestamp_offsets_max_change);
    // Tighten the range a bit to ensure we don't exceed things when we add and
    // subtract timestamps later. Note that timestamps are measured in seconds
    // and fractions of a second since epoch and can be quite large so precision
    // loss can easily happen.
    double delta = (max_timestamp_offset[cam_type] - min_timestamp_offset[cam_type]);
    delta = std::max(std::min(delta/10.0, 1.0e-5), 0.0);
    min_timestamp_offset[cam_type] += delta;
    max_timestamp_offset[cam_type] -= delta;
    // Print the timestamp offset allowed ranges
    // vw::vw_out() << std::setprecision(8) << R.cam_names[cam_type]
    //           << ": [" << min_timestamp_offset[cam_type]
    //           << ", " << max_timestamp_offset[cam_type] << "]\n";
  }

}

// Assuming that the rig constraint is not used, initialize the 'cams' structure
// by copying each image and its other data in that structure as expected
// by later code. See also lookupImagesAndBrackets() when some selection based
// on bracketing takes place.
void lookupImagesNoBrackets(// Inputs
                            rig::RigSet const& R,
                            std::vector<MsgMap> const& image_data,
                            std::vector<MsgMap> const& depth_data,
                            // Outputs
                            std::vector<rig::cameraImage>& cams,
                            std::vector<double>& min_timestamp_offset,
                            std::vector<double>& max_timestamp_offset) {

  vw::vw_out() << "Looking up the images." << std::endl;
  int num_cam_types = R.cam_names.size();
  
  // Initialize the outputs
  cams.clear();
  min_timestamp_offset.resize(num_cam_types, -1.0e+100);
  max_timestamp_offset.resize(num_cam_types,  1.0e+100);

  // A lot of care is needed with positions. This remembers how we travel in time
  // for each camera type so we have fewer messages to search.
  // But if a mistake is done below it will mess up this bookkeeping.
  std::vector<MsgMapIter> image_start_positions(num_cam_types);
  std::vector<MsgMapIter> depth_start_positions(num_cam_types);
  for (int cam_it = 0; cam_it < num_cam_types; cam_it++) {
    image_start_positions[cam_it] = image_data[cam_it].begin();
    depth_start_positions[cam_it] = depth_data[cam_it].begin();
  }

  // Populate the data for each camera image
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {

    int cam_it = -1;
    for (auto map_it = image_data[cam_type].begin(); map_it != image_data[cam_type].end();
         map_it++) {
      cam_it++;
      
      rig::cameraImage cam;
      cam.camera_type   = cam_type;
      cam.timestamp     = (map_it->second).timestamp;
      cam.ref_timestamp = cam.timestamp; // no rig, so no timestamp offset
      // These two values below should not be needed with no rig
      cam.beg_ref_index = cam_it;
      cam.end_ref_index = cam_it;

      // Start looking up the image timestamp from this position. Some care
      // is needed here as we advance in time in image_start_positions[cam_type].
      double found_time = -1.0;
      // This has to succeed since this timestamp originally came from an existing image
      bool have_lookup =  
        rig::lookupImage(cam.timestamp, image_data[cam_type],
                         // Outputs
                         cam.image, cam.image_name, 
                         image_start_positions[cam_type],  // this will move forward
                         found_time);
      if (!have_lookup)
        LOG(FATAL) << std::fixed << std::setprecision(17)
                   << "Cannot look up camera at time " << cam.timestamp << ".\n";

      // The exact time is expected
      if (found_time != cam.timestamp)
        LOG(FATAL) << std::fixed << std::setprecision(17)
                   << "Cannot look up camera at time " << cam.timestamp << ".\n";
      
      // Look up the closest cloud in time (either before or after cam.timestamp)
      // This need not succeed.
      cam.cloud_timestamp = -1.0;  // will change
      if (!depth_data.empty()) 
        rig::lookupImage(cam.timestamp,  // start looking from this time forward
                               depth_data[cam_type],
                               // Outputs
                               cam.depth_cloud, cam.depth_name, 
                               depth_start_positions[cam_type],  // this will move forward
                               cam.cloud_timestamp);             // found time

      // Accept this camera
      cams.push_back(cam);
    }  // end loop over camera types
  }    // end loop over ref images

  return;
}

// Look up images, with or without the rig constraint. See individual functions
// below for more details.
void lookupImagesOneRig(// Inputs
                        bool no_rig, double bracket_len,
                        double timestamp_offsets_max_change,
                        bool bracket_single_image,
                        rig::RigSet const& R,
                        std::vector<MsgMap> const& image_maps,
                        std::vector<MsgMap> const& depth_maps,
                        // Outputs
                        std::vector<double>                 & ref_timestamps,
                        std::vector<Eigen::Affine3d>        & world_to_ref,
                        std::vector<rig::cameraImage> & cams,
                        std::vector<Eigen::Affine3d>        & world_to_cam,
                        std::vector<double>                 & min_timestamp_offset,
                        std::vector<double>                 & max_timestamp_offset) {
  
  rig::lookupFilesPoses(// Inputs
                        R, image_maps, depth_maps,
                        // Outputs
                        ref_timestamps, world_to_ref);
  
  if (!no_rig) 
    lookupImagesAndBrackets(// Inputs
                            bracket_len,  
                            timestamp_offsets_max_change,  
                            bracket_single_image,
                            R, ref_timestamps,  image_maps, depth_maps,  
                            // Outputs
                            cams, min_timestamp_offset, max_timestamp_offset);
  else
    lookupImagesNoBrackets(// Inputs
                           R, image_maps, depth_maps,  
                           // Outputs
                           cams, min_timestamp_offset, max_timestamp_offset);
  
  // See how many timestamps we have for each camera
  std::map<int, int> num_images;
  int num_cam_types = R.cam_names.size();
  for (int cam_type_it = 0; cam_type_it < num_cam_types; cam_type_it++)
    num_images[cam_type_it] = 0;
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++)
    num_images[cams[cam_it].camera_type]++;
  bool is_good = true;
  for (int cam_type_it = 0; cam_type_it < num_cam_types; cam_type_it++) {
    vw::vw_out() << "Number of images for sensor: " << R.cam_names[cam_type_it] << ": "
              << num_images[cam_type_it] << std::endl;

    if (num_images[cam_type_it] == 0)
      is_good = false;
  }
  if (!is_good)
    vw::vw_out(vw::WarningMessage) << "Could not find images for all sensors.\n";

  // The images may need to be resized to be the same
  // size as in the calibration file. Sometimes the full-res images
  // can be so blurry that interest point matching fails, hence the
  // resizing.
  for (size_t it = 0; it < cams.size(); it++)
    rig::adjustImageSize(R.cam_params[cams[it].camera_type], cams[it].image);

  // Sort by the timestamp in reference camera time. This is essential
  // for matching each image to other images close in time. Note
  // that this does not affect the book-keeping of beg_ref_index
  // and end_ref_it in this vector because those indices point to
  // world_to_ref and ref_timestamp, which do not change.
  // Note that this happens for each rig. So, different rigs
  // have the ref timestamps individually sorted, and later
  // these will be concatenated.
  std::sort(cams.begin(), cams.end(), rig::timestampLess);

  // Parse the transform from the world to each cam, which were known
  // on input. Later, if use_initial_rig_transform is specified,
  // these will be computed based on the rig.  Since
  // image_maps[cam_type] is sorted chronologically, travel in time
  // along at as we move along the cams array. Use the two arrays
  // below to remember where we left off.
  // TODO(oalexan1): This is fragile. It relies on cams being sorted by time.
  // Make the cams array have a world_to_cam entry and remove the loop below.
  world_to_cam.resize(cams.size());
  std::vector<MsgMapIter> beg_pos(num_cam_types); 
  std::vector<MsgMapIter> end_pos(num_cam_types); 
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    beg_pos[cam_type] = image_maps[cam_type].begin();
    end_pos[cam_type] = image_maps[cam_type].end();
  }
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++) {
    int cam_type = cams[cam_it].camera_type;
    for (auto pos = beg_pos[cam_type]; pos != end_pos[cam_type]; pos++) {
      if (cams[cam_it].timestamp == pos->first) {
        world_to_cam[cam_it] = (pos->second).world_to_cam;
        beg_pos[cam_type] = pos;  // save for next time
        break;
      }
    }
  }  
  return; 
}

// Look up images for a set of rigs. This requires looking up images for individual rigs,
// then concatenating the results and adjusting the book-keeping.
void lookupImages(// Inputs
                  bool no_rig, double bracket_len,
                  double timestamp_offsets_max_change,
                  bool bracket_single_image,
                  rig::RigSet const& R,
                  std::vector<MsgMap> const& image_maps,
                  std::vector<MsgMap> const& depth_maps,
                  // Outputs
                  std::vector<double>                 & ref_timestamps,
                  std::vector<Eigen::Affine3d>        & world_to_ref,
                  std::vector<rig::cameraImage> & cams,
                  std::vector<Eigen::Affine3d>        & world_to_cam,
                  std::vector<double>                 & min_timestamp_offset,
                  std::vector<double>                 & max_timestamp_offset) {

  // Wipe the outputs
  ref_timestamps.clear();
  world_to_ref.clear();
  cams.clear();
  world_to_cam.clear();
  min_timestamp_offset.clear();
  max_timestamp_offset.clear();

  for (size_t rig_id = 0; rig_id < R.cam_set.size(); rig_id++) {

    // Create a single rig
    rig::RigSet sub_rig = R.subRig(rig_id);

    // Prepare the inputs for the subrig
    std::vector<MsgMap> sub_image_maps;
    std::vector<MsgMap> sub_depth_maps;
    for (size_t sub_it = 0; sub_it < sub_rig.cam_names.size(); sub_it++) {
      std::string sensor_name = sub_rig.cam_names[sub_it];
      int rig_set_it = R.sensorIndex(sensor_name); // index in the larger rig
      sub_image_maps.push_back(image_maps[rig_set_it]);
      sub_depth_maps.push_back(depth_maps[rig_set_it]);
    }

    std::vector<double>                 sub_ref_timestamps;
    std::vector<Eigen::Affine3d>        sub_world_to_ref;
    std::vector<rig::cameraImage> sub_cams;
    std::vector<Eigen::Affine3d>        sub_world_to_cam;
    std::vector<double>                 sub_min_timestamp_offset;
    std::vector<double>                 sub_max_timestamp_offset;

    // Do the work for the subrig
    lookupImagesOneRig(// Inputs
                       no_rig, bracket_len, timestamp_offsets_max_change, 
                       bracket_single_image,
                       sub_rig,  
                       sub_image_maps, sub_depth_maps,  
                       // Outputs
                       sub_ref_timestamps, sub_world_to_ref, sub_cams,  
                       sub_world_to_cam, sub_min_timestamp_offset, sub_max_timestamp_offset);

    // Save the endpoints for ref timestamps and all cams, before concatenation
    size_t prev_ref_end = ref_timestamps.size();
    size_t prev_end = cams.size();

    // Append the answers
    ref_timestamps.insert(ref_timestamps.end(), sub_ref_timestamps.begin(),
                          sub_ref_timestamps.end());
    world_to_ref.insert(world_to_ref.end(), sub_world_to_ref.begin(), sub_world_to_ref.end());
    cams.insert(cams.end(), sub_cams.begin(), sub_cams.end());
    world_to_cam.insert(world_to_cam.end(), sub_world_to_cam.begin(), sub_world_to_cam.end());
    min_timestamp_offset.insert(min_timestamp_offset.end(), sub_min_timestamp_offset.begin(),
                                sub_min_timestamp_offset.end());
    max_timestamp_offset.insert(max_timestamp_offset.end(), sub_max_timestamp_offset.begin(),
                                sub_max_timestamp_offset.end());

    // Update the bookkeeping in 'cams'
    for (size_t cam_it = prev_end; cam_it < cams.size(); cam_it++) {

      // Find the current sensor index in the larger rig set
      int subrig_sensor_index = cams[cam_it].camera_type;
      std::string subrig_sensor = sub_rig.cam_names[subrig_sensor_index];
      int rig_sensor_index = R.sensorIndex(subrig_sensor);
      cams[cam_it].camera_type = rig_sensor_index;

      // Update the pointers to indices in ref_timestamps
      cams[cam_it].beg_ref_index += prev_ref_end;     
      cams[cam_it].end_ref_index += prev_ref_end;
    }
  }

  return;
}
  
}  // end namespace rig
