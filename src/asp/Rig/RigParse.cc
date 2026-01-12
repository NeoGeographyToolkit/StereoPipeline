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

#include <asp/Rig/RigParse.h>
#include <asp/Rig/system_utils.h>

#include <vw/Core/Log.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace fs = boost::filesystem;

namespace rig {

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

// The parent subdirectory. Example: mydir/nav_cam/file.jpg will return
// 'nav_cam'.
std::string parentSubdir(std::string const& image_file) {
  return fs::path(image_file).parent_path().filename().string();
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
void removeNonAlphaNum(std::string & s) {
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
  
  removeNonAlphaNum(group);

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

  removeNonAlphaNum(group);
  
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

// A little function to replace separators with space. Note that the backslash
// is a separator, in case, it used as a continuation line.
void replace_separators_with_space(std::string & str) {
  std::string sep = "\\:, \t\r\n";
  for (size_t it = 0; it < sep.size(); it++) 
    std::replace(str.begin(), str.end(), sep[it], ' ');
}
  
// A function to parse a string like
// 'cam1:focal_length,optical_center,distortion cam2:focal_length' and
// extract the intrinsics to float. Separators can be space, comma,
// colon.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float_str,
                               std::vector<std::string> const& cam_names,
                               std::vector<std::set<std::string>>& intrinsics_to_float) {
  // Wipe the output
  intrinsics_to_float.clear();

  std::string input_str = intrinsics_to_float_str; // so we can edit it

  replace_separators_with_space(input_str);

  std::istringstream iss(input_str);
  std::string curr_cam = "";
  std::string val;
  
  // Temporary map of sets for collection. This will ensure variable order
  // of inputs is supported.
  std::map<std::string, std::set<std::string>> local_map;
  while (iss >> val) {
    // See if this is a camera name
    bool have_cam_name = false;
    for (size_t it = 0; it < cam_names.size(); it++) {
      if (val == cam_names[it]) {
        curr_cam = val;
        have_cam_name = true;
        break;
      }
    }

    if (have_cam_name) // recorded the camera name
      continue;
    
    if (val != "focal_length" && val != "optical_center" && val != "distortion")
      LOG(FATAL) << "Unexpected value when parsing intrinsics to float: " << val << "\n";

    if (curr_cam == "") 
      LOG(FATAL) << "Incorrectly set option for floating intrinsics.\n";

    local_map[curr_cam].insert(val);
  }

  // Export this
  intrinsics_to_float.resize(cam_names.size());
  for (size_t it = 0; it < cam_names.size(); it++)
    intrinsics_to_float[it] = local_map[cam_names[it]];
}

// A  function to split a string like 'haz_cam sci_cam' into
// its two constituents and validate against the list of known cameras.
void parse_camera_names(std::vector<std::string> const& cam_names,
                        std::string const&
                        depth_to_image_transforms_to_float_str,
                        std::set<std::string>&
                        depth_to_image_transforms_to_float) {
  // Wipe the output
  depth_to_image_transforms_to_float.clear();

  std::string input_str = depth_to_image_transforms_to_float_str; // so we can edit it
  replace_separators_with_space(input_str);
  
  std::istringstream iss(input_str);
  std::string curr_cam = "";
  std::string val;
  
  while (iss >> val) {
    bool have_cam_name = false;
    for (size_t it = 0; it < cam_names.size(); it++) {
      if (val == cam_names[it]) {
        have_cam_name = true;
        break;
      }
    }
    
    if (!have_cam_name) 
      LOG(FATAL) << "Error: A specified sensor name is not among the known sensors. "
                 << "Offending camera: " << val << "\n";
    
    depth_to_image_transforms_to_float.insert(val);
  }

  return;
}

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name) {
  size_t beg = file_name.rfind("/");
  size_t end = file_name.rfind(".");
  if (beg == std::string::npos || end == std::string::npos || beg > end) {
    std::cout << "Could not parse file name: " + file_name;
    exit(1);
  }

  std::string frameStr = file_name.substr(beg + 1, end - beg - 1);
  return atof(frameStr.c_str());
}

// Convert a string of space-separated numbers to a vector
void strToVec(std::string const& str, std::vector<double> & vec) {

  vec.clear();
  std::istringstream iss(str);
  double val = 0.0;
  while (iss >> val)
    vec.push_back(val);
}

// Extract control points and the images they correspond 2 from
// a Hugin project file
void ParseHuginControlPoints(std::string const& hugin_file,
                             std::vector<std::string> * images,
                             Eigen::MatrixXd * points) {
  
  // Initialize the outputs
  (*images).clear();
  *points = Eigen::MatrixXd(6, 0); // this will be resized as points are added

  std::ifstream hf(hugin_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Parse for images
    if (line.find("i ") == 0) {
      size_t it = line.find("n\"");
      if (it == std::string::npos)
        LOG(FATAL) << "ParseHuginControlPoints(): Invalid line: " << line;
      it += 2;
      std::string image;
      while (it < line.size() && line[it] != '"') {
        image += line[it];
        it++;
      }
      (*images).push_back(image);
    }

    // Parse control points
    if (line.find("c ") == 0) {
      // First wipe all letters
      std::string orig_line = line;
      char * ptr = const_cast<char*>(line.c_str());
      for (size_t i = 0; i < line.size(); i++) {
        // Wipe some extra chars
        if ( (ptr[i] >= 'a' && ptr[i] <= 'z') ||
             (ptr[i] >= 'A' && ptr[i] <= 'Z') )
          ptr[i] = ' ';
      }

      // Out of a line like:
      // c n0 N1 x367 y240 X144.183010710425 Y243.04008545843 t0
      // we store the numbers, 0, 1, 367, 240, 144.183010710425 243.04008545843
      // as a column.
      // The stand for left image cid, right image cid,
      // left image x, left image y, right image x, right image y.
      double a, b, c, d, e, f;
      if (sscanf(ptr, "%lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f) != 6)
        LOG(FATAL) << "ParseHuginControlPoints(): Could not scan line: " << line;

      // The left and right images must be different
      if (a == b)
        LOG(FATAL) << "The left and right images must be distinct. "
                   << "Offending line in " << hugin_file << " is:\n"
                   << orig_line << "\n";

      num_points++;
      (*points).conservativeResize(Eigen::NoChange_t(), num_points);
      (*points).col(num_points - 1) << a, b, c, d, e, f;
    }
  }

  return;
}

// A little helper function
bool is_blank(std::string const& line) {
  return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
}

// Parse a file having on each line xyz coordinates
void ParseXYZ(std::string const& xyz_file,
              Eigen::MatrixXd * xyz) {

  // Initialize the outputs
  *xyz = Eigen::MatrixXd(3, 1);

  std::ifstream hf(xyz_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseXYZ(): Could not open hugin file: " << xyz_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Ignore lines starting with comments and empty lines
    if (line.find("#") == 0 || is_blank(line)) continue;

    // Apparently sometimes empty lines show up as if of length 1
    if (line.size() == 1)
      continue;

    // Replace commas with spaces
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';
    double x, y, z;
    if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
      LOG(FATAL) << "ParseXYZ(): Could not scan line: '" << line << "'\n";

    num_points++;
    (*xyz).conservativeResize(Eigen::NoChange_t(), num_points);
    (*xyz).col(num_points-1) << x, y, z;
  }
}
  
}  // namespace rig
