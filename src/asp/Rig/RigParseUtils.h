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

#ifndef ASP_RIG_RIG_PARSE_UTILS_H
#define ASP_RIG_RIG_PARSE_UTILS_H

#include <Eigen/Core>

#include <string>
#include <vector>
#include <set>
#include <map>

namespace rig {

// Given a file with name 
// <dir><text><digits>.<digits><text>ref_cam<text>.jpg
// or 
// <dir>/<cam name>/<digits>.<digits>.jpg
// find the cam type
// Return the index in the basename where the cam name starts or std::string::npos if not found.
size_t findCamType(std::string const& image_file,
                   std::vector<std::string> const& cam_names,
                   // Output
                   int & cam_type);

// Must keep only alphanumeric characters in the s
void removeNonAlphaNum(std::string & s);

// Given a file with name 
// <dir><text><digits>.<digits><text>ref_cam<text>.jpg
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
                             std::string & group);

// Given a file with name 
// <dir>/<group><separator><separator>ref_cam<text>.ext
// parse the group and the cam index.
void findCamTypeAndGroup(std::string const& image_file,
                        std::vector<std::string> const& cam_names,
                        // Outputs
                        int         & cam_type,
                        std::string & group);

// Parse a file each line of which contains a filename, a sensor name, and a timestamp.
// Then reorder the data based on input file names. 
bool parseImageSensorList(std::string const& image_sensor_list,
                          std::vector<std::string> const& image_files,
                          std::vector<std::string> const& cam_names,
                          bool flexible_strategy,
                          // Outputs
                          std::vector<int> & cam_types,
                          std::vector<double> & timestamps);

// A little function to replace separators with space. Note that the backslash
// is a separator, in case, it used as a continuation line.
void replace_separators_with_space(std::string & str);

// A function to parse a string like
// 'cam1:focal_length,optical_center,distortion cam2:focal_length' and
// extract the intrinsics to float. Separators can be space, comma,
// colon.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float_str,
                               std::vector<std::string> const& cam_names,
                               std::vector<std::set<std::string>>& intrinsics_to_float);

// A  function to split a string like 'haz_cam sci_cam' into
// its two constituents and validate against the list of known cameras.
void parse_camera_names(std::vector<std::string> const& cam_names,
                        std::string const&
                        depth_to_image_transforms_to_float_str,
                        std::set<std::string>&
                        depth_to_image_transforms_to_float);

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name);

// Convert a string of space-separated numbers to a vector
void strToVec(std::string const& str, std::vector<double> & vec);

// Extract control points and the images they correspond 2 from
// a Hugin project file
void ParseHuginControlPoints(std::string const& hugin_file,
                             std::vector<std::string> * images,
                             Eigen::MatrixXd * points);

// Parse a file having on each line xyz coordinates
void ParseXYZ(std::string const& xyz_file,
              Eigen::MatrixXd * xyz);

}  // namespace rig

#endif  // ASP_RIG_RIG_PARSE_UTILS_H
