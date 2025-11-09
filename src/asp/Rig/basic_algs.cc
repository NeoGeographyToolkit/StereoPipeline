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

#include <Rig/basic_algs.h>

#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace rig {

// Convert keypoints to Eigen format
void vec2eigen(std::vector<std::pair<float, float>> const& vec,
               Eigen::Matrix2Xd & mat) {

  mat = Eigen::MatrixXd(2, vec.size());
  for (size_t it = 0; it < vec.size(); it++) {
    mat.col(it) = Eigen::Vector2d(vec[it].first, vec[it].second);
  }
}

// Convert keypoints from Eigen format
void eigen2vec(Eigen::Matrix2Xd const& mat,
               std::vector<std::pair<float, float>> & vec) {

  vec.clear();
  vec.resize(mat.cols());
  for (size_t it = 0; it < vec.size(); it++)
    vec.at(it) = std::make_pair<float, float>(mat(0, it), mat(1, it));
}

// Read a vector of strings from a file, with spaces and newlines
// acting as separators.  Store them in a set.
void readList(std::string const& file, std::set<std::string> & list) {
  list.clear();
  std::ifstream fh(file);
  std::string val;
  while (fh >> val)
    list.insert(val);
  fh.close();
}

// The parent subdirectory. Example: mydir/nav_cam/file.jpg will return
// 'nav_cam'.
std::string parentSubdir(std::string const& image_file) {
  return fs::path(image_file).parent_path().filename().string();
}

// Replace .<extension> with <suffix>  
std::string changeFileSuffix(std::string filename, std::string new_suffix) {
  // Find the last '.' character in the filename
  size_t last_dot = filename.find_last_of(".");
  if (last_dot == std::string::npos) {
    // No dot found, return original filename
    return filename;
  } else {
    // Replace extension with new suffix
    std::string new_filename = filename.substr(0, last_dot) + new_suffix;
    return new_filename;
  }
}  

// See the header for details.
void calcBracketIndicies(const std::vector<double>& times, double t,
                         int & low, int & high) {

  if (times.empty()) 
    LOG(FATAL) << "Cannot find bracketing indices for an empty vector.\n";
    
  int len = times.size();
  if (t <= times[0]) {
    low = 0;
    high = 1;
    return;
  } else if (t >= times[len-1]) {
    low = len-2;
    high = len-1;
    return;
  } 
  
  low = 0;
  high = len - 1;

  // Search for first iterator it such that t <= times[it]
  auto it = std::lower_bound(times.begin(), times.end(), t);
  high = std::distance(times.begin(), it);
  low = high - 1;

  // Sanity check
  bool is_good = (times[low] <= t && t <= times[high]);
  if (low < 0 || high >= len || low + 1 != high || !is_good) 
    LOG(FATAL) << "Book-keeping failure in bracketing. Must not happen.\n";

  return;  
}

}  // end namespace rig
