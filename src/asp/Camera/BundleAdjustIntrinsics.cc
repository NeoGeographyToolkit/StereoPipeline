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

/// \file BundleAdjustIntrinsics.cc
///

#include <asp/Camera/BundleAdjustIntrinsics.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace vw;

namespace asp {

void distortion_sanity_check(std::vector<int> const& num_dist_params,
                             IntrinsicOptions const& intrinsics_opts,
                             std::vector<double> const& intrinsics_limits) {

  // If nothing is shared, there is nothing to do

  // If all distortion params are shared, all sizes must agree
  if (!intrinsics_opts.share_intrinsics_per_sensor &&
      intrinsics_opts.distortion_shared) {
    for (size_t it = 1; it < num_dist_params.size(); it++) {
      if (num_dist_params[it] != num_dist_params[0])
           vw_throw(ArgumentErr() << "When sharing distortion parameters, "
             << "they must have the same size.\n");
    }
  }

  // If distortion is shared per sensor
  if (intrinsics_opts.share_intrinsics_per_sensor) {
    std::vector<std::set<int>> dist_sizes(intrinsics_opts.num_sensors);
    for (size_t cam_it = 0; cam_it < num_dist_params.size(); cam_it++) {
      int sensor_it = intrinsics_opts.cam2sensor[cam_it];
      dist_sizes[sensor_it].insert(num_dist_params[cam_it]); // all found sizes
    }
    // Now check that each dist_sizes[sensor_it] has size 1
    for (size_t sensor_it = 0; sensor_it < intrinsics_opts.num_sensors; sensor_it++) {
      if (dist_sizes[sensor_it].size() != 1)
        vw_throw(ArgumentErr() << "When sharing distortion parameters per sensor, "
         << "they must have the same size for all cameras of the same sensor.\n");
    }
  }

  // Intrinsics limits only can be used for now when all distortion vectors have
  // the same size. This could be fixed but it is a rarely used option
  // and would require a lot of bookkeeping.
  if (!intrinsics_limits.empty()) {
    bool have_same_size = true;
    for (size_t it = 1; it < num_dist_params.size(); it++) {
      if (num_dist_params[it] != num_dist_params[0]) {
        have_same_size = false;
        break;
      }
    }
    if (!have_same_size)
      vw_throw(ArgumentErr() << "When using --intrinsics-limits, all cameras "
               << "must have the same number of distortion coefficients.\n");
  }

  return;
}

// A little function to replace separators with space. Note that the backslash
// is a separator, in case, it used as a continuation line.
void replace_separators_with_space(std::string & str) {
  std::string sep = "\\:;, \t\r\n";
  for (size_t it = 0; it < sep.size(); it++)
    std::replace(str.begin(), str.end(), sep[it], ' ');
}

// Split a string into a vector of strings with space as separator
std::vector<std::string> split_str_with_space(std::string const& str) {
  std::istringstream is(str);
  std::vector<std::string> ret;
  std::string val;
  while (is >> val)
    ret.push_back(val);
  return ret;
}

// Function that tells if a string is a non-negative integer
bool is_str_non_neg_integer(std::string const& str) {
  if (str.empty()) return false;
  for (size_t it = 0; it < str.size(); it++) {
    if (!isdigit(str[it])) return false;
  }
  return true;
}

// Map a named optical-bar motion sub-group to its indices in the 9-element
// distortion vector [speed, MCF, scan_time, velocity(3), final_pose(3)]. This
// floats a subset of the velocity-mode scan-motion model. Returns true if the
// name is recognized. The layout matches pack_optical_bar_to_arrays. Validation
// that the cameras are actually optical bar happens later, in handleBaArgs.
bool optical_bar_float_param(std::string const& name, std::vector<int> & idx) {
  idx.clear();
  if (name == "velocity")                   { idx = {3, 4, 5}; return true; }
  if (name == "motion_compensation_factor") { idx = {1};       return true; }
  if (name == "final_pose")                 { idx = {6, 7, 8}; return true; }
  return false;
}

// Parse format:
// "1:focal_length,optical_center 2:focal_length,other_intrinsics 3:none"
// Applies when optimizing intrinsics per sensor.
// The numbers are sensor indices, starting with 1.
void fine_grained_parse(bool share_intrinsics_per_sensor,
                        int num_sensors,
                        std::vector<std::string> const& options,
                        // Outputs
                        std::vector<bool> & float_center,
                        std::vector<bool> & float_focus,
                        std::vector<bool> & float_distortion,
                        std::set<int>     & float_distortion_indices) {

  // Sanity checks
  if (!share_intrinsics_per_sensor)
    vw_throw(ArgumentErr() << "Intrinsics are not being optimized per sensor. Remove any "
             << "fields of the form 1:, etc., from the options for floating intrinsics.\n");

  if (num_sensors <= 0)
    vw_throw(ArgumentErr() << "Expecting a positive number of sensors.\n");
  if (options.empty())
    vw_throw(ArgumentErr() << "Expecting at least one option.\n");

  // Wipe the outputs
  float_center.clear();
  float_focus.clear();
  float_distortion.clear();

  // It is convenient to initialize all to false. 0th element must always exist.
  for (int i = 0; i < std::max(num_sensors, 1); i++) {
    float_center.push_back(false);
    float_focus.push_back(false);
    float_distortion.push_back(false);
  }

  // First entity must be an integer. That is sensor id, starting from 1.
  if (!is_str_non_neg_integer(options[0]))
    vw_throw(ArgumentErr() << "Expecting an integer as the first option.\n");

  int sensor_id = atoi(options[0].c_str()) - 1; // subtract 1 to make it zero-based

  // check for duplicate ids
  std::set<int> seen_ids;
  // Iterate over options
  for (size_t it = 0; it < options.size(); it++) {

    // If it is an integer, update the sensor id, and continue.
    // Subtract 1 to make it zero-based.
    if (is_str_non_neg_integer(options[it])) {
      sensor_id = atoi(options[it].c_str()) - 1;

      // Sensor id must be in bounds
      if (sensor_id < 0 || sensor_id >= num_sensors)
        vw_throw(ArgumentErr() << "Sensor id " << options[it] << " is out of bounds.\n");

      // If seen, that's a problem
      if (seen_ids.find(sensor_id) != seen_ids.end())
        vw_throw(ArgumentErr() << "Sensor id " << options[it] << " is repeated.\n");
      // Record as seen
      seen_ids.insert(sensor_id);

      continue;
    }

    // Handle the optical center
    if (options[it] == "optical_center") {
      float_center[sensor_id] = true;
      continue;
    }

    // Handle the focal length
    if (options[it] == "focal_length") {
      float_focus[sensor_id] = true;
      continue;
    }

    // Handle the distortion (other_intrinsics)
    if (options[it] == "other_intrinsics" || options[it] == "distortion") {
      float_distortion[sensor_id] = true;
      continue;
    }

    // For all, populate all fields
    if (options[it] == "all") {
      float_center[sensor_id]     = true;
      float_focus[sensor_id]      = true;
      float_distortion[sensor_id] = true;
      continue;
    }

    // Named optical bar motion sub-groups (velocity, motion_compensation_factor,
    // final_pose) float a subset of the distortion vector. Optical-bar-only,
    // validated in handleBaArgs.
    {
      std::vector<int> ob_idx;
      if (optical_bar_float_param(options[it], ob_idx)) {
        float_distortion[sensor_id] = true;
        for (size_t k = 0; k < ob_idx.size(); k++)
          float_distortion_indices.insert(ob_idx[k]);
        continue;
      }
    }

    // For none, just skip
    if (options[it] == "none") {
      continue;
    }

    vw_throw(ArgumentErr() << "Found unknown option when parsing which "
              << "sensor intrinsics to float: " << options[it] << ".\n");
  }
}

// Parse format:
// "focal_length optical_center other_intrinsics"
// Applies to all sensors and when not optimizing intrinsics per sensor.
void coarse_grained_parse(int num_sensors,
                          std::vector<std::string> const& options,
                          // Outputs
                          std::vector<bool> & float_center,
                          std::vector<bool> & float_focus,
                          std::vector<bool> & float_distortion,
                          std::set<int>     & float_distortion_indices) {

  // Wipe the outputs
  float_center.clear();
  float_focus.clear();
  float_distortion.clear();

  if (num_sensors < 0)
    vw_throw(ArgumentErr() << "Cameras were not parsed correctly.\n");

   // It is convenient to initialize all to false. 0th element must always exist.
  for (int i = 0; i < std::max(num_sensors, 1); i++) {
    float_center.push_back(false);
    float_focus.push_back(false);
    float_distortion.push_back(false);
  }

  // For now, populate only for sensor with id 0
  int sensor_id = 0;

  // Iterate over options
  for (size_t it = 0; it < options.size(); it++) {

     // Must not have an integer here
     if (is_str_non_neg_integer(options[0]))
        vw_throw(ArgumentErr() << "When parsing intrinsics to float, expecting a "
                 << "string, not an integer. Check your inputs.\n");

    // Handle the optical center
    if (options[it] == "optical_center") {
      float_center[sensor_id] = true;
      continue;
    }

    // Handle the focal length
    if (options[it] == "focal_length") {
      float_focus[sensor_id] = true;
      continue;
    }

    // Handle the distortion (other_intrinsics)
    if (options[it] == "other_intrinsics" || options[it] == "distortion") {
      float_distortion[sensor_id] = true;
      continue;
    }

    // For all, populate all fields
    if (options[it] == "all") {
      float_center[sensor_id]     = true;
      float_focus[sensor_id]      = true;
      float_distortion[sensor_id] = true;
      continue;
    }

    // Named optical bar motion sub-groups (velocity, motion_compensation_factor,
    // final_pose) float a subset of the distortion vector. Optical-bar-only,
    // validated in handleBaArgs.
    {
      std::vector<int> ob_idx;
      if (optical_bar_float_param(options[it], ob_idx)) {
        float_distortion[sensor_id] = true;
        for (size_t k = 0; k < ob_idx.size(); k++)
          float_distortion_indices.insert(ob_idx[k]);
        continue;
      }
    }

    // For none, just skip
    if (options[it] == "none") {
      continue;
    }

    // We should not arrive here
    vw_throw(ArgumentErr() << "Found unknown option when parsing which "
              << "sensor intrinsics to float: " << options[it] << ".\n");
  }

  // Distribute for all sensors.
  // This will happen only if we share intrinsics per sensor.
  for (int sensor_id = 0; sensor_id < num_sensors; sensor_id++) {
     float_center[sensor_id]     = float_center[0];
     float_focus[sensor_id]      = float_focus[0];
     float_distortion[sensor_id] = float_distortion[0];
  }

  return;
}

void print_float(bool do_float) {
  if (do_float)
    vw_out() << "floated\n";
  else
    vw_out() << "fixed\n";
}

void print_float_vec(std::vector<bool> const& intrinsics, std::string const& name) {
  vw_out() << name << ": ";
  for (size_t it = 0; it < intrinsics.size(); it++) {
    if (intrinsics[it])
      vw_out() << "floated ";
    else
      vw_out() << "fixed ";
  }
  vw_out() << "\n";
}

void print_shared(bool shared) {
  if (shared)
    vw_out() << "shared\n";
  else
    vw_out() << "not shared\n";
}

/// For each option, the string must include a subset of the entries:
///  "focal_length, optical_center, distortion_params"
/// - Need the extra boolean to handle the case where --intrinsics-to-share
///   is provided as "" in order to share none of them.
void load_intrinsics_options(bool        solve_intrinsics,
                             bool        shared_is_specified,
                             std::string intrinsics_to_float_str, // make a copy
                             std::string intrinsics_to_share_str, // make a copy
                             asp::IntrinsicOptions & intrinsics_options) {

  // Share everything unless told otherwise or not solving for intrinsics
  intrinsics_options.focus_shared        = solve_intrinsics;
  intrinsics_options.center_shared       = solve_intrinsics;
  intrinsics_options.distortion_shared   = solve_intrinsics;

  // We need these to be initialized even when not solving for intrinsics, as
  // the intrinsics are always added to the cost function when
  // --inline-adjustments is used. In that case they are just expected to be
  // kept fixed.
  intrinsics_options.float_center.resize(1, false);
  intrinsics_options.float_focus.resize(1, false);
  intrinsics_options.float_distortion.resize(1, false);

  if (((intrinsics_to_float_str != "" && intrinsics_to_float_str != "none") ||
      (intrinsics_to_share_str != "" && intrinsics_to_share_str != "none"))
      && !solve_intrinsics) {
    vw::vw_throw(vw::ArgumentErr() << "To be able set intrinsics to float or share, "
             << "the option --solve-intrinsics must be on.\n");
  }

  if (!solve_intrinsics)
    return;

  // If the user did not specify which intrinsics to float, float all of them.
  boost::to_lower(intrinsics_to_float_str);
  if (intrinsics_to_float_str == "" || intrinsics_to_float_str == "all")
    intrinsics_to_float_str = "focal_length optical_center other_intrinsics";
  // This is the right place in which to turn 'none' to empty string,
  // which now will mean float nothing.
  if (intrinsics_to_float_str == "none")
    intrinsics_to_float_str = "";

  // If the user did not specify which intrinsics to share, share all of them.
  boost::to_lower(intrinsics_to_share_str);
  if (!shared_is_specified) {
    intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
  } else {
    // Otherwise, 'all' also means share all of them, 'none' means share none
    if (intrinsics_to_share_str == "all")
      intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
    if (intrinsics_to_share_str == "none")
      intrinsics_to_share_str = "";
  }

  if (intrinsics_options.share_intrinsics_per_sensor && shared_is_specified)
    vw_out() << "When sharing intrinsics per sensor, option "
              << "--intrinsics-to-share is ignored. The intrinsics will "
              << "always be shared for a sensor and never across sensors.\n";

  // If sharing intrinsics per sensor, the only supported mode is that 
  // the intrinsics are always shared per sensor and never across sensors.
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    intrinsics_options.focus_shared      = false;
    intrinsics_options.center_shared     = false;
    intrinsics_options.distortion_shared = false;
  }

  // Replace any separators (:;, \t\r\n) with spaces. It can be convenient to
  // use commas and colons as separators when passing in the options from the command line.
  asp::replace_separators_with_space(intrinsics_to_float_str);
  asp::replace_separators_with_space(intrinsics_to_share_str);

  // Parse float options. Supported formats:
  // "1:focal_length,optical_center 2:focal_length,other_intrinsics 3:none"  
  // "focal_length optical_center other_intrinsics"
  // In the first case, the numbers are sensor indices, starting with 1.
  std::vector<std::string> float_options = asp::split_str_with_space(intrinsics_to_float_str);
  if (!float_options.empty() && asp::is_str_non_neg_integer(float_options[0])) {
    asp::fine_grained_parse(intrinsics_options.share_intrinsics_per_sensor,
                            intrinsics_options.num_sensors,
                            float_options,
                            intrinsics_options.float_center,
                            intrinsics_options.float_focus,
                            intrinsics_options.float_distortion,
                            intrinsics_options.float_distortion_indices);
  } else {
    asp::coarse_grained_parse(intrinsics_options.num_sensors,
                              float_options,
                              intrinsics_options.float_center,
                              intrinsics_options.float_focus,
                              intrinsics_options.float_distortion,
                              intrinsics_options.float_distortion_indices);
  }

  // Useful reporting
  std::string center_name = "Optical center";
  std::string focus_name  = "Focal length";
  std::string dist_name   = "Other intrinsics (distortion)";
  if (intrinsics_options.share_intrinsics_per_sensor) {
    vw_out() << "Intrinsics are shared for all cameras with given sensor.\n";
    vw_out() << "Number of sensors: " << intrinsics_options.num_sensors << "\n";
    vw_out() << "For each sensor:\n";
    print_float_vec(intrinsics_options.float_center, center_name);
    print_float_vec(intrinsics_options.float_focus, focus_name);
    print_float_vec(intrinsics_options.float_distortion, dist_name);
  } else {
    vw_out() << center_name << ": "; print_float(intrinsics_options.float_center[0]);
    vw_out() << focus_name  << ": "; print_float(intrinsics_options.float_focus[0]);
    vw_out() << dist_name   << ": "; print_float(intrinsics_options.float_distortion[0]);
  }

  // No parsing is done when sharing intrinsics per sensor, per above 
  std::string val;
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    std::istringstream is(intrinsics_to_share_str);
    while (is >> val) {
      if (val == "focal_length")
        intrinsics_options.focus_shared = true;
      else if (val == "optical_center")
        intrinsics_options.center_shared = true;
      else if (val == "other_intrinsics" || val == "distortion")
        intrinsics_options.distortion_shared = true;
      else if (val == "velocity")
        // Share one velocity vector across every camera in the run (one block =
        // one orbit pass). The per-camera velocity slots are frozen and the shared
        // vector is optimized. Set up in handleBaArgs and BaState.
        intrinsics_options.velocity_shared = true;
      else if (val == "motion_compensation_factor" || val == "final_pose")
        // Sharing these is not supported. The fore and aft cameras usually have
        // different values (the motion compensation sign and the end attitude).
        vw_throw(ArgumentErr() << "Sharing the optical bar motion parameter '"
          << val << "' is not supported.\n");
      else
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to share: "
          << val << ".\n");
    }
  }

  // Useful info
  std::string sensor_mode = " (across sensors): ";
  if (intrinsics_options.share_intrinsics_per_sensor)
    sensor_mode = " (per sensor): "; // useful clarification
  vw_out() << center_name << sensor_mode; print_shared(intrinsics_options.center_shared);
  vw_out() << focus_name << sensor_mode;  print_shared(intrinsics_options.focus_shared);
  vw_out() << dist_name << sensor_mode;   print_shared(intrinsics_options.distortion_shared);

} // End function load_intrinsics_options

/// Parse the string of limits and make sure they are all valid pairs.
void parse_intrinsics_limits(std::string const& intrinsics_limits_str,
                             std::vector<double> & intrinsics_limits) {

  intrinsics_limits.clear();
  std::istringstream is(intrinsics_limits_str);
  double val;
  int    count = 0;
  while (is >> val) {
    intrinsics_limits.push_back(val);
    if (count % 2 == 1) {
      if (intrinsics_limits[count] < intrinsics_limits[count-1])
        vw_throw(vw::ArgumentErr()
                  << "Error: Intrinsic limit pairs must be min before max.\n");
    }
    count++;
  }

  if (count % 2 != 0)
    vw::vw_throw(vw::ArgumentErr()
              << "Error: Intrinsic limits must always be provided in min max pairs.\n");
}

} // end namespace asp
