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

#ifndef RIG_IMAGE_LOOKUP_H_
#define RIG_IMAGE_LOOKUP_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace rig {
  // Forward declaration
  class CameraParameters;
}

namespace asp {
  class nvmData;
}

namespace rig {

// Forward declarations  
class cameraImage;
class ImageMessage;
class RigSet;

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

// For each image, find its sensor name and timestamp. The info can be in a list or
// from the file or directory structure. If flexible_strategy is true, then 
// can try from list first, and if that fails, then from file/directory structure.
void readImageSensorTimestamp(std::string const& image_sensor_list, 
                              std::vector<std::string> const& image_files,
                              std::vector<std::string> const& cam_names,
                              bool flexible_strategy,
                              // Outputs
                              std::vector<int> & cam_types,
                              std::vector<double> & timestamps);

// Look up images, with or without the rig constraint. See individual
// functions below for more details.
typedef std::map<double, rig::ImageMessage> MsgMap;
typedef MsgMap::const_iterator MsgMapIter;
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
                  std::vector<double>                 & max_timestamp_offset);
  
// Find pointers to the camera and reference images that bracket the
// camera image. Great care is needed here. Two cases are considered,
// if there is a rig or not. If no_rig is true, then the reference images are
// the same as the camera images. 
void calcBracketing(// Inputs
                  bool no_rig, int cid, int cam_type,
                  std::vector<rig::cameraImage> const& cams,
                  std::vector<double> const& ref_timestamps,
                  rig::RigSet   const& R,
                  // Will not be changed but need access
                  std::vector<double> & world_to_cam_vec,
                  std::vector<double> & world_to_ref_vec,
                  std::vector<double> & ref_to_cam_vec,
                  std::vector<double> & ref_identity_vec,
                  std::vector<double> & right_identity_vec,
                  // Outputs
                  double* & beg_cam_ptr, 
                  double* & end_cam_ptr, 
                  double* & ref_to_cam_ptr,
                  double  & beg_ref_timestamp, 
                  double  & end_ref_timestamp,
                  double  & cam_timestamp);

}  // namespace rig

#endif  // RIG_IMAGE_LOOKUP_H_
