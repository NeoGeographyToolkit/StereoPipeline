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

#ifndef RIG_CALIBRATOR_CAMERA_IMAGE_H_
#define RIG_CALIBRATOR_CAMERA_IMAGE_H_

#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rig {

// A class to encompass all known information about a camera image
struct cameraImage {
  // An index to look up the type of camera. This will equal the
  // value ref_camera_type if and only if this is a reference
  // camera.
  int camera_type;

  // The timestamp for this camera (in floating point seconds since epoch)
  // measured by the clock/cpu which is particular to this camera.
  double timestamp;

  // The timestamp with an adjustment added to it to be in
  // reference camera time
  double ref_timestamp;

  // The timestamp of the closest cloud for this image, measured
  // with the same clock as the 'timestamp' value.
  double cloud_timestamp;

  // Indices to look up the reference cameras bracketing this camera
  // in time in array ref_timestamps. These can have the same value
  // for the reference camera and for the last camera if its time
  // stamp is also the last ref cam timestamp, as then there's no more
  // indices.  this is a reference camera.
  int beg_ref_index;
  int end_ref_index;

  // The image for this camera, in grayscale
  cv::Mat image;

  // The corresponding depth cloud, for an image + depth camera.
  // Will be empty and uninitialized for a camera lacking depth.
  cv::Mat depth_cloud;

  // These will be populated automatically if missing
  std::string image_name, depth_name;
};

// A struct to collect together some attributes of an image or depth cloud
// (stored as an image with 3 channels)
struct ImageMessage {
  cv::Mat image;
  double timestamp;
  std::string name;
  Eigen::Affine3d world_to_cam;
};
  
}  // namespace rig

#endif  // RIG_CALIBRATOR_CAMERA_IMAGE_H_
