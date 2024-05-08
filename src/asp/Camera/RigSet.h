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

/// \file RigSet.h

// Models a set of rigs

#ifndef __ASP_CAMERA_RIGSET_H__
#define __ASP_CAMERA_RIGSET_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace asp {

namespace camera {  

enum DistortionType {
  NO_DISTORTION,      // no distortion params
  FOV_DISTORTION,     // 1 distortion param
  FISHEYE_DISTORTION, // 4 distortion params
  RADTAN_DISTORTION,  // 4 or 5 distortion params
  RPC_DISTORTION      // many distortion params
};

// These are the names of the distortion models as strings in the config files
const std::string NO_DISTORTION_STR      = "no_distortion";
const std::string FOV_DISTORTION_STR     = "fov";
const std::string FISHEYE_DISTORTION_STR = "fisheye";
const std::string RADTAN_DISTORTION_STR  = "radtan";
const std::string RPC_DISTORTION_STR     = "rpc";

struct CameraParameters {
  double focal_length;
  Eigen::Vector2d optical_center;
  Eigen::Vector2d distortion_params;
  Eigen::Vector2i image_size;
  DistortionType m_distortion_type;
  Eigen::Vector2i m_distorted_crop_size;
  Eigen::Vector2i m_undistorted_size;

  CameraParameters() {} // empty constructor; will create an undefined model
    
  CameraParameters(Eigen::Vector2i const& image_size,
                   double                 focal_length,
                   Eigen::Vector2d const& optical_center,
                   Eigen::VectorXd const& distortion,
                   DistortionType distortion_type);
  
    // Domain of validity of distortion model (normally all image)
    // Centered around image center.
    void SetDistortedCropSize(Eigen::Vector2i const& crop_size);
    void SetUndistortedSize(Eigen::Vector2i const& image_size);
};

}  // end namespace camera
 
struct RigSet {

  // For rig i, cam_set[i] will list the sensor/camera names in that rig.
  // cam_set[i][0] is the reference sensor. All sensor
  // names are unique. For convenience, this is also duplicated
  // as 'cam_names', where all sensors are concatenated. That vector
  // is often more convenient.
  std::vector<std::vector<std::string>> cam_set;

  // Must be in one-to-one correspondence with all vectors below.
  std::vector<std::string> cam_names;

  // A transform from a reference sensor to a sensor in the rig
  // (including to itself, that's the identity transform). This
  // concatenates all such transforms for all rigs, in the order
  // given by concatenating cam_set[0], cam_set[1], etc.
  std::vector<Eigen::Affine3d> ref_to_cam_trans;

  // Depth-to-image transform for each sensor. It is the identity
  // if there are no depth transforms. One per sensor. All
  // concatenated.
  std::vector<Eigen::Affine3d> depth_to_image;

  // The value to add to each ref sensor time to get a given sensor
  // time. One per sensor. All concatenated.
  std::vector<double> ref_to_cam_timestamp_offsets;
    
  // Each sensor's intrinsics. All concatenated. 
  std::vector<camera::CameraParameters> cam_params;

  // If this sensor is a reference sensor for one of the rig.
  bool isRefSensor(std::string const& sensor_name) const;

  // Return the id of the rig given the index of the camera
  // in cam_names.
  int rigId(int cam_id) const;

  // The name of the ref sensor for the rig having the given sensor id
  std::string refSensor(int cam_id) const;

  // Index in the list of sensors of the sensor with given name
  int sensorIndex(std::string const& sensor_name) const;

  // Create a rig set having a single rig  
  RigSet subRig(int rig_id) const;
  
  // Sanity checks
  void validate() const;
};
  
// Save the optimized rig configuration
void writeRigConfig(std::string const& out_dir, bool model_rig, RigSet const& R);
  
// Read a rig configuration. Check if the transforms among the sensors
// on the rig is not 0, in that case will use it.
void readRigConfig(std::string const& rig_config, bool have_rig_transforms,
                   RigSet & R);
  
} // end namespace asp

#endif // __ASP_CAMERA_RIGSET_H__
