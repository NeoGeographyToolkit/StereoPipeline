/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
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
#ifndef ASP_RIG_CAMERA_MODEL_H
#define ASP_RIG_CAMERA_MODEL_H

#include <asp/Rig/RigCameraParams.h>

#include <Eigen/Geometry>
#include <string>

namespace rig {

// A model of a camera, with transformation matrix and camera parameters. Need
// to carefully apply distortion / undistortion as needed. It is not automatic.

class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
      const rig::CameraParameters & params);
  CameraModel(const Eigen::Affine3d & transform,
      const rig::CameraParameters & params);
  explicit CameraModel(const rig::CameraParameters & params);
  ~CameraModel();

  const rig::CameraParameters& GetParameters() const;
  Eigen::Vector3d GetPosition() const;
  const Eigen::Affine3d& GetWorldToCam() const;

 private:
  void InitTransform(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation);
  // The transform m_world_to_cam goes from the world to the camera.
  Eigen::Affine3d m_world_to_cam;
  rig::CameraParameters m_params;
};

}  // namespace rig

#endif  // ASP_RIG_CAMERA_MODEL_H
