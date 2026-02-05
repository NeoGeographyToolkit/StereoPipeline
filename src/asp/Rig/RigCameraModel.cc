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

#include <asp/Rig/RigCameraModel.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>

namespace rig {

// A model of a camera, with transformation matrix and camera parameters. Need
// to carefully apply distortion / undistortion as needed. It is not automatic.

CameraModel::CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
        const rig::CameraParameters & params) : m_params(params) {
  InitTransform(position, rotation);
}

CameraModel::CameraModel(const rig::CameraParameters & params) : m_params(params) {
  InitTransform(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity());
}

CameraModel::CameraModel(const Eigen::Affine3d & world_to_cam,
                         const rig::CameraParameters & params):
  m_world_to_cam(world_to_cam), m_params(params) {}

void CameraModel::InitTransform(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation) {
  m_world_to_cam.setIdentity();
  m_world_to_cam.translate(-(rotation * position));
  m_world_to_cam.rotate(rotation);
}

CameraModel::~CameraModel() {}

Eigen::Vector3d CameraModel::GetPosition() const {
  return -m_world_to_cam.rotation().inverse() * m_world_to_cam.translation();
}

const Eigen::Affine3d& CameraModel::GetWorldToCam() const {
  return m_world_to_cam;
}

const rig::CameraParameters& CameraModel::GetParameters() const {
  return m_params;
}

}  // namespace rig
