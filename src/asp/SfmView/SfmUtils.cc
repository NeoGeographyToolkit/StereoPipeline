// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include <asp/SfmView/SfmUtils.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace sfm {

SfmCameraInfo::SfmCameraInfo() {
  flen = 0.0f;
  std::fill(trans, trans + 3, 0.0);
  std::fill(rot, rot + 9, 0.0);
  rot[0] = rot[4] = rot[8] = 1.0; // identity
}

Eigen::Vector3f SfmCameraInfo::fill_camera_pos() const {
  Eigen::Vector3f pos;
  pos[0] = float(-rot[0] * trans[0] - rot[3] * trans[1] - rot[6] * trans[2]);
  pos[1] = float(-rot[1] * trans[0] - rot[4] * trans[1] - rot[7] * trans[2]);
  pos[2] = float(-rot[2] * trans[0] - rot[5] * trans[1] - rot[8] * trans[2]);
  return pos;
}

Eigen::Vector3f SfmCameraInfo::fill_viewing_direction() const {
  Eigen::Vector3f viewdir;
  for (int i = 0; i < 3; ++i)
    viewdir[i] = float(rot[6 + i]);
  return viewdir;
}

Eigen::Matrix4f SfmCameraInfo::fill_cam_to_world() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
  // 3x3 block is R^T (transpose of world-to-camera rotation)
  m(0, 0) = float(rot[0]); m(0, 1) = float(rot[3]); m(0, 2) = float(rot[6]);
  m(1, 0) = float(rot[1]); m(1, 1) = float(rot[4]); m(1, 2) = float(rot[7]);
  m(2, 0) = float(rot[2]); m(2, 1) = float(rot[5]); m(2, 2) = float(rot[8]);
  // Translation is -R^T * t
  m(0, 3) = float(-(rot[0]*trans[0] + rot[3]*trans[1] + rot[6]*trans[2]));
  m(1, 3) = float(-(rot[1]*trans[0] + rot[4]*trans[1] + rot[7]*trans[2]));
  m(2, 3) = float(-(rot[2]*trans[0] + rot[5]*trans[1] + rot[8]*trans[2]));
  m(3, 3) = 1.0f;
  return m;
}

// Read camera parameters from a .tsai pinhole model file.
// Only reads the extrinsic parameters (camera center C, rotation R)
// and converts them to the world-to-camera convention (rot, trans).
void SfmCameraInfo::read_tsai(std::string const& filename) {

  std::ifstream cam_file(filename);
  if (cam_file.fail())
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not open file: " + filename);

  // Use a fixed focal length for frustum display.
  // The true value is not needed for camera pose visualization.
  this->flen = 1.0f;

  std::string line;

  // Check for version number on the first line
  std::getline(cam_file, line);
  if (line.find("VERSION") != std::string::npos) {
    int file_version = 1;
    std::sscanf(line.c_str(), "VERSION_%d", &file_version);
    std::getline(cam_file, line);
    if (file_version == 4) {
      if (line.find("PINHOLE") == std::string::npos)
        throw std::invalid_argument(
          "SfmCameraInfo::read_tsai: Expected PINHOLE type, got: " + line);
      std::getline(cam_file, line);
    }
  } else {
    // First line should be fu = ...
    float dummy = 0.0f;
    if (std::sscanf(line.c_str(), "fu = %f", &dummy) != 1)
      throw std::invalid_argument(
        "SfmCameraInfo::read_tsai: File must start with VERSION or fu = . "
        "Got: " + line);
  }

  // Parse fu, fv
  double f_u = 0.0, f_v = 0.0;
  if (std::sscanf(line.c_str(), "fu = %lf", &f_u) != 1)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read x focal length.");

  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "fv = %lf", &f_v) != 1)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read y focal length.");

  // cu, cv (principal point - not used for display)
  double cu = 0.0, cv = 0.0;
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "cu = %lf", &cu) != 1)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read x principal point.");
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "cv = %lf", &cv) != 1)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read y principal point.");

  // u_direction, v_direction, w_direction (not used for display)
  double dummy3[3] = {0.0, 0.0, 0.0};
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "u_direction = %lf %lf %lf",
      &dummy3[0], &dummy3[1], &dummy3[2]) != 3)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read u direction.");
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "v_direction = %lf %lf %lf",
      &dummy3[0], &dummy3[1], &dummy3[2]) != 3)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read v direction.");
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "w_direction = %lf %lf %lf",
      &dummy3[0], &dummy3[1], &dummy3[2]) != 3)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read w direction.");

  // SfmCamera center in world coordinates
  double camera_center[3] = {0.0, 0.0, 0.0};
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "C = %lf %lf %lf",
      &camera_center[0], &camera_center[1], &camera_center[2]) != 3)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read camera center C.");

  // Rotation matrix (camera-to-world, row-major)
  double rotation[9] = {0.0};
  std::getline(cam_file, line);
  if (std::sscanf(line.c_str(), "R = %lf %lf %lf %lf %lf %lf %lf %lf %lf",
      &rotation[0], &rotation[1], &rotation[2],
      &rotation[3], &rotation[4], &rotation[5],
      &rotation[6], &rotation[7], &rotation[8]) != 9)
    throw std::invalid_argument(
      "SfmCameraInfo::read_tsai: Could not read rotation matrix R.");

  // Transpose: camera-to-world -> world-to-camera
  this->rot[0] = rotation[0];
  this->rot[1] = rotation[3];
  this->rot[2] = rotation[6];
  this->rot[3] = rotation[1];
  this->rot[4] = rotation[4];
  this->rot[5] = rotation[7];
  this->rot[6] = rotation[2];
  this->rot[7] = rotation[5];
  this->rot[8] = rotation[8];

  // trans = -world2camera * camera_center
  this->trans[0] = -(rot[0]*camera_center[0] + rot[1]*camera_center[1]
                   + rot[2]*camera_center[2]);
  this->trans[1] = -(rot[3]*camera_center[0] + rot[4]*camera_center[1]
                   + rot[5]*camera_center[2]);
  this->trans[2] = -(rot[6]*camera_center[0] + rot[7]*camera_center[1]
                   + rot[8]*camera_center[2]);
}

// View

View::Ptr View::create() {
  return Ptr(new View);
}

std::string const& View::get_name() const {
  return name_;
}

SfmCameraInfo const& View::get_camera() const {
  return camera_;
}

void View::set_camera(SfmCameraInfo const& cam) {
  camera_ = cam;
}

void View::set_dirty(bool /*dirty*/) {
}

void View::load_view(std::string const& camera_path) {
  // Extract basename for the view name
  std::string::size_type pos = camera_path.find_last_of("/\\");
  if (pos != std::string::npos)
    name_ = camera_path.substr(pos + 1);
  else
    name_ = camera_path;

  camera_ = SfmCameraInfo();
  camera_.read_tsai(camera_path);
}

// Scene

Scene::ViewList const& Scene::get_views() const {
  return views_;
}

Scene::ViewList& Scene::get_views() {
  return views_;
}

View::Ptr Scene::get_view_by_id(std::size_t id) {
  if (id < views_.size())
    return views_[id];
  return View::Ptr();
}

Scene::Ptr Scene::create(
  std::vector<std::string> const& camera_files) {
  Ptr scene(new Scene);
  scene->views_.resize(camera_files.size());
  for (std::size_t i = 0; i < camera_files.size(); ++i) {
    scene->views_[i] = View::create();
    scene->views_[i]->load_view(camera_files[i]);
  }
  return scene;
}

} // namespace sfm
