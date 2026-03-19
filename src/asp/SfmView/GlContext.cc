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

#include <asp/SfmView/GlContext.h>

#include <algorithm>
#include <iostream>

namespace sfm {

void CamTrackball::set_camera(SfmCamera* camera) {
  this->cam = camera;
}

Eigen::Vector3f CamTrackball::get_campos(void) const {
  return this->tb_center + this->tb_tocam * this->tb_radius;
}

Eigen::Vector3f CamTrackball::get_viewdir(void) const {
  return -this->tb_tocam;
}

Eigen::Vector3f const& CamTrackball::get_upvec(void) const {
  return this->tb_upvec;
}

CamTrackball::CamTrackball(void) {
  this->cam = nullptr;
  this->tb_radius = 1.0f;
  this->tb_center = Eigen::Vector3f::Zero();
  this->tb_tocam = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  this->tb_upvec = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
}

bool CamTrackball::consume_event(MouseEvent const& event) {
  bool is_handled = false;

  if (event.type == MOUSE_EVENT_PRESS) {
    if (event.button == MOUSE_BUTTON_LEFT) {
      this->rot_mouse_x = event.x;
      this->rot_mouse_y = event.y;
      this->rot_tb_tocam = this->tb_tocam;
      this->rot_tb_upvec = this->tb_upvec;
    } else if (event.button == MOUSE_BUTTON_MIDDLE) {
      this->zoom_mouse_y = event.y;
      this->zoom_tb_radius = this->tb_radius;
    }
    is_handled = true;
  } else if (event.type == MOUSE_EVENT_MOVE) {
    if (event.button_mask & MOUSE_BUTTON_LEFT) {
      if (event.x == this->rot_mouse_x && event.y == this->rot_mouse_y) {
        this->tb_tocam = this->rot_tb_tocam;
        this->tb_upvec = this->rot_tb_upvec;
      } else {
        this->handle_tb_rotation(event.x, event.y);
      }
      is_handled = true;
    }

    if (event.button_mask & MOUSE_BUTTON_MIDDLE) {
      int mouse_diff = this->zoom_mouse_y - event.y;
      float zoom_speed = this->zoom_tb_radius / 100.0f;
      float cam_diff = (float)mouse_diff * zoom_speed;
      float new_radius = this->zoom_tb_radius + cam_diff;
      this->tb_radius = sfm::clamp(new_radius, this->cam->z_near, this->cam->z_far);
      is_handled = true;
    }
  } else if (event.type == MOUSE_EVENT_WHEEL_UP) {
    this->tb_radius = this->tb_radius + this->tb_radius / 10.0f;
    this->tb_radius = std::min(this->cam->z_far, this->tb_radius);
    is_handled = true;
  } else if (event.type == MOUSE_EVENT_WHEEL_DOWN) {
    this->tb_radius = this->tb_radius - this->tb_radius / 10.0f;
    this->tb_radius = std::max(this->cam->z_near, this->tb_radius);
    is_handled = true;
  }

  return is_handled;
}

void CamTrackball::handle_tb_rotation(int x, int y) {
  // Get ball normals.
  Eigen::Vector3f bn_start = this->get_ball_normal(
    this->rot_mouse_x, this->rot_mouse_y);
  Eigen::Vector3f bn_now = this->get_ball_normal(x, y);

  // Rotation axis and angle.
  Eigen::Vector3f axis = bn_now.cross(bn_start);
  float angle = std::acos(bn_now.dot(bn_start));

  // Rotate axis to world coords. Build inverse viewing matrix from
  // values stored at the time of mouse click.
  Eigen::Matrix4f cam_to_world;
  Eigen::Vector3f campos = this->tb_center + this->rot_tb_tocam * this->tb_radius;
  Eigen::Vector3f viewdir = -this->rot_tb_tocam;
  cam_to_world = sfm::matrix_inverse_viewtrans(
    campos, viewdir, this->rot_tb_upvec);
  axis = sfm::mult_homogeneous(cam_to_world, axis, 0.0f);
  axis.normalize();

  // Rotate camera and up vector around axis.
  Eigen::Matrix3f rot = matrix_rotation_from_axis_angle(axis, angle);
  this->tb_tocam = rot * this->rot_tb_tocam;
  this->tb_upvec = rot * this->rot_tb_upvec;
}

Eigen::Vector3f CamTrackball::get_ball_normal(int x, int y) {
  // Calculate normal on unit sphere.
  Eigen::Vector3f sn;
  sn[0] = 2.0f * (float)x / (float)(this->cam->width - 1) - 1.0f;
  sn[1] = 1.0f - 2.0f * (float)y / (float)(this->cam->height - 1);
  float z2 = 1.0f - sn[0] * sn[0] - sn[1] * sn[1];
  sn[2] = z2 > 0.0f ? std::sqrt(z2) : 0.0f;

  return sn.normalized();
}

// GlContext

void GlContext::init(void) {
  this->controller.set_camera(&this->camera);
  this->init_impl();
}

void GlContext::resize(int new_width, int new_height) {
  std::swap(new_width, this->width);
  std::swap(new_height, this->height);
  this->resize_impl(new_width, new_height);
}

void GlContext::paint(void) {
  this->paint_impl();
}

bool GlContext::mouse_event(MouseEvent const& event) {
  bool is_handled = this->controller.consume_event(event);
  this->update_camera();
  return is_handled;
}

void GlContext::resize_impl(int /*old_width*/, int /*old_height*/) {
  glFunctions()->glViewport(0, 0, this->width, this->height);
  this->camera.width = this->width;
  this->camera.height = this->height;

  float aspect = (float)this->width / (float)this->height;
  float minside = 0.05f;
  if (this->width > this->height) {
    this->camera.top = minside;
    this->camera.right = minside * aspect;
  } else {
    this->camera.top = minside / aspect;
    this->camera.right = minside;
  }

  this->camera.update_proj_mat();
}

void GlContext::update_camera(void) {
  this->camera.pos = this->controller.get_campos();
  this->camera.viewing_dir = this->controller.get_viewdir();
  this->camera.up_vec = this->controller.get_upvec();
  this->camera.update_view_mat();
}

}
