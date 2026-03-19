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

#ifndef __ASP_SFMVIEW_GL_CONTEXT_H__
#define __ASP_SFMVIEW_GL_CONTEXT_H__

#include <asp/SfmView/SfmMath.h>
#include <asp/SfmView/GlCommon.h>

namespace sfm {

// Trackball camera control that consumes mouse events.
class CamTrackball {
public:
  CamTrackball(void);

  void set_camera(SfmCamera* camera);
  bool consume_event(MouseEvent const& event);

  sfm::Vec3f get_campos(void) const;
  sfm::Vec3f get_viewdir(void) const;
  sfm::Vec3f const& get_upvec(void) const;

private:
  void handle_tb_rotation(int x, int y);
  sfm::Vec3f get_ball_normal(int x, int y);

  SfmCamera* cam;

  float tb_radius;
  sfm::Vec3f tb_center;
  sfm::Vec3f tb_tocam;
  sfm::Vec3f tb_upvec;

  int rot_mouse_x;
  int rot_mouse_y;
  sfm::Vec3f rot_tb_tocam;
  sfm::Vec3f rot_tb_upvec;

  float zoom_tb_radius;
  int zoom_mouse_y;
};

// Rendering context with trackball camera control.
// Subclass and override init_impl, resize_impl, paint_impl.
class GlContext {
public:
  virtual ~GlContext(void) {}

  void init(void);
  void resize(int new_width, int new_height);
  void paint(void);
  bool mouse_event(MouseEvent const& event);

protected:
  virtual void init_impl(void) = 0;
  virtual void resize_impl(int old_width, int old_height);
  virtual void paint_impl(void) = 0;
  void update_camera(void);

  SfmCamera camera;
  CamTrackball controller;
  int width = 0;
  int height = 0;
};

}

#endif // __ASP_SFMVIEW_GL_CONTEXT_H__
