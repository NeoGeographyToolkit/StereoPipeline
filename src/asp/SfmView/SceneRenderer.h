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

#ifndef __ASP_SFMVIEW_SCENE_RENDERER_H__
#define __ASP_SFMVIEW_SCENE_RENDERER_H__

#include <asp/SfmView/GlCommon.h>
#include <asp/SfmView/SfmUtils.h>
#include <asp/SfmView/GlContext.h>
#include <asp/SfmView/MeshRenderer.h>
#include <asp/SfmView/GlWidget.h>

#include <QAction>
#include <QSlider>

// 3D renderer: draws camera frusta, ground plane, viewing direction.
class SceneRenderer: public QWidget, public sfm::GlContext {
  Q_OBJECT

public:
  SceneRenderer(GlWidget* gl_widget);
  void set_scene(sfm::Scene::Ptr scene);
  void set_view(sfm::View::Ptr view);
  void reset_scene(void);

  QAction* get_action_frusta(void);
  QAction* get_action_viewdir(void);
  QAction* get_action_ground(void);
  QSlider* get_frusta_size_slider(void);

protected:
  void init_impl(void);
  void resize_impl(int old_width, int old_height);
  void paint_impl(void);

private slots:
  void reset_viewdir_renderer(void);
  void reset_frusta_renderer(void);
  void reset_ground_renderer(void);
  void on_scene_changed(void);

private:
  void load_shaders(void);
  void send_uniform(sfm::SfmCamera const& cam);
  void create_frusta_renderer(void);
  void create_ground_renderer(void);
  void create_viewdir_renderer(void);

  GlWidget* gl_widget;
  QOpenGLShaderProgram* wireframe_shader = nullptr;
  sfm::Scene::Ptr scene;
  sfm::View::Ptr view;

  QAction* action_frusta;
  QAction* action_viewdir;
  QAction* action_ground;
  QSlider* frusta_size_slider;
  sfm::MeshRenderer::Ptr frusta_renderer;
  sfm::MeshRenderer::Ptr ground_renderer;
  sfm::MeshRenderer::Ptr viewdir_renderer;

  // Cached original poses (before GL transformation).
  // Cleared on scene change, preserved across slider changes.
  std::vector<Eigen::Vector3d> orig_cam_centers;
  std::vector<Eigen::Matrix3d> orig_cam2world_vec;
};

#endif // __ASP_SFMVIEW_SCENE_RENDERER_H__
