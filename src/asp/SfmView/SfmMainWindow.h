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

#ifndef __ASP_SFMVIEW_SFM_MAIN_WINDOW_H__
#define __ASP_SFMVIEW_SFM_MAIN_WINDOW_H__

#include <asp/SfmView/GlWidget.h>
#include <asp/SfmView/SceneRenderer.h>
#include <asp/SfmView/SceneOverview.h>

#include <QMainWindow>

#include <string>
#include <vector>

class SfmMainWindow: public QMainWindow {
  Q_OBJECT

private:
  QDockWidget* dock_scene;

  SceneOverview* scene_overview;
  GlWidget* gl_widget;
  SceneRenderer* scene_renderer;

  QAction* action_quit;
  QAction* action_about;

  QMenu* menu_file;
  QMenu* menu_view;
  QMenu* menu_help;

  void create_actions(void);
  void create_menus(void);
  void perform_close_scene(void);

private slots:
  void on_scene_selected(sfm::Scene::Ptr scene);
  void on_view_selected(sfm::View::Ptr view);
  void on_about(void);
  void on_frusta_size(void);

  void closeEvent(QCloseEvent* event);

public:
  SfmMainWindow(int width, int height);
  ~SfmMainWindow(void);

  void load_scene(std::vector<std::string> const& images,
                  std::vector<std::string> const& cameras);
};

#endif // __ASP_SFMVIEW_SFM_MAIN_WINDOW_H__
