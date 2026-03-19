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

#ifndef __ASP_SFMVIEW_SCENE_MANAGER_H__
#define __ASP_SFMVIEW_SCENE_MANAGER_H__

#include <asp/SfmView/SfmUtils.h>

#include <QObject>

// Singleton signal hub for scene and view selection.
// SfmMainWindow selects the scene, SceneOverview selects views.
class SceneManager: public QObject {
  Q_OBJECT

private:
  sfm::Scene::Ptr scene;
  sfm::View::Ptr view;

signals:
  void scene_selected(sfm::Scene::Ptr scene);
  void view_selected(sfm::View::Ptr view);

public:
  SceneManager(void);
  ~SceneManager(void);
  static SceneManager& get(void);

  void select_scene(sfm::Scene::Ptr scene);
  void select_view(sfm::View::Ptr view);
  sfm::Scene::Ptr get_scene(void);
  void reset_scene(void);
  void reset_view(void);
};

#endif // __ASP_SFMVIEW_SCENE_MANAGER_H__
