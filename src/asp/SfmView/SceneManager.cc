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

#include <asp/SfmView/SceneManager.h>

SceneManager::SceneManager(void) {
}

SceneManager::~SceneManager(void) {
}

SceneManager& SceneManager::get(void) {
  static SceneManager instance;
  return instance;
}

void SceneManager::select_scene(sfm::Scene::Ptr scene) {
  this->scene = scene;
  emit this->scene_selected(scene);
}

void SceneManager::select_view(sfm::View::Ptr view) {
  this->view = view;
  emit this->view_selected(view);
}

sfm::Scene::Ptr SceneManager::get_scene(void) {
  return this->scene;
}

void SceneManager::reset_scene(void) {
  this->select_scene(sfm::Scene::Ptr());
}

void SceneManager::reset_view(void) {
  this->select_view(sfm::View::Ptr());
}
