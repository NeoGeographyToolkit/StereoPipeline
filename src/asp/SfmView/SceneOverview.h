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

#ifndef __ASP_SFMVIEW_SCENE_OVERVIEW_H__
#define __ASP_SFMVIEW_SCENE_OVERVIEW_H__

#include <asp/SfmView/SfmUtils.h>

#include <QListWidget>

class SceneOverview: public QWidget {
  Q_OBJECT

protected slots:
  void on_scene_changed(sfm::Scene::Ptr scene);
  void on_row_changed(int id);

private:
  void add_view_to_layout(std::size_t id, sfm::View::Ptr view);
  QListWidget* viewlist;

public:
  SceneOverview(QWidget* parent);
  QSize sizeHint(void) const;
};

#endif // __ASP_SFMVIEW_SCENE_OVERVIEW_H__
