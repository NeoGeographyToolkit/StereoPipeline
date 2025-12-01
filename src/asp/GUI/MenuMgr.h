// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
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

/// \file MenuMgr.h
///
/// Handles menu creation and management for MainWidget.
///
#ifndef __STEREO_GUI_MENU_MGR_H__
#define __STEREO_GUI_MENU_MGR_H__

// Qt
#include <QMenu>
#include <QAction>
#include <QObject>

namespace asp {

class MainWidget; // Forward declaration

// This struct will contain all the menu-related members and their creation
struct MenuMgr {

  // Constructor to create and connect menu actions
  MenuMgr(MainWidget* parent_widget);
  QMenu* formCustomMenu(MainWidget* parent_widget);
  void setupContextMenu();

  MainWidget* m_parent_widget;

  // Right-click context menu
  QMenu  * m_contextMenu;
  QMenu  * m_customMenu;
  QAction* m_addMatchPoint;
  QAction* m_deleteMatchPoint;
  QAction* m_moveMatchPoint;
  QAction* m_toggleHillshadeImageRightClick;
  QAction* m_setThreshold;
  QAction* m_setHillshadeParams;
  QAction* m_saveScreenshot;
  QAction* m_toggleHillshadeFromImageList;
  QAction* m_zoomToImageFromTable;
  QAction* m_bringImageOnTopFromTable;
  QAction* m_pushImageToBottomFromTable;
  QAction* m_changePolyColor;
  QAction* m_allowMultipleSelections_action;
  QAction* m_deleteSelection;
  QAction* m_hideImagesNotInRegion;
  QAction* m_saveVectorLayerAsShapeFile;
  QAction* m_saveVectorLayerAsTextFile;
  QAction* m_deleteVertex;
  QAction* m_deleteVertices;
  QAction* m_insertVertex;
  QAction* m_moveVertex;
  QAction* m_showIndices;
  QAction* m_mergePolys;
  QAction* m_showPolysFilled;

}; // End struct MenuMgr

} // End namespace asp

#endif  // __STEREO_GUI_MENU_MGR_H__
