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

// \file WindowMenuMgr.h
//
// Handles menu bar creation and management for MainWindow.
//
#ifndef __STEREO_GUI_WINDOW_MENU_MGR_H__
#define __STEREO_GUI_WINDOW_MENU_MGR_H__

// Qt
#include <QMenu>
#include <QAction>

namespace asp {

class MainWindow; // Forward declaration

// Menu bar for MainWindow
struct WindowMenuMgr {

  WindowMenuMgr() = default;

  // Build the menu bar and connect signals to MainWindow slots
  void init(MainWindow* win);

  // Menus
  QMenu *m_file_menu;
  QMenu *m_view_menu;
  QMenu *m_matches_menu;
  QMenu *m_threshold_menu;
  QMenu *m_profile_menu;
  QMenu *m_vector_layer_menu;
  QMenu *m_help_menu;

  // Actions
  QAction *m_about_action;
  QAction *m_thresholdCalc_action;
  QAction *m_thresholdGetSet_action;
  QAction *m_setLineWidth_action;
  QAction *m_setPolyColor_action;
  QAction *m_sizeToFit_action;
  QAction *m_zoomIn_action;
  QAction *m_zoomOut_action;
  QAction *m_viewSingleWindow_action;
  QAction *m_viewAllSideBySide_action;
  QAction *m_viewSeveralSideBySide_action;
  QAction *m_viewAsTiles_action;
  QAction *m_zoomToProjWin_action;
  QAction *m_viewHillshadedImages_action;
  QAction *m_viewGeoreferencedImages_action;
  QAction *m_overlayGeoreferencedImages_action;
  QAction *m_viewThreshImages_action;
  QAction *m_contourImages_action;
  QAction *m_saveVectorLayerAsShapeFile_action;
  QAction *m_saveVectorLayerAsTextFile_action;
  QAction *m_zoomAllToSameRegion_action;
  QAction *m_viewNextImage_action;
  QAction *m_viewPrevImage_action;
  QAction *m_viewMatches_action;
  QAction *m_viewPairwiseMatches_action;
  QAction *m_viewPairwiseCleanMatches_action;
  QAction *m_addDelMatches_action;
  QAction *m_saveMatches_action;
  QAction *m_writeGcp_action;
  QAction *m_save_screenshot_action;
  QAction *m_select_region_action;
  QAction *m_change_cursor_action;
  QAction *m_run_stereo_action;
  QAction *m_run_parallel_stereo_action;
  QAction *m_exit_action;
  QAction *m_profileMode_action;
  QAction *m_polyEditMode_action;

}; // End struct WindowMenuMgr

} // End namespace asp

#endif  // __STEREO_GUI_WINDOW_MENU_MGR_H__
