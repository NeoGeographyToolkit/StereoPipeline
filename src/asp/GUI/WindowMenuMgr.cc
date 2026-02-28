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

// \file WindowMenuMgr.cc
//
// Handles menu bar creation and management for MainWindow.
//
#include <asp/GUI/WindowMenuMgr.h>
#include <asp/GUI/MainWindow.h>
#include <asp/Core/StereoSettings.h>

#include <QMenuBar>

namespace asp {

void WindowMenuMgr::init(MainWindow* win) {

  QMenuBar* menu = win->menuBar();

  // Exit or Quit
  m_exit_action = new QAction(QObject::tr("Exit"), win);
  m_exit_action->setShortcut(QObject::tr("Q"));
  m_exit_action->setStatusTip(QObject::tr("Exit the application"));
  QObject::connect(m_exit_action, SIGNAL(triggered()), win, SLOT(forceQuit()));

  // Save screenshot
  m_save_screenshot_action = new QAction(QObject::tr("Save screenshot"), win);
  m_save_screenshot_action->setStatusTip(QObject::tr("Save screenshot"));
  QObject::connect(m_save_screenshot_action, SIGNAL(triggered()),
                   win, SLOT(save_screenshot()));

  // Select region
  m_select_region_action = new QAction(QObject::tr("Select region"), win);
  m_select_region_action->setStatusTip(QObject::tr("Select rectangular region"));
  QObject::connect(m_select_region_action, SIGNAL(triggered()),
                   win, SLOT(select_region()));

  // Change cursor shape (a workaround for Qt not setting the cursor correctly in vnc)
  m_change_cursor_action = new QAction(QObject::tr("Change cursor shape"), win);
  m_change_cursor_action->setStatusTip(QObject::tr("Change cursor shape"));
  QObject::connect(m_change_cursor_action, SIGNAL(triggered()),
                   win, SLOT(change_cursor()));
  m_change_cursor_action->setShortcut(QObject::tr("C"));

  // Run parallel_stereo
  m_run_parallel_stereo_action = new QAction(QObject::tr("Run parallel_stereo"), win);
  m_run_parallel_stereo_action->setStatusTip(QObject::tr("Run parallel_stereo on selected clips"));
  QObject::connect(m_run_parallel_stereo_action, SIGNAL(triggered()),
                   win, SLOT(run_parallel_stereo()));
  m_run_parallel_stereo_action->setShortcut(QObject::tr("R"));

  // Run stereo
  m_run_stereo_action = new QAction(QObject::tr("Run stereo"), win);
  m_run_stereo_action->setStatusTip(QObject::tr("Run stereo on selected clips"));
  QObject::connect(m_run_stereo_action, SIGNAL(triggered()),
                   win, SLOT(run_stereo()));

  // Zoom to full view
  m_sizeToFit_action = new QAction(QObject::tr("Zoom to full view"), win);
  m_sizeToFit_action->setStatusTip(QObject::tr("Change the view to encompass the images"));
  QObject::connect(m_sizeToFit_action, SIGNAL(triggered()),
                   win, SLOT(sizeToFit()));
  m_sizeToFit_action->setShortcut(QObject::tr("F"));

  m_viewSingleWindow_action = new QAction(QObject::tr("Single window"), win);
  m_viewSingleWindow_action->setStatusTip(QObject::tr("View images in a single window"));
  m_viewSingleWindow_action->setCheckable(true);
  m_viewSingleWindow_action->setChecked(win->m_view_type == VIEW_IN_SINGLE_WINDOW);
  m_viewSingleWindow_action->setShortcut(QObject::tr("W"));
  QObject::connect(m_viewSingleWindow_action, SIGNAL(triggered()),
                   win, SLOT(viewSingleWindow()));

  m_viewAllSideBySide_action = new QAction(QObject::tr("All side-by-side"), win);
  m_viewAllSideBySide_action->setStatusTip(QObject::tr("View all images side-by-side"));
  m_viewAllSideBySide_action->setCheckable(true);
  m_viewAllSideBySide_action->setChecked(win->m_view_type == VIEW_SIDE_BY_SIDE &&
                                         !sideBySideWithDialog());
  m_viewAllSideBySide_action->setShortcut(QObject::tr("S"));
  QObject::connect(m_viewAllSideBySide_action, SIGNAL(triggered()),
                   win, SLOT(viewAllSideBySide()));

  m_viewSeveralSideBySide_action = new QAction(QObject::tr("Several side-by-side"), win);
  m_viewSeveralSideBySide_action->setStatusTip(QObject::tr("View several images side-by-side"));
  m_viewSeveralSideBySide_action->setCheckable(true);
  m_viewSeveralSideBySide_action->setChecked(
    asp::stereo_settings().view_several_side_by_side);
  QObject::connect(m_viewSeveralSideBySide_action, SIGNAL(triggered()),
                   win, SLOT(viewSeveralSideBySide()));

  m_viewAsTiles_action = new QAction(QObject::tr("As tiles on grid"), win);
  m_viewAsTiles_action->setStatusTip(QObject::tr("View images as tiles on grid"));
  m_viewAsTiles_action->setCheckable(true);
  m_viewAsTiles_action->setChecked(win->m_view_type == VIEW_AS_TILES_ON_GRID);
  m_viewAsTiles_action->setShortcut(QObject::tr("T"));
  QObject::connect(m_viewAsTiles_action, SIGNAL(triggered()),
                   win, SLOT(viewAsTiles()));

  // View hillshaded images
  m_viewHillshadedImages_action = new QAction(QObject::tr("Hillshaded images"), win);
  m_viewHillshadedImages_action->setStatusTip(QObject::tr("View hillshaded images"));
  m_viewHillshadedImages_action->setCheckable(true);
  m_viewHillshadedImages_action->setChecked(
    win->app_data.display_mode == HILLSHADED_VIEW);
  m_viewHillshadedImages_action->setShortcut(QObject::tr("H"));
  QObject::connect(m_viewHillshadedImages_action, SIGNAL(triggered()),
                   win, SLOT(viewHillshadedImages()));

  // View as georeferenced
  m_viewGeoreferencedImages_action =
    new QAction(QObject::tr("View as georeferenced images"), win);
  m_viewGeoreferencedImages_action->setStatusTip(
    QObject::tr("View as georeferenced images"));
  m_viewGeoreferencedImages_action->setCheckable(true);
  m_viewGeoreferencedImages_action->setChecked(win->app_data.use_georef);
  m_viewGeoreferencedImages_action->setShortcut(QObject::tr("G"));
  QObject::connect(m_viewGeoreferencedImages_action, SIGNAL(triggered()),
                   win, SLOT(viewGeoreferencedImages()));

  // View overlaid georeferenced images
  m_overlayGeoreferencedImages_action =
    new QAction(QObject::tr("Overlay georeferenced images"), win);
  m_overlayGeoreferencedImages_action->setStatusTip(
    QObject::tr("Overlay georeferenced images"));
  m_overlayGeoreferencedImages_action->setCheckable(true);
  m_overlayGeoreferencedImages_action->setChecked(
    win->app_data.use_georef &&
    (win->m_view_type == VIEW_IN_SINGLE_WINDOW));
  m_overlayGeoreferencedImages_action->setShortcut(QObject::tr("O"));
  QObject::connect(m_overlayGeoreferencedImages_action, SIGNAL(triggered()),
                   win, SLOT(overlayGeoreferencedImages()));

  // Zoom all images to same region
  m_zoomAllToSameRegion_action =
    new QAction(QObject::tr("Zoom all images to same region"), win);
  m_zoomAllToSameRegion_action->setStatusTip(
    QObject::tr("Zoom all images to same region"));
  m_zoomAllToSameRegion_action->setCheckable(true);
  m_zoomAllToSameRegion_action->setChecked(
    asp::stereo_settings().zoom_all_to_same_region);
  m_zoomAllToSameRegion_action->setShortcut(QObject::tr("Z"));
  QObject::connect(m_zoomAllToSameRegion_action, SIGNAL(triggered()),
                   win, SLOT(setZoomAllToSameRegion()));

  // View next image
  m_viewNextImage_action = new QAction(QObject::tr("View next image"), win);
  m_viewNextImage_action->setStatusTip(QObject::tr("View next image"));
  m_viewNextImage_action->setCheckable(false);
  m_viewNextImage_action->setShortcut(QObject::tr("N"));
  QObject::connect(m_viewNextImage_action, SIGNAL(triggered()),
                   win, SLOT(viewNextImage()));

  // View prev image
  m_viewPrevImage_action = new QAction(QObject::tr("View previous image"), win);
  m_viewPrevImage_action->setStatusTip(QObject::tr("View previous image"));
  m_viewPrevImage_action->setCheckable(false);
  m_viewPrevImage_action->setShortcut(QObject::tr("P"));
  QObject::connect(m_viewPrevImage_action, SIGNAL(triggered()),
                   win, SLOT(viewPrevImage()));

  m_zoomToProjWin_action = new QAction(QObject::tr("Zoom to proj win"), win);
  m_zoomToProjWin_action->setStatusTip(QObject::tr("Zoom to proj win"));
  m_zoomToProjWin_action->setCheckable(false);
  QObject::connect(m_zoomToProjWin_action, SIGNAL(triggered()),
                   win, SLOT(zoomToProjWin()));

  // IP matches
  m_viewMatches_action = new QAction(QObject::tr("View IP matches"), win);
  m_viewMatches_action->setStatusTip(QObject::tr("View IP matches"));
  m_viewMatches_action->setCheckable(true);
  m_viewMatches_action->setChecked(asp::stereo_settings().view_matches);
  QObject::connect(m_viewMatches_action, SIGNAL(triggered()),
                   win, SLOT(viewMatchesFromMenu()));

  m_viewPairwiseMatches_action =
    new QAction(QObject::tr("View pairwise IP matches"), win);
  m_viewPairwiseMatches_action->setStatusTip(QObject::tr("View pairwise IP matches"));
  m_viewPairwiseMatches_action->setCheckable(true);
  m_viewPairwiseMatches_action->setChecked(
    asp::stereo_settings().pairwise_matches);
  QObject::connect(m_viewPairwiseMatches_action, SIGNAL(triggered()),
                   win, SLOT(viewPairwiseMatchesSlot()));

  m_viewPairwiseCleanMatches_action =
    new QAction(QObject::tr("View pairwise clean IP matches"), win);
  m_viewPairwiseCleanMatches_action->setStatusTip(
    QObject::tr("View pairwise clean IP matches"));
  m_viewPairwiseCleanMatches_action->setCheckable(true);
  m_viewPairwiseCleanMatches_action->setChecked(
    asp::stereo_settings().pairwise_clean_matches);
  QObject::connect(m_viewPairwiseCleanMatches_action, SIGNAL(triggered()),
                   win, SLOT(viewPairwiseCleanMatchesSlot()));

  m_addDelMatches_action = new QAction(QObject::tr("Add/delete IP matches"), win);
  m_addDelMatches_action->setStatusTip(QObject::tr("Add/delete interest point matches"));
  QObject::connect(m_addDelMatches_action, SIGNAL(triggered()),
                   win, SLOT(addDelMatches()));

  m_saveMatches_action = new QAction(QObject::tr("Save IP matches"), win);
  m_saveMatches_action->setStatusTip(QObject::tr("Save interest point matches"));
  QObject::connect(m_saveMatches_action, SIGNAL(triggered()),
                   win, SLOT(saveMatches()));

  m_writeGcp_action =
    new QAction(QObject::tr("Save GCP and IP matches"), win);
  m_writeGcp_action->setStatusTip(
    QObject::tr("Save interest point matches as GCP for bundle_adjust"));
  QObject::connect(m_writeGcp_action, SIGNAL(triggered()),
                   win, SLOT(writeGroundControlPoints()));

  // Threshold calculation by clicking on pixels and setting the threshold
  // as the largest determined pixel value
  m_thresholdCalc_action = new QAction(QObject::tr("Threshold detection"), win);
  m_thresholdCalc_action->setStatusTip(QObject::tr("Threshold detection"));
  m_thresholdCalc_action->setCheckable(true);
  QObject::connect(m_thresholdCalc_action, SIGNAL(triggered()),
                   win, SLOT(thresholdCalc()));

  // Thresholded image visualization
  m_viewThreshImages_action =
    new QAction(QObject::tr("View thresholded images"), win);
  m_viewThreshImages_action->setStatusTip(QObject::tr("View thresholded images"));
  m_viewThreshImages_action->setCheckable(true);
  m_viewThreshImages_action->setChecked(
    win->app_data.display_mode == THRESHOLDED_VIEW);
  QObject::connect(m_viewThreshImages_action, SIGNAL(triggered()),
                   win, SLOT(viewThreshImages()));

  // View/set image threshold
  m_thresholdGetSet_action = new QAction(QObject::tr("View/set thresholds"), win);
  m_thresholdGetSet_action->setStatusTip(QObject::tr("View/set thresholds"));
  QObject::connect(m_thresholdGetSet_action, SIGNAL(triggered()),
                   win, SLOT(thresholdGetSet()));

  // 1D profile mode
  m_profileMode_action = new QAction(QObject::tr("1D profile mode"), win);
  m_profileMode_action->setStatusTip(QObject::tr("Profile mode"));
  m_profileMode_action->setCheckable(true);
  m_profileMode_action->setChecked(false);
  QObject::connect(m_profileMode_action, SIGNAL(triggered()),
                   win, SLOT(profileMode()));

  // Polygon edit mode
  m_polyEditMode_action = new QAction(QObject::tr("Polygon edit mode"), win);
  m_polyEditMode_action->setStatusTip(QObject::tr("Polygon edit mode"));
  m_polyEditMode_action->setCheckable(true);
  m_polyEditMode_action->setChecked(false);
  QObject::connect(m_polyEditMode_action, SIGNAL(triggered()),
                   win, SLOT(polyEditMode()));

  // Set line width
  m_setLineWidth_action = new QAction(QObject::tr("Set line width"), win);
  m_setLineWidth_action->setStatusTip(QObject::tr("Set line width"));
  QObject::connect(m_setLineWidth_action, SIGNAL(triggered()),
                   win, SLOT(setLineWidth()));

  // Set color of polygons
  m_setPolyColor_action = new QAction(QObject::tr("Set color of polygons"), win);
  m_setPolyColor_action->setStatusTip(QObject::tr("Set color of polygons"));
  QObject::connect(m_setPolyColor_action, SIGNAL(triggered()),
                   win, SLOT(setPolyColor()));

  // Contour image
  m_contourImages_action =
    new QAction(QObject::tr("Find contour at threshold"), win);
  m_contourImages_action->setStatusTip(QObject::tr("Find contour at threshold"));
  QObject::connect(m_contourImages_action, SIGNAL(triggered()),
                   win, SLOT(contourImages()));

  // Save vector layer as shape file
  m_saveVectorLayerAsShapeFile_action =
    new QAction(QObject::tr("Save vector layer as shapefile"), win);
  m_saveVectorLayerAsShapeFile_action->setStatusTip(
    QObject::tr("Save vector layer as shapefile"));
  QObject::connect(m_saveVectorLayerAsShapeFile_action, SIGNAL(triggered()),
                   win, SLOT(saveVectorLayerAsShapeFile()));

  // Save vector layer as text file
  m_saveVectorLayerAsTextFile_action =
    new QAction(QObject::tr("Save vector layer as text file"), win);
  m_saveVectorLayerAsTextFile_action->setStatusTip(
    QObject::tr("Save vector layer as text file"));
  QObject::connect(m_saveVectorLayerAsTextFile_action, SIGNAL(triggered()),
                   win, SLOT(saveVectorLayerAsTextFile()));

  // The About box
  m_about_action = new QAction(QObject::tr("About stereo_gui"), win);
  m_about_action->setStatusTip(QObject::tr("Show the stereo_gui about box"));
  QObject::connect(m_about_action, SIGNAL(triggered()),
                   win, SLOT(about()));

  // File menu
  m_file_menu = menu->addMenu(QObject::tr("&File"));
  m_file_menu->addAction(m_save_screenshot_action);
  m_file_menu->addAction(m_select_region_action);
  m_file_menu->addAction(m_change_cursor_action);
  m_file_menu->addAction(m_exit_action);

  // Run menu
  m_file_menu = menu->addMenu(QObject::tr("&Run"));
  m_file_menu->addAction(m_run_parallel_stereo_action);
  m_file_menu->addAction(m_run_stereo_action);

  // View menu
  m_view_menu = menu->addMenu(QObject::tr("&View"));
  m_view_menu->addAction(m_sizeToFit_action);
  m_view_menu->addAction(m_viewSingleWindow_action);
  m_view_menu->addAction(m_viewAllSideBySide_action);
  m_view_menu->addAction(m_viewSeveralSideBySide_action);
  m_view_menu->addAction(m_viewAsTiles_action);
  m_view_menu->addAction(m_viewHillshadedImages_action);
  m_view_menu->addAction(m_viewGeoreferencedImages_action);
  m_view_menu->addAction(m_overlayGeoreferencedImages_action);
  m_view_menu->addAction(m_zoomAllToSameRegion_action);
  m_view_menu->addAction(m_viewNextImage_action);
  m_view_menu->addAction(m_viewPrevImage_action);
  m_view_menu->addAction(m_zoomToProjWin_action);

  // Matches menu
  m_matches_menu = menu->addMenu(QObject::tr("&IP matches"));
  m_matches_menu->addAction(m_viewMatches_action);
  m_matches_menu->addAction(m_viewPairwiseMatches_action);
  m_matches_menu->addAction(m_viewPairwiseCleanMatches_action);
  m_matches_menu->addAction(m_addDelMatches_action);
  m_matches_menu->addAction(m_saveMatches_action);
  m_matches_menu->addAction(m_writeGcp_action);

  // Threshold menu
  m_threshold_menu = menu->addMenu(QObject::tr("&Threshold"));
  m_threshold_menu->addAction(m_thresholdCalc_action);
  m_threshold_menu->addAction(m_viewThreshImages_action);
  m_threshold_menu->addAction(m_thresholdGetSet_action);

  // Profile menu
  m_profile_menu = menu->addMenu(QObject::tr("Profile"));
  m_profile_menu->addAction(m_profileMode_action);

  // Vector layer menu
  m_vector_layer_menu = menu->addMenu(QObject::tr("Vector layer"));
  m_vector_layer_menu->addAction(m_polyEditMode_action);
  m_vector_layer_menu->addAction(m_setLineWidth_action);
  m_vector_layer_menu->addAction(m_setPolyColor_action);
  m_vector_layer_menu->addAction(m_contourImages_action);
  m_vector_layer_menu->addAction(m_saveVectorLayerAsShapeFile_action);
  m_vector_layer_menu->addAction(m_saveVectorLayerAsTextFile_action);

  // Help menu
  m_help_menu = menu->addMenu(QObject::tr("&Help"));
  m_help_menu->addAction(m_about_action);
}

} // End namespace asp
