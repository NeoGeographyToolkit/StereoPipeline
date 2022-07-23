// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


/// \file MainWindow.h
///
///
#ifndef __STEREO_GUI_MAINWINDOW_H__
#define __STEREO_GUI_MAINWINDOW_H__

#include <QMainWindow>
#include <string>
#include <vector>

// Boost
#include <boost/program_options.hpp>

#include <vw/Math/Vector.h>
#include <vw/InterestPoint/InterestData.h>
#include <asp/Core/Common.h>
#include <asp/GUI/GuiUtilities.h>

// Forward declarations
class QAction;
class QLabel;
class QTabWidget;
class QSplitter;

namespace vw { namespace gui {

  enum ViewType {VIEW_SIDE_BY_SIDE, VIEW_IN_SINGLE_WINDOW, VIEW_AS_TILES_ON_GRID};

  class MainWidget;
  class chooseFilesDlg;

  /// This class handles the menues at the top bar and other application level details.
  class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(vw::GdalWriteOptions const& opt,
               std::vector<std::string> const& images,
               std::string& output_prefix, // non-const, so we can change it
               int grid_cols,
               vw::Vector2i const& window_size, bool single_window,
               bool use_georef, bool hillshade, bool delete_temporary_files_on_exit,
               int argc, char ** argv);
    virtual ~MainWindow() {}

  private slots:
    void forceQuit                  (); // Ensure the program shuts down.
    void sizeToFit                  ();
    void viewSingleWindow           ();
    void viewSideBySide             ();
    void viewAsTiles                ();
    void viewMatchesFromMenu();
    void toggleViewMatches          ();
    void zoomAllToSameRegionAction(int widget_id);
    void viewMatches();
    void viewPairwiseMatchesSlot();
    void viewPairwiseCleanMatchesSlot();
    void addDelMatches              ();
    void saveMatches                ();
    void writeGroundControlPoints   (); ///< Write a ground control point file for bundle_adjust
    void save_screenshot            ();
    void select_region              ();
    void change_cursor              ();
    void run_stereo                 ();
    void run_parallel_stereo        ();
    void thresholdCalc              ();
    void thresholdGetSet            ();
    void setPolyColor               ();
    void setLineWidth               ();
    void viewThreshImages           ();
    void contourImages              ();
    void saveVectorLayer            ();
    void viewHillshadedImages       ();
    void viewGeoreferencedImages    ();
    void overlayGeoreferencedImages ();
    void setZoomAllToSameRegion     ();
    void setZoomAllToSameRegionAux(bool do_zoom);
    void viewNextImage              ();
    void viewPrevImage              ();

    void profileMode                ();
    void polyEditMode               ();
    void uncheckProfileModeCheckbox ();
    void uncheckPolyEditModeCheckbox();
    void about                      ();

  protected:
    void keyPressEvent(QKeyEvent *event);
    bool eventFilter(QObject *obj, QEvent *e);


  private slots:
  void perhapsCreateLayout(); 

  private:

    void createLayout();
    void run_stereo_or_parallel_stereo(std::string const& cmd);

    /// Go through m_matches and retain only IPs detected in the first image.
    /// - If require_all is set, only keep IPs detected in all images.
    size_t consolidate_matches(bool require_all = true);

    void createMenus();

    // Event handlers
    void resizeEvent(QResizeEvent *);
    void closeEvent (QCloseEvent  *);

    // See if in the middle of editing matches
    bool editingMatches() const;

    void updateMatchesMenuEntries();
    
    bool sanityChecks(int num_images);

    void viewPairwiseMatchesOrCleanMatches();
    
    vw::GdalWriteOptions m_opt;
    std::string               m_output_prefix;
    double                    m_widRatio;    // ratio of sidebar to entire win wid
    std::vector<MainWidget*>  m_widgets;     // one of these for each seperate image pane.
    chooseFilesDlg *          m_chooseFiles; // left sidebar for selecting files

    QMenu *m_file_menu;
    QMenu *m_view_menu;
    QMenu *m_matches_menu;
    QMenu *m_threshold_menu;
    QMenu *m_profile_menu;
    QMenu *m_vector_layer_menu;
    QMenu *m_help_menu;

    QAction *m_about_action;
    QAction *m_thresholdCalc_action;
    QAction *m_thresholdGetSet_action;
    QAction *m_setLineWidth_action;
    QAction *m_setPolyColor_action;
    QAction *m_sizeToFit_action;
    QAction *m_viewSingleWindow_action;
    QAction *m_viewSideBySide_action;
    QAction *m_viewAsTiles_action;
    QAction *m_viewHillshadedImages_action;
    QAction *m_viewGeoreferencedImages_action;
    QAction *m_overlayGeoreferencedImages_action;
    QAction *m_viewThreshImages_action;
    QAction *m_contourImages_action;
    QAction *m_saveVectorLayer_action;
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

    ViewType m_view_type,
             m_view_type_old;
    int      m_grid_cols, m_grid_cols_old;
    bool     m_use_georef, m_view_thresholded,
      m_delete_temporary_files_on_exit;
    bool     m_allowMultipleSelections;
    int      m_argc;
    char **  m_argv;
    bool     m_matches_exist;
    DisplayMode m_display_mode;
    
    std::vector<std::string>  m_image_files; // Loaded image files
    std::vector<imageData>    m_images;      // Handles to loaded images

    /// Structures to keep track of all interest point matches.
    MatchList         m_matchlist;
    pairwiseMatchList m_pairwiseMatches;
    pairwiseMatchList m_pairwiseCleanMatches;
    
    int       m_editMatchPointVecIndex; ///< Point being edited

    int m_cursor_count;

    // This will be set to true each time we enter the mode when images
    // are shown side-by-side with a dialog. The user can later choose
    // to show any number.
    bool m_show_two_images_when_side_by_side_with_dialog;
  };

}} // namespace vw::gui

#endif // __STEREO_GUI_MAINWINDOW_H__
