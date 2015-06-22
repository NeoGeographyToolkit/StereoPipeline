// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file stereo_gui_MainWindow.h
///
/// The Vision Workbench image viewer main window class.
///
#ifndef __STEREO_GUI_MAINWINDOW_H__
#define __STEREO_GUI_MAINWINDOW_H__

#include <QMainWindow>
#include <string>
#include <vector>

// Boost
#include <boost/program_options.hpp>

// Forward declarations
class QAction;
class QLabel;
class QTabWidget;

namespace vw {
namespace gui {

  class MainWidget;
  class chooseFilesDlg;

  class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(std::vector<std::string> const& images, std::string const& geom,
               bool ignore_georef, bool hillshade, int argc, char ** argv);
    virtual ~MainWindow() {}

  private slots:
    void forceQuit(); // Ensure the program shuts down.
    void size_to_fit();
    void run_stereo();
    void about();

  protected:
    void keyPressEvent(QKeyEvent *event);

  private:

    void create_menus();

    // Event handlers
    void resizeEvent(QResizeEvent *);
    void closeEvent (QCloseEvent *);

    std::vector<std::string> m_images;
    double           m_widRatio;    // ratio of sidebar to entire win wid
    MainWidget     * m_main_widget;
    MainWidget     * m_left_widget;
    MainWidget     * m_right_widget;
    chooseFilesDlg * m_chooseFiles; // left sidebar for selecting files

    QMenu *m_file_menu;
    QMenu *m_view_menu;
    QMenu *m_help_menu;

    QAction *m_about_action;
    QAction *m_size_to_fit_action;
    QAction *m_run_stereo_action;
    QAction *m_exit_action;

    int m_argc;
    char ** m_argv;
  };

}} // namespace vw::gui

#endif // __STEREO_GUI_MAINWINDOW_H__
