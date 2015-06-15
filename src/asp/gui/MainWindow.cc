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


/// \file stereo_gui_MainWindow.cc
///
/// The Vision Workbench image viewer main window class.
///
#include <QtGui>
#include <vw/gui/MainWindow.h>
#include <vw/gui/MainWidget.h>
#include <vw/gui/Utils.h>
using namespace vw::gui;

#include <vw/config.h>
#include <vw/Image/MaskViews.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/PixelMask.h>

#include <sstream>
namespace po = boost::program_options;

MainWindow::MainWindow(std::vector<std::string> const& images,
                       std::string const& geom,
                       bool ignore_georef, bool hillshade) :
  m_images(images), m_widRatio(0.3), m_main_widget(NULL),
  m_chooseFiles(NULL) {

  int windowWidX, windowWidY;
  extractWindowDims(geom, windowWidX, windowWidY);
  resize(windowWidX, windowWidY);

  // Set the window title and add tabs
  std::string window_title = "Vision Workbench Viewer";
  this->setWindowTitle(window_title.c_str());

  // Set up the basic layout of the window and its menus.
  create_menus();

  if (images.size() > 1){

    // Split the window into two, with the sidebar on the left

    QWidget * centralFrame;
    centralFrame = new QWidget(this);
    setCentralWidget(centralFrame);

    QSplitter *splitter = new QSplitter(centralFrame);
#if 0
    m_chooseFiles = new chooseFilesDlg(this);
    m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());
    m_main_widget = new MainWidget(centralFrame, images, m_chooseFiles,
                                   ignore_georef, hillshade);
    splitter->addWidget(m_chooseFiles);
    splitter->addWidget(m_main_widget);
#else
    // for doing stereo
    std::vector<std::string> left_images, right_images;
    left_images.push_back(images[0]);
    right_images.push_back(images[1]);
    m_left_widget = new MainWidget(centralFrame, left_images, m_chooseFiles,
                                   ignore_georef, hillshade);
    m_right_widget = new MainWidget(centralFrame, right_images, m_chooseFiles,
                                   ignore_georef, hillshade);

    splitter->addWidget(m_left_widget);
    splitter->addWidget(m_right_widget);
#endif
    QGridLayout *layout = new QGridLayout(centralFrame);
    layout->addWidget (splitter, 0, 0, 0);
    centralFrame->setLayout (layout);
  }else{
    // Set up MainWidget
    m_main_widget = new MainWidget(this, images, NULL, ignore_georef, hillshade);
    setCentralWidget(m_main_widget);
  }

}

//********************************************************************
//                      MAIN WINDOW SETUP
//********************************************************************

void MainWindow::create_menus() {

  QMenuBar* menu = menuBar();

  // Exit or Quit
  m_exit_action = new QAction(tr("Exit"), this);
  m_exit_action->setShortcut(tr("Q"));
  m_exit_action->setStatusTip(tr("Exit the application"));
  connect(m_exit_action, SIGNAL(triggered()), this, SLOT(forceQuit()));

  // Size to fit
  m_size_to_fit_action = new QAction(tr("Size to fit"), this);
  m_size_to_fit_action->setStatusTip(tr("Change the view to encompass the images."));
  connect(m_size_to_fit_action, SIGNAL(triggered()), this, SLOT(size_to_fit()));
  m_size_to_fit_action->setShortcut(tr("F"));

  // The About Box
  m_about_action = new QAction(tr("About stereo_gui"), this);
  m_about_action->setStatusTip(tr("Show the stereo_gui about box."));
  connect(m_about_action, SIGNAL(triggered()), this, SLOT(about()));

  // File menu
  m_file_menu = menu->addMenu(tr("&File"));
  m_file_menu->addAction(m_exit_action);

  // View menu
  menu->addSeparator();
  m_view_menu = menu->addMenu(tr("&View"));
  m_view_menu->addAction(m_size_to_fit_action);

  // Help menu
  menu->addSeparator();
  m_help_menu = menu->addMenu(tr("&Help"));
  m_help_menu->addAction(m_about_action);
}

void MainWindow::resizeEvent(QResizeEvent *){
  if (m_chooseFiles)
    m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());
}

void MainWindow::closeEvent(QCloseEvent *){
  forceQuit();
}

void MainWindow::forceQuit(){
  exit(0); // A fix for an older buggy version of Qt
}

void MainWindow::size_to_fit(){
  if (m_main_widget)
    m_main_widget->size_to_fit();
}

void MainWindow::about() {
  std::ostringstream about_text;
  about_text << "<h3>Vision Workbench Image Viewer (stereo_gui)</h3>"
             << "<p>Version " << VW_PACKAGE_VERSION << "</p>"
             << "<p>Copyright &copy; 2014 NASA Ames Research Center</p>";
  QMessageBox::about(this, tr("About Vision Workbench Viewer"),
                     tr(about_text.str().c_str()));

}

void MainWindow::keyPressEvent(QKeyEvent *event) {

  std::ostringstream s;

  switch (event->key()) {
  case Qt::Key_Escape:  // Quit
    close();
    break;
  }
}
