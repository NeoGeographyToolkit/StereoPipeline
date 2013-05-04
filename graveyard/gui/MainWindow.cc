// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


#include <QtGui>

#include "gui/StereoGuiSession.h"
#include "gui/ProgressBar.h"
#include "gui/MainWindow.h"
#include "gui/InputWidget.h"
#include "gui/AlignmentWidget.h"
// #include "gui/PreprocessWidget.h"
// #include "gui/CostFunctionWidget.h"
// #include "gui/SearchWindowWidget.h"
// #include "gui/CorrelationWidget.h"
#include <vw/FileIO.h>
#include <vw/Image.h>

MainWindow::MainWindow() {

  // Set up the basic layout of the window and its menus
  create_actions();
  create_menus();
  create_status_bar();
  
  // Set the window title and add tabs
  this->setWindowTitle("Ames Stereo Pipeline v2.0");
  m_tab_widget = new QTabWidget(this);
  m_tab_widget->addTab(genInputTab(), "Input");
  m_tab_widget->addTab(genAlignmentTab(), "Alignment");
//   m_tab_widget->addTab(genPreprocessTab(), "Preprocess");
//   m_tab_widget->addTab(genCostFunctionTab(), "Cost Function");
//   m_tab_widget->addTab(genCorrelateTab(), "Correlate");
  setCentralWidget(m_tab_widget);
  connect(m_tab_widget, SIGNAL(currentChanged(int)), this, SLOT(tab_switch(int)));

  // Maximize the main window
  this->showMaximized();
}

//********************************************************************
//                      MAIN WINDOW SETUP
//********************************************************************
void MainWindow::tab_switch(int index) {
  switch (index) {
  case 0:    // Input
    
    break;
  case 1:    // Preprocessing
    
    break;
  }
}

void MainWindow::create_actions() {

  // The About Box
  about_action = new QAction(tr("About StereoPipeline"), this);
  about_action->setStatusTip(tr("Show the StereoPipeline about box"));
  connect(about_action, SIGNAL(triggered()), this, SLOT(about()));

  // Exit or Quit
  exit_action = new QAction(tr("E&xit"), this);
  exit_action->setShortcut(tr("Ctrl+Q"));
  exit_action->setStatusTip(tr("Exit the application"));
  connect(exit_action, SIGNAL(triggered()), this, SLOT(close()));
}

void MainWindow::create_menus() {
  
  // File Menu
  file_menu = menuBar()->addMenu(tr("&File"));
  file_menu->addAction(exit_action);

  // Edit Menu
  edit_menu = menuBar()->addMenu(tr("&Edit"));

  // Help menu
  menuBar()->addSeparator();
  help_menu = menuBar()->addMenu(tr("&Help"));
  help_menu->addAction(about_action);
}

void MainWindow::create_status_bar() {
  status_label = new QLabel("Welcome to the Stereo Pipeline");
  status_label->setAlignment(Qt::AlignHCenter);
  statusBar()->addWidget(status_label);

  // WARNING: Memory leak as currently written.  Fix this somewhow...
  StereoGuiProgressCallback *clbk = new StereoGuiProgressCallback(this, "Testing");
  statusBar()->addWidget(clbk->widget());
}

void MainWindow::update_status_bar(std::string const& s) {
  status_label->setText(QString(s.c_str()));
}

void MainWindow::about() {
  QMessageBox::about(this, tr("About Stereo Pipeline"),
                     tr("<h2>NASA Ames Stereo Pipeline 2.0</h2>"
                        "<p>Copyright &copy; 2008 NASA Ames Research Center</p>"));
                       
}

void MainWindow::keyPressEvent(QKeyEvent *event) {

  int idx;
  switch (event->key()) {
  case Qt::Key_Right:   
    idx = m_tab_widget->currentIndex();
    if (++idx >= m_tab_widget->count())
      idx = 0;
    m_tab_widget->setCurrentIndex(idx);
    break;

  case Qt::Key_Left: 
    idx = m_tab_widget->currentIndex();
    if (--idx < 0)
      idx = m_tab_widget->count()-1;
    m_tab_widget->setCurrentIndex(idx);
    break;
  default: 
    QWidget::keyPressEvent(event);
  }
}


//********************************************************************
//                      MAIN WINDOW TABS
//********************************************************************
QWidget *MainWindow::genInputTab() {

  // Set up the input widget
  InputWidget *w = new InputWidget(this);

  // If the user has supplied input filenames on the command line, we
  // pass them along here so that they can be automatically loaded.
  if (stereo_gui_session().left_input_image() != "") 
    w->load_left_image(stereo_gui_session().left_input_image() );
  if (stereo_gui_session().right_input_image() != "") 
    w->load_right_image(stereo_gui_session().right_input_image() );
  w->show();

  return w;
}

QWidget *MainWindow::genAlignmentTab() {

  // Set up the alignment widget for each input image
  AlignmentWidget *widget = new AlignmentWidget(this);
  widget->show();

  return widget;
}


// QWidget *MainWindow::genPreprocessTab() {
//   // Set up the preprocess widgets for each input image
//   PreprocessWidget *left = new PreprocessWidget("Left Image", this);
//   left->show();

//   PreprocessWidget *right = new PreprocessWidget("Right Image", this);
//   right->show();

//   QHBoxLayout *layout = new QHBoxLayout;
//   layout->setContentsMargins(0, 0, 0, 0);
//   layout->addWidget(left);
//   layout->addWidget(right);

//   QWidget *widget = new QWidget;
//   widget->setLayout(layout);
//   return widget;
// }

// QWidget *MainWindow::genCostFunctionTab() {
//   CostFunctionWidget *costPreview = new CostFunctionWidget(this);
//   SearchWindowWidget *searchPreview = new SearchWindowWidget(NULL, this);
  
//   QHBoxLayout *layout = new QHBoxLayout;
//   layout->addWidget(costPreview);
//   layout->addWidget(searchPreview);

//   QWidget *widget = new QWidget;
//   widget->setLayout(layout);

//   return widget;
// }

// QWidget *MainWindow::genCorrelateTab() {
//   CorrelationWidget *correlationPreview = new CorrelationWidget(NULL, this);
//   SearchWindowWidget *searchPreview = new SearchWindowWidget(NULL, this);

//   QHBoxLayout *layout = new QHBoxLayout;
//   layout->addWidget(correlationPreview);
//   layout->addWidget(searchPreview);

//   QWidget *widget = new QWidget;
//   widget->setLayout(layout);

//   return widget;
// }


