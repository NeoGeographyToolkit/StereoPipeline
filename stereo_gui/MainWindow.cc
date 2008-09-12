#include <QtGui>

#include "MainWindow.h"
#include "InputWidget.h"
#include "PreprocessWidget.h"
#include "PreviewGLWidget.h"
#include <vw/FileIO.h>
#include <vw/Image.h>

MainWindow::MainWindow(int /* argc */, char** /* argv */) {

  // Set up the basic layout of the window and its menus
  create_actions();
  create_menus();
  create_status_bar();
  
  // Set the window title and add tabs
  this->setWindowTitle("Ames Stereo Pipeline v3.0");
  QTabWidget* tab_widget = new QTabWidget(this);
  tab_widget->addTab(genInputTab(), "Input");
  tab_widget->addTab(genPreprocessTab(), "Preprocess");
  setCentralWidget(tab_widget);
  connect(tab_widget, SIGNAL(currentChanged(int)), this, SLOT(tab_switch(int)));

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
}

void MainWindow::update_status_bar(std::string const& s) {
  status_label->setText(QString(s.c_str()));
}

void MainWindow::about() {
  QMessageBox::about(this, tr("About Stereo Pipeline"),
                     tr("<h2>NASA Ames Stereo Pipeline 3.0</h2>"
                        "<p>Copyright &copy; 2008 NASA Ames Research Center</p>"));
                       
}


//********************************************************************
//                      MAIN WINDOW TABS
//********************************************************************
QWidget *MainWindow::genInputTab() {
  // Set up the preprocess widgets for each input image
  InputWidget *left = new InputWidget("Left Image", this);
  left->show();

  InputWidget *right = new InputWidget("Right Image", this);
  right->show();

  QHBoxLayout *layout = new QHBoxLayout;
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(left);
  layout->addWidget(right);

  QWidget *widget = new QWidget;
  widget->setLayout(layout);
  return widget;
}


QWidget *MainWindow::genPreprocessTab() {
  // Set up the preprocess widgets for each input image
  PreprocessWidget *left = new PreprocessWidget("Left Image", this);
  left->show();

  PreprocessWidget *right = new PreprocessWidget("Right Image", this);
  right->show();

  QHBoxLayout *layout = new QHBoxLayout;
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(left);
  layout->addWidget(right);

  QWidget *widget = new QWidget;
  widget->setLayout(layout);
  return widget;
}

