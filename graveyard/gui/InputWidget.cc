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
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Stereo.h>

using namespace vw;

#include "gui/InputWidget.h"
#include "gui/PreviewGLWidget.h"
#include "gui/QCompatFormLayout.h"

InputWidget::InputWidget(QWidget *parent) : QWidget(parent) {

  // Image Previews
  m_left_preview = new PreviewGLWidget(this);
  m_right_preview = new PreviewGLWidget(this);
  QHBoxLayout *preview_layout = new QHBoxLayout;
  preview_layout->addWidget(m_left_preview);
  preview_layout->addWidget(m_right_preview);
  QWidget *previews = new QWidget(this);
  previews->setLayout(preview_layout);

  // Left input controls
  QLabel *file_label = new QLabel("Filename: ");
  m_left_filename_edit = new QLineEdit;
  QPushButton *left_pushbutton = new QPushButton("Browse");
  QHBoxLayout *left_control_layout = new QHBoxLayout;
  left_control_layout->addWidget(file_label);
  left_control_layout->addWidget(m_left_filename_edit);
  left_control_layout->addWidget(left_pushbutton);
  QGroupBox *left_control_box = new QGroupBox("Left Image");
  left_control_box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  left_control_box->setLayout(left_control_layout);

  // Right input controls
  m_right_filename_edit = new QLineEdit;
  QPushButton *right_pushbutton = new QPushButton("Browse");
  QHBoxLayout *right_control_layout = new QHBoxLayout;
  right_control_layout->addWidget(file_label);
  right_control_layout->addWidget(m_right_filename_edit);
  right_control_layout->addWidget(right_pushbutton);
  QGroupBox *right_control_box = new QGroupBox("Right Image");
  right_control_box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  right_control_box->setLayout(right_control_layout);

  // Overall controls layout
  QHBoxLayout *controls_layout = new QHBoxLayout;
  controls_layout->addWidget(left_control_box);
  controls_layout->addWidget(right_control_box);
  QWidget *controls = new QWidget(this);
  controls->setLayout(controls_layout);

  // Main frame layout
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(previews);
  main_layout->addWidget(controls);
  this->setLayout(main_layout);

  connect(left_pushbutton, SIGNAL(clicked()), this, SLOT( left_browse_button_clicked() ));
  connect(right_pushbutton, SIGNAL(clicked()), this, SLOT( right_browse_button_clicked() ));
  connect(m_left_filename_edit, SIGNAL(returnPressed()), this, SLOT( reload_left_image() ));
  connect(m_right_filename_edit, SIGNAL(returnPressed()), this, SLOT( reload_right_image() ));
}

void InputWidget::left_browse_button_clicked() {
  QString filename = QFileDialog::getOpenFileName(this, "Open...", "", "Images (*.png *.jpg *.tif *.cub *.img)");

  if (filename != "") {
    m_left_filename_edit->setText(filename);
    load_left_image(filename.toStdString());
  }
}

void InputWidget::right_browse_button_clicked() {
  QString filename = QFileDialog::getOpenFileName(this, "Open...", "", "Images (*.png *.jpg *.tif *.cub *.img)");

  if (filename != "") {
    m_right_filename_edit->setText(filename);
    load_right_image(filename.toStdString());
  }
}


void InputWidget::reload_left_image() {
  std::string filename = m_left_filename_edit->text().toStdString();
  load_left_image(filename);
}

void InputWidget::reload_right_image() {
  std::string filename = m_right_filename_edit->text().toStdString();
  load_right_image(filename);
}

void InputWidget::load_left_image(std::string filename) {
  
  // Bail if the filename was empty, although maybe we should clear the input instead?
  if (filename == "") 
    return;
  
  m_left_filename_edit->setText(filename.c_str());

  try {
    m_left_preview->load_image_from_file(filename);

    std::list<Vector2> points;
    points.push_back(Vector2(100,100));
    points.push_back(Vector2(200,100));
    points.push_back(Vector2(200,200));
    points.push_back(Vector2(100,200));
    m_left_preview->add_crosshairs(points, Vector3(1.0,0.0,0.0));
    
    m_left_preview->sizeToFit();
  } catch(vw::Exception& e) {
    QMessageBox::critical(this, "Error opening image", e.what());
    m_left_filename_edit->clear();
  }
}


void InputWidget::load_right_image(std::string filename) {
  // Bail if the filename was empty, although maybe we should clear the input instead?
  if (filename == "") 
    return;
  
  m_right_filename_edit->setText(filename.c_str());

  try {
    DiskImageView<PixelRGB<float32> > input_image(filename);
    m_right_preview->setImage(input_image);

    std::list<Vector2> points;
    points.push_back(Vector2(100,100));
    points.push_back(Vector2(200,100));
    points.push_back(Vector2(200,200));
    points.push_back(Vector2(100,200));
    m_right_preview->add_crosshairs(points, Vector3(1.0,0.0,0.0));
    
    m_right_preview->sizeToFit();
  } catch(vw::Exception& e) {
    QMessageBox::critical(this, "Error opening image", e.what());
    m_right_filename_edit->clear();
  }
}


