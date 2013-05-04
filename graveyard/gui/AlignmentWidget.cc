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

#include "StereoSettings.h"
#include "gui/StereoGuiSession.h"
#include "gui/AlignmentWidget.h"
#include "gui/PreviewGLWidget.h"
#include "gui/QCompatFormLayout.h"

AlignmentWidget::AlignmentWidget(QWidget *parent) : QWidget(parent) {

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(generate_previews());
  mainLayout->addWidget(generate_controls());
  this->setLayout(mainLayout);
  
}

QWidget* AlignmentWidget::generate_previews() {

  // Comparison for the two aligned images (in two separate windows)
  m_left_preview = new PreviewGLWidget(this);
  m_right_preview = new PreviewGLWidget(this);
  QHBoxLayout *align_preview_layout = new QHBoxLayout;
  align_preview_layout->addWidget(m_left_preview);
  align_preview_layout->addWidget(m_right_preview);
  QWidget *aligned_preview = new QWidget(this);
  aligned_preview->setLayout(align_preview_layout);

  // Comparison for the two aligned images (in the same window)
  m_compare_preview = new PreviewGLWidget(this);

  // Comparison for the two original images (with interest point overlay)
  m_left_ip_preview = new PreviewGLWidget(this);
  m_right_ip_preview = new PreviewGLWidget(this);
  QHBoxLayout *align_ip_layout = new QHBoxLayout;
  align_ip_layout->addWidget(m_left_ip_preview);
  align_ip_layout->addWidget(m_right_ip_preview);
  QWidget *ip_preview = new QWidget(this);
  ip_preview->setLayout(align_ip_layout);

  // The tab widget allows the user to cycle through the three views
  // above.
  QTabWidget *tab = new QTabWidget(this);
  tab->setTabPosition(QTabWidget::South);
  tab->addTab(aligned_preview, "Aligned");
  tab->addTab(m_compare_preview, "Compare");
  tab->addTab(ip_preview, "Interest Points");

  // Image Previews Group Box
  QHBoxLayout *master_layout = new QHBoxLayout;
  master_layout->addWidget(tab);
  QGroupBox *previews = new QGroupBox("Image Previews");
  previews->setLayout(master_layout);

  // Set up the signal and slot connections
  connect(stereo_gui_session().qobject_ptr(), SIGNAL(left_aligned_image_changed(std::string)), m_left_preview, SLOT( load_image_from_file(std::string) ));
  connect(stereo_gui_session().qobject_ptr(), SIGNAL(right_aligned_image_changed(std::string)), m_right_preview, SLOT( load_image_from_file(std::string) ));

  return previews;
}

QWidget* AlignmentWidget::generate_controls() {

  // Set up feature-based alignment group box
  QVBoxLayout *feature_layout = new QVBoxLayout;
  QPushButton *feature_button = new QPushButton("Align Images");
  feature_button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  feature_layout->addWidget(new QLabel("Description of feature based alignment..."));
  feature_layout->addWidget(feature_button);

  QWidget* feature_widget = new QWidget(this);
  feature_widget->setLayout(feature_layout);
  
  // Set up orthoimage alignment group box
  QVBoxLayout *orthoimage_layout = new QVBoxLayout;
  QPushButton *orthoimage_button = new QPushButton("Align Images");
  orthoimage_button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  orthoimage_layout->addWidget(new QLabel("Description of orthoimage based alignment..."));
  orthoimage_layout->addWidget(orthoimage_button);

  QWidget *orthoimage_widget = new QWidget(this);
  orthoimage_widget->setLayout(orthoimage_layout);

  // Set up tabe widget for both control boxes
  QTabWidget *controls = new QTabWidget(this);
  controls->setTabPosition(QTabWidget::South); 
  controls->addTab(feature_widget, "Feature-based Alignment");
  controls->addTab(orthoimage_widget, "Orthoimage-based Alignment");
  controls->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  // Set up the signal and slot connections
  connect(feature_button, SIGNAL(clicked()), this, SLOT( feature_button_clicked() ));
  connect(orthoimage_button, SIGNAL(clicked()), this, SLOT( orthoimage_button_clicked() ));

  return controls;
}


void AlignmentWidget::feature_button_clicked() {
  std::string pre_preprocess_file1, pre_preprocess_file2;
  stereo_gui_session().session()->pre_preprocessing_hook(stereo_gui_session().left_input_image(), 
                                                         stereo_gui_session().right_input_image(),
                                                         pre_preprocess_file1, pre_preprocess_file2);

  std::cout << "\nGenerating image masks..." << std::flush;
  DiskImageView<PixelGray<uint8> > left_rectified_image(pre_preprocess_file1);
  DiskImageView<PixelGray<uint8> > right_rectified_image(pre_preprocess_file2);
  int mask_buffer = std::max(stereo_settings().h_kern, stereo_settings().v_kern);
  ImageViewRef<uint8> Lmask = channel_cast_rescale<uint8>(stereo::disparity::generate_mask(left_rectified_image, mask_buffer));
  ImageViewRef<uint8> Rmask = channel_cast_rescale<uint8>(stereo::disparity::generate_mask(right_rectified_image, mask_buffer));
  std::cout << "Done.\n";
  write_image(stereo_gui_session().output_prefix() + "-lMask.tif", Lmask);
  write_image(stereo_gui_session().output_prefix() + "-rMask.tif", Rmask);
  
  stereo_gui_session().set_left_aligned_image(pre_preprocess_file1);
  stereo_gui_session().set_right_aligned_image(pre_preprocess_file2);
}

void AlignmentWidget::orthoimage_button_clicked() {
  
}
