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
#include <cmath>

#include "gui/CostFunctionWidget.h"
#include "gui/QCompatFormLayout.h"
#include "gui/PreviewGLWidget.h"

CostFunctionWidget::CostFunctionWidget(QWidget *parent) : QWidget(parent) {
  this->blockSignals(true);

  m_imagePreview = new PreviewGLWidget(this);
  m_leftImagePreview = new PreviewGLWidget(this);

  QTabWidget *previewTab = new QTabWidget;
  previewTab->addTab(m_imagePreview, "Cost Landscape");
  previewTab->addTab(m_leftImagePreview, "Left Image");

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(previewTab);
  mainLayout->addWidget(genSettingsBox("Cost Function Preview Settings"));

  this->setLayout(mainLayout);

  connect(m_xDisparitySlider, SIGNAL(valueChanged(int)), m_xDisparitySpin, SLOT(setValue(int)));
  connect(m_yDisparitySlider, SIGNAL(valueChanged(int)), m_yDisparitySpin, SLOT(setValue(int)));
  connect(m_xDisparitySpin, SIGNAL(valueChanged(int)), m_xDisparitySlider, SLOT(setValue(int)));
  connect(m_yDisparitySpin, SIGNAL(valueChanged(int)), m_yDisparitySlider, SLOT(setValue(int)));
  
//   connect(m_xDisparitySpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setXDisparityPreview(int)));
//   connect(m_yDisparitySpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setYDisparityPreview(int)));

//   connect(m_cs, SIGNAL(xDisparityPreviewChanged(int)), m_xDisparitySpin, SLOT(setValue(int)));
//   connect(m_cs, SIGNAL(yDisparityPreviewChanged(int)), m_yDisparitySpin, SLOT(setValue(int)));

//   connect(m_cs, SIGNAL(disparityPreviewChanged(int, int)), this, SLOT(recalculateCost()));

//   connect(m_cs, SIGNAL(costFunctionObjectChanged(boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >)), this, SLOT(recalculateCost()));
  
  connect(m_imagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));
  connect(m_leftImagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));

  connect(previewTab, SIGNAL(currentChanged(int)), m_imagePreview, SLOT(fitToWindow()));
  connect(previewTab, SIGNAL(currentChanged(int)), m_leftImagePreview, SLOT(fitToWindow()));
}

QGroupBox *CostFunctionWidget::genSettingsBox(QString const& name) {
  QCompatFormLayout *layout = new QCompatFormLayout;
  
  QLabel *xDisparityLabel = new QLabel("X Disparity:");
  layout->addRow(xDisparityLabel, genXDisparityKnobs());
  
  QLabel *yDisparityLabel = new QLabel("Y Disparity:");
  layout->addRow(yDisparityLabel, genYDisparityKnobs());
  
  QGroupBox *box =  new QGroupBox(name);
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  box->setLayout(layout);
  return box;
}

QHBoxLayout *CostFunctionWidget::genXDisparityKnobs() {
  m_xDisparitySpin = new QSpinBox;
  m_xDisparitySpin->setValue(0);
  m_xDisparitySpin->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  
  m_xDisparitySlider = new QSlider(Qt::Horizontal);
  m_xDisparitySlider->setValue(0);
  m_xDisparitySlider->setPageStep(1);

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(m_xDisparitySlider);
  layout->addWidget(m_xDisparitySpin);
  return layout;
}

QHBoxLayout *CostFunctionWidget::genYDisparityKnobs() {
  m_yDisparitySpin = new QSpinBox;
  m_yDisparitySpin->setValue(0);
  m_yDisparitySpin->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  
  m_yDisparitySlider = new QSlider(Qt::Horizontal);
  m_yDisparitySlider->setValue(0);
  m_yDisparitySlider->setPageStep(1);

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(m_yDisparitySlider);
  layout->addWidget(m_yDisparitySpin);
  return layout;
}

void CostFunctionWidget::recalculateCost() {
//   if (m_cs->hasBothImagesLoaded()) {
//     int width = m_cs->leftImage().cols();
//     int height = m_cs->leftImage().rows();
//     int dx = m_xDisparitySpin->value();
//     int dy = m_yDisparitySpin->value();

//     vw::BBox2i corr_window((dx < 0) ? (-dx) : 0,
//                            (dy < 0) ? (-dy) : 0,
//                            width - abs(dx),
//                            height - abs(dy));
    
//     vw::ImageView<vw::float32> result(width, height);

//     vw::crop(result, corr_window) = m_cs->getCostFunctionObject()->calculate(corr_window, vw::Vector2i(dx, dy));

//     m_imagePreview->setImage(result, true);

//     updatePreview();
//   }
}

void CostFunctionWidget::updatePreview() {
//   m_imagePreview->setCrosshairPosition(m_cs->searchWindowPreviewPoint());
//   m_imagePreview->setCrosshairEnabled(m_cs->crosshairEnabled());
    
//   m_imagePreview->updatePreview(); 

//   m_leftImagePreview->setCrosshairPosition(m_cs->searchWindowPreviewPoint());
//   m_leftImagePreview->setCrosshairEnabled(m_cs->crosshairEnabled());

//   m_leftImagePreview->updatePreview();
}

void CostFunctionWidget::imageClicked(int x, int y) {
//   vw::Vector2i loc(x, y);
  
//   if (loc == m_cs->searchWindowPreviewPoint() && m_cs->crosshairEnabled()) {
//     m_cs->setCrosshairEnabled(false);
//   }
//   else {
//     m_cs->setCrosshairEnabled(true);
//   }

//   m_cs->setSearchWindowPreviewPoint(vw::Vector2i(x, y));

//   updatePreview();
}

void CostFunctionWidget::updateWidgets() {
//   int width = m_cs->leftImage().cols();
//   int height = m_cs->leftImage().rows();

//   m_xDisparitySpin->setRange(-width, width);
//   m_yDisparitySpin->setRange(-height, height);
//   m_xDisparitySlider->setRange(-width, width);
//   m_yDisparitySlider->setRange(-height, height);

//   m_leftImagePreview->setImage(m_cs->leftImage());
  
//   recalculateCost();

//   m_imagePreview->fitToWindow();
}
