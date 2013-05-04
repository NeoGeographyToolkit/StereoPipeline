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

#include "CorrelationWidget.h"

CorrelationWidget::CorrelationWidget(StereoGuiSession *cs, QWidget *parent) : QWidget(parent) {
  this->blockSignals(true);

  m_cs = cs;

  m_xImagePreview = new PreviewGLWidget(this);
  m_yImagePreview = new PreviewGLWidget(this);
  m_leftImagePreview = new PreviewGLWidget(this);

  QTabWidget *previewTab = new QTabWidget;
  previewTab->addTab(m_xImagePreview, "X Disparity");
  previewTab->addTab(m_yImagePreview, "Y Disparity");
  previewTab->addTab(m_leftImagePreview, "Left Image");

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(previewTab);
  mainLayout->addWidget(genSettingsBox("Search Window Preview Settings"));

  this->setLayout(mainLayout);

  //  m_correlateThread = new QCorrelateThread(this);

//   connect(m_correlateThread, SIGNAL(correlationFinished()), this, SLOT(correlationFinished()));
//   connect(m_correlateThread, SIGNAL(progressUpdate(int)), m_progressBar, SLOT(setValue(int)));
  
//   connect(m_cancelButton, SIGNAL(clicked()), m_correlateThread, SLOT(abortCorrelation()));
  connect(m_cancelButton, SIGNAL(clicked()), m_progressBar, SLOT(reset()));

  connect(m_doCorrelateButton, SIGNAL(clicked()), this, SLOT(doCorrelate()));

  connect(m_xImagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));
  connect(m_yImagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));
  connect(m_leftImagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));

  connect(previewTab, SIGNAL(currentChanged(int)), m_xImagePreview, SLOT(fitToWindow()));
  connect(previewTab, SIGNAL(currentChanged(int)), m_yImagePreview, SLOT(fitToWindow()));
  connect(previewTab, SIGNAL(currentChanged(int)), m_leftImagePreview, SLOT(fitToWindow()));
}

QGroupBox *CorrelationWidget::genSettingsBox(QString const& name) {
  m_doCorrelateButton = new QPushButton("Correlate!!");

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(m_doCorrelateButton);
  layout->addLayout(genProgressLayout());

  QGroupBox *box = new QGroupBox(name);
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  box->setLayout(layout);

  return box;
}

QHBoxLayout *CorrelationWidget::genProgressLayout() {
  m_progressBar = new QProgressBar;
  m_progressBar->setRange(0, 100);
  m_progressBar->setValue(0);

  m_cancelButton = new QPushButton("Cancel");

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(m_progressBar);
  layout->addWidget(m_cancelButton);

  return layout;
}

void CorrelationWidget::doCorrelate() {
  if (m_cs->hasBothImagesLoaded()) {
    //    m_correlateThread->doCorrelate(m_cs->getCostFunctionObject(), m_cs->searchWindow());
  }
}

void CorrelationWidget::updatePreview() {
//   m_xImagePreview->setCrosshairPosition(m_cs->searchWindowPreviewPoint());
//   m_xImagePreview->setCrosshairEnabled(m_cs->crosshairEnabled());
//   m_xImagePreview->updatePreview();
  
//   m_yImagePreview->setCrosshairPosition(m_cs->searchWindowPreviewPoint());
//   m_yImagePreview->setCrosshairEnabled(m_cs->crosshairEnabled());
//   m_yImagePreview->updatePreview();

//   m_leftImagePreview->setCrosshairPosition(m_cs->searchWindowPreviewPoint());
//   m_leftImagePreview->setCrosshairEnabled(m_cs->crosshairEnabled());
//   m_leftImagePreview->updatePreview();
}

void CorrelationWidget::imageClicked(int x, int y) {
  vw::Vector2i loc(x, y);
  
  if (loc == m_cs->searchWindowPreviewPoint() && m_cs->crosshairEnabled()) {
    m_cs->setCrosshairEnabled(false);
  }
  else {
    m_cs->setCrosshairEnabled(true);
  }

  m_cs->setSearchWindowPreviewPoint(vw::Vector2i(x, y));

  updatePreview();
}

void CorrelationWidget::updateWidgets() {
  //  m_leftImagePreview->setImage(m_cs->leftImage(), true);
  updatePreview();
}

void CorrelationWidget::correlationFinished() {
  //  vw::ImageView<vw::PixelDisparity<vw::float32> > disparity_map = m_correlateThread->result();

//   vw::ImageView<vw::float32> xResult = clamp(select_channel(disparity_map, 0), m_cs->searchWindow().min().x(), m_cs->searchWindow().max().x());
//   vw::ImageView<vw::float32> yResult = clamp(select_channel(disparity_map, 1), m_cs->searchWindow().min().y(), m_cs->searchWindow().max().y());

//   m_xImagePreview->setImage(xResult, true);
//   m_yImagePreview->setImage(yResult, true);

//   m_xImagePreview->fitToWindow();
//   m_yImagePreview->fitToWindow();
//   m_leftImagePreview->fitToWindow();

//   m_cs->setDisparityMap(disparity_map);
 
  updatePreview();
}
