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

#include "gui/SearchWindowWidget.h"
#include "gui/QCompatFormLayout.h"

SearchWindowWidget::SearchWindowWidget(StereoGuiSession *cs, QWidget *parent) : QWidget(parent) {
  this->blockSignals(true);

  m_cs = cs;

  m_imagePreview = new PreviewGLWidget(this);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(m_imagePreview);
  mainLayout->addWidget(genSettingsBox("Search Window Preview Settings"));

  this->setLayout(mainLayout);

  connect(m_kernSizeSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setKernSize(int)));
  connect(m_xSearchSizeSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setXSearchSize(int)));
  connect(m_ySearchSizeSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setYSearchSize(int)));
  connect(m_xSearchOffsetSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setXSearchOffset(int)));
  connect(m_ySearchOffsetSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setYSearchOffset(int)));
  connect(m_costTypeBox, SIGNAL(valueChanged(StereoGuiSession::CostType)), m_cs, SLOT(setCostType(StereoGuiSession::CostType)));
  connect(m_costBlurSizeSpin, SIGNAL(valueChanged(int)), m_cs, SLOT(setCostBlurSize(int)));

  connect(m_cs, SIGNAL(kernSizeChanged(int)), m_kernSizeSpin, SLOT(setValue(int))); 
  connect(m_cs, SIGNAL(xSearchSizeChanged(int)), m_xSearchSizeSpin, SLOT(setValue(int)));
  connect(m_cs, SIGNAL(ySearchSizeChanged(int)), m_ySearchSizeSpin, SLOT(setValue(int)));
  connect(m_cs, SIGNAL(xSearchOffsetChanged(int)), m_xSearchOffsetSpin, SLOT(setValue(int)));
  connect(m_cs, SIGNAL(ySearchOffsetChanged(int)), m_ySearchOffsetSpin, SLOT(setValue(int)));
  connect(m_cs, SIGNAL(costTypeChanged(StereoGuiSession::CostType)), m_costTypeBox, SLOT(setValue(StereoGuiSession::CostType)));
  connect(m_cs, SIGNAL(costBlurSizeChanged(int)), m_costBlurSizeSpin, SLOT(setValue(int)));

  connect(m_cs, SIGNAL(costFunctionObjectChanged(boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >)), this, SLOT(correlatePixel()));
  connect(m_cs, SIGNAL(searchWindowPreviewPointChanged(vw::Vector2i)), this, SLOT(correlatePixel()));
  connect(m_cs, SIGNAL(searchWindowChanged(vw::BBox2i)), this, SLOT(correlatePixel()));

//   connect(m_imagePreview, SIGNAL(imageClicked(int, int)), this, SLOT(imageClicked(int, int)));
}

QGroupBox *SearchWindowWidget::genSettingsBox(QString const& name) {
  QGroupBox *box = new QGroupBox(name);
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(genSettingsKnobs());

  box->setLayout(layout);

  return box;
}

QHBoxLayout *SearchWindowWidget::genSettingsKnobs() {
  QLabel *kernSizeLabel = new QLabel("Kernel Size:");
  m_kernSizeSpin = new QSpinBox;

  QLabel *xSearchSizeLabel = new QLabel("X Search Size:");
  m_xSearchSizeSpin = new QSpinBox;
  m_xSearchSizeSpin->setRange(0, 1000);

  QLabel *ySearchSizeLabel = new QLabel("Y Search Size:");
  m_ySearchSizeSpin = new QSpinBox;
  m_ySearchSizeSpin->setRange(0, 1000);

  QLabel *xSearchOffsetLabel = new QLabel("X Search Offset:");
  m_xSearchOffsetSpin = new QSpinBox;

  QLabel *ySearchOffsetLabel = new QLabel("Y Search Offset:");
  m_ySearchOffsetSpin = new QSpinBox;

  QLabel *costTypeLabel = new QLabel("Cost Type:");
  m_costTypeBox = new CostTypeComboBox;

  QLabel *costBlurSizeLabel = new QLabel("Cost Blur: ");
  m_costBlurSizeSpin = new QSpinBox;

  QLabel *currPixelLabel = new QLabel("Current Pixel:");
  m_currPixelLabel = new QLabel("INVALID");
  
  QCompatFormLayout *leftLayout = new QCompatFormLayout;
  leftLayout->addRow(costTypeLabel, m_costTypeBox);
  leftLayout->addRow(xSearchSizeLabel, m_xSearchSizeSpin);
  leftLayout->addRow(xSearchOffsetLabel, m_xSearchOffsetSpin);
  leftLayout->addRow(costBlurSizeLabel, m_costBlurSizeSpin);

  QCompatFormLayout *rightLayout = new QCompatFormLayout;
  rightLayout->addRow(kernSizeLabel, m_kernSizeSpin);
  rightLayout->addRow(ySearchSizeLabel, m_ySearchSizeSpin);
  rightLayout->addRow(ySearchOffsetLabel, m_ySearchOffsetSpin);
  rightLayout->addRow(currPixelLabel, m_currPixelLabel);


  QHBoxLayout *layout = new QHBoxLayout;
  layout->addLayout(leftLayout);
  layout->addLayout(rightLayout);

  return layout;
}

void SearchWindowWidget::correlatePixel() {
  if (m_cs->hasBothImagesLoaded()) {
    int sample_size = m_cs->getCostFunctionObject()->sample_size();
    vw::BBox2i pixelsAccessed(m_cs->searchWindowPreviewPoint().x() + m_cs->xSearchOffset() - sample_size / 2,
                              m_cs->searchWindowPreviewPoint().y() + m_cs->ySearchOffset() - sample_size / 2,
                              m_cs->xSearchSize() + sample_size,
                              m_cs->ySearchSize() + sample_size);

    vw::BBox2i imageBounds(0, 0, m_cs->leftImage().cols(), m_cs->rightImage().rows());

    if (imageBounds.contains(pixelsAccessed)) {
      vw::ImageView<vw::float32> result;

      result = vw::stereo::correlate_pixel(m_cs->getCostFunctionObject(), m_cs->searchWindow(), m_cs->searchWindowPreviewPoint());

//       m_imagePreview->setImage(result, true, m_cs->xSearchOffset(), m_cs->ySearchOffset());
//       m_imagePreview->fitToWindow();
    
      int minX = 0, minY = 0;
      for (int x = 0; x < result.cols(); x++) {
        for (int y = 0; y < result.rows(); y++) {
          if (result(x, y) < result(minX, minY)) {
            minX = x;
            minY = y;
          }
        }
      }

//       m_imagePreview->setCrosshairPosition(minX, minY);
//       m_imagePreview->setCrosshairEnabled(true);

      updatePreview();

      m_currPixelLabel->setText(QString("( %1, %2 )").arg(m_cs->searchWindowPreviewPoint().x()).arg(m_cs->searchWindowPreviewPoint().y()));
    }
    else {
      m_currPixelLabel->setText("INVALID");
    }
  }
}

void SearchWindowWidget::updatePreview() {
  // This function isn't really necessary at this point,
  // But I've but it here since all the other forms I've created have it
//   m_imagePreview->updatePreview();
}

void SearchWindowWidget::updateWidgets() {
  m_xSearchOffsetSpin->setRange(-m_cs->leftImage().cols(), m_cs->leftImage().cols());
  m_ySearchOffsetSpin->setRange(-m_cs->leftImage().rows(), m_cs->leftImage().rows());

  m_kernSizeSpin->setValue(m_cs->kernSize());

  m_xSearchSizeSpin->setValue(m_cs->xSearchSize());
  m_ySearchSizeSpin->setValue(m_cs->ySearchSize());

  m_xSearchOffsetSpin->setValue(m_cs->xSearchOffset());
  m_ySearchOffsetSpin->setValue(m_cs->ySearchOffset());

  m_costTypeBox->setValue(m_cs->costType());

  m_costBlurSizeSpin->setValue(m_cs->costBlurSize());

  correlatePixel();
}

void SearchWindowWidget::imageClicked(int x, int y) {
  m_cs->setDisparityPreview(x + m_cs->xSearchOffset(), y + m_cs->ySearchOffset());
}
