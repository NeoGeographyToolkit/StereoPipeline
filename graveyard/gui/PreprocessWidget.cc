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

#include "gui/QCompatFormLayout.h"
#include "gui/PreprocessWidget.h"
#include "gui/PreviewGLWidget.h"
#include "gui/Noise.h"



template<class ImageT>
ImageView<typename ImageT::pixel_type> box_filter(ImageViewBase<ImageT> const& img,  Vector2i const& kernSize) {
  typedef typename ImageT::pixel_type PixelT;

  ImageView<PixelT> src = img.impl();
    
  ImageView<PixelT> result(src.cols(), src.rows());
    
  Vector<PixelT> cSum(src.cols());

  // Seed the column sum buffer
  for (int x = 0; x < src.cols(); x++) {
    cSum(x) = 0;
    for (int ky = 0; ky < kernSize.y(); ky++) {
      cSum(x) += src(x, ky);
    }
  }
    
  for (int y = 0; y < src.rows() - kernSize.y(); y++) {
    // Seed the row sum
    PixelT rsum = 0;
    for (int i = 0; i < kernSize.x(); i++) {
      rsum += cSum(i);
    }

    for (int x = 0; x < src.cols() - kernSize.x(); x++) {
      result(x + kernSize.x() / 2, y + kernSize.y() / 2) = rsum;
      // Update the row sum
      rsum += cSum(x + kernSize.x()) - cSum(x);
    }

    // Update the column sum
    for (int i = 0; i < src.cols(); i++) {
      cSum(i) += src(i, y + kernSize.y()) - src(i, y);
    }
  }
   
  return result / (kernSize.x() * kernSize.y()); 
}

template <class ImageT>
ImageViewRef<typename ImageT::pixel_type> calc_preproc_filter(ImageViewBase<ImageT> const& image, stereopipeline::PreprocFilterType preproc_filter, double size = 0) {
  switch (preproc_filter) {
  case stereopipeline::NO_PREPROC_FILTER:
    return image.impl();
    break;
  case stereopipeline::LOG_PREPROC_FILTER:
    return laplacian_filter(gaussian_filter(image.impl(), size));
    break;
  case stereopipeline::SLOG_PREPROC_FILTER:
    return threshold(laplacian_filter(gaussian_filter(image.impl(), size)), 0.0);
    break;
  case stereopipeline::GAUSSIAN_PREPROC_FILTER:
    return gaussian_filter(image.impl(), size);
    break;
  case stereopipeline::BOX_PREPROC_FILTER:
    return box_filter(image.impl(), Vector2i(int(size), int(size)));
    break;
  default:
    VW_ASSERT(0, ArgumentErr() << "Unrecognized Preprocessing Filter");
    return ImageViewRef<typename ImageT::pixel_type>();
  }
}


PreprocessWidget::PreprocessWidget(QString const& name, QWidget *parent) : QWidget(parent) {
  m_glPreview = new PreviewGLWidget(this);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(m_glPreview);
  mainLayout->addWidget(genSettingsBox(name + " Settings"));
  this->setLayout(mainLayout);

  connect(m_fileBrowseButton, SIGNAL(clicked()), this, SLOT(fileBrowseButtonClicked()));
  connect(m_fileNameEdit, SIGNAL(returnPressed()), this, SLOT(loadImage()));
  
  connect(m_filterTypeBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateImage())); 
  connect(m_gaussNoiseSpin, SIGNAL(valueChanged(double)), this, SLOT(updateImage()));
  connect(m_spNoiseSpin, SIGNAL(valueChanged(double)), this, SLOT(updateImage()));
  connect(m_gammaSpin, SIGNAL(valueChanged(double)), this, SLOT(updateImage()));
  connect(m_filterSizeSpin, SIGNAL(valueChanged(double)), this, SLOT(updateImage()));
}

QGroupBox *PreprocessWidget::genSettingsBox(QString const& name) {
  QLabel *gaussLabel = new QLabel("Gaussian Noise:");
  m_gaussNoiseSpin = new QDoubleSpinBox;
  m_gaussNoiseSpin->setValue(0.0);
  m_gaussNoiseSpin->setRange(0.0, 3.0);
  m_gaussNoiseSpin->setSingleStep(0.1);

  QLabel *spLabel = new QLabel("Salt and Pepper Noise:");
  m_spNoiseSpin = new QDoubleSpinBox;
  m_spNoiseSpin->setValue(0.0);
  m_spNoiseSpin->setRange(0.0, 1.0);
  m_spNoiseSpin->setSingleStep(0.1);

  QLabel *gammaLabel = new QLabel("Gamma:");
  m_gammaSpin = new QDoubleSpinBox;
  m_gammaSpin->setValue(1.0);
  m_gammaSpin->setRange(0.0, 10.0);
  m_gammaSpin->setSingleStep(0.1);

  QLabel *filterTypeLabel = new QLabel("Filter Type:");
  m_filterTypeBox = new PreprocessComboBox(this);

  QLabel *filterSizeLabel = new QLabel("Filter Kernel Size:");
  m_filterSizeSpin = new QDoubleSpinBox;
  m_filterSizeSpin->setValue(1.5);
  m_filterSizeSpin->setRange(0.0, 1000);
  m_filterSizeSpin->setSingleStep(0.1);

  QCompatFormLayout *layout = new QCompatFormLayout;

  layout->addRow(genFileOpenLayout());
  layout->addRow(gaussLabel, m_gaussNoiseSpin);
  layout->addRow(spLabel, m_spNoiseSpin);
  layout->addRow(gammaLabel, m_gammaSpin);
  layout->addRow(filterTypeLabel, m_filterTypeBox);
  layout->addRow(filterSizeLabel, m_filterSizeSpin);

  QGroupBox *box = new QGroupBox(name);
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  box->setLayout(layout);

  return box;
}

QHBoxLayout *PreprocessWidget::genFileOpenLayout() {
  QLabel *fileLabel = new QLabel("Filename: ");

  m_fileNameEdit = new QLineEdit;
  m_fileBrowseButton = new QPushButton("Browse");
  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(fileLabel);
  layout->addWidget(m_fileNameEdit);
  layout->addWidget(m_fileBrowseButton);
  return layout;
}

void PreprocessWidget::updateImage() {
  if (hasImageLoaded()) {
    vw::ImageViewRef<PixelRGB<vw::float32> > pipeline = vw::channel_cast<vw::float32>(m_inputImage);

  //   if (m_gaussNoiseSpin->value() != 0) {
//       pipeline = gaussian_noise(pipeline, m_gaussNoiseSpin->value());
//     }

    if (m_spNoiseSpin->value() != 0) {
      pipeline = salt_pepper_noise(pipeline, m_spNoiseSpin->value());
    }

    if (m_gammaSpin->value() != 1) {
      pipeline = pow(pipeline, vw::float32(m_gammaSpin->value()));
    }

    // I used to not need raster, and could put pipeline directly into
    // the preproc_filter I wonder why this isn't working anymore
    vw::ImageView<PixelRGB<vw::float32> > raster = pipeline; 
    m_resultImage = calc_preproc_filter(raster, 
                                        m_filterTypeBox->value(), 
                                        m_filterSizeSpin->value());
    m_glPreview->setImage(m_resultImage);
  }  
  
}

void PreprocessWidget::fileBrowseButtonClicked() {
  QString filename = QFileDialog::getOpenFileName(this, "Open...", "", "Images (*.png *.jpg *.tif *.cub *.img)");

  if (filename != "") {
    m_fileNameEdit->setText(filename);
    loadImage();
  }
}

void PreprocessWidget::loadImage() {
  QString filename = m_fileNameEdit->text();

  if (filename != "") {
    try {
      read_image(m_inputImage, filename.toStdString());
      m_fileNameEdit->setText(filename);
      updateImage();
      m_glPreview->sizeToFit();
    }
    catch(vw::Exception& e) {
      QMessageBox::critical(this, "Error opening image", e.what());
    }
  }
}
