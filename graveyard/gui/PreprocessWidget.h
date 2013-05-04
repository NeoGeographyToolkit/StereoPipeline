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


#ifndef __PREPROCESS_WIDGET_H__
#define __PREPROCESS_WIDGET_H__

#include <vw/Image/ImageView.h>

// Forward Declarations
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QString>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
class PreviewGLWidget;

namespace stereopipeline {
  enum PreprocFilterType {
    NO_PREPROC_FILTER,
    LOG_PREPROC_FILTER,
    SLOG_PREPROC_FILTER,
    GAUSSIAN_PREPROC_FILTER,
    BOX_PREPROC_FILTER
  };
}

class PreprocessComboBox : public QComboBox {
  Q_OBJECT

public:
  PreprocessComboBox(QWidget *parent = 0) : QComboBox(parent) {
    addItem("NONE", QVariant(int(stereopipeline::NO_PREPROC_FILTER)));
    addItem("LOG", QVariant(int(stereopipeline::LOG_PREPROC_FILTER)));
    addItem("SLOG", QVariant(int(stereopipeline::SLOG_PREPROC_FILTER)));
    addItem("GAUSSIAN", QVariant(int(stereopipeline::GAUSSIAN_PREPROC_FILTER)));
    addItem("BOX", QVariant(int(stereopipeline::BOX_PREPROC_FILTER)));
  }
  
  stereopipeline::PreprocFilterType value() const {
    return static_cast<stereopipeline::PreprocFilterType>(itemData(currentIndex()).toInt());
  }
};

class PreprocessWidget : public QWidget {
  Q_OBJECT

  // Private member variables
  PreviewGLWidget *m_glPreview;
  vw::ImageView<vw::PixelRGB<vw::float32> > m_inputImage;
  vw::ImageView<vw::PixelRGB<vw::float32> > m_resultImage;
  
  QPushButton *m_fileBrowseButton;
  QLineEdit *m_fileNameEdit;
  QDoubleSpinBox *m_gaussNoiseSpin;
  QDoubleSpinBox *m_spNoiseSpin;
  QDoubleSpinBox *m_gammaSpin;
  PreprocessComboBox *m_filterTypeBox;
  QDoubleSpinBox *m_filterSizeSpin;
  
  // Private methods
  QGroupBox *genSettingsBox(QString const& name);
  QHBoxLayout *genFileOpenLayout();
    
public:
  PreprocessWidget(QString const& name, QWidget *parent = 0);
  
  vw::ImageView<vw::PixelRGB<vw::float32> > result() const { return m_resultImage; }
  
  
  bool hasImageLoaded() const { return m_inputImage.cols() != 0; }

private slots:
  void fileBrowseButtonClicked();
  void updateImage();
  void loadImage();
};

#endif // __PREPROCESS_WIDGET_H__
