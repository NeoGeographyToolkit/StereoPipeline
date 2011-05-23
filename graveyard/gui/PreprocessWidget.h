// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
