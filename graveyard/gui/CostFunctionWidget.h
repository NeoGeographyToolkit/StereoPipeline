// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __COST_FUNCTION_WIDGET_H__
#define __COST_FUNCTION_WIDGET_H__

#include <QSpinBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QComboBox>
#include <QGroupBox>
#include <QSlider>
#include <QCheckBox>

#include <vw/Image.h>

// Forward Declaration
class PreviewGLWidget;

class CostFunctionWidget : public QWidget {
  Q_OBJECT

  PreviewGLWidget *m_imagePreview;
  PreviewGLWidget *m_leftImagePreview;
  
  QSpinBox *m_xDisparitySpin;
  QSpinBox *m_yDisparitySpin;

  QSlider *m_xDisparitySlider;
  QSlider *m_yDisparitySlider;
  
  QGroupBox *genSettingsBox(QString const& name);
  QHBoxLayout *genXDisparityKnobs();
  QHBoxLayout *genYDisparityKnobs();
  
public:
  CostFunctionWidget(QWidget *parent = 0);
                                         
private slots:
  void imageClicked(int x, int y);
  void recalculateCost();
  void updatePreview();
  
public slots:
  void updateWidgets();
};
#endif
