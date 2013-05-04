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
