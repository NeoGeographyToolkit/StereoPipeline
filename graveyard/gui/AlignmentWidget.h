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


#ifndef __ALIGNMENT_WIDGET_H__
#define __ALIGNMENT_WIDGET_H__

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

class AlignmentWidget : public QWidget {
  Q_OBJECT

  PreviewGLWidget *m_right_preview;
  PreviewGLWidget *m_left_preview;
  PreviewGLWidget *m_compare_preview;
  PreviewGLWidget *m_left_ip_preview;
  PreviewGLWidget *m_right_ip_preview;

  QWidget* generate_previews();
  QWidget* generate_controls();

public:
  AlignmentWidget(QWidget *parent = 0);

private slots:
  void feature_button_clicked();
  void orthoimage_button_clicked();
  
};

#endif // __ALIGNMENT_WIDGET_H__
