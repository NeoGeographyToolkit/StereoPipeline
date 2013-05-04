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


#ifndef __CORRELATION_WIDGET_H__
#define __CORRELATION_WIDGET_H__

#include <QSpinBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QGroupBox>
#include <QPushButton>
#include <QProgressBar>

#include <vw/Image.h>

#include "gui/PreviewGLWidget.h"
#include "gui/KyleStereo.h"
#include "gui/StereoGuiSession.h"
//#include "QCorrelateThread.h"

class CorrelationWidget : public QWidget {
  Q_OBJECT

  private:
    StereoGuiSession *m_cs;
    PreviewGLWidget *m_xImagePreview, *m_yImagePreview, *m_leftImagePreview;
    QPushButton *m_doCorrelateButton;
    QProgressBar *m_progressBar;
    QPushButton *m_cancelButton;

  //    QCorrelateThread *m_correlateThread;

    QGroupBox *genSettingsBox(QString const& name);
    QHBoxLayout *genProgressLayout();

  public:
    CorrelationWidget(StereoGuiSession *cs, QWidget *parent = 0);

  public slots:
    void updateWidgets();

  private slots:
    void correlationFinished();
    void updatePreview();
    void imageClicked(int x, int y);
    void doCorrelate();
};

#endif
