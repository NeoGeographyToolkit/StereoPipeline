// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
