// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
