// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef __ASP_SFMVIEW_GL_WIDGET_H__
#define __ASP_SFMVIEW_GL_WIDGET_H__

#include <asp/SfmView/GlContext.h>

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QTimer>

#include <set>

class GlWidget: public QOpenGLWidget {
  Q_OBJECT

public:
  GlWidget(QWidget* parent = nullptr);
  ~GlWidget();

  void set_context(sfm::GlContext* context);

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

public slots:
  void repaint_async(void);

protected:
  void initializeGL(void);
  void paintGL(void);
  void resizeGL(int width, int height);

  void mousePressEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent* event);

private:
  sfm::GlContext* context;
  int gl_width;
  int gl_height;
  qreal device_pixel_ratio;
  bool cx_init;
  std::set<sfm::GlContext*> init_set;
  QTimer* repaint_timer;
};

#endif // __ASP_SFMVIEW_GL_WIDGET_H__
