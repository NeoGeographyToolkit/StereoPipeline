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


#ifndef __PREVIEW_GL_WIDGET_H__
#define __PREVIEW_GL_WIDGET_H__

#include <QGLWidget>
#include <QGLFormat>
#include <QPoint>
#include <vw/Core/Log.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Statistics.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>

#include <string>
#include <list>

class QMouseEvent;
class QWheelEvent;
class QPoint;


class PointList {
  std::list<vw::Vector2> m_points;
  vw::Vector3 m_color;
public:
  PointList(vw::Vector3 const& color) : m_color(color) {}
  PointList(std::list<vw::Vector2> const& points, vw::Vector3 const& color) : 
    m_color(color) {
    this->push_back(points);
  }
  
  std::list<vw::Vector2> const& points() const { return m_points; }
  vw::Vector3 color() const { return m_color; }

  void push_back(vw::Vector2 pt) { m_points.push_back(pt); }
  void push_back(std::list<vw::Vector2> pts) { 
    std::list<vw::Vector2>::iterator iter  = pts.begin();
    while (iter != pts.end()) {
      m_points.push_back(*iter);
      ++iter;
    }
  }
};


class PreviewGLWidget : public QGLWidget {
  Q_OBJECT

public:
  PreviewGLWidget(QWidget *parent) : QGLWidget(parent) {
    setupPreviewGLWidget();
  }

  template <class ViewT>
  PreviewGLWidget(QWidget *parent, vw::ImageViewBase<ViewT> const& view) : QGLWidget(parent) {
    if (!QGLFormat::hasOpenGL()) {
      vw::vw_out(0) << "This system has no OpenGL support.\nExiting\n\n";
      exit(1);
    }

    setupPreviewGLWidget();
    setImage(view);
  }
  virtual ~PreviewGLWidget();

  // Set a default size for this widget.  This is usually overridden.
  virtual QSize sizeHint () const { return QSize(500,500); }

  template <class ViewT>
  void setImage(vw::ImageViewBase<ViewT> const& view) {
    m_image = vw::channel_cast<vw::float32>(view.impl());
    vw::min_max_channel_values(m_image, m_image_min, m_image_max);
    initializeGL();
    update();
  }

  void sizeToFit();
  void zoom(float scale);
  void normalizeImage();

  void add_crosshairs(std::list<vw::Vector2> const& points, vw::Vector3 const& color);
  void clear_crosshairs(); 

public slots:

  void load_image_from_file(std::string const& filename) {
    vw::DiskImageView<vw::PixelRGB<vw::float32> > input_image(filename);
    m_image = input_image;
    vw::min_max_channel_values(m_image, m_image_min, m_image_max);
    initializeGL();
    update();
    sizeToFit();
  }


protected:

  // Setup
  void setupPreviewGLWidget();
  void initializeGL();
  void resizeGL(int width, int height);

  // Event handlers
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseDoubleClickEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent *event);
  void enterEvent(QEvent *event);
  void leaveEvent(QEvent *event);
  void keyPressEvent(QKeyEvent *event);
  
private:
  // Drawing is driven by QPaintEvents, which call out to drawImage()
  // and drawLegend()
  void drawImage();
  void drawLegend(QPainter *painter);
  void updateCurrentMousePosition();
  
  // Image & OpenGL
  vw::ImageView<vw::PixelRGB<vw::float32> > m_image;
  GLuint m_texture;
  GLuint m_glsl_program;
  bool m_draw_texture;
  bool m_show_legend;
  bool m_bilinear_filter;
  bool m_use_colormap;
  
  // Adjustment mode
  enum AdjustmentMode { TransformAdjustment, GainAdjustment, OffsetAdjustment, GammaAdjustment };
  AdjustmentMode m_adjust_mode;

  // Mouse positions and legend information
  QPoint lastPos;
  QPoint currentImagePos;
  std::string m_legend_status;

  // Dimensions & stats
  int m_viewport_width;
  int m_viewport_height;
  vw::float32 m_image_min;
  vw::float32 m_image_max;
  
  // Image Parameters
  vw::BBox2 m_current_viewport;
  float m_gain;
  float m_offset;
  float m_gamma;

  // Crosshair overlays
  std::vector<PointList> m_crosshairs;

  enum DisplayChannel { DisplayRGBA = 0, DisplayR, DisplayG, DisplayB, DisplayA };
  int m_display_channel;
};

#endif  // __PREVIEW_GL_WIDGET_H__
