#ifndef __PREVIEW_GL_WIDGET_H__
#define __PREVIEW_GL_WIDGET_H__

#include <QGLWidget>
#include <QGLFormat>
#include <QPoint>
#include <vw/Core/Log.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Statistics.h>
#include <vw/Math/BBox.h>

#include <string>

class QMouseEvent;
class QWheelEvent;
class QPoint;

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

  enum DisplayChannel { DisplayRGBA = 0, DisplayR, DisplayG, DisplayB, DisplayA };
  int m_display_channel;
};

#endif  // __PREVIEW_GL_WIDGET_H__
