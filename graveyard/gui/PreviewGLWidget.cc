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


#include <QtGui>

// VW
#include <vw/Image.h>
#include <vw/FileIO.h>
using namespace vw;

#include "gui/PreviewGLWidget.h"


// --------------------------------------------------------------
//                       GLSL DEBUGGING
// --------------------------------------------------------------

void printShaderInfoLog(GLuint obj)
{
  int infologLength = 0;
  int charsWritten  = 0;
  char *infoLog;

  glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

  if (infologLength > 0)
    {
      infoLog = (char *)malloc(infologLength);
      glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
      std::ostringstream err;
      err << "<h4>An error occured while compiling the GLSL shader:</h4><p><h5><tt>" << infoLog << "</tt></h5>";
      QMessageBox::critical(0, "GLSL Shader Error", 
                            err.str().c_str());
      free(infoLog);
    }
}

void printProgramInfoLog(GLuint obj)
{
  int infologLength = 0;
  int charsWritten  = 0;
  char *infoLog;

  glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

  if (infologLength > 0)
    {
      infoLog = (char *)malloc(infologLength);
      glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
      std::ostringstream err;
      err << "<h4>An error occured while linking the GLSL program:</h4><p><h5><tt>" << infoLog << "</tt></h5>";
      QMessageBox::critical(0, "GLSL Program Error", 
                            err.str().c_str());
      printf("%s\n",infoLog);
      free(infoLog);
    }
}

// --------------------------------------------------------------
//               PreviewGLWidget Public Methods
// --------------------------------------------------------------
PreviewGLWidget::~PreviewGLWidget() {
  makeCurrent();
  glDeleteTextures(1,&m_texture);
}

void PreviewGLWidget::sizeToFit() {
  float aspect = float(m_viewport_width) / m_viewport_height;
  int maxdim = std::max(m_image.cols(),m_image.rows());
  if (m_image.cols() > m_image.rows()) {
    float width = maxdim;
    float height = maxdim/aspect;
    float extra = height - m_image.rows();
    m_current_viewport = BBox2(Vector2(0.0, -extra/2), 
                                Vector2(width, height-extra/2));
  } else {
    float width = maxdim*aspect;
    float height = maxdim;
    float extra = width - m_image.cols();
    m_current_viewport = BBox2(Vector2(-extra/2, 0.0), 
                                Vector2(width-extra/2, height));
  }
  update();
}

void PreviewGLWidget::zoom(float scale) {
  float mid_x = m_current_viewport.min().x() + m_current_viewport.width()/2;
  float mid_y = m_current_viewport.min().y() + m_current_viewport.height()/2;
  
  // Check to make sure we haven't hit our zoom limits...
  if (m_current_viewport.width()/scale > 1.0 && 
      m_current_viewport.height()/scale > 1.0 &&
      m_current_viewport.width()/scale < 4*m_image.cols() && 
      m_current_viewport.height()/scale < 4*m_image.rows()) {
    m_current_viewport.min().x() = (m_current_viewport.min().x() - mid_x) / scale + mid_x;
    m_current_viewport.max().x() = (m_current_viewport.max().x() - mid_x) / scale + mid_x;
    m_current_viewport.min().y() = (m_current_viewport.min().y() - mid_y) / scale + mid_y;
    m_current_viewport.max().y() = (m_current_viewport.max().y() - mid_y) / scale + mid_y;
    update();
  }
  m_show_legend = false;
}

void PreviewGLWidget::normalizeImage() {
  m_offset = -m_image_min;
  m_gain = 1/(m_image_max-m_image_min);
  update();
}

// --------------------------------------------------------------
//             PreviewGLWidget Private Methods
// --------------------------------------------------------------

void PreviewGLWidget::drawImage() {

  // Make this context current, and store the current OpenGL state
  // before we start to modify it.
  makeCurrent();
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  
  // Activate our GLSL fragment program and set up the uniform
  // variables in the shader
  glUseProgram(m_glsl_program);
  GLint gain_loc = glGetUniformLocation(m_glsl_program,"gain");
  glUniform1f(gain_loc,m_gain);
  GLint offset_loc = glGetUniformLocation(m_glsl_program,"offset");
  glUniform1f(offset_loc,m_offset);
  GLint gamma_loc = glGetUniformLocation(m_glsl_program,"gamma");
  glUniform1f(gamma_loc,m_gamma);
  GLint display_channel_loc = glGetUniformLocation(m_glsl_program,"display_channel");
  glUniform1i(display_channel_loc,m_display_channel);

  // Set the background color and viewport.
  qglClearColor(QColor(0, 25, 50)); // Bluish-green background
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_viewport_width,m_viewport_height);

  // Set up the orthographic view of the scene.  The exact extent of
  // the view onto the scene depends on the current panning and zoom
  // in the UI.
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(m_current_viewport.min().x(), m_current_viewport.max().x(), 
          -m_current_viewport.max().y(), -m_current_viewport.min().y(),
          -1.0, 1.0);

  // Set up the modelview matrix, and bind the image as the texture we
  // are about to use.
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  if (m_draw_texture) {
    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D, m_texture );
  }
  if (m_bilinear_filter) {
    // When the texture area is small, bilinear filter the closest mipmap
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  } else {
    // When the texture area is small, pick the nearest neighbor in the closest mipmap
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  }


  // Draw the rectangle onto which we will draw the image as a
  // texture.
  qglColor(Qt::white);
  glBegin(GL_QUADS);
  glTexCoord2d( 0.0 , 0.0); 
  glVertex2d( 0.0 , 0.0);
  glTexCoord2d( 0.0 , float(m_image.rows()) / m_image.rows() ); 
  glVertex2d( 0.0 , -(m_image.rows()) );
  glTexCoord2d( float(m_image.cols()) / m_image.cols() , float(m_image.rows()) / m_image.rows() ); 
  glVertex2d( m_image.cols() , -(m_image.rows()) );
  glTexCoord2d( float(m_image.cols()) / m_image.cols() , 0.0 ); 
  glVertex2d( m_image.cols() , 0.0 );
  glEnd();

  // Disable texture mapping and GLSL shaders
  glDisable( GL_TEXTURE_2D );
  glUseProgram(0);

  // Draw crosshairs
  glLineWidth(1.0);
  for (unsigned i = 0; i < m_crosshairs.size(); ++i) {
    Vector3 color = m_crosshairs[i].color();
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINES);
    std::list<Vector2>::const_iterator iter = m_crosshairs[i].points().begin();
    while (iter != m_crosshairs[i].points().end() ) {
      Vector2 point = *iter;
      glVertex2d( point[0]-3 , -point[1]);
      glVertex2d( point[0]+3 , -point[1]);
      glVertex2d( point[0], -point[1]-3);
      glVertex2d( point[0], -point[1]+3);
      ++iter;
    }
    glEnd();
  }    
  
  // Restore the previous OpenGL state so that we don't trample on the
  // QPainter elements of the window.
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
}

void PreviewGLWidget::drawLegend(QPainter* painter) {

  // Extract the value for the pixel currently under the mouse
  PixelRGB<float32> pix_value;
  if (currentImagePos.x() >= 0 && currentImagePos.x() < m_image.cols() &&
      currentImagePos.y() >= 0 && currentImagePos.y() < m_image.rows()) {
    pix_value = m_image(currentImagePos.x(), currentImagePos.y());
  }

  const int Margin = 11;
  const int Padding = 6;

  QTextDocument textDocument;
  textDocument.setDefaultStyleSheet("* { color: #00FF00; font-family: courier, serif }");
  std::ostringstream legend_text;
  legend_text << "<h5 align=\"right\">" << m_legend_status << "<br>"
              << pix_value << "<br>" << " @ " << currentImagePos.x() << " " << currentImagePos.y() << "<br>"
              << "Range: " << m_image_min << " | " << m_offset << " " 
                           << (m_offset + 1/m_gain) << " | " << m_image_max << "</h5>";
  textDocument.setHtml(legend_text.str().c_str());
  textDocument.setTextWidth(textDocument.size().width());
  
  QRect rect(QPoint(0,0), textDocument.size().toSize()
             + QSize(2 * Padding, 2 * Padding));
  painter->translate(width() - rect.width() - Margin,
                    height() - rect.height() - Margin);
  //   painter->setPen(QColor(255, 239, 239));
  //   painter->drawRect(rect);
  painter->translate(Padding, Padding);
  textDocument.drawContents(painter);
}

void PreviewGLWidget::updateCurrentMousePosition() {
  float x_loc = m_current_viewport.min().x() + m_current_viewport.width() * float(lastPos.x()) / m_viewport_width;
  float y_loc = m_current_viewport.min().y() + m_current_viewport.height() * float(lastPos.y()) / m_viewport_height;
  currentImagePos = QPoint(x_loc,y_loc);
}


// --------------------------------------------------------------
//             PreviewGLWidget Setup Methods
// --------------------------------------------------------------
void PreviewGLWidget::setupPreviewGLWidget() {
  setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

  // Set some reasonable defaults
  m_draw_texture = true;
  m_show_legend = false;
  m_bilinear_filter = true;
  m_use_colormap = false;
  m_adjust_mode = TransformAdjustment;
  m_display_channel = DisplayRGBA;
  
  // Set up shader parameters
  m_gain = 1.0;
  m_offset = 0.0;
  m_gamma = 1.0;
  
  // Allocate the opengl texture
  glGenTextures(1,&m_texture);

  // Set mouse tracking
  this->setMouseTracking(true);

  // Set the size policy that the widget can grow or shrink and still
  // be useful.
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PreviewGLWidget::initializeGL() {  
  makeCurrent();

  glShadeModel(GL_FLAT);

  // Set up the texture mode to replace (rather than blend...)
  glBindTexture(GL_TEXTURE_2D, m_texture);
  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); 

  // Copy the texture data over into texture memory.
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image.cols(), m_image.rows(), 0, 
               GL_RGB, GL_FLOAT, &(m_image(0,0)) );
  
  std::ifstream input_file("/Users/mbroxton/projects/StereoPipeline/trunk/src/gui/PreviewGLWidget.frag");
  if (!input_file.is_open())
    vw_throw(IOErr() << "Could not open GLSL shader file.");
  std::string fragment_prog, line;
  while (!input_file.eof()) {
    getline(input_file,line);
    fragment_prog += (line + "\n");
  }
  input_file.close();
  const char* fragment_prog_ptr = fragment_prog.c_str();

  // For debugging:
  //  std::cout << "***\n" << fragment_prog << "***\n";
  
  GLuint m_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(m_fragment_shader, 1, &fragment_prog_ptr, NULL);
  glCompileShader(m_fragment_shader);
  printShaderInfoLog(m_fragment_shader);
  
  m_glsl_program = glCreateProgram();
  glAttachShader(m_glsl_program, m_fragment_shader);
  glLinkProgram(m_glsl_program);
  printProgramInfoLog(m_glsl_program);
}

void PreviewGLWidget::resizeGL(int width, int height) {
  m_viewport_width = width;
  m_viewport_height = height;
  sizeToFit();
}


// --------------------------------------------------------------
//             PreviewGLWidget Event Handlers
// --------------------------------------------------------------

void PreviewGLWidget::paintEvent(QPaintEvent * /* event */) { 
  QPainter painter(this);
  drawImage();
  if (m_show_legend)
    drawLegend(&painter);
}

void PreviewGLWidget::mousePressEvent(QMouseEvent *event) { 
  m_show_legend = true;
  grabKeyboard();
  lastPos = event->pos();
  updateCurrentMousePosition();
}

void PreviewGLWidget::mouseMoveEvent(QMouseEvent *event) {
  float ticks;

  // Left mouse button moves the image around
  if (event->buttons() & Qt::LeftButton) {
    float x_diff = float(event->x() - lastPos.x()) / m_viewport_width;
    float y_diff = float(event->y() - lastPos.y()) / m_viewport_height;

    std::ostringstream s; 
    switch (m_adjust_mode) {

    case TransformAdjustment:
      m_current_viewport.min().x() -= x_diff * m_current_viewport.width();
      m_current_viewport.min().y() -= y_diff * m_current_viewport.height();
      m_current_viewport.max().x() -= x_diff * m_current_viewport.width();
      m_current_viewport.max().y() -= y_diff * m_current_viewport.height();
      break;

    case GainAdjustment:
      // The number '5' below adjust the sensitivity.
      ticks = pow(2, 5 * x_diff * (m_image_max-m_image_min));
      if (m_gain * ticks > 1e-8 && m_gain * ticks < 1e8)
        m_gain *= ticks;
      s << "Gain: " << m_gain << "   " << x_diff << "   " << ticks << "\n";
      m_legend_status = s.str();
      break;

    case OffsetAdjustment:
      m_offset += x_diff * (m_image_max - m_image_min);
      s << "Offset: " << m_offset << "\n";
      m_legend_status = s.str();
      break;

    case GammaAdjustment:
      // The number '5.0' below adjust the sensitivity.
      ticks = pow(2, x_diff * 5.0);
      if (m_gamma * ticks > 0.01 && m_gamma * ticks < 10.0)
        m_gamma *= ticks;
      s << "Gamma: " << m_gamma << "\n";
      m_legend_status = s.str();
      break;
    }

  } else if (event->buttons() & Qt::RightButton) {
    m_gain += GLfloat(event->x() - lastPos.x()) / m_viewport_width *10;
  } 

  // Regardless, we store the current position for the text legend.
  lastPos = event->pos();
  updateCurrentMousePosition();
  update();
}

void PreviewGLWidget::mouseDoubleClickEvent(QMouseEvent * /*event*/) {
  m_draw_texture = !m_draw_texture;
  update();
}

void PreviewGLWidget::wheelEvent(QWheelEvent *event) {
  int num_degrees = event->delta() / 8;
  float num_ticks = num_degrees / 15;

  float scale = pow(2,num_ticks/5);
  zoom(scale);

  m_show_legend = true;
  grabKeyboard();
  lastPos = event->pos();
  updateCurrentMousePosition();
}


void PreviewGLWidget::enterEvent(QEvent */*event*/) {
  m_show_legend = true;
  grabKeyboard();
  update();
}

void PreviewGLWidget::leaveEvent(QEvent */*event*/) {
  m_show_legend = false;
  releaseKeyboard();
  update();
}

void PreviewGLWidget::keyPressEvent(QKeyEvent *event) {

  std::ostringstream s; 
  
  switch (event->key()) {
  case Qt::Key_Plus:   // Zoom inxo
    zoom(2.0);
    break;
  case Qt::Key_Minus:  // Zoom out
    zoom(0.5);
    break;
  case Qt::Key_F:  // Size to fit
    sizeToFit();
    break;
  case Qt::Key_N:  // Toggle bilinear/nearest neighbor interp
    m_bilinear_filter = !m_bilinear_filter;
    update();
    break;
  case Qt::Key_C:  // Activate colormap
    m_use_colormap = !m_use_colormap;
    update();
    break;
  case Qt::Key_R:  // Normalize the image
    normalizeImage();
    update();
    break;
  case Qt::Key_G:  // Gain adjustment mode
    if (m_adjust_mode == GainAdjustment) {
      m_adjust_mode = TransformAdjustment;
      m_legend_status = "";
    } else {
      m_adjust_mode = GainAdjustment;
      s << "Gain: " << m_gain;
      m_legend_status = s.str();
    }
    update();
    break;
  case Qt::Key_O:  // Offset adjustment mode
    if (m_adjust_mode == OffsetAdjustment) {
      m_adjust_mode = TransformAdjustment;
      m_legend_status = "";
    } else {
      m_adjust_mode = OffsetAdjustment;
      s << "Offset: " << m_offset;
      m_legend_status = s.str();
    }
    update();
    break;
  case Qt::Key_V:  // Gamma adjustment mode
    if (m_adjust_mode == GammaAdjustment) {
      m_adjust_mode = TransformAdjustment;
      m_legend_status = "";
    } else {
      m_adjust_mode = GammaAdjustment;
      s << "Gamma: " << m_gamma;
      m_legend_status = s.str();
    }
    update();
    break;
  case Qt::Key_1:  // Normalize the image
    m_display_channel = DisplayR;
    update();
    break;
  case Qt::Key_2:  // Normalize the image
    m_display_channel = DisplayG;
    update();
    break;
  case Qt::Key_3:  // Normalize the image
    m_display_channel = DisplayB;
    update();
    break;
  case Qt::Key_4:  // Normalize the image
    m_display_channel = DisplayA;
    update();
    break;
  case Qt::Key_0:  // Normalize the image
    m_display_channel = DisplayRGBA;
    update();
    break;
  default: 
    QWidget::keyPressEvent(event);
  }
}

void PreviewGLWidget::add_crosshairs(std::list<Vector2> const& points, Vector3 const& color) {
  m_crosshairs.push_back(PointList(points, color));
  update();
}

void PreviewGLWidget::clear_crosshairs() {
  m_crosshairs.clear(); 
  update();
}



