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

#include <asp/SfmView/GlCommon.h>
#include <asp/SfmView/GlWidget.h>

#include <QApplication>

#include <ctime>
#include <iostream>

GlWidget::GlWidget(QWidget *parent):
  QOpenGLWidget(parent),
  context(nullptr),
  gl_width(500),
  gl_height(500),
  cx_init(false) {
  this->setFocusPolicy(Qt::StrongFocus);
  this->makeCurrent();
  this->device_pixel_ratio = static_cast<QGuiApplication*>(
    QApplication::instance())->devicePixelRatio();

  // This timer triggers a repaint after all events in the window system's
  // event queue have been processed. Thus a snappy 3D view is provided.
  this->repaint_timer = new QTimer(this);
  this->repaint_timer->setSingleShot(true);
  connect(this->repaint_timer, SIGNAL(timeout()), this, SLOT(repaint()));
}

GlWidget::~GlWidget(void) {
}

void GlWidget::initializeGL() {
}

void GlWidget::resizeGL(int width, int height) {
  width *= this->device_pixel_ratio;
  height *= this->device_pixel_ratio;

  this->gl_width = width;
  this->gl_height = height;
  if (this->context != nullptr)
    this->context->resize(width, height);
}

void GlWidget::paintGL() {
  if (this->context == 0)
    return;

  // Current context may need initialization.
  if (this->cx_init) {
    if (this->init_set.find(this->context) == this->init_set.end()) {
      this->context->init();
      this->context->resize(this->gl_width, this->gl_height);
      this->init_set.insert(this->context); // Mark initialized
    }
    this->cx_init = false;
  }

  // Paint it!
  this->context->paint();
}

QSize GlWidget::minimumSizeHint() const {
  return QSize(50, 50);
}

QSize GlWidget::sizeHint() const {
  return QSize(400, 400);
}

void GlWidget::repaint_async(void) {
  // Don't issue an immediate repaint but let the timer trigger
  // a repaint after all events have been processed.
  if (this->repaint_timer->isActive())
    return;

  this->repaint_timer->start();
}

void GlWidget::set_context(sfm::GlContext* context) {
  this->context = context;
  this->cx_init = true;
}

void GlWidget::mousePressEvent(QMouseEvent *event) {
  this->makeCurrent();
  sfm::MouseEvent e;
  e.type = sfm::MOUSE_EVENT_PRESS;
  e.button = (sfm::MouseButton)event->button();
  e.button_mask = event->buttons();
  e.x = event->localPos().x() * this->device_pixel_ratio;
  e.y = event->localPos().y() * this->device_pixel_ratio;
  this->context->mouse_event(e);
  this->repaint_async();
}

void GlWidget::mouseReleaseEvent(QMouseEvent *event) {
  this->makeCurrent();
  sfm::MouseEvent e;
  e.type = sfm::MOUSE_EVENT_RELEASE;
  e.button = (sfm::MouseButton)event->button();
  e.button_mask = event->buttons();
  e.x = event->localPos().x() * this->device_pixel_ratio;
  e.y = event->localPos().y() * this->device_pixel_ratio;
  this->context->mouse_event(e);
  this->repaint_async();
}

void GlWidget::mouseMoveEvent(QMouseEvent *event) {
  this->makeCurrent();
  sfm::MouseEvent e;
  e.type = sfm::MOUSE_EVENT_MOVE;
  e.button = (sfm::MouseButton)event->button();
  e.button_mask = event->buttons();
  e.x = event->localPos().x() * this->device_pixel_ratio;
  e.y = event->localPos().y() * this->device_pixel_ratio;
  this->context->mouse_event(e);
  this->repaint_async();
}

void GlWidget::wheelEvent(QWheelEvent* event) {
  this->makeCurrent();
  sfm::MouseEvent e;
  if (event->angleDelta().y() < 0)
    e.type = sfm::MOUSE_EVENT_WHEEL_DOWN;
  else
    e.type = sfm::MOUSE_EVENT_WHEEL_UP;
  e.button = sfm::MOUSE_BUTTON_NONE;
  e.button_mask = event->buttons();
  e.x = event->position().x() * this->device_pixel_ratio;
  e.y = event->position().y() * this->device_pixel_ratio;
  this->context->mouse_event(e);
  this->repaint_async();
}
