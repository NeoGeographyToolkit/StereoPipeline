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

#ifndef __ASP_SFMVIEW_GL_COMMON_H__
#define __ASP_SFMVIEW_GL_COMMON_H__

// OpenGL via Qt
#include <QOpenGLExtraFunctions>
#include <QOpenGLContext>

#include <stdexcept>
#include <string>

// Get GL functions from the current Qt GL context (Qt6).
inline QOpenGLExtraFunctions* glFunctions() {
  return QOpenGLContext::currentContext()->extraFunctions();
}

namespace sfm {

enum MouseEventType {
  MOUSE_EVENT_PRESS,
  MOUSE_EVENT_RELEASE,
  MOUSE_EVENT_MOVE,
  MOUSE_EVENT_WHEEL_UP,
  MOUSE_EVENT_WHEEL_DOWN
};

enum MouseButton {
  MOUSE_BUTTON_NONE   = 0,
  MOUSE_BUTTON_LEFT   = 1 << 0,
  MOUSE_BUTTON_RIGHT  = 1 << 1,
  MOUSE_BUTTON_MIDDLE = 1 << 2
};

struct MouseEvent {
  MouseEventType type;
  MouseButton button;
  int button_mask;
  int x;
  int y;
};

} // namespace sfm

#endif // __ASP_SFMVIEW_GL_COMMON_H__
