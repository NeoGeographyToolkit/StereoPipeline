#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>

#include <vw/Core/Debugging.h>
#include <vw/Core/Exception.h>

#include "HardwareRenderer.h"

using namespace vw;
using namespace stereo;

static void glut_display_dummy() {}

HardwareRenderer::HardwareRenderer(int bufferWidth, int bufferHeight,float *buffer)
{
  printf("OpenGLManagerInit\n");
  // GLUT Init
  int argc = 1;
  char* argv[1];
    argv[0] = "";
    glutInit(&argc, argv);
    //    if(dummyWindow) {
      glutInitWindowSize(600, 400);
      glutInitDisplayMode(GLUT_RGBA);
      
      glutCreateWindow("GLUT Dummy Window");
      glutDisplayFunc(glut_display_dummy);
      //    }
    printf("OpenGLManagerInit\n");

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(GL_DITHER);
    glDisable(GL_BLEND);
   
   // Generate framebuffer object and render buffer IDs
   glGenFramebuffersEXT(1, (GLuint*)m_frameBufferIDs);
   glGenRenderbuffersEXT(1, (GLuint*)m_renderBufferIDs);

  vw_out(0) << "Getting list of GL extensions.\n";
  const GLubyte *strExt = glGetString (GL_EXTENSIONS); 
  GLboolean has_framebuffer_ext = gluCheckExtension ((const GLubyte*)"EXT_framebuffer_object",strExt); 
  if (has_framebuffer_ext == 0) {
    vw_throw(NoImplErr() << "ERROR: Framebuffer Object extension not supported. Aborting.\n");
    return;
  }

  vw_out(0) << "Generating and binding.\n";

  // Make the current framebuffer object framebuffer[0]
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_frameBufferIDs[0]);
  // Make the current renderbuffer object renderbuffer[0]
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_renderBufferIDs[0]);

#ifndef USE_ZBUFFER
  // Associate storage with the currently bound renderbuffer
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA,
                           bufferWidth, bufferHeight);
  // Associate the currently bound render buffer with the currently
  // bound framebuffer
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                               GL_RENDERBUFFER_EXT, m_renderBufferIDs[0]);
#else
  // Associate storage with the currently bound renderbuffer
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24,
                           bufferWidth, bufferHeight);
  // Associate the currently bound render buffer with the currently
  // bound framebuffer
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                               GL_RENDERBUFFER_EXT, m_renderBufferIDs[0]);
#endif

  vw_out(0) << "Checking for completeness.\n";
  if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT) {
    vw_out(vw::WarningMessage) << "Aborting framebuffer object rendering\n";
    return;
  }

  glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE_EXT, (GLint*)&m_maxRenderBufferSize);
  printf("The max render buffer size is: %d\n", m_maxRenderBufferSize);

  m_buffer = buffer;
  m_bufferWidth = bufferWidth;
  m_bufferHeight = bufferHeight;

  // Initialize GL state. Note: you must set the viewport otherwise it
  // defaults to the size of the window associated with the current GL
  // context.
  glViewport(0, 0, bufferWidth, bufferHeight);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Enable vertex & color arrays
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  // Return to the normal GL buffer...
//   glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

HardwareRenderer::~HardwareRenderer()
{
  printf("\nHardwareRenderer::~HardwareRenderer()\n");
  // Return to the normal GL buffer...
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
  // Disable vertex arrays
  glEnableClientState(GL_VERTEX_ARRAY);
}

void HardwareRenderer::Clear(int bufferSelect) const {
  glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
  glClear(GL_COLOR_BUFFER_BIT);
}

float *HardwareRenderer::Buffer() {
  UpdateBuffer();
  return m_buffer;
}

void HardwareRenderer::UpdateBuffer() {
  // Read out the framebuffer object...
  glReadPixels(0, 0, m_bufferWidth, m_bufferHeight,
               GL_RED, GL_FLOAT, (void *) m_buffer);
}

void HardwareRenderer::Ortho2D(const double left, const double right,
                               const double bottom, const double top) {
  gluOrtho2D(left, right, bottom, top);
}

void HardwareRenderer::Color(const float red, const float green, const float blue) {
  glColor3f(red, green, blue);
}

void HardwareRenderer::Color(const unsigned char red, const unsigned char green,
                             const unsigned char blue) {
  glColor3ub(red, green, blue);
}

void HardwareRenderer::SetVertexPointer(const int numComponents, const int stride,
                                        float * const vertices) {
  glVertexPointer(numComponents, GL_FLOAT, stride, vertices);
}

void HardwareRenderer::SetColorPointer(const int numComponents, const int stride,
                                       float * const colors) {
  glColorPointer(numComponents, GL_FLOAT, stride, colors);
}

void HardwareRenderer::DrawPolygon(const int startIndex, const int numVertices) {
  // Draw the graphics primitives...
  glDrawArrays(GL_POLYGON, startIndex, numVertices);
}
