// __BEGIN_LICENSE__
//
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2008 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file SoftwareRenderer.h
///

#ifndef __VW_STEREO_SOFTWARE_RENDERER_H__
#define __VW_STEREO_SOFTWARE_RENDERER_H__

namespace vw {
namespace stereo {

  struct SoftwareRenderer {

      SoftwareRenderer(int bufferWidth, int bufferHeight, float *buffer = 0);
      ~SoftwareRenderer();
      void Ortho2D(const double left, const double right,
                   const double bottom, const double top);
      void Clear(const float val);
      void SetVertexPointer(const int numComponents, float * const vertices);
      void SetColorPointer(const int numComponents, float * const colors);
      void DrawPolygon(const int startIndex, const int numVertices);

    private:
      int m_numVertexComponents;
      float *m_vertexPointer;
      int m_triangleVertexStep;
      int m_numColorComponents;
      int m_triangleColorStep;
      float *m_colorPointer;
      int m_bufferWidth, m_bufferHeight;
      float *m_buffer;
      int m_shadeMode;
      float m_currentFlatColor[3];
      float m_clearColor[4];
      double m_transformNDC[3][2];
      double m_transformViewport[3][2];
      void *m_graphicsState;
    };
  }
}

#endif  // __VW_STEREO_SOFTWARE_RENDERER_H__
