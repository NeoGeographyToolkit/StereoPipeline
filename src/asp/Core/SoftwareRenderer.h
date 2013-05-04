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
