#ifndef __VW_STEREO_SOFTWARE_RENDERER_H__
#define __VW_STEREO_SOFTWARE_RENDERER_H__

namespace vw {
namespace stereo {

    enum { eShadeFlat = 0x0, eShadeSmooth = 0x1 };
    enum { ePackedArray = 0 };
    enum { eColorBufferBit = 0 };
    class SoftwareRenderer
    {
    public:
      SoftwareRenderer(int bufferWidth, int bufferHeight, float *buffer = 0);
      ~SoftwareRenderer();
      float *Buffer() const { return m_buffer; }
      // NOTE: buffer must be at least as big as m_buffer!
      void Buffer(float *buffer);
      void Ortho2D(const double left, const double right,
		   const double bottom, const double top);
      void ClearColor(const float red, const float green, const float blue,
		      const float alpha);
      void Clear(int bufferSelect);
      void Color(const float red, const float green, const float blue);
      void Color(const unsigned char red, const unsigned char green,
		 const unsigned char blue);
      void SetVertexPointer(const int numComponents, const int stride,
			    float * const vertices);
      void SetColorPointer(const int numComponents, const int stride,
			   float * const colors);
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

#endif	// __VW_STEREO_SOFTWARE_RENDERER_H__
