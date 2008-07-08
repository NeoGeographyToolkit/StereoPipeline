// Off hardware assisted offscreen rendering implementation.
//
// Note that this implementation uses framebuffer objects bound to render
// buffers, and that render buffers are limited to RGB or RGBA (i.e., no
// floating point luminance buffers), and typically limited in size to
// 4096x4096.
//
// Finally note that framebuffer objects require a direct rendering
// context!
#ifndef _HARDWARE_RENDERER_H_
#define _HARDWARE_RENDERER_H_

namespace vw { 
namespace stereo {
  enum { eShadeFlat = 0x0, eShadeSmooth = 0x1 };
  enum { ePackedArray = 0 };
  enum { eColorBufferBit = 0 };
  class HardwareRenderer
  {
  public:
    HardwareRenderer();
    HardwareRenderer(int bufferWidth, int bufferHeight, float *buffer = 0);
    ~HardwareRenderer();
    float *Buffer();
    void UpdateBuffer();
    void ClearColor(const float red, const float green,
                    const float blue, const float alpha)
    {
      m_clearColor[0] = red;
      m_clearColor[1] = green;
      m_clearColor[2] = blue;
      m_clearColor[3] = alpha;
    }

    void Clear(int bufferSelect) const;

    void Ortho2D(const double left, const double right,
                 const double bottom, const double top);
    void Color(const float red, const float green, const float blue);
    void Color(const unsigned char red, const unsigned char green,
               const unsigned char blue);
    void SetVertexPointer(const int numComponents, const int stride,
                          float * const vertices);
    void SetColorPointer(const int numComponents, const int stride,
                         float * const colors);
      void DrawPolygon(const int startIndex, const int numVertices);
  private:
    int m_maxRenderBufferSize;
    float m_clearColor[4];
    unsigned int m_numFrameBuffers;
    unsigned int m_numRenderBuffers;
    unsigned int m_frameBufferIDs[1];
    unsigned int m_renderBufferIDs[1];
    int m_bufferWidth, m_bufferHeight;
    float *m_buffer;
  };

}} 

#endif	// _HARDWARE_RENDERER_H_
