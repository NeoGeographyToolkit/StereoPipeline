#include <cuda.h>
#include <iostream>

/* --------------------------- target code ------------------------------*/  
#define THREAD_COUNT 128

struct params { 
    float *input; 
    float *output; 
    int n; 
}; 
 
__global__ void flip_main (struct params p) 
{ 
  int i; 
  for (i = threadIdx.x; i < p.n; i += THREAD_COUNT) { 
    p.output[i] = 1.0f - p.input[i]; 
  } 
} 
 
/* --------------------------- host code ------------------------------*/ 

// Invert a 1 channel, 32-bit floating point image.
void invert_image (float* img, int width, int height) { 
    cudaError_t         cudaStat; 
    float*              out = 0; 
    float*              in = 0; 
    struct params   funcParams; 
    int N = width * height;
 
    cout << "Allocating GPU Memory.\n";
    cudaStat = cudaMalloc ((void **)&in, N * sizeof(in[0])); 
    cudaStat = cudaMalloc ((void **)&out, N * sizeof(out[0])); 

    cout << "Copying data.\n";
    cudaStat = cudaMemcpy (in, img, N * sizeof(img[0]), cudaMemcpyHostToDevice); 
 
    funcParams.output = out; 
    funcParams.input = in; 
    funcParams.n = N; 
     
    cout << "Running kernel.\n";
    flip_main<<<1,THREAD_COUNT>>>(funcParams); 

    cout << "Copying result.\n";
    cudaStat = cudaMemcpy (img, out, N * sizeof(out[0]), cudaMemcpyDeviceToHost); 
} 


//------------

  /// Base class from which specific image resources derive.
  class CudaImageResource { 
    float* m_buffer;
    ImageFormat m_format;

  public:
  
    CudaImageResource(ImageFormat format): 
      m_format(format) {
      int32 size = m_format.cols * m_format.rows * m_format.planes;
      cudaStat = cudaMalloc ((void **)&m_buffer, size * sizeof(float)); 
    }

    virtual ~CudaImageResource() {
      cudaFree(m_buffer);
    };

    /// Returns the number of columns in an image resource.
    virtual int32 cols() const { return m_cols; }

    /// Returns the number of rows in an image resource.
    virtual int32 rows() const { return m_rows; }

    /// Returns the number of planes in an image resource.
    virtual int32 planes() const { return m_planes; }

    /// Returns the number of channels in a image resource.
    int32 channels() const { return num_channels( pixel_format() ); }

    /// Returns the native pixel format of the resource.
    virtual PixelFormatEnum pixel_format() const { return m_format.pixel_format; }

    /// Returns the native channel type of the resource.
    virtual ChannelTypeEnum channel_type() const { return m_format.channel_type; }

    /// Read the image resource at the given location into the given buffer.
    virtual void read( ImageBuffer const& buf, BBox2i const& bbox ) const {
      cudaStat = cudaMemcpy (img, out, N * sizeof(out[0]), cudaMemcpyDeviceToHost); 
    }

    /// Write the given buffer to the image resource at the given location.
    virtual void write( ImageBuffer const& buf, BBox2i const& bbox ) {
      cudaStat = cudaMemcpy (in, img, N * sizeof(img[0]), cudaMemcpyHostToDevice); 
    }

    /// Returns the optimal block size/alignment for partial reads or writes.
    virtual Vector2i native_block_size() const { return Vector2i(cols(),rows()); }

    /// Force any changes to be written to the resource.
    virtual void flush() {}

  };
