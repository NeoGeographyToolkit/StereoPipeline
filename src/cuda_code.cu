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

