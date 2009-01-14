// VW
//#include <vw/vw.h>

// includes, system
//#include <iostream>

// includes, project
#include <cuda.h>

/* --------------------------- target code ------------------------------*/  
#define ACOS_THREAD_CNT 128
#define N 128

struct acosParams { 
    float *arg; 
    float *res; 
    int n; 
}; 
 
__global__ void acos_main (struct acosParams parms) 
{ 
    int i; 
    for (i = threadIdx.x; i < parms.n; i += ACOS_THREAD_CNT) { 
        parms.res[i] = acosf(parms.arg[i]); 
    } 
} 
 
/* --------------------------- host code ------------------------------*/ 
 
int main (int argc, char *argv[]) 
{ 
    cudaError_t         cudaStat; 
    float*              acosRes = 0; 
    float*              acosArg = 0; 
    float* arg[N];
    float* res[N];
    struct acosParams   funcParams; 
 
    //    ... fill arguments array ‘arg’ .... 
 
    //cout << "Hello, world3.\n";
    cudaStat = cudaMalloc ((void **)&acosArg, N * sizeof(acosArg[0])); 
    cudaStat = cudaMemcpy (acosArg, arg, N * sizeof(arg[0]), cudaMemcpyHostToDevice); 
 
    funcParams.res = acosRes; 
    funcParams.arg = acosArg; 
    funcParams.n = N; 
     
    acos_main<<<1,ACOS_THREAD_CNT>>>(funcParams); 

    cudaStat = cudaMemcpy (res, acosRes, N * sizeof(res[0]), cudaMemcpyDeviceToHost); 
 
    //    ... process result array ‘res’ .... 
} 

