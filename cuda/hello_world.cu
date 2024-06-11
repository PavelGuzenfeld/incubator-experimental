//Basic Hellow World with cuda

#include <stdio.h>

__global__ void helloFromGPU(void)
{
    printf("Hello World from GPU!\n");
}


int main(void)
{
    printf("Hello World from CPU!\n");

    helloFromGPU<<<1, 10>>>(); //launch 10 blocks of 1 thread each

    cudaDeviceReset();//wait for GPU to finish before exiting
    return 0;
}
