//
//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"
//#include "AddVector.cuh"
//#include "box2d.h"
//#include <stdio.h>
//#include <iostream>
//
//const double MapWidth = 2;
//const double MapHeight = 3;
//
////typedef float float32;
//
//using namespace std;
//
//
//// Helper function for using CUDA to add vectors in parallel.
////cudaError_t addWithCuda(int *c, const int *a, const int *b, size_t size)
////{
////	int *dev_a = 0;
////	int *dev_b = 0;
////	int *dev_c = 0;
////	cudaError_t cudaStatus;
////
////	// Choose which GPU to run on, change this on a multi-GPU system.
////	cudaStatus = cudaSetDevice(0);
////
////	// Allocate GPU buffers for three vectors (two input, one output)    .
////	cudaStatus = cudaMalloc((void**)&dev_c, size * sizeof(int));
////	cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(int));
////	cudaStatus = cudaMalloc((void**)&dev_b, size * sizeof(int));
////	// Copy input vectors from host memory to GPU buffers.
////	cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
////	cudaStatus = cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);
////	// Launch a kernel on the GPU with one thread for each element.
////	addKernel<<<1, size>>>(dev_c, dev_a, dev_b);
////
////	// cudaDeviceSynchronize waits for the kernel to finish, and returns
////	// any errors encountered during the launch.
////	cudaStatus = cudaDeviceSynchronize();
////	// Copy output vector from GPU buffer to host memory.
////	cudaStatus = cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);
////
////	cudaFree(dev_c);
////	cudaFree(dev_a);
////	cudaFree(dev_b);
////
////	return cudaStatus;
////}
//
////__global__ void addKernel(int *c, int *a, int *b)
////{
////	int i = threadIdx.x;  
////	c[i] = a[i] + b[i];
////}
//
//int main()
//{
//	/*const int arraySize = 5;
//	const int a[arraySize] = { 1, 2, 3, 4, 5 };
//	const int b[arraySize] = { 10, 20, 30, 40, 50 };
//	int c[arraySize] = { 0 };
//
//	cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
//	printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
//	c[0], c[1], c[2], c[3], c[4]);*/
//
//	int c = MapWidth * MapHeight;
//
//	float32 *re = new float32[c];
//	float32 *ha = new float32[c];
//	float32 *hb = new float32[c];
//
//	for (int i = 0; i < c; i++)
//	{
//		ha[i] = 3.0;
//		hb[i] = 4.0;
//	}
//
//	addVectorI(re, ha, hb, c);
//	for (int i = 0; i < c; i++) 
//	{
//		cout << re[i] << " ";
//	}
//
//	system("pause");
//    return 0;
//}
//
