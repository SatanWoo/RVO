#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "AddVector.cuh"

__global__ void addKernelI(float32 *c, float32 *a, float32 *b, int size)
{
	int ID = threadIdx.x;
	if (ID < size) {
		c[ID] = a[ID] + b[ID];
	}
}

__global__ void addKernelC(float32 *c, float32 k, int size)
{
	int ID = threadIdx.x;
	if (ID < size) {
		c[ID] = k;
	}
}

__global__ void addKernelV(b2Vec2 *c, b2Vec2 vec, int size) 
{
	int ID = threadIdx.x;
	if (ID < size) {
		c[ID] = vec;
	}
}

void addVectorI(float32 *re, float32 *a, float32 *b, int size)
{
	float32 *dev_a = 0;
	float32 *dev_b = 0;
	float32 *dev_c = 0;

	cudaSetDevice(0);

	cudaMalloc((void**)&dev_c, size * sizeof(int));
	cudaMalloc((void**)&dev_a, size * sizeof(int));
	cudaMalloc((void**)&dev_b, size * sizeof(int));

	cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);

	addKernelI<<<1, size>>>(dev_c, dev_a, dev_b, size);

	cudaDeviceSynchronize();
	cudaMemcpy(re, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);

	cudaFree(dev_c);
	cudaFree(dev_a);
	cudaFree(dev_b);
}

void addVectorC(float32 *c, float32 k, int size)
{
	float32 *dev_c = 0;

	cudaSetDevice(0);
	cudaMalloc((void**)&dev_c, size * sizeof(int));
	cudaMemcpy(dev_c, c, size * sizeof(int), cudaMemcpyHostToDevice);

	addKernelC<<<1, size>>>(dev_c, k, size);

	cudaDeviceSynchronize();
	cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);

	cudaFree(dev_c);
}

void addVectorV(b2Vec2 *c, b2Vec2 vec, int size)
{
	b2Vec2 *dev_c = 0;

	cudaSetDevice(0);
	cudaMalloc((void**)&dev_c, size * sizeof(b2Vec2));
	cudaMemcpy(dev_c, c, size * sizeof(b2Vec2), cudaMemcpyHostToDevice);

	addKernelV<<<1, size>>>(dev_c, vec, size);

	cudaDeviceSynchronize();
	cudaMemcpy(c, dev_c, size * sizeof(b2Vec2), cudaMemcpyDeviceToHost);

	cudaFree(dev_c);
}