#include "lights.h"
#include <cuda_runtime.h>
#include <iostream>

__device__ __constant__ PointLightGPU gpuLights[MAX_LIGHTS];

__device__ __constant__ int gpuLightCount;

void UploadLightsGPU(const PointLightGPU* lights, int count)
{

	cudaError_t err =
		cudaMemcpyToSymbol(gpuLights, lights, count * sizeof(PointLightGPU));

	if (err != cudaSuccess)
	{
	std::cout
		<< cudaGetErrorString(err)
		<< std::endl;
	}

	cudaError_t err1 =
		cudaMemcpyToSymbol(gpuLightCount, &count, sizeof(int));

	if (err1 != cudaSuccess)
	{
		std::cout
			<< cudaGetErrorString(err1)
			<< std::endl;
	}
}