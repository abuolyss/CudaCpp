#pragma once
#include <cuda_runtime.h>
#include "Structs.h"


struct PointLight
{
	Vertice3 position;
	Vertice3 color;
	float intensity;
};

struct PointLightGPU
{
	float posx;
	float posy;
	float posz;

	float colorx;
	float colory;
	float colorz;

	float intensity;

	__host__ __device__ inline float DistanceSquared(const Vertice3& other) const
	{
		float dx = posx - other.x;
		float dy = posy - other.y;
		float dz = posz - other.z;

		return dx * dx + dy * dy + dz * dz;
	}

	__host__ __device__ inline float Distance(const Vertice3& other) const
	{
		float dx = posx - other.x;
		float dy = posy - other.y;
		float dz = posz - other.z;

		return sqrtf(dx * dx + dy * dy + dz * dz);
	}
};

constexpr int MAX_LIGHTS = 64;

extern __device__ __constant__ PointLightGPU gpuLights[MAX_LIGHTS];
extern __device__ __constant__ int gpuLightCount;

//extern __constant__ PointLight gpuLights[MAX_LIGHTS];
void UploadLightsGPU(const PointLightGPU* lights, int count);