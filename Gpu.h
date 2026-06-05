#pragma once
#include "BVH.h"
#include "Structs.h"
#include "lights.h"

extern __device__ int shadowRayCount;
extern __device__ int hitPixels;
void GetCudaStats(int& shadowRays, int& pixels);
void ResetCudaStats();

void LaunchRender(Uint32* framebuffer,  int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp);

struct StackEntry
{
	int nodeIndex;
	float nearT;
};

struct SurfaceHit
{
	int triangleIndex = -1;

	float distance = FLT_MAX;

	float barycentricU = 0.0f;
	float barycentricV = 0.0f;
};


struct ShadingData
{
	Vertice3 hitPoint;
	Vertice3 normal;
	Vertice3 shadowRayOrigin;
};


