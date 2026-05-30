#pragma once
#include "BVH.h"
void LaunchRender(Uint32* framebuffer,  int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp);


struct StackEntry
{
	int nodeIndex;
	float nearT;
};