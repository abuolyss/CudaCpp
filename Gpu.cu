#pragma once

#include "BVH.h"
#include "Gpu.h"
#include "Structs.h"
#include "lights.h"
#include <cfloat>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>


__global__ void RenderScene(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp);

__device__ static void TraverseBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 cameraPos, Vertice3 invDir, Vertice3 dir, float& closestDistance, int& closestTriangleIndex, float& outu, float& outv);

__host__ void LaunchRender(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp)
{
	dim3 blockSize(16, 16);
	dim3 gridSize(
		(screenWidth + blockSize.x - 1) / blockSize.x,
		(screenHeight + blockSize.y - 1) / blockSize.y
	);
	RenderScene << <gridSize, blockSize >> > (framebuffer, numTriangles, screenWidth, screenHeight, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodeCount, cameraForward, cameraRight, cameraUp);

	cudaDeviceSynchronize();
}

/// <summary>
/// Möller–Trumbore ray-triangle intersection algorithm implementation. This function checks if a ray defined by its origin and direction intersects with a triangle defined by its vertices. If an intersection occurs, it calculates the distance from the ray origin to the intersection point and returns true, otherwise it returns false.
/// </summary>
/// <param name="rayOrigin"></param>
/// <param name="rayDirection"></param>
/// <param name="tri"></param>
/// <param name="outIntersectionPoint"></param>
/// <returns></returns>
__device__ inline bool RayIntersectsTriangle(Vertice3 rayOrigin, Vertice3 rayDirection, float& outIntersectionPoint, float& u, float& v, const TriangleSoA triangleSoA, int triangleIndex)
{
	float E1x = triangleSoA.E1x[triangleIndex];
	float E1y = triangleSoA.E1y[triangleIndex];
	float E1z = triangleSoA.E1z[triangleIndex];

	float E2x = triangleSoA.E2x[triangleIndex];
	float E2y = triangleSoA.E2y[triangleIndex];
	float E2z = triangleSoA.E2z[triangleIndex];

	float ax = triangleSoA.ax[triangleIndex];
	float ay = triangleSoA.ay[triangleIndex];
	float az = triangleSoA.az[triangleIndex];

	float Px = rayDirection.y * E2z - rayDirection.z * E2y;
	float Py = rayDirection.z * E2x - rayDirection.x * E2z;
	float Pz = rayDirection.x * E2y - rayDirection.y * E2x;


	float det = E1x * Px + E1y * Py + E1z * Pz;
	const float EPSILON = 1e-7f;

	if (fabsf(det) < EPSILON)
		return false;

	float invDet = 1.0f / det;

	float Tx = rayOrigin.x - ax;
	float Ty = rayOrigin.y - ay;
	float Tz = rayOrigin.z - az;

	u = (Tx * Px + Ty * Py + Tz * Pz) * invDet;
	if (u < 0 || u > 1)
		return false;

	float Qx = Ty * E1z - Tz * E1y;
	float Qy = Tz * E1x - Tx * E1z;
	float Qz = Tx * E1y - Ty * E1x;

	v = (rayDirection.x * Qx + rayDirection.y * Qy + rayDirection.z * Qz) * invDet;

	if (v < 0 || u + v > 1)
		return false;


	outIntersectionPoint = (E2x * Qx + E2y * Qy + E2z * Qz) * invDet;
	if (outIntersectionPoint > 0.0001f)

		return true;


	return false;
}

__device__ inline bool HitAABB(Vertice3 rayOrigin, Vertice3 rayDir, AABB box, float& tmin)
{
	float tx1 = (box.min.x - rayOrigin.x) * rayDir.x;
	float tx2 = (box.max.x - rayOrigin.x) * rayDir.x;

	tmin = fminf(tx1, tx2);
	float tmax = fmaxf(tx1, tx2);

	float ty1 = (box.min.y - rayOrigin.y) * rayDir.y;
	float ty2 = (box.max.y - rayOrigin.y) * rayDir.y;

	tmin = fmaxf(tmin, fminf(ty1, ty2));
	tmax = fminf(tmax, fmaxf(ty1, ty2));

	float tz1 = (box.min.z - rayOrigin.z) * rayDir.z;
	float tz2 = (box.max.z - rayOrigin.z) * rayDir.z;

	tmin = fmaxf(tmin, fminf(tz1, tz2));
	tmax = fminf(tmax, fmaxf(tz1, tz2));

	return (tmax >= tmin && tmax > 0.0f);
}

__global__ void RenderScene(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp)
{

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= screenWidth || y >= screenHeight)
		return;

	float viewportDistance = 1;
	float viewportWidth = 1.7778;
	float viewportHeight = 1;

	int closestTriangleIndex = -1;
	float closestDistance = FLT_MAX;

	float u = (float)x / screenWidth;
	float v = (float)y / screenHeight;
	u = u * 2 - 1;
	v = 1.0f - ((float)y / screenHeight) * 2.0f;

	Vertice3 dir = cameraForward * viewportDistance + cameraRight * (u * viewportWidth) + cameraUp * (v * viewportHeight);


	//gpuNodes;
	//gpuTriIndices;
	dir.NormalizeValue();

	Vertice3 invDir = Vertice3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);

	int triangleTests = 0;
	int aabbcount = 0;

	float outu, outv;

	//For each ray, traverse the BVH to find the closest triangle intersection
	TraverseBVH(gpuNodes, gpuTriIndices, triangleSoA, cameraPos, invDir, dir, closestDistance, closestTriangleIndex, outu, outv);




	if (closestTriangleIndex != -1)
	{

		float w = 1.0f - outu - outv;

		float normalx =
			triangleSoA.N0x[closestTriangleIndex] * w +
			triangleSoA.N1x[closestTriangleIndex] * outu +
			triangleSoA.N2x[closestTriangleIndex] * outv;

		float normaly =
			triangleSoA.N0y[closestTriangleIndex] * w +
			triangleSoA.N1y[closestTriangleIndex] * outu +
			triangleSoA.N2y[closestTriangleIndex] * outv;

		float normalz =
			triangleSoA.N0z[closestTriangleIndex] * w +
			triangleSoA.N1z[closestTriangleIndex] * outu +
			triangleSoA.N2z[closestTriangleIndex] * outv;
		Vertice3 normal = Vertice3(normalx, normaly, normalz);
			
		normal.NormalizeValue();
			PointLight light;
		light.position = Vertice3(6, 2, -1);
		light.color = Vertice3(1, 1, 1);
		light.intensity = 4.0f;
		Vertice3 hitPoint = cameraPos + dir * closestDistance;
		Vertice3 lightDir = light.position - hitPoint;
			lightDir.NormalizeValue();

			int shadowRayHit = -1;
			float outShadowu, outShadowv;

			Vertice3 shadowOrigin =
				hitPoint + normal * 0.001f;

		closestDistance = FLT_MAX;

		float lightDistance =
			sqrtf(light.position.DistanceSquared(hitPoint));

		Vertice3 shadowRayDirInv = lightDir.Reciprocal();
		TraverseBVH(gpuNodes, gpuTriIndices, triangleSoA, shadowOrigin, shadowRayDirInv, lightDir, closestDistance, shadowRayHit, outShadowu, outShadowv);

		if (shadowRayHit != -1 &&
			closestDistance < lightDistance)
		{
			framebuffer[y * (int)screenWidth + x] = 0x030303FF;
			//framebuffer[y * screenWidth + x] = 0xFF0000FF;
		}
		else
		{
		






			float invLength = rsqrtf(normalx * normalx + normaly * normaly + normalz * normalz);

			normalx *= invLength;
			normaly *= invLength;
			normalz *= invLength;







			float distanceSq =
				light.position.DistanceSquared(hitPoint);

			float attenuation =
				1.0f / distanceSq;


			float diffuse = fmaxf((normalx * lightDir.x + normaly * lightDir.y + normalz * lightDir.z), 0.0f);
			diffuse *= attenuation;

			Uint32 packed = triangleSoA.packedColor[closestTriangleIndex];

			float r = ((packed >> 24) & 0xFF) / 255.0f;
			float g = ((packed >> 16) & 0xFF) / 255.0f;
			float b = ((packed >> 8) & 0xFF) / 255.0f;
			float a = ((packed) & 0xFF) / 255.0f;

			r *= light.color.x * diffuse * light.intensity;
			g *= light.color.y * diffuse * light.intensity;
			b *= light.color.z * diffuse * light.intensity;

			r = fminf(r, 1.0f);
			g = fminf(g, 1.0f);
			b = fminf(b, 1.0f);

			//Gamma correction below this
			r = sqrtf(r);
			g = sqrtf(g);
			b = sqrtf(b);



			framebuffer[y * (int)screenWidth + x] =
				((Uint32)(r * 255.0f) << 24) |
				((Uint32)(g * 255.0f) << 16) |
				((Uint32)(b * 255.0f) << 8) |
				0xFF;
			//framebuffer[y * screenWidth + x] = 0x00FF00FF;
		}
	}
	else {
		framebuffer[y * (int)screenWidth + x] = 0x303030FF;
	}

}

__device__ static void TraverseBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 cameraPos, Vertice3 invDir, Vertice3 dir, float& closestDistance, int& closestTriangleIndex, float& outu, float& outv)
{

	StackEntry stack[64];
	int stackPtr = 0;
	int nodeIndex = 0;
	float nodeNear = 0;

	float uu, vv;
	stack[stackPtr++] = { 0, 0.0f };

	while (stackPtr > 0)
	{

		StackEntry entry = stack[--stackPtr];

		nodeIndex = entry.nodeIndex;
		nodeNear = entry.nearT;

		BVHNode node = gpuNodes[nodeIndex];

		if (nodeNear > closestDistance)
			continue;
		if (node.triCount == 0)
		{
			int leftChild = node.leftFirst;
			int rightChild = node.leftFirst + 1;

			float leftNear;
			float rightNear;

			bool hitLeft =
				HitAABB(cameraPos,
					invDir,
					gpuNodes[leftChild].aabb,
					leftNear);
			//aabbcount++;
			bool hitRight =
				HitAABB(cameraPos,
					invDir,
					gpuNodes[rightChild].aabb,
					rightNear);
			//aabbcount++;
			if (hitLeft && hitRight)
			{
				if (leftNear < rightNear)
				{
					stack[stackPtr++] = { rightChild, rightNear };
					stack[stackPtr++] = { leftChild, leftNear };
				}
				else
				{
					stack[stackPtr++] = { leftChild, leftNear };
					stack[stackPtr++] = { rightChild, rightNear };
				}
			}
			else if (hitLeft)
			{
				if (leftNear <= closestDistance)
					stack[stackPtr++] = { leftChild, leftNear };
			}
			else if (hitRight)
			{
				if (rightNear <= closestDistance)
					stack[stackPtr++] = { rightChild, rightNear };
			}
		}
		else
		{
			float intersectionPoint = 0.0f;
			for (int i = 0; i < node.triCount; i++)
			{
				int triIndex =
					gpuTriIndices[node.leftFirst + i];

				if ((RayIntersectsTriangle(cameraPos, dir, intersectionPoint, uu, vv, triangleSoA, triIndex)))
				{
					if (intersectionPoint < closestDistance)
					{
						outu = uu;
						outv = vv;
						closestDistance = intersectionPoint;
						closestTriangleIndex = triIndex;
					}
				}
			}

		}
	}
}