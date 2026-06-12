#pragma once

#include "BVH.h"
#include "Gpu.h"
#include "Structs.h"
#include "lights.h"
#include <cfloat>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>

__device__ int shadowRayCount = 0;
__device__ int hitPixels = 0;

__global__ void RenderScene(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp, Material* gpuMaterials, Texture* gpuTextures);

__device__ static bool HitsBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 cameraPos, Vertice3 invDir, Vertice3 dir, float maxDistance);

__device__ static void TraverseBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 cameraPos, Vertice3 invDir, Vertice3 dir, float& closestDistance, int& closestTriangleIndex, float& outu, float& outv);

__device__ __forceinline__ ShadingData BuildShadingData(const TriangleSoA& triangles, const SurfaceHit& hit, const Vertice3& cameraPosition, const Vertice3& rayDirection);

void GetCudaStats(int& shadowRays, int& pixels)
{
	cudaMemcpyFromSymbol(
		&shadowRays,
		shadowRayCount,
		sizeof(int));

	cudaMemcpyFromSymbol(
		&pixels,
		hitPixels,
		sizeof(int));
}

void ResetCudaStats()
{
	int zero = 0;
	cudaMemcpyToSymbol(shadowRayCount, &zero, sizeof(int));
	cudaMemcpyToSymbol(hitPixels, &zero, sizeof(int));
}

__host__ void LaunchRender(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp, Material* gpuMaterials, Texture* gpuTextures)
{
	dim3 blockSize(8, 8);
	dim3 gridSize(
		(screenWidth + blockSize.x - 1) / blockSize.x,
		(screenHeight + blockSize.y - 1) / blockSize.y
	);
	RenderScene << <gridSize, blockSize >> > (framebuffer, numTriangles, screenWidth, screenHeight, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodeCount, cameraForward, cameraRight, cameraUp, gpuMaterials, gpuTextures);

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
__device__ __forceinline__ bool RayIntersectsTriangle(Vertice3 rayOrigin, Vertice3 rayDirection, float& outIntersectionPoint, float& u, float& v, const TriangleSoA triangleSoA, int triangleIndex)
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

__device__ __forceinline__ bool HitAABB(Vertice3 rayOrigin, Vertice3 rayDir, AABB box, float& tmin)
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

__global__ void RenderScene(Uint32* framebuffer, int numTriangles, int screenWidth, int screenHeight, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp, Material* gpuMaterials, Texture* gpuTextures)
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


	dir.NormalizeValue();

	Vertice3 invDir = Vertice3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);

	float outu, outv;

	TraverseBVH(gpuNodes, gpuTriIndices, triangleSoA, cameraPos, invDir, dir, closestDistance, closestTriangleIndex, outu, outv);


	if (closestTriangleIndex != -1)
	{
		atomicAdd(&hitPixels, 1);

		SurfaceHit hit;

		hit.triangleIndex = closestTriangleIndex;
		hit.distance = closestDistance;
		hit.barycentricU = outu;
		hit.barycentricV = outv;

		ShadingData shading = BuildShadingData(triangleSoA, hit, cameraPos, dir);

		float totalLightr = 0.0f;
		float totalLightg = 0.0f;
		float totalLightb = 0.0f;

		int closestLights[4] =
		{
			-1, -1, -1, -1
		};

		float closestDistances[4] =
		{
			FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
		};

		for (int i = 0; i < gpuLightCount; i++)
		{
			float dist2 =
				gpuLights[i].DistanceSquared(shading.hitPoint);

			for (int slot = 0; slot < 4; slot++)
			{
				if (dist2 < closestDistances[slot])
				{
					for (int shift = 3; shift > slot; shift--)
					{
						closestDistances[shift] =
							closestDistances[shift - 1];

						closestLights[shift] =
							closestLights[shift - 1];
					}

					closestDistances[slot] = dist2;
					closestLights[slot] = i;

					break;
				}
			}
		}

		for (int lightIndex = 0; lightIndex < 4; lightIndex++)
		{
			int i = closestLights[lightIndex];

			if (i < 0)
				continue;



			const PointLightGPU& light = gpuLights[i];

			float distanceSq = light.DistanceSquared(shading.hitPoint);

			if (light.intensity / distanceSq < 0.02f)
				continue;

			float maxRange = light.intensity;

			if (distanceSq > maxRange * maxRange * 2)
				continue;

			Vertice3 lightDir(
				light.posx - shading.hitPoint.x,
				light.posy - shading.hitPoint.y,
				light.posz - shading.hitPoint.z);

			float invDistance = rsqrtf(distanceSq);

			Vertice3 lightDirNormalized(
				lightDir.x * invDistance,
				lightDir.y * invDistance,
				lightDir.z * invDistance);

			float lightDistance = distanceSq * invDistance;

			Vertice3 shadowRayDirInv =
				lightDirNormalized.Reciprocal();

			float NdotL = fmaxf(
				shading.normal.x * lightDirNormalized.x +
				shading.normal.y * lightDirNormalized.y +
				shading.normal.z * lightDirNormalized.z,
				0.0f);

			if (NdotL <= 0.0f)
				continue;

			float attenuation = 1.0f / distanceSq;

			atomicAdd(&shadowRayCount, 1);

			////////////////////////////////////////////////////////////////////////////////////////////



			//bool shadowed = HitsBVH(gpuNodes, gpuTriIndices, triangleSoA, shading.shadowRayOrigin, shadowRayDirInv, lightDirNormalized, lightDistance);

			//if (!shadowed)
			//{
			//	NdotL *= attenuation;

			//	totalLightr +=
			//		NdotL * light.intensity * light.colorx;

			//	totalLightg +=
			//		NdotL * light.intensity * light.colory;

			//	totalLightb +=
			//		NdotL * light.intensity * light.colorz;
			//}
			float shadowHitDistance = lightDistance;
			int shadowHitTriangleIndex = -1;
			float shadowU = 0.0f;
			float shadowV = 0.0f;

			TraverseBVH(
				gpuNodes,
				gpuTriIndices,
				triangleSoA,
				shading.shadowRayOrigin,
				shadowRayDirInv,
				lightDirNormalized,
				shadowHitDistance,
				shadowHitTriangleIndex,
				shadowU,
				shadowV);

			bool shadowed = shadowHitTriangleIndex != -1 && shadowHitDistance <= lightDistance;

			if (!shadowed)
			{
				NdotL *= attenuation;

				totalLightr += NdotL * light.intensity * light.colorx;
				totalLightg += NdotL * light.intensity * light.colory;
				totalLightb += NdotL * light.intensity * light.colorz;
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
		}

		totalLightr = fminf(totalLightr, 1.0f);
		totalLightg = fminf(totalLightg, 1.0f);
		totalLightb = fminf(totalLightb, 1.0f);



		//	float a = ((packed) & 0xFF) / 255.0f;

		float r = totalLightr;
		float g = totalLightg;
		float b = totalLightb;


		float baryU = hit.barycentricU;
		float baryV = hit.barycentricV;
		float baryW = 1.0f - baryU - baryV;
		float hitU =
			baryW * triangleSoA.UV0u[hit.triangleIndex] +
			baryU * triangleSoA.UV1u[hit.triangleIndex] +
			baryV * triangleSoA.UV2u[hit.triangleIndex];

		float hitV =
			baryW * triangleSoA.UV0v[hit.triangleIndex] +
			baryU * triangleSoA.UV1v[hit.triangleIndex] +
			baryV * triangleSoA.UV2v[hit.triangleIndex];


		Material hitMaterial = gpuMaterials[triangleSoA.materialId[hit.triangleIndex]];
		if (hitMaterial.textureId < 0) //no texture, use vertex color
		{
			Uint32 packed = triangleSoA.packedColor[closestTriangleIndex];
			r *= ((packed >> 24) & 0xFF) / 255.0f;
			g *= ((packed >> 16) & 0xFF) / 255.0f;
			b *= ((packed >> 8) & 0xFF) / 255.0f;

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

		}
		else //texture sampling
		{
			Uint32 sampledColor = SampleTexture(gpuTextures[hitMaterial.textureId], hitU, hitV);

			r *= ((sampledColor >> 24) & 0xFF) / 255.0f;
			g *= ((sampledColor >> 16) & 0xFF) / 255.0f;
			b *= ((sampledColor >> 8) & 0xFF) / 255.0f;

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
		}
	}
	else
	{
		framebuffer[y * (int)screenWidth + x] = 0x000000FF;
	}

}

__device__ static void TraverseBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 cameraPos, Vertice3 invDir, Vertice3 dir, float& closestDistance, int& closestTriangleIndex, float& outu, float& outv)
{

	StackEntry stack[32];
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

		const BVHNode& node = gpuNodes[nodeIndex];

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
__device__ static bool HitsBVH(const BVHNode* gpuNodes, const int* gpuTriIndices, const TriangleSoA triangleSoA, Vertice3 rayStart, Vertice3 inverseDir, Vertice3 dir, float maxDistance)
{

	StackEntry stack[32];
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

		const BVHNode& node = gpuNodes[nodeIndex];

		if (nodeNear > maxDistance)
			continue;
		if (node.triCount == 0)
		{
			int leftChild = node.leftFirst;
			int rightChild = node.leftFirst + 1;

			float leftNear;
			float rightNear;

			bool hitLeft =
				HitAABB(rayStart,
					inverseDir,
					gpuNodes[leftChild].aabb,
					leftNear);
			//aabbcount++;
			bool hitRight =
				HitAABB(rayStart,
					inverseDir,
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
				if (leftNear <= maxDistance)
					stack[stackPtr++] = { leftChild, leftNear };
			}
			else if (hitRight)
			{
				if (rightNear <= maxDistance)
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

				if ((RayIntersectsTriangle(rayStart, dir, intersectionPoint, uu, vv, triangleSoA, triIndex)))
				{
					if (intersectionPoint < maxDistance)
					{

						return true;
					}
				}
			}

		}
	}
	return false;
}

__device__ __forceinline__ ShadingData BuildShadingData(
	const TriangleSoA& triangles,
	const SurfaceHit& hit,
	const Vertice3& cameraPosition,
	const Vertice3& rayDirection)
{
	ShadingData result;

	float w = 1.0f - hit.barycentricU - hit.barycentricV;

	result.normal.x = triangles.N0x[hit.triangleIndex] * w +
		triangles.N1x[hit.triangleIndex] * hit.barycentricU +
		triangles.N2x[hit.triangleIndex] * hit.barycentricV;

	result.normal.y = triangles.N0y[hit.triangleIndex] * w +
		triangles.N1y[hit.triangleIndex] * hit.barycentricU +
		triangles.N2y[hit.triangleIndex] * hit.barycentricV;

	result.normal.z = triangles.N0z[hit.triangleIndex] * w +
		triangles.N1z[hit.triangleIndex] * hit.barycentricU +
		triangles.N2z[hit.triangleIndex] * hit.barycentricV;

	float inverseLength = rsqrtf(result.normal.x * result.normal.x +
		result.normal.y * result.normal.y +
		result.normal.z * result.normal.z);

	result.normal.x *= inverseLength;
	result.normal.y *= inverseLength;
	result.normal.z *= inverseLength;

	result.hitPoint =
		cameraPosition +
		rayDirection * hit.distance;

	if (result.normal.Dot(-rayDirection) < 0.0f)
	{
		result.normal = result.normal * -1.0f;
	}

	result.shadowRayOrigin =
		result.hitPoint +
		result.normal * 0.001f;

	return result;
}


__device__ inline Uint32 SampleTexture(const Texture& texture, float u, float v)
{
	u = u - floorf(u);
	v = v - floorf(v);

	int x = (int)(u * texture.width);
	int y = (int)((1.0f - v) * texture.height);

	if (x >= texture.width)
		x = texture.width - 1;
	if (y >= texture.height)
		y = texture.height - 1;

	return texture.pixels[
		y * texture.width + x];
}