#pragma once

#include "BVH.h"
#include "Structs.h"
#include "lights.h"
#include <cfloat>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>


__global__ void RenderScene(Uint32* framebuffer, int numTriangles, float pixelCount, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp);


__host__ void LaunchRender(Uint32* framebuffer, int numTriangles, float pixelCount, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp)
{
	dim3 blockSize(16, 16);
	dim3 gridSize(
		(pixelCount + blockSize.x - 1) / blockSize.x,
		(pixelCount + blockSize.y - 1) / blockSize.y
	);
	RenderScene << <gridSize, blockSize >> > (framebuffer, numTriangles, pixelCount, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodeCount, cameraForward, cameraRight, cameraUp);

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

__device__ inline bool HitAABB(Vertice3 rayOrigin, Vertice3 rayDir, AABB box)
{
	float tx1 = (box.min.x - rayOrigin.x) / rayDir.x;
	float tx2 = (box.max.x - rayOrigin.x) / rayDir.x;

	float tmin = fminf(tx1, tx2);
	float tmax = fmaxf(tx1, tx2);

	float ty1 = (box.min.y - rayOrigin.y) / rayDir.y;
	float ty2 = (box.max.y - rayOrigin.y) / rayDir.y;

	tmin = fmaxf(tmin, fminf(ty1, ty2));
	tmax = fminf(tmax, fmaxf(ty1, ty2));

	float tz1 = (box.min.z - rayOrigin.z) / rayDir.z;
	float tz2 = (box.max.z - rayOrigin.z) / rayDir.z;

	tmin = fmaxf(tmin, fminf(tz1, tz2));
	tmax = fminf(tmax, fmaxf(tz1, tz2));

	return (tmax > tmin && tmax > 0.0f);
}

__global__ void RenderScene(Uint32* framebuffer, int numTriangles, float pixelsOneDirect, Vertice3 cameraPos, const TriangleSoA triangleSoA, BVHNode* gpuNodes, int* gpuTriIndices, int nodeCount, Vertice3 cameraForward, Vertice3 cameraRight, Vertice3 cameraUp)
{

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= pixelsOneDirect || y >= pixelsOneDirect)
		return;

	float viewportDistance = 1;
	float viewportWidth = 2;
	float viewportHeight = 2;

	int closestTriangleIndex = -1;
	float closestDistance = FLT_MAX;

	float u = (float)x / pixelsOneDirect;
	float v = (float)y / pixelsOneDirect;
	u = u * 2 - 1;
	v = 1.0f - ((float)y / pixelsOneDirect) * 2.0f;

	Vertice3 dir = cameraForward * viewportDistance + cameraRight * (u * viewportWidth) + cameraUp * (v * viewportHeight);

	//gpuNodes;
	//gpuTriIndices;
	dir.NormalizeValue();





	int stack[64];
	int stackPtr = 0;

	float uu, vv;
	float outu, outv;
	stack[stackPtr++] = 0;

	while (stackPtr > 0)
	{
		//if (stackPtr >= 62)
		//	break;
		int nodeIndex = stack[--stackPtr];


		BVHNode node = gpuNodes[nodeIndex];

		if (!HitAABB(cameraPos, dir, node.aabb))
			continue;

		if (node.triCount != 0)
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
		else
		{
			stack[stackPtr++] = node.leftFirst;
			stack[stackPtr++] = node.leftFirst + 1;
		}
	}


	//closestTriangleIndex = -1;
	//closestDistance = FLT_MAX;

	//float intersectionPoint;

	//for (int triIndex = 0; triIndex < numTriangles; triIndex++)
	//{
	//	if (RayIntersectsTriangle(
	//		cameraPos,
	//		dir,
	//		intersectionPoint,
	//		triangleSoA,
	//		triIndex))
	//	{
	//		if (intersectionPoint < closestDistance)
	//		{
	//			closestDistance = intersectionPoint;
	//			closestTriangleIndex = triIndex;
	//		}
	//	}
	//}

	if (closestTriangleIndex != -1)
	{
		PointLight light;
		light.position = Vertice3(0, 70, 50);
		light.color = Vertice3(1, 1, 1);
		light.intensity = 5.0f;
		Vertice3 hitPoint = cameraPos + dir * closestDistance;
		Vertice3 lightDir = light.position - hitPoint;

		lightDir.NormalizeValue();

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



		float invLength = rsqrtf(normalx * normalx + normaly * normaly + normalz * normalz);

		normalx *= invLength;
		normaly *= invLength;
		normalz *= invLength;

		float distanceSq =
			light.position.DistanceSquared(hitPoint);

		float attenuation =
			1.0f / distanceSq;


		float diffuse = fmaxf((normalx * lightDir.x + normaly * lightDir.y + normalz * lightDir.z), 0.0f);
		
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
		
		//Gamma correction belw this
		r = sqrtf(r);
		g = sqrtf(g);
		b = sqrtf(b);



		
		
					//				//								//						//
		framebuffer[y * (int)pixelsOneDirect + x] =
				((Uint32)(r * 255.0f) << 24) |
				((Uint32)(g * 255.0f) << 16) |
				((Uint32)(b * 255.0f) << 8) |   
				0xFF; 
					//				//								//						//

		//framebuffer[y * (int)pixelCount + x] = triangleSoA.packedColor[closestTriangleIndex];


			//int c = closestTriangleIndex & 255;

			//framebuffer[y * (int)pixelsOneDirect + x] =
			//	(c << 24) |
			//	(c << 16) |
			//	(c << 8) |
			//	0xFF;


			//float c = normaly * 0.5f + 0.5f;

			//Uint32 color = (Uint32)(c * 255.0f);

			//framebuffer[y * (int)pixelsOneDirect + x] =
			//	(color << 24) |
			//	(color << 16) |
			//	(color << 8) |
			//	0xFF;
	}
	else
		framebuffer[y * (int)pixelsOneDirect + x] = 0x000000FF;
}
//__host__ __device__ inline void NormalizeValue()
//{
//#ifdef __CUDA_ARCH__
//	float invLength = rsqrtf(x * x + y * y + z * z);
//#else
//	float invLength = 1.0f / sqrtf(x * x + y * y + z * z);
//
//#endif
//	x *= invLength;
//	y *= invLength;
//	z *= invLength;
////}