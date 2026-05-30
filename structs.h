#pragma once

#include <SDL3/SDL.h>
#include <cmath>
#include <cuda_runtime.h>
#include <math.h>


struct Vertice3
{
	//coords for vector or vertices
	float x, y, z;

	__host__ __device__ inline Vertice3() : x(0), y(0), z(0) {}

	__host__ __device__ inline Vertice3(float x, float y, float z)
		: x(x), y(y), z(z) {
	}

	__host__ __device__ inline Vertice3 operator+(const Vertice3& other) const
	{
		return Vertice3(x + other.x, y + other.y, z + other.z);
	}

	__host__ __device__ inline Vertice3 operator-(const Vertice3& other) const
	{
		return Vertice3(x - other.x, y - other.y, z - other.z);
	}

	__host__ __device__ inline Vertice3 operator*(float scalar) const
	{
		return Vertice3(x * scalar, y * scalar, z * scalar);
	}

	__host__ __device__ inline void operator+=(const Vertice3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
	}

	__host__ __device__ inline void operator-=(const Vertice3& other)
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
	}

	__host__ __device__ inline float operator*(const Vertice3& other) const
	{
		return fmaf(z, other.z, fmaf(y, other.y, x * other.x));
	}

	__host__ __device__ inline bool operator==(const Vertice3& other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}

	__host__ __device__ inline bool operator!=(const Vertice3& other) const
	{
		return (x != other.x || y != other.y || z != other.z);
	}


	__host__ __device__ inline Vertice3 Cross(const Vertice3& other) const
	{
		return Vertice3(
			y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x
		);
	}
	//is this vector further from another vector
	__host__ __device__ inline float DistanceSquared(const Vertice3& other) const
	{
		Vertice3 diff = *this - other;
		return diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
	}
	__host__ __device__ inline void NormalizeValue()
	{
#ifdef __CUDA_ARCH__
		float invLength = rsqrtf(x * x + y * y + z * z);
#else
		float invLength = 1.0f / sqrtf(x * x + y * y + z * z);

#endif
		x *= invLength;
		y *= invLength;
		z *= invLength;
	}
};

struct Triangle
{
	//coords of each virsune 
	Vertice3 x, y, z;
	//normals
	Vertice3 nx, ny, nz;
	Uint32 color;
	Vertice3 N;

	__host__ __device__ inline Triangle(Vertice3 x = {}, Vertice3 y = {}, Vertice3 z = {}, Uint32 color = 0)
		: x(x), y(y), z(z), color(color) {
		N = (y - x).Cross(z - x);
		N.NormalizeValue();
	}

	__host__ __device__ inline bool operator==(const Triangle& other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}
	__host__ __device__ inline bool operator!=(const Triangle& other) const
	{
		return (x != other.x || y != other.y || z != other.z);
	}
};

struct TriangleSoA
{
	float* ax, * ay, * az;
	float* bx, * by, * bz;
	float* cx, * cy, * cz;

	float* E1x, * E1y, * E1z;
	float* E2x, * E2y, * E2z;

	float* N0x;
	float* N0y;
	float* N0z;

	float* N1x;
	float* N1y;
	float* N1z;

	float* N2x;
	float* N2y;
	float* N2z;

	Uint32* packedColor;
};
