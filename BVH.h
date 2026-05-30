#pragma once


#include "structs.h"
#include <vector>
#include <algorithm>


struct AABB
{
	Vertice3 min;
	Vertice3 max;
};

struct BVHNode
{
	AABB aabb;

	// if tri count >0 then leaf node leftfirst = which triangle
	// else
	// leftfirst = leftchild
	// leftfirst+1 = right child 
	int leftFirst;
	int triCount;
};


static void ComputeBounds(AABB& aabb, int start, int end, std::vector<int>& triIndices, TriangleSoA& triangleSoA)
{
	float minx = FLT_MAX;
	float miny = FLT_MAX;
	float minz = FLT_MAX;

	float maxx = -FLT_MAX;
	float maxy = -FLT_MAX;
	float maxz = -FLT_MAX;
	for (int i = start; i < end; i++)
	{
		int triIndex = triIndices[i];
		minx = fminf(minx, fminf(triangleSoA.ax[triIndex], fminf(triangleSoA.bx[triIndex], triangleSoA.cx[triIndex])));
		miny = fminf(miny, fminf(triangleSoA.ay[triIndex], fminf(triangleSoA.by[triIndex], triangleSoA.cy[triIndex])));
		minz = fminf(minz, fminf(triangleSoA.az[triIndex], fminf(triangleSoA.bz[triIndex], triangleSoA.cz[triIndex])));
		maxx = fmaxf(maxx, fmaxf(triangleSoA.ax[triIndex], fmaxf(triangleSoA.bx[triIndex], triangleSoA.cx[triIndex])));
		maxy = fmaxf(maxy, fmaxf(triangleSoA.ay[triIndex], fmaxf(triangleSoA.by[triIndex], triangleSoA.cy[triIndex])));
		maxz = fmaxf(maxz, fmaxf(triangleSoA.az[triIndex], fmaxf(triangleSoA.bz[triIndex], triangleSoA.cz[triIndex])));
	}
	aabb.min = Vertice3(minx, miny, minz);
	aabb.max = Vertice3(maxx, maxy, maxz);
}

static void SortBVH(
	std::vector<int>& triIndices,
	int start,
	int end,
	TriangleSoA& triangleSoA)
{
	float minX = FLT_MAX;
	float minY = FLT_MAX;
	float minZ = FLT_MAX;

	float maxX = -FLT_MAX;
	float maxY = -FLT_MAX;
	float maxZ = -FLT_MAX;

	for (int i = start; i < end; i++)
	{
		int tri = triIndices[i];

		float cx =
			(triangleSoA.ax[tri] +
				triangleSoA.bx[tri] +
				triangleSoA.cx[tri]) / 3.0f;

		float cy =
			(triangleSoA.ay[tri] +
				triangleSoA.by[tri] +
				triangleSoA.cy[tri]) / 3.0f;

		float cz =
			(triangleSoA.az[tri] +
				triangleSoA.bz[tri] +
				triangleSoA.cz[tri]) / 3.0f;

		minX = std::min(minX, cx);
		minY = std::min(minY, cy);
		minZ = std::min(minZ, cz);

		maxX = std::max(maxX, cx);
		maxY = std::max(maxY, cy);
		maxZ = std::max(maxZ, cz);
	}

	float extentX = maxX - minX;
	float extentY = maxY - minY;
	float extentZ = maxZ - minZ;

	int axis = 0;

	if (extentY > extentX)
		axis = 1;

	if (extentZ > (axis == 0 ? extentX : extentY))
		axis = 2;

	std::sort(
		triIndices.begin() + start,
		triIndices.begin() + end,
		[&](int a, int b)
		{
			float ax =
				(triangleSoA.ax[a] +
					triangleSoA.bx[a] +
					triangleSoA.cx[a]) / 3.0f;

			float ay =
				(triangleSoA.ay[a] +
					triangleSoA.by[a] +
					triangleSoA.cy[a]) / 3.0f;

			float az =
				(triangleSoA.az[a] +
					triangleSoA.bz[a] +
					triangleSoA.cz[a]) / 3.0f;

			float bx =
				(triangleSoA.ax[b] +
					triangleSoA.bx[b] +
					triangleSoA.cx[b]) / 3.0f;

			float by =
				(triangleSoA.ay[b] +
					triangleSoA.by[b] +
					triangleSoA.cy[b]) / 3.0f;

			float bz =
				(triangleSoA.az[b] +
					triangleSoA.bz[b] +
					triangleSoA.cz[b]) / 3.0f;

			switch (axis)
			{
			case 0:
				return ax < bx;

			case 1:
				return ay < by;

			default:
				return az < bz;
			}
		});
}

static void BuildBVH(int nodeIndex, int Start, int End, std::vector<int>& triIndices, std::vector<BVHNode>& nodes, TriangleSoA& triangleSoA)
{
	BVHNode& node = nodes[nodeIndex];

	int triCount = End - Start;

	ComputeBounds(node.aabb, Start, End, triIndices, triangleSoA);

	if (triCount <= 4)
	{
		node.leftFirst = Start;
		node.triCount = triCount;
		return;
	}

	SortBVH(triIndices, Start, End, triangleSoA);

	int middle = Start + (End - Start) / 2;

	int leftChildIndex = nodes.size();
	nodes.push_back(BVHNode());

	int rightChildIndex = nodes.size();
	nodes.push_back(BVHNode());

	nodes[nodeIndex].leftFirst = leftChildIndex;
	nodes[nodeIndex].triCount = 0;

	BuildBVH(leftChildIndex, Start, middle, triIndices, nodes, triangleSoA);

	BuildBVH(rightChildIndex, middle, End, triIndices, nodes, triangleSoA);
}

static void GenerateBVH(TriangleSoA& triangleSoA, std::vector<BVHNode>& nodes, std::vector<int>& triIndices, int triangleCount)
{

	for (int i = 0; i < triangleCount; i++)
	{
		triIndices.push_back(i);
	}

	nodes.clear();

	nodes.push_back(BVHNode());

	BuildBVH(0, 0, triangleCount, triIndices, nodes, triangleSoA);
}