#pragma once


#include "structs.h"
#include <algorithm>
#include <vector>


struct AABB
{
	Vertice3 min;
	Vertice3 max;
};

struct BVHNode
{
	AABB aabb;

	int triCount;
	// if tri count >0 then leaf node leftfirst = which triangle
	// else
	// leftfirst = leftchild
	// leftfirst+1 = right child 
	int leftFirst;
};

struct TriangleBuildData
{
	Vertice3 centroid;
	AABB bounds;
};


static void ComputeBounds(AABB& aabb, int start, int end, std::vector<int>& triIndices,  const std::vector<TriangleBuildData>& buildData)
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
		minx = fminf(minx, buildData[triIndex].bounds.min.x);
		miny = fminf(miny, buildData[triIndex].bounds.min.y);
		minz = fminf(minz, buildData[triIndex].bounds.min.z);
		maxx = fmaxf(maxx, buildData[triIndex].bounds.max.x);
		maxy = fmaxf(maxy, buildData[triIndex].bounds.max.y);
		maxz = fmaxf(maxz, buildData[triIndex].bounds.max.z);
	}
	aabb.min = Vertice3(minx, miny, minz);
	aabb.max = Vertice3(maxx, maxy, maxz);
}

static void SortBVH(
	std::vector<int>& triIndices,
	int start,
	int end,
	const std::vector<Triangle>& triangles)
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
			(triangles[tri].x.x + triangles[tri].y.x + triangles[tri].z.x) / 3.0f;

		float cy =
			(triangles[tri].x.y + triangles[tri].y.y + triangles[tri].z.y) / 3.0f;

		float cz =
			(triangles[tri].x.z + triangles[tri].y.z + triangles[tri].z.z) / 3.0f;


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
				(triangles[a].x.x + triangles[a].y.x + triangles[a].z.x) / 3.0f;

			float ay =
				(triangles[a].x.y + triangles[a].y.y + triangles[a].z.y) / 3.0f;

			float az =
				(triangles[a].x.z + triangles[a].y.z + triangles[a].z.z) / 3.0f;

			float bx =
				(triangles[b].x.x + triangles[b].y.x + triangles[b].z.x) / 3.0f;

			float by =
				(triangles[b].x.y + triangles[b].y.y + triangles[b].z.y) / 3.0f;

			float bz =
				(triangles[b].x.z + triangles[b].y.z + triangles[b].z.z) / 3.0f;



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

static void BuildBVH(int nodeIndex, int Start, int End, std::vector<int>& triIndices, std::vector<BVHNode>& nodes, const std::vector<Triangle>& triangles, const std::vector<TriangleBuildData>& buildData)
{

	//	1. Precompute triangle centroids
	//	2. Precompute triangle AABBs
	//	3. Implement 16 - bin SAH on longest axis
	//	4. Replace SortBVH + middle split
	//	5. Benchmark
	//	6. Extend SAH to evaluate X, Y, Z

	BVHNode& node = nodes[nodeIndex];

	int triCount = End - Start;

	ComputeBounds(node.aabb, Start, End, triIndices, buildData);

	if (triCount <= 4)
	{
		node.leftFirst = Start;
		node.triCount = triCount;
		return;
	}

	SortBVH(triIndices, Start, End, triangles);

	int middle = Start + (End - Start) / 2;

	int leftChildIndex = nodes.size();
	nodes.push_back(BVHNode());

	int rightChildIndex = nodes.size();
	nodes.push_back(BVHNode());

	nodes[nodeIndex].leftFirst = leftChildIndex;
	nodes[nodeIndex].triCount = 0;

	BuildBVH(leftChildIndex, Start, middle, triIndices, nodes, triangles, buildData);

	BuildBVH(rightChildIndex, middle, End, triIndices, nodes, triangles, buildData);
}

static void GenerateBVH(const std::vector<Triangle>& triangles, std::vector<BVHNode>& nodes, std::vector<int>& triIndices)
{
	std::vector<TriangleBuildData> buildData(triangles.size());

	for (int i = 0; i < triangles.size(); i++)
	{
		buildData[i].centroid =
		{
			(triangles[i].x.x + triangles[i].y.x + triangles[i].z.x) / 3.0f,
			(triangles[i].x.y + triangles[i].y.y + triangles[i].z.y) / 3.0f,
			(triangles[i].x.z + triangles[i].y.z + triangles[i].z.z) / 3.0f
		};
		float minx = fminf(triangles[i].x.x, fminf(triangles[i].y.x, triangles[i].z.x));

		float miny = fminf(triangles[i].x.y, fminf(triangles[i].y.y, triangles[i].z.y));

		float minz = fminf(triangles[i].x.z, fminf(triangles[i].y.z, triangles[i].z.z));

		float maxx = fmaxf(triangles[i].x.x, fmaxf(triangles[i].y.x, triangles[i].z.x));

		float maxy = fmaxf(triangles[i].x.y, fmaxf(triangles[i].y.y, triangles[i].z.y));

		float maxz = fmaxf(triangles[i].x.z, fmaxf(triangles[i].y.z, triangles[i].z.z));

		buildData[i].bounds.min = Vertice3(minx, miny, minz);
		buildData[i].bounds.max = Vertice3(maxx, maxy, maxz);
	}

	for (int i = 0; i < triangles.size(); i++)
	{
		triIndices.push_back(i);
	}

	nodes.clear();

	nodes.push_back(BVHNode());

	BuildBVH(0, 0, triangles.size(), triIndices, nodes, triangles, buildData);
}