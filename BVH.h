#pragma once


#include "structs.h"
#include <algorithm>
#include <vector>
#include <iostream>



struct AABB
{
	Vertice3 min;
	Vertice3 max;
};

struct BVHNode
{
	AABB aabb;

	int triCount = 0;
	int leftFirst = 0;
};

struct TriangleBuildData
{
	Vertice3 centroid;
	AABB bounds;
};

constexpr int BINAABB_COUNT = 16;

struct Bin
{
	AABB bounds;
	int count = 0;
};

static float SurfaceArea(const AABB& box)
{
	Vertice3 e = box.max - box.min;

	return 2.0f *
		(
			e.x * e.y +
			e.x * e.z +
			e.y * e.z
			);
}

static void InitEmptyAABB(AABB& box)
{
	box.min = Vertice3(
		FLT_MAX,
		FLT_MAX,
		FLT_MAX);

	box.max = Vertice3(
		-FLT_MAX,
		-FLT_MAX,
		-FLT_MAX);
}

static void ExpandAABB(AABB& dst, const AABB& src)
{
	dst.min.x = fminf(dst.min.x, src.min.x);
	dst.min.y = fminf(dst.min.y, src.min.y);
	dst.min.z = fminf(dst.min.z, src.min.z);

	dst.max.x = fmaxf(dst.max.x, src.max.x);
	dst.max.y = fmaxf(dst.max.y, src.max.y);
	dst.max.z = fmaxf(dst.max.z, src.max.z);
}

struct SAHBin
{
	AABB bounds;
	int count;
};

static bool FindBestSplitSAH(
	int start,
	int end,
	const std::vector<int>& triIndices,
	const std::vector<TriangleBuildData>& buildData,
	int& outAxis,
	int& outSplit)
{
	float bestCost = FLT_MAX;

	outAxis = -1;
	outSplit = -1;

	for (int axis = 0; axis < 3; axis++)
	{
		float centroidMin = FLT_MAX;
		float centroidMax = -FLT_MAX;

		for (int i = start; i < end; i++)
		{
			int tri = triIndices[i];

			float c =
				axis == 0 ? buildData[tri].centroid.x :
				axis == 1 ? buildData[tri].centroid.y :
				buildData[tri].centroid.z;

			centroidMin = std::min(centroidMin, c);
			centroidMax = std::max(centroidMax, c);
		}

		float extent =
			centroidMax - centroidMin;

		if (extent < 1e-6f)
			continue;

		SAHBin bins[BINAABB_COUNT];

		for (int i = 0; i < BINAABB_COUNT; i++)
		{
			bins[i].count = 0;
			InitEmptyAABB(bins[i].bounds);
		}

		float scale =
			(float)BINAABB_COUNT / extent;

		for (int i = start; i < end; i++)
		{
			int tri = triIndices[i];

			float c =
				axis == 0 ? buildData[tri].centroid.x :
				axis == 1 ? buildData[tri].centroid.y :
				buildData[tri].centroid.z;

			int bin =
				(int)((c - centroidMin) * scale);

			bin =
				std::max(0,
					std::min(bin, BINAABB_COUNT - 1));

			bins[bin].count++;

			ExpandAABB(
				bins[bin].bounds,
				buildData[tri].bounds);
		}

		AABB leftBounds[BINAABB_COUNT - 1];
		AABB rightBounds[BINAABB_COUNT - 1];

		int leftCount[BINAABB_COUNT - 1];
		int rightCount[BINAABB_COUNT - 1];

		AABB running;
		InitEmptyAABB(running);

		int count = 0;

		for (int i = 0; i < BINAABB_COUNT - 1; i++)
		{
			count += bins[i].count;

			ExpandAABB(
				running,
				bins[i].bounds);

			leftBounds[i] = running;
			leftCount[i] = count;
		}

		InitEmptyAABB(running);

		count = 0;

		for (int i = BINAABB_COUNT - 1; i > 0; i--)
		{
			count += bins[i].count;

			ExpandAABB(
				running,
				bins[i].bounds);

			rightBounds[i - 1] = running;
			rightCount[i - 1] = count;
		}

		for (int split = 0;
			split < BINAABB_COUNT - 1;
			split++)
		{
			if (leftCount[split] == 0 ||
				rightCount[split] == 0)
				continue;

			float cost =
				SurfaceArea(leftBounds[split]) *
				leftCount[split]
				+
				SurfaceArea(rightBounds[split]) *
				rightCount[split];

			if (cost < bestCost)
			{
				bestCost = cost;
				outAxis = axis;
				outSplit = split;
			}
		}
	}

	return outAxis != -1;
}
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


	int axis;
	int split;

	bool success =
		FindBestSplitSAH(
			Start,
			End,
			triIndices,
			buildData,
			axis,
			split);


	std::vector<int>::iterator midIter;

	if (success)
	{
		midIter = std::partition(
			triIndices.begin() + Start,
			triIndices.begin() + End,
			[&](int tri)
			{
				float c;

				if (axis == 0)
					c = buildData[tri].centroid.x;
				else if (axis == 1)
					c = buildData[tri].centroid.y;
				else
					c = buildData[tri].centroid.z;

				float axisMin =
					axis == 0 ? node.aabb.min.x :
					axis == 1 ? node.aabb.min.y :
					node.aabb.min.z;

				float axisMax =
					axis == 0 ? node.aabb.max.x :
					axis == 1 ? node.aabb.max.y :
					node.aabb.max.z;

				float scale =
					(float)BINAABB_COUNT /
					(axisMax - axisMin);

				int bin =
					(int)((c - axisMin) * scale);

				bin = std::max(0, std::min(bin, BINAABB_COUNT - 1));

				return bin <= split;
			});
	}
	else
	{
		std::cout << "SAH failed to find a good split, falling back to middle split." << std::endl;
		SortBVH(triIndices, Start, End, triangles);
		midIter = triIndices.begin() + Start + (End - Start) / 2;
	}

	int middle = (int)(midIter - triIndices.begin());

	if (middle == Start || middle == End)
	{
		SortBVH(triIndices, Start, End, triangles);
		middle = Start + triCount / 2;
	}
	//SortBVH(triIndices, Start, End, triangles);

	//int middle = Start + (End - Start) / 2;

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
