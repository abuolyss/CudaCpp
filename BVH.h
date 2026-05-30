#pragma once


#include "structs.h"
#include <vector>


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
