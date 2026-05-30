#include <SDL3/SDL.h>
//#include <cmath>
#include "Structs.h"
#include <vector>
#include <optional>
#include <iostream>
#include <cuda_runtime.h>
#include "Gpu.h"
#include "objImport.h"
#include <SDL3/SDL_mouse.h>
#include "BVH.h"

static void ConvertTriangles(const std::vector<Triangle>& triangles, TriangleSoA& triangleSoA);

static void ComputeBounds(AABB& aabb, int start, int end, std::vector<int>& triIndices, TriangleSoA& triangleSoA);

static void SortBVH(std::vector<int>& triIndices, int start, int end, TriangleSoA& triangleSoA);

static int BuildBVH(int Start, int End, std::vector<int>& triIndices, std::vector<BVHNode>& nodes, TriangleSoA& triangleSoA);

static void GenerateBVH(TriangleSoA& triangleSoA, std::vector<BVHNode>& nodes, std::vector<int>& triIndices, int triangleCount);

static float pixelCount = 1000;


int main()
{
	std::cout << "Program launched." << std::endl;
	SDL_Init(SDL_INIT_VIDEO);
	SDL_Window* window = SDL_CreateWindow("Render RayTrace", (int)pixelCount, (int)pixelCount, 0);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
	SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, (int)pixelCount, (int)pixelCount);
	SDL_DEFINE_PIXELFORMAT(SDL_PIXELTYPE_PACKED32, SDL_PACKEDORDER_RGBA, SDL_PACKEDLAYOUT_8888, 32, 4);


	Uint32* gpuFramebuffer;
	cudaMallocManaged(&gpuFramebuffer, pixelCount * pixelCount * sizeof(Uint32));
	Vertice3 cameraPos(0, 0, 0);


	std::vector<Triangle> triangles;


	ReadObjFile(triangles, 255, 20, 147);

	std::cout << "Converting triangles into triangle SoA." << std::endl;
	TriangleSoA triangleSoA;
	ConvertTriangles(triangles, triangleSoA);




	std::cout << "Creating the BVH data." << std::endl;
	std::vector<BVHNode> nodes;
	std::vector<int> triIndices;

	for (int i = 0; i < triangles.size(); i++)
	{
		triIndices.push_back(i);
	}

	GenerateBVH(triangleSoA, nodes, triIndices, triangles.size());
	BVHNode* gpuNodes;
	int* gpuTriIndices;

	cudaMalloc(&gpuNodes, nodes.size() * sizeof(BVHNode));
	cudaMemcpy(gpuNodes, nodes.data(), nodes.size() * sizeof(BVHNode), cudaMemcpyHostToDevice);

	cudaMalloc(&gpuTriIndices, triIndices.size() * sizeof(int));
	cudaMemcpy(gpuTriIndices, triIndices.data(), triIndices.size() * sizeof(int), cudaMemcpyHostToDevice);





	std::cout << "Starting the game loop." << std::endl;
	SDL_SetWindowRelativeMouseMode(window, true);

	int frames = 0;
	Uint64 lastTime = SDL_GetTicks();

	SDL_MouseMotionEvent mouseMotion;
	float mouseX, mouseY;
	float yaw = 0.0f;
	float pitch = 0.0f;
	float sensitivity = 0.002f;
	const float speed = 0.5f;
	Vertice3 cameraForward;
	Vertice3 worldUp(0, 1, 0);
	Vertice3 cameraRight;
	Vertice3 cameraUp;

	bool running = true;
	SDL_Event event;
	while (running)
	{
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_EVENT_QUIT)
				running = false;

		}

		const bool* keys = SDL_GetKeyboardState(NULL);

		frames++;



		SDL_GetRelativeMouseState(&mouseX, &mouseY);
		yaw += mouseX * sensitivity;
		pitch -= mouseY * sensitivity;
		if (pitch > 1.5f) pitch = 1.5f;
		if (pitch < -1.5f) pitch = -1.5f;

		cameraForward.x = cosf(pitch) * sinf(yaw);
		cameraForward.y = sinf(pitch);
		cameraForward.z = cosf(pitch) * cosf(yaw);
		cameraForward.NormalizeValue();

		cameraRight = worldUp.Cross(cameraForward);
		cameraRight.NormalizeValue();

		cameraUp = cameraForward.Cross(cameraRight);
		cameraUp.NormalizeValue();



		if (SDL_GetTicks() - lastTime >= 1000)
		{
			std::cout << "FPS: " << frames << std::endl;

			frames = 0;
			lastTime = SDL_GetTicks();
		}

		if (keys[SDL_SCANCODE_ESCAPE])
		{
			running = false;
			continue;
		}
		if (keys[SDL_SCANCODE_W])
		{
			cameraPos += cameraForward * speed;
		}
		if (keys[SDL_SCANCODE_S])
		{
			cameraPos -= cameraForward * speed;
		}
		if (keys[SDL_SCANCODE_A])
		{
			cameraPos -= cameraRight * speed;
		}
		if (keys[SDL_SCANCODE_D])
		{
			cameraPos += cameraRight * speed;
		}
		if (keys[SDL_SCANCODE_SPACE])
		{
			cameraPos.y += speed;
		}

		if (keys[SDL_SCANCODE_LSHIFT])
		{
			cameraPos.y -= speed;
		}


		LaunchRender(gpuFramebuffer, triangles.size(), pixelCount, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodes.size(), cameraForward, cameraRight, cameraUp);
		//cudaDeviceSynchronize();
		//cudaError_t err = cudaDeviceSynchronize();

		//std::cout << cudaGetErrorString(err);
		//cudaError_t err;
		//err = cudaGetLastError();
		//std::cout << err;


		SDL_UpdateTexture(texture, NULL, gpuFramebuffer, (int)pixelCount * sizeof(Uint32));

		SDL_RenderTexture(renderer, texture, NULL, NULL);
		SDL_RenderPresent(renderer);
	}
	std::cout << "Program closed, freeing the memory." << std::endl << std::endl;
	cudaFree(gpuNodes);
	cudaFree(gpuTriIndices);
	cudaFree(gpuFramebuffer);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}


static void ConvertTriangles(const std::vector<Triangle>& triangles, TriangleSoA& triangleSoA)
{
	cudaMallocManaged(&triangleSoA.ax, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.ay, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.az, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.bx, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.by, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.bz, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.cx, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.cy, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.cz, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.E1x, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.E1y, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.E1z, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.E2x, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.E2y, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.E2z, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.N0x, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N0y, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N0z, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.N1x, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N1y, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N1z, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.N2x, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N2y, triangles.size() * sizeof(float));
	cudaMallocManaged(&triangleSoA.N2z, triangles.size() * sizeof(float));

	cudaMallocManaged(&triangleSoA.packedColor, triangles.size() * sizeof(Uint32));

	for (size_t i = 0; i < triangles.size(); i++)
	{
		const Triangle& tri = triangles[i];

		triangleSoA.ax[i] = tri.x.x;
		triangleSoA.ay[i] = tri.x.y;
		triangleSoA.az[i] = tri.x.z;

		triangleSoA.bx[i] = tri.y.x;
		triangleSoA.by[i] = tri.y.y;
		triangleSoA.bz[i] = tri.y.z;

		triangleSoA.cx[i] = tri.z.x;
		triangleSoA.cy[i] = tri.z.y;
		triangleSoA.cz[i] = tri.z.z;

		triangleSoA.E1x[i] = tri.y.x - tri.x.x;
		triangleSoA.E1y[i] = tri.y.y - tri.x.y;
		triangleSoA.E1z[i] = tri.y.z - tri.x.z;

		triangleSoA.E2x[i] = tri.z.x - tri.x.x;
		triangleSoA.E2y[i] = tri.z.y - tri.x.y;
		triangleSoA.E2z[i] = tri.z.z - tri.x.z;

		triangleSoA.N0x[i] = tri.nx.x;
		triangleSoA.N0y[i] = tri.nx.y;
		triangleSoA.N0z[i] = tri.nx.z;

		triangleSoA.N1x[i] = tri.ny.x;
		triangleSoA.N1y[i] = tri.ny.y;
		triangleSoA.N1z[i] = tri.ny.z;

		triangleSoA.N2x[i] = tri.nz.x;
		triangleSoA.N2y[i] = tri.nz.y;
		triangleSoA.N2z[i] = tri.nz.z;

		triangleSoA.packedColor[i] = tri.color;
	}

}




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

static void SortBVH(std::vector<int>& triIndices, int start, int end, TriangleSoA& triangleSoA)
{
	std::sort(
		triIndices.begin() + start,
		triIndices.begin() + end,
		[&](int a, int b)
		{
			float centroidA =
				(triangleSoA.ax[a] +
					triangleSoA.bx[a] +
					triangleSoA.cx[a]) / 3.0f;

			float centroidB =
				(triangleSoA.ax[b] +
					triangleSoA.bx[b] +
					triangleSoA.cx[b]) / 3.0f;

			return centroidA < centroidB;
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
	nodes.clear();

	nodes.push_back(BVHNode());

	BuildBVH(0, 0, triangleCount, triIndices, nodes, triangleSoA);
}