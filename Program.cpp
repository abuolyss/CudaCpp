#include <SDL3/SDL.h>
//#include <cmath>
#include "Structs.h"
#include <vector>
#include <iostream>
#include <cuda_runtime.h>
#include "Gpu.h"
#include "objImport.h"
#include <SDL3/SDL_mouse.h>
#include "BVH.h"
#include "SDLmanager.cpp"
#include <windows.h>

static void ConvertTriangles(const std::vector<Triangle>& triangles, TriangleSoA& triangleSoA);

static void ComputeBounds(AABB& aabb, int start, int end, std::vector<int>& triIndices, TriangleSoA& triangleSoA);

static void SortBVH(std::vector<int>& triIndices, int start, int end, TriangleSoA& triangleSoA);

static int BuildBVH(int Start, int End, std::vector<int>& triIndices, std::vector<BVHNode>& nodes, TriangleSoA& triangleSoA);

static void GenerateBVH(TriangleSoA& triangleSoA, std::vector<BVHNode>& nodes, std::vector<int>& triIndices, int triangleCount);

static void FPSCounter();

static int screenWidth = 1920;
static int screenHeight = 1080;


int main()
{
	std::cout << "Program launched." << std::endl;

	SDL_Window* window;
	SDL_Renderer* renderer;
	SDL_Texture* texture;


	Uint32* cpuFramebuffer = new Uint32[screenWidth * screenHeight];

	
	
	SDLInitialize(screenWidth, screenHeight, &window, &renderer, &texture);
	
	


	std::cout << "Reading triangle .Obj file." << std::endl;
	std::vector<Triangle> triangles;
	ReadObjFile(triangles, 192, 192, 192);
	//ReadObjFile(triangles, 255, 20, 147);



	std::cout << "Converting triangles into triangle SoA." << std::endl;
	TriangleSoA triangleSoA;
	ConvertTriangles(triangles, triangleSoA);

	std::cout << "Creating the BVH data." << std::endl;

	std::vector<int> triIndices;
	std::vector<BVHNode> nodes;
	GenerateBVH(triangleSoA, nodes, triIndices, triangles.size());



	BVHNode* gpuNodes;
	int* gpuTriIndices;
	
	Uint32* gpuFramebuffer;
	cudaMalloc(&gpuFramebuffer, screenWidth * screenHeight * sizeof(Uint32));

	cudaMalloc(&gpuNodes, nodes.size() * sizeof(BVHNode));
	cudaMemcpy(gpuNodes, nodes.data(), nodes.size() * sizeof(BVHNode), cudaMemcpyHostToDevice);

	cudaMalloc(&gpuTriIndices, triIndices.size() * sizeof(int));
	cudaMemcpy(gpuTriIndices, triIndices.data(), triIndices.size() * sizeof(int), cudaMemcpyHostToDevice);


	//std::cout << triangles.size() << std::endl;
	//std::cout << nodes.size() << std::endl;
	std::cout << "Waiting for profiler..." << std::endl;
	Sleep(2000);

	std::cout << "Starting the game loop." << std::endl;

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
	Vertice3 cameraPos(0, 0, 0);

	SDL_SetWindowRelativeMouseMode(window, true);
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

		FPSCounter();

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

		//auto startg = std::chrono::high_resolution_clock::now();
		LaunchRender(gpuFramebuffer, triangles.size(), screenWidth, screenHeight, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodes.size(), cameraForward, cameraRight, cameraUp);
		//	std::cout<< "gpu + " << std::chrono::high_resolution_clock::now() - startg << std::endl;

			//cudaDeviceSynchronize();
			//cudaError_t err = cudaDeviceSynchronize();

			//std::cout << cudaGetErrorString(err);
			//cudaError_t err;
			//err = cudaGetLastError();
			//std::cout << err;

		SDLRenderLoop(screenWidth, screenHeight, window, renderer, texture, cpuFramebuffer, gpuFramebuffer);
	}
	
	std::cout << "Program closed, freeing the memory." << std::endl << std::endl;
	cudaFree(gpuNodes);
	cudaFree(gpuTriIndices);
	cudaFree(gpuFramebuffer);
	delete[] cpuFramebuffer;
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}

static void FPSCounter()
{
	static int frames = 0;
	static Uint64 lastTime = SDL_GetTicks();
	frames++;
	if (SDL_GetTicks() - lastTime >= 1000)
	{
		std::cout << "FPS: " << frames << std::endl;
		frames = 0;
		lastTime = SDL_GetTicks();
	}
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



