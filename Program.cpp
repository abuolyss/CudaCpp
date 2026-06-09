#include "BVH.h"
#include "Gpu.h"
#include "MtlImport.h"
#include "SDLmanager.h"
#include "Structs.h"
#include "lights.h"
#include "objImport.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_mouse.h>
#include <chrono>
#include <cuda_runtime.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <windows.h>



static void UploadLights(const std::vector<PointLight>& cpuLights);

static void ConvertTriangles(const std::vector<Triangle>& triangles, TriangleSoA& triangleSoA);

static void ComputeBounds(AABB& aabb, int start, int end, std::vector<int>& triIndices, TriangleSoA& triangleSoA);

static void SortBVH(std::vector<int>& triIndices, int start, int end, TriangleSoA& triangleSoA);

static int BuildBVH(int Start, int End, std::vector<int>& triIndices, std::vector<BVHNode>& nodes, TriangleSoA& triangleSoA);

static void GenerateBVH(TriangleSoA& triangleSoA, std::vector<BVHNode>& nodes, std::vector<int>& triIndices, int triangleCount);

static void FPSCounter(std::chrono::steady_clock::time_point start, std::chrono::nanoseconds gputime);

static void ReadAssets(AssetData& assetData, Texture*& gpuTextureTable, Material*& gpuMaterialTable);


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




	std::cout << "Reading assets." << std::endl;
	AssetData assetData;

	Texture* gpuTextureTable = nullptr;
	Material* gpuMaterialTable = nullptr;

	ReadAssets(assetData, gpuTextureTable, gpuMaterialTable);


	std::cout << "Reading triangle .Obj file." << std::endl;
	std::vector<Triangle> triangles;

	ReadObjFile(triangles, assetData, 192, 192, 192);

	for (int i = 0; i < assetData.textures.size(); i++)
	{
		std::cout
			<< i << " "
			<< assetData.textures[i].width
			<< "x"
			<< assetData.textures[i].height
			<< '\n';
	}


	std::cout << "Creating the BVH data." << std::endl;

	std::vector<int> triIndices;
	std::vector<BVHNode> nodes;
	GenerateBVH(triangles, nodes, triIndices);

	std::cout << "Converting triangles into triangle SoA." << std::endl;
	TriangleSoA triangleSoA;
	ConvertTriangles(triangles, triangleSoA);


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
	//std::cout << "Waiting for profiler..." << std::endl;
	//Sleep(2000);

	std::cout << "Setting up the lights." << std::endl;


	std::vector<PointLight> cpuLights;

	PointLight light1;
	light1.position.x = 3.5;
	light1.position.y = 6.5;
	light1.position.z = 20.5;

	light1.color.x = 1;
	light1.color.y = 1;
	light1.color.z = 1;

	light1.intensity = 30;

	cpuLights.push_back(light1);

	PointLight light2;
	light2.position.x = 3.5;
	light2.position.y = 6.5;
	light2.position.z = 43;

	light2.color.x = 1;
	light2.color.y = 0;
	light2.color.z = 0;

	light2.intensity = 30;

	cpuLights.push_back(light2);

	PointLight light3;
	light3.position.x = 3.5;
	light3.position.y = 6.5;
	light3.position.z = -25;

	light3.color.x = 0.2;
	light3.color.y = 0;
	light3.color.z = 1;

	light3.intensity = 30;

	cpuLights.push_back(light3);
	PointLight light4;
	light4.position.x = 15;
	light4.position.y = 6.5;
	light4.position.z = 43;

	light4.color.x = 0;
	light4.color.y = 0.8;
	light4.color.z = 1;

	light4.intensity = 30;

	cpuLights.push_back(light4);

	UploadLights(cpuLights);


	//cudaMemcpyToSymbol(
	//	gpuLights,
	//	cpuLights,
	//	sizeof(PointLight));

	int shadowRayCount = 0;
	int hitPixelCount = 0;


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
		auto start = std::chrono::high_resolution_clock::now();
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
		ResetCudaStats();
		auto startg = std::chrono::high_resolution_clock::now();
		LaunchRender(gpuFramebuffer, triangles.size(), screenWidth, screenHeight, cameraPos, triangleSoA, gpuNodes, gpuTriIndices, nodes.size(), cameraForward, cameraRight, cameraUp, gpuMaterialTable, gpuTextureTable);
		auto gputime = std::chrono::high_resolution_clock::now() - startg;

		GetCudaStats(shadowRayCount, hitPixelCount);
		std::cout << "Shadow rays: "
			<< shadowRayCount
			<< ", Hit pixels: "
			<< hitPixelCount
			<< std::endl;

		//cudaDeviceSynchronize();
		//cudaError_t err = cudaDeviceSynchronize();

		//std::cout << cudaGetErrorString(err);
		//cudaError_t err;
		//err = cudaGetLastError();
		//std::cout << err;

		SDLRenderLoop(screenWidth, screenHeight, window, renderer, texture, cpuFramebuffer, gpuFramebuffer);


		FPSCounter(start, gputime);

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

static void FPSCounter(std::chrono::steady_clock::time_point start, std::chrono::nanoseconds gputime)
{
	static int frames = 0;
	static Uint64 lastTime = SDL_GetTicks();
	frames++;
	if (SDL_GetTicks() - lastTime >= 1000)
	{
		std::cout << "FPS: " << frames << std::endl;
		frames = 0;
		lastTime = SDL_GetTicks();
		std::cout << "Cuda time: " << gputime;
		std::cout << ", CPU time: " << std::chrono::high_resolution_clock::now() - gputime - start << std::endl;



	}
}

static void ConvertTriangles(
	const std::vector<Triangle>& triangles,
	TriangleSoA& triangleSoA)
{
	size_t count = triangles.size();

	std::vector<float> ax(count), ay(count), az(count);
	std::vector<float> bx(count), by(count), bz(count);
	std::vector<float> cx(count), cy(count), cz(count);

	std::vector<float> E1x(count), E1y(count), E1z(count);
	std::vector<float> E2x(count), E2y(count), E2z(count);

	std::vector<float> N0x(count), N0y(count), N0z(count);
	std::vector<float> N1x(count), N1y(count), N1z(count);
	std::vector<float> N2x(count), N2y(count), N2z(count);

	std::vector<float> UV0u(count);
	std::vector<float> UV0v(count);

	std::vector<float> UV1u(count);
	std::vector<float> UV1v(count);

	std::vector<float> UV2u(count);
	std::vector<float> UV2v(count);

	std::vector<uint8_t> materialId(count);

	std::vector<Uint32> packedColor(count);

	for (size_t i = 0; i < count; i++)
	{
		const Triangle& tri = triangles[i];

		ax[i] = tri.x.x;
		ay[i] = tri.x.y;
		az[i] = tri.x.z;

		bx[i] = tri.y.x;
		by[i] = tri.y.y;
		bz[i] = tri.y.z;

		cx[i] = tri.z.x;
		cy[i] = tri.z.y;
		cz[i] = tri.z.z;

		E1x[i] = tri.y.x - tri.x.x;
		E1y[i] = tri.y.y - tri.x.y;
		E1z[i] = tri.y.z - tri.x.z;

		E2x[i] = tri.z.x - tri.x.x;
		E2y[i] = tri.z.y - tri.x.y;
		E2z[i] = tri.z.z - tri.x.z;

		N0x[i] = tri.nx.x;
		N0y[i] = tri.nx.y;
		N0z[i] = tri.nx.z;

		N1x[i] = tri.ny.x;
		N1y[i] = tri.ny.y;
		N1z[i] = tri.ny.z;

		N2x[i] = tri.nz.x;
		N2y[i] = tri.nz.y;
		N2z[i] = tri.nz.z;

		UV0u[i] = tri.uv0.u;
		UV0v[i] = tri.uv0.v;

		UV1u[i] = tri.uv1.u;
		UV1v[i] = tri.uv1.v;

		UV2u[i] = tri.uv2.u;
		UV2v[i] = tri.uv2.v;

		materialId[i] = tri.materialId;

		packedColor[i] = tri.color;
	}

	cudaMalloc(&triangleSoA.ax, count * sizeof(float));
	cudaMalloc(&triangleSoA.ay, count * sizeof(float));
	cudaMalloc(&triangleSoA.az, count * sizeof(float));

	cudaMalloc(&triangleSoA.bx, count * sizeof(float));
	cudaMalloc(&triangleSoA.by, count * sizeof(float));
	cudaMalloc(&triangleSoA.bz, count * sizeof(float));

	cudaMalloc(&triangleSoA.cx, count * sizeof(float));
	cudaMalloc(&triangleSoA.cy, count * sizeof(float));
	cudaMalloc(&triangleSoA.cz, count * sizeof(float));

	cudaMalloc(&triangleSoA.E1x, count * sizeof(float));
	cudaMalloc(&triangleSoA.E1y, count * sizeof(float));
	cudaMalloc(&triangleSoA.E1z, count * sizeof(float));

	cudaMalloc(&triangleSoA.E2x, count * sizeof(float));
	cudaMalloc(&triangleSoA.E2y, count * sizeof(float));
	cudaMalloc(&triangleSoA.E2z, count * sizeof(float));

	cudaMalloc(&triangleSoA.N0x, count * sizeof(float));
	cudaMalloc(&triangleSoA.N0y, count * sizeof(float));
	cudaMalloc(&triangleSoA.N0z, count * sizeof(float));

	cudaMalloc(&triangleSoA.N1x, count * sizeof(float));
	cudaMalloc(&triangleSoA.N1y, count * sizeof(float));
	cudaMalloc(&triangleSoA.N1z, count * sizeof(float));

	cudaMalloc(&triangleSoA.N2x, count * sizeof(float));
	cudaMalloc(&triangleSoA.N2y, count * sizeof(float));
	cudaMalloc(&triangleSoA.N2z, count * sizeof(float));

	cudaMalloc(&triangleSoA.UV0u, count * sizeof(float));
	cudaMalloc(&triangleSoA.UV0v, count * sizeof(float));

	cudaMalloc(&triangleSoA.UV1u, count * sizeof(float));
	cudaMalloc(&triangleSoA.UV1v, count * sizeof(float));

	cudaMalloc(&triangleSoA.UV2u, count * sizeof(float));
	cudaMalloc(&triangleSoA.UV2v, count * sizeof(float));

	cudaMalloc(&triangleSoA.materialId, count * sizeof(uint8_t));

	cudaMalloc(&triangleSoA.packedColor, count * sizeof(Uint32));

	cudaMemcpy(triangleSoA.ax, ax.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.ay, ay.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.az, az.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.bx, bx.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.by, by.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.bz, bz.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.cx, cx.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.cy, cy.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.cz, cz.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.E1x, E1x.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.E1y, E1y.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.E1z, E1z.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.E2x, E2x.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.E2y, E2y.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.E2z, E2z.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.N0x, N0x.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N0y, N0y.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N0z, N0z.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.N1x, N1x.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N1y, N1y.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N1z, N1z.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.N2x, N2x.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N2y, N2y.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.N2z, N2z.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.UV0u, UV0u.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.UV0v, UV0v.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.UV1u, UV1u.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.UV1v, UV1v.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.UV2u, UV2u.data(), count * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(triangleSoA.UV2v, UV2v.data(), count * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.materialId, materialId.data(), count * sizeof(uint8_t), cudaMemcpyHostToDevice);

	cudaMemcpy(triangleSoA.packedColor,
		packedColor.data(),
		count * sizeof(Uint32),
		cudaMemcpyHostToDevice);
}

static void UploadLights(const std::vector<PointLight>& cpuLights)
{
	std::vector<PointLightGPU> uploadLights;

	for (const auto& light : cpuLights)
	{
		PointLightGPU gpu;

		gpu.posx = light.position.x;
		gpu.posy = light.position.y;
		gpu.posz = light.position.z;

		gpu.colorx = light.color.x;
		gpu.colory = light.color.y;
		gpu.colorz = light.color.z;

		gpu.intensity = light.intensity;

		uploadLights.push_back(gpu);
	}

	UploadLightsGPU(uploadLights.data(), uploadLights.size());
}

static void ReadAssets(AssetData& assetData, Texture*& gpuTextureTable, Material*& gpuMaterialTable)
{
	assetData = ReadMtlFile();



	// Build CPU texture table that contains GPU pixel pointers
	std::vector<Texture> gpuTextures;
	gpuTextures.resize(assetData.textures.size());

	for (size_t i = 0; i < assetData.textures.size(); i++)
	{
		gpuTextures[i].width =
			assetData.textures[i].width;

		gpuTextures[i].height =
			assetData.textures[i].height;

		size_t size =
			assetData.textures[i].width *
			assetData.textures[i].height *
			sizeof(Uint32);

		cudaMalloc(
			&gpuTextures[i].pixels,
			size);

		cudaMemcpy(
			gpuTextures[i].pixels,
			assetData.textures[i].pixels,
			size,
			cudaMemcpyHostToDevice);
	}

	// Upload texture table
	cudaMalloc(
		&gpuTextureTable,
		gpuTextures.size() *
		sizeof(Texture));

	cudaMemcpy(
		gpuTextureTable,
		gpuTextures.data(),
		gpuTextures.size() *
		sizeof(Texture),
		cudaMemcpyHostToDevice);

	// Upload materials
	cudaMalloc(
		&gpuMaterialTable,
		assetData.materialList.size() *
		sizeof(Material));

	cudaMemcpy(
		gpuMaterialTable,
		assetData.materialList.data(),
		assetData.materialList.size() *
		sizeof(Material),
		cudaMemcpyHostToDevice);
}