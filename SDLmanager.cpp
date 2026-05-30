#include <SDL3/SDL.h>
#include "SDLmanager.h"
#include <cuda_runtime.h>

static void SDLInitialize(int screenWidth, int screenHeight, SDL_Window** window, SDL_Renderer** renderer, SDL_Texture** texture)
{
	SDL_Init(SDL_INIT_VIDEO);
	*window = SDL_CreateWindow("Render RayTrace", (int)screenWidth, (int)screenHeight, 0);
	*renderer = SDL_CreateRenderer(*window, NULL);
	*texture = SDL_CreateTexture(*renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, (int)screenWidth, (int)screenHeight);
	SDL_DEFINE_PIXELFORMAT(SDL_PIXELTYPE_PACKED32, SDL_PACKEDORDER_RGBA, SDL_PACKEDLAYOUT_8888, 32, 4);
}

static void SDLRenderLoop(int screenWidth, int screenHeight, SDL_Window* window, SDL_Renderer* renderer, SDL_Texture* texture, Uint32* cpuFramebuffer, Uint32* gpuFramebuffer)
{
	cudaMemcpy(cpuFramebuffer, gpuFramebuffer, screenWidth * screenHeight * sizeof(Uint32), cudaMemcpyDeviceToHost);
	SDL_UpdateTexture(texture, NULL, cpuFramebuffer, (int)screenWidth * sizeof(Uint32));
	//SDL_UpdateTexture(texture, NULL, gpuFramebuffer, (int)pixelCount * sizeof(Uint32));
	//std::cout << "cpu update texture + " << std::chrono::high_resolution_clock::now() - startc << std::endl;

	//startc = std::chrono::high_resolution_clock::now();
	SDL_RenderTexture(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
	//std::cout << "cpu render + " << std::chrono::high_resolution_clock::now() - startc << std::endl;
}