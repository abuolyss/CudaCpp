#pragma once
#include <SDL3/SDL.h>


void SDLRenderLoop(int screenWidth, int screenHeight, SDL_Window* window, SDL_Renderer* renderer, SDL_Texture* texture, Uint32* cpuFramebuffer, Uint32* gpuFramebuffer);
void SDLInitialize(int screenWidth, int screenHeight, SDL_Window** window, SDL_Renderer** renderer, SDL_Texture** texture);