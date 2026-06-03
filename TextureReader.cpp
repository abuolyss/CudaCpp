#include <SDL3_image/SDL_image.h>
#include <iostream>
#include "TextureReader.h"

void LoadTexture()
{
	SDL_Surface* image = IMG_Load("test.png");

	if (!image)
	{
		std::cout << SDL_GetError() << '\n';
	}
}