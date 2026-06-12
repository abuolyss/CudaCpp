#include "MtlImport.h"
#include "Structs.h"
#include <SDL3_image/SDL_image.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>


Texture LoadTexture(std::filesystem::path texturePath);

AssetData ReadMtlFile()
{

	std::cout << "Starting the read of .mtl file." << std::endl;
	AssetData assetData;
	std::unordered_map<std::string, int> textureIds;

	std::ifstream Read("Assets/Level1/Level1.mtl");

	if (!Read.is_open())
	{
		std::cout << "Failed to open the file." << std::endl;

		return AssetData();
	}

	std::unordered_map<std::string, Material> materials;
	std::string currentMaterialName;

	std::string Line;

	while (std::getline(Read, Line)) {

		std::stringstream ss(Line);
		std::string type;
		ss >> type;

		// Output the text from the file
		if (type == "newmtl")
		{
			ss >> currentMaterialName;

			Material material{};

			assetData.materialIds[currentMaterialName] =
				assetData.materialList.size();

			assetData.materialList.push_back(material);
		}
		else if (type == "map_Kd")
		{
			std::string texturePath;
			std::getline(ss >> std::ws, texturePath);

			int textureIndex;

			auto it = textureIds.find(texturePath);

			if (it == textureIds.end())
			{
				textureIndex =
					assetData.textures.size();

				textureIds[texturePath] =
					textureIndex;

				std::filesystem::path fullPath =
					std::filesystem::path("Assets/Level1") /
					texturePath;

				assetData.textures.push_back(
					LoadTexture(fullPath));
			}
			else
			{
				textureIndex = it->second;
			}

			int materialId =
				assetData.materialIds[currentMaterialName];

			assetData.materialList[materialId]
				.textureId = textureIndex;
		}

	}
	std::cout << "Read is done." << std::endl;
	return assetData;
}


Texture LoadTexture(std::filesystem::path texturePath)
{
	SDL_Surface* image = IMG_Load(texturePath.string().c_str());

	if (!image)
	{
		std::cout << SDL_GetError() << '\n';
		return Texture{};
	}

	Texture texture;

	image = SDL_ConvertSurface(
		image,
		SDL_PIXELFORMAT_RGBA8888);

	texture.width = image->w;
	texture.height = image->h;
	texture.pixels = new Uint32[texture.width * texture.height];
	memcpy(texture.pixels, image->pixels, texture.width * texture.height * sizeof(Uint32));


	SDL_DestroySurface(image);

	std::cout
		<< texturePath.filename()
		<< " "
		<< texture.width
		<< "x"
		<< texture.height
		<< std::endl;

	return texture;
}