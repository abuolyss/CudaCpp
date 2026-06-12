#include "Structs.h"
#include "objImport.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


void ReadObjFile(std::vector<Triangle>& triangles, AssetData& assetData, Uint8 red, Uint8 green, Uint8 blue)
{
	std::cout << "Starting the read of .Obj file." << std::endl;

	std::ifstream Read("Assets/Level1/Level1.obj");

	if (!Read.is_open())
	{
		std::cout << "Failed to open the file." << std::endl;
		// Handle the error, e.g., print a message and return
		return;
	}
	std::string Line;

	std::vector<Vertice3> vertices;
	std::vector<Vertice3> normals;
	std::vector<UV> uvs;
	std::string currentMaterialName;

	float x, y, z;
	int a, b, c;

	size_t trianglesWithUV = 0;


	auto ParseVertexIndex = [](const std::string& s)
		{
			return std::stoi(s.substr(0, s.find('/')));
		};

	auto ParseNormalIndex = [](const std::string& s)
		{
			size_t first = s.find('/');
			if (first == std::string::npos)
				return -1;

			size_t second = s.find('/', first + 1);
			if (second == std::string::npos)
				return -1;

			return std::stoi(s.substr(second + 1));
		};

	auto ParseUVIndex = [](const std::string& s)
		{
			size_t first = s.find('/');

			if (first == std::string::npos)
				return -1;

			size_t second = s.find('/', first + 1);

			if (second == first + 1)
				return -1;

			return std::stoi(
				s.substr(first + 1, second - first - 1)
			);
		};

	while (std::getline(Read, Line)) {

		std::stringstream ss(Line);
		std::string type;
		ss >> type;

		// Output the text from the file
		if (type == "v") {
			//std::cout << "Vertex: " << Line.substr(2) << std::endl;

			ss >> x >> y >> z;
			vertices.push_back(Vertice3(x, y, z));

		}
		else if (type == "vn")
		{
			ss >> x >> y >> z;

			normals.push_back(
				Vertice3(x, y, z)
			);
		}
		else if (type == "vt")
		{
			float u, v;
			ss >> u >> v;

			uvs.push_back({ u, v });
		}
		else if (type == "usemtl")
		{
			ss >> currentMaterialName;
		}

		else if (type == "f")
		{
			std::string s1, s2, s3;
			ss >> s1 >> s2 >> s3;

			int v0 = ParseVertexIndex(s1);
			int v1 = ParseVertexIndex(s2);
			int v2 = ParseVertexIndex(s3);

			int n0 = ParseNormalIndex(s1);
			int n1 = ParseNormalIndex(s2);
			int n2 = ParseNormalIndex(s3);

			int uv0 = ParseUVIndex(s1);
			int uv1 = ParseUVIndex(s2);
			int uv2 = ParseUVIndex(s3);

			Triangle tri(
				vertices[v0 - 1],
				vertices[v1 - 1],
				vertices[v2 - 1],
				Uint32(red << 24 | green << 16 | blue << 8 | 255)
			);

			if (n0 > 0 && n1 > 0 && n2 > 0)
			{
				tri.nx = normals[n0 - 1];
				tri.ny = normals[n1 - 1];
				tri.nz = normals[n2 - 1];
			}

			if (uv0 > 0 && uv1 > 0 && uv2 > 0)
			{
				trianglesWithUV++;

				tri.uv0 = uvs[uv0 - 1];
				tri.uv1 = uvs[uv1 - 1];
				tri.uv2 = uvs[uv2 - 1];
			}

			tri.materialId = assetData.materialIds[currentMaterialName];

			triangles.push_back(tri);
		}
	}
	std::cout << "Read is done." << std::endl;

	std::cout << "Vertices: " << vertices.size() << std::endl;
	std::cout << "Normals: " << normals.size() << std::endl;
	std::cout << "Triangles: " << triangles.size() << std::endl;
	std::cout << "Triangles with UVs: " << trianglesWithUV << std::endl;
	std::cout << "UVs: " << uvs.size() << std::endl;
	if (!uvs.empty())
	{
		std::cout << "First UV: "
			<< uvs[0].u << ", "
			<< uvs[0].v << std::endl;
	}
}


