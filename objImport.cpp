#include "objImport.h"
#include "Structs.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <filesystem>


void ReadObjFile( std::vector<Triangle>& triangles, Uint8 red, Uint8 green, Uint8 blue)
{
	std::cout << "Starting the read of .Obj file." << std::endl;

	std::ifstream Read("City.obj");

	if (!Read.is_open())
	{
		std::cout << "Failed to open the file." << std::endl;
		// Handle the error, e.g., print a message and return
		return;
	}
	std::string Line;

	std::vector<Vertice3> vertices;
	std::vector<Vertice3> normals;

	float x, y, z;
	int a, b, c;
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

			triangles.push_back(tri);
		}
	} 
	std::cout << "Read is done." << std::endl;

	std::cout << "Vertices: " << vertices.size() << std::endl;
	std::cout << "Normals: " << normals.size() << std::endl;
	std::cout << "Triangles: " << triangles.size() << std::endl;
}


