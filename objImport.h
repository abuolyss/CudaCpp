#pragma once

#include "objImport.h"
#include "Structs.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <filesystem>

void ReadObjFile(std::vector<Triangle>& triangles, Uint8 red, Uint8 green, Uint8 blue);