#pragma once
#include "StlMesh.h"

StlMesh loadStl(const char* path);

// Internal implementation (in StlLoader.cpp)
namespace StlLoader {
    StlMesh loadBinary(const char* path);
    StlMesh loadAscii(const char* path);
}
