#pragma once
#include <vector>

struct StlTriangle {
    float normal[3];
    float v1[3], v2[3], v3[3];
};

struct StlMesh {
    std::vector<StlTriangle> triangles;
    float bboxMin[3] = { 0, 0, 0 };
    float bboxMax[3] = { 0, 0, 0 };
    bool valid = false;

    void computeBBox();
    void draw() const; // immediate-mode OpenGL rendering
};
