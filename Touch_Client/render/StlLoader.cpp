#include "StlLoader.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "../config/glut_fix.h"

// ===== BBox computation =====
void StlMesh::computeBBox() {
    if (triangles.empty()) return;
    bboxMin[0] = bboxMin[1] = bboxMin[2] = 1e10f;
    bboxMax[0] = bboxMax[1] = bboxMax[2] = -1e10f;
    for (auto& t : triangles) {
        for (int i = 0; i < 3; i++) {
            if (t.v1[i] < bboxMin[i]) bboxMin[i] = t.v1[i];
            if (t.v1[i] > bboxMax[i]) bboxMax[i] = t.v1[i];
            if (t.v2[i] < bboxMin[i]) bboxMin[i] = t.v2[i];
            if (t.v2[i] > bboxMax[i]) bboxMax[i] = t.v2[i];
            if (t.v3[i] < bboxMin[i]) bboxMin[i] = t.v3[i];
            if (t.v3[i] > bboxMax[i]) bboxMax[i] = t.v3[i];
        }
    }
}

// ===== Rendering =====
void StlMesh::draw() const {
    if (!valid) return;
    glBegin(GL_TRIANGLES);
    for (auto& t : triangles) {
        glNormal3fv(t.normal);
        glVertex3fv(t.v1);
        glVertex3fv(t.v2);
        glVertex3fv(t.v3);
    }
    glEnd();
}

// ===== Binary STL parsing =====
StlMesh StlLoader::loadBinary(const char* path) {
    StlMesh mesh;
    FILE* f = fopen(path, "rb");
    if (!f) { std::cerr << "[STL] Cannot open: " << path << std::endl; return mesh; }

    // Skip 80-byte header
    fseek(f, 80, SEEK_SET);

    // Read triangle count
    unsigned int count = 0;
    fread(&count, sizeof(unsigned int), 1, f);

    mesh.triangles.reserve(count);

    for (unsigned int i = 0; i < count; i++) {
        StlTriangle t;
        fread(t.normal, sizeof(float), 3, f);
        fread(t.v1, sizeof(float), 3, f);
        fread(t.v2, sizeof(float), 3, f);
        fread(t.v3, sizeof(float), 3, f);
        fseek(f, 2, SEEK_CUR); // attribute byte count
        mesh.triangles.push_back(t);
    }

    fclose(f);
    mesh.computeBBox();
    mesh.valid = !mesh.triangles.empty();
    std::cout << "[STL] Binary: " << count << " triangles" << std::endl;
    return mesh;
}

// ===== ASCII STL parsing =====
StlMesh StlLoader::loadAscii(const char* path) {
    StlMesh mesh;
    FILE* f = fopen(path, "r");
    if (!f) { std::cerr << "[STL] Cannot open: " << path << std::endl; return mesh; }

    char line[256];
    StlTriangle t;
    int vertexIdx = 0;

    while (fgets(line, sizeof(line), f)) {
        // Skip leading whitespace
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;

        if (strncmp(p, "facet normal", 12) == 0) {
            sscanf_s(p, "facet normal %f %f %f", &t.normal[0], &t.normal[1], &t.normal[2]);
            vertexIdx = 0;
        }
        else if (strncmp(p, "vertex", 6) == 0) {
            float* v = (vertexIdx == 0) ? t.v1 : (vertexIdx == 1) ? t.v2 : t.v3;
            sscanf_s(p, "vertex %f %f %f", &v[0], &v[1], &v[2]);
            vertexIdx++;
            if (vertexIdx == 3) {
                mesh.triangles.push_back(t);
            }
        }
    }

    fclose(f);
    mesh.computeBBox();
    mesh.valid = !mesh.triangles.empty();
    std::cout << "[STL] ASCII: " << mesh.triangles.size() << " triangles" << std::endl;
    return mesh;
}

// ===== Auto-detect format =====
StlMesh loadStl(const char* path) {
    // Read first 5 bytes: if starts with "solid" -> ASCII, otherwise Binary
    FILE* f = fopen(path, "rb");
    if (!f) return StlMesh();

    char header[5] = {};
    fread(header, 1, 5, f);
    fclose(f);

    if (strncmp(header, "solid", 5) == 0) {
        return StlLoader::loadAscii(path);
    } else {
        return StlLoader::loadBinary(path);
    }
}
