#pragma once
#include <GL/glut.h>
#include "StlMesh.h"
#include "../core/AppState.h"

class RobotModel {
public:
    bool loadModels(const char* directory);
    void draw(const AppState::RobotPose& joints);
    void setFallbackMode() { m_useFallback = true; }
    bool isLoaded() const { return m_loaded; }

private:
    void drawLink(const StlMesh& mesh);
    void drawFallbackLink(float w, float h, float d); // cuboid
    void drawFallbackBase();                           // cylinder

    StlMesh m_links[7];  // 0=base, 1~6=links
    bool m_linkLoaded[7] = {};
    bool m_loaded = false;
    bool m_useFallback = false;

    // CR3 geometric parameters (mm) — used for fallback geometry and kinematics
    static constexpr float BASE_HEIGHT = 60.0f;
    static constexpr float LINK1_Z = 100.0f;
    static constexpr float LINK2_LENGTH = 220.0f;
    static constexpr float LINK3_LENGTH = 210.0f;
    static constexpr float LINK4_Z = 50.0f;
    static constexpr float LINK5_Z = 50.0f;
    static constexpr float LINK6_Z = 40.0f;
};
