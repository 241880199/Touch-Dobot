#pragma once
#include "../config/glut_fix.h"
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
    void drawLinkOrFallback(int idx, void (*fallbackFn)());
    static void drawFallbackBase();
    static void drawFallbackLink1();
    static void drawFallbackLink2();
    static void drawFallbackLink3();
    static void drawFallbackLink4();
    static void drawFallbackLink5();
    static void drawFallbackLink6();

    StlMesh m_links[7];  // 0=base, 1~6=links
    bool m_linkLoaded[7] = {};
    bool m_loaded = false;
    bool m_useFallback = false;

    // ===== CR3 URDF kinematic parameters =====
    // Joint origins (meters, from DOBOT_6Axis_ROS2_V3/cra_description/urdf/cr3_robot.xacro)
    static constexpr float J1_Z     = 0.1283f;   // joint1 origin: (0, 0, 0.1283)
    static constexpr float J3_X     = -0.274f;   // joint3 origin: (-0.274, 0, 0)
    static constexpr float J4_X     = -0.23f;    // joint4 origin: (-0.23, 0, 0.1283)
    static constexpr float J4_Z     = 0.1283f;
    static constexpr float J5_Y     = -0.116f;   // joint5 origin: (0, -0.116, 0)
    static constexpr float J6_Y     = 0.105f;    // joint6 origin: (0, 0.105, 0)

    // STL mesh scale factor: URDF/STL are in meters, scene is in millimeters
    static constexpr float MESH_SCALE = 1000.0f;
};
