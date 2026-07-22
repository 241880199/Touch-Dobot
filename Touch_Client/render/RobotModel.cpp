#include "RobotModel.h"
#include "StlLoader.h"
#include <cstdio>
#include <iostream>
#include <cmath>

bool RobotModel::loadModels(const char* dir) {
    const char* names[] = { "base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6" };
    char path[512];
    bool anyLoaded = false;

    for (int i = 0; i < 7; i++) {
        snprintf(path, sizeof(path), "%s/%s.STL", dir, names[i]);
        m_links[i] = loadStl(path);
        m_linkLoaded[i] = m_links[i].valid;
        if (m_linkLoaded[i]) anyLoaded = true;
    }

    m_loaded = anyLoaded;
    if (!m_loaded) {
        std::cout << "[RobotModel] STL loading failed, enabling geometric fallback mode" << std::endl;
        m_useFallback = true;
    } else {
        std::cout << "[RobotModel] STL models loaded (" << dir << ")" << std::endl;
    }
    return m_loaded || m_useFallback;
}

void RobotModel::drawLink(const StlMesh& mesh) {
    mesh.draw();
}

void RobotModel::drawLinkOrFallback(int idx, void (*fallbackFn)()) {
    if (m_linkLoaded[idx]) {
        drawLink(m_links[idx]);
    } else if (m_useFallback) {
        fallbackFn();
    }
}

// ===== Fallback geometry (URDF-proportioned primitives, meters like STL) =====

static void drawFallbackLink(float w, float h, float d) {
    glPushMatrix();
    glScalef(w / 2.0f, h / 2.0f, d / 2.0f);
    glutSolidCube(2.0f);
    glPopMatrix();
}

void RobotModel::drawFallbackBase() {
    // Base: wide tapered cylinder, height ≈ J1_Z (0.128m)
    glPushMatrix();
    glRotatef(-90, 1, 0, 0); // gluCylinder Z→Y
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, 0.075, 0.065, 0.12, 16, 1);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void RobotModel::drawFallbackLink1() {
    drawFallbackLink(0.08f, 0.13f, 0.08f);  // waist: height ≈ J1_Z
}

void RobotModel::drawFallbackLink2() {
    drawFallbackLink(0.06f, 0.27f, 0.05f);  // upper arm: length ≈ |J3_X|
}

void RobotModel::drawFallbackLink3() {
    drawFallbackLink(0.045f, 0.23f, 0.04f);  // forearm: length ≈ |J4_X|
}

void RobotModel::drawFallbackLink4() {
    drawFallbackLink(0.03f, 0.05f, 0.03f);
}

void RobotModel::drawFallbackLink5() {
    drawFallbackLink(0.025f, 0.05f, 0.025f);
}

void RobotModel::drawFallbackLink6() {
    drawFallbackLink(0.02f, 0.04f, 0.02f);
}

// ===== Main draw: CR3 URDF kinematics =====

void RobotModel::draw(const AppState::RobotPose& joints) {
    if (!m_loaded && !m_useFallback) return;

    glColor3f(0.25f, 0.28f, 0.32f); // dark grey metallic

    glPushMatrix();

    // Scale: STL/URDF are in meters, scene is in millimeters
    glScalef(MESH_SCALE, MESH_SCALE, MESH_SCALE);

    // === Base (world origin, no transform) ===
    drawLinkOrFallback(0, drawFallbackBase);

    // === Joint 1: waist rotation about Z ===
    // URDF: origin xyz="0 0 0.1283", rpy="0 0 0", axis="0 0 1"
    glTranslatef(0, 0, J1_Z);
    // rpy: yaw=0, pitch=0, roll=0
    glRotatef((float)joints.j1, 0, 0, 1);
    drawLinkOrFallback(1, drawFallbackLink1);

    // === Joint 2: shoulder pitch ===
    // URDF: origin xyz="0 0 0", rpy="1.5708 1.5708 0", axis="0 0 1"
    // rpy=(π/2, π/2, 0) → rotation applied: Rz(0)*Ry(π/2)*Rx(π/2)
    glRotatef(0,   0, 0, 1);  // yaw:    Rz(0°)
    glRotatef(90,  0, 1, 0);  // pitch:  Ry(90°)
    glRotatef(90,  1, 0, 0);  // roll:   Rx(90°)
    glRotatef((float)joints.j2, 0, 0, 1);
    drawLinkOrFallback(2, drawFallbackLink2);

    // === Joint 3: elbow ===
    // URDF: origin xyz="-0.274 0 0", rpy="0 0 0", axis="0 0 1"
    glTranslatef(J3_X, 0, 0);
    // rpy: yaw=0, pitch=0, roll=0
    glRotatef((float)joints.j3, 0, 0, 1);
    drawLinkOrFallback(3, drawFallbackLink3);

    // === Joint 4: wrist 1 ===
    // URDF: origin xyz="-0.23 0 0.1283", rpy="0 0 -1.5708", axis="0 0 1"
    glTranslatef(J4_X, 0, J4_Z);
    glRotatef(-90,  0, 0, 1);  // yaw:    Rz(-90°)
    // pitch=0, roll=0
    glRotatef((float)joints.j4, 0, 0, 1);
    drawLinkOrFallback(4, drawFallbackLink4);

    // === Joint 5: wrist 2 ===
    // URDF: origin xyz="0 -0.116 0", rpy="1.5708 0 0", axis="0 0 1"
    glTranslatef(0, J5_Y, 0);
    // yaw=0, pitch=0
    glRotatef(90,  1, 0, 0);  // roll:   Rx(90°)
    glRotatef((float)joints.j5, 0, 0, 1);
    drawLinkOrFallback(5, drawFallbackLink5);

    // === Joint 6: end flange ===
    // URDF: origin xyz="0 0.105 0", rpy="-1.5708 0 0", axis="0 0 1"
    glTranslatef(0, J6_Y, 0);
    // yaw=0, pitch=0
    glRotatef(-90, 1, 0, 0);  // roll:   Rx(-90°)
    glRotatef((float)joints.j6, 0, 0, 1);
    drawLinkOrFallback(6, drawFallbackLink6);

    glPopMatrix();
}
