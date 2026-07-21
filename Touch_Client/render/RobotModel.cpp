#include "RobotModel.h"
#include "StlLoader.h"
#include <cstdio>
#include <iostream>
#include <cmath>

bool RobotModel::loadModels(const char* dir) {
    const char* names[] = { "base", "link1", "link2", "link3", "link4", "link5", "link6" };
    char path[512];
    bool anyLoaded = false;

    for (int i = 0; i < 7; i++) {
        snprintf(path, sizeof(path), "%s/%s.stl", dir, names[i]);
        m_links[i] = loadStl(path);
        m_linkLoaded[i] = m_links[i].valid;
        if (m_linkLoaded[i]) anyLoaded = true;
    }

    m_loaded = anyLoaded;
    if (!m_loaded) {
        std::cout << "[RobotModel] STL loading failed, enabling geometric fallback mode" << std::endl;
        m_useFallback = true;
    } else {
        std::cout << "[RobotModel] loading complete" << std::endl;
    }
    return m_loaded || m_useFallback;
}

void RobotModel::drawLink(const StlMesh& mesh) {
    mesh.draw();
}

void RobotModel::drawFallbackLink(float w, float h, float d) {
    glPushMatrix();
    glScalef(w / 2.0f, h / 2.0f, d / 2.0f);
    glutSolidCube(2.0f);
    glPopMatrix();
}

void RobotModel::drawFallbackBase() {
    // base with flange
    glPushMatrix();
    glRotatef(-90, 1, 0, 0); // gluCylinder defaults to Z axis, rotate to Y
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, 60, 70, BASE_HEIGHT, 16, 1);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void RobotModel::draw(const AppState::RobotPose& joints) {
    if (!m_loaded && !m_useFallback) return;

    glColor3f(0.25f, 0.28f, 0.32f); // dark grey metallic

    glPushMatrix();

    // === Base (world origin, no rotation or translation) ===
    if (m_linkLoaded[0])        drawLink(m_links[0]);
    else if (m_useFallback)     drawFallbackBase();

    // === J1: rotate around Z (waist) ===
    glTranslatef(0, 0, BASE_HEIGHT + LINK1_Z * 0.5f);
    glRotatef((float)joints.rz, 0, 0, 1);
    glTranslatef(0, 0, -LINK1_Z * 0.5f);
    if (m_linkLoaded[1])        drawLink(m_links[1]);
    else if (m_useFallback)     drawFallbackLink(80, LINK1_Z, 80);

    // === J2: rotate around Y (shoulder) ===
    glTranslatef(0, 0, LINK1_Z);
    glRotatef((float)joints.ry, 0, 1, 0);
    glTranslatef(0, 0, LINK2_LENGTH * 0.3f);
    if (m_linkLoaded[2])        drawLink(m_links[2]);
    else if (m_useFallback)     drawFallbackLink(60, LINK2_LENGTH, 50);

    // === J3: rotate around Y (elbow) ===
    glTranslatef(0, 0, LINK2_LENGTH * 0.7f);
    glRotatef((float)(joints.ry * 0.5), 0, 1, 0); // approximate: J3 coupled with J2
    glTranslatef(0, 0, LINK3_LENGTH * 0.3f);
    if (m_linkLoaded[3])        drawLink(m_links[3]);
    else if (m_useFallback)     drawFallbackLink(45, LINK3_LENGTH, 40);

    // === J4: rotate around Z (wrist 1) ===
    glTranslatef(0, 0, LINK3_LENGTH * 0.7f);
    glRotatef((float)joints.rx, 0, 0, 1);
    glTranslatef(0, 0, LINK4_Z * 0.5f);
    if (m_linkLoaded[4])        drawLink(m_links[4]);
    else if (m_useFallback)     drawFallbackLink(30, LINK4_Z, 30);

    // === J5: rotate around Y (wrist 2) ===
    glTranslatef(0, 0, LINK4_Z);
    glRotatef((float)(joints.ry * 0.3), 0, 1, 0);
    glTranslatef(0, 0, LINK5_Z * 0.5f);
    if (m_linkLoaded[5])        drawLink(m_links[5]);
    else if (m_useFallback)     drawFallbackLink(25, LINK5_Z, 25);

    // === J6: rotate around Z (end effector) ===
    glTranslatef(0, 0, LINK5_Z);
    glRotatef((float)joints.rz, 0, 0, 1);
    glTranslatef(0, 0, LINK6_Z * 0.5f);
    if (m_linkLoaded[6])        drawLink(m_links[6]);
    else if (m_useFallback)     drawFallbackLink(20, LINK6_Z, 20);

    glPopMatrix();
}
