#include <GL/glut.h>
#include "SceneRenderer.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include "../relay/CoordinateTransform.h"

namespace SceneRenderer {

static RobotModel s_robotModel;

RobotModel& getRobotModel() { return s_robotModel; }

void init() {
    // 加载 STL 模型（如果 models/cr3/ 目录存在），否则自动 fallback 几何体
    if (!s_robotModel.loadModels("models/cr3")) {
        s_robotModel.setFallbackMode();
    }
}

void drawFloor() {
    glColor4f(0.22f, 0.25f, 0.30f, 0.40f);
    glLineWidth(1.0f);

    float size = 300.0f;
    float step = 50.0f;
    int lines = (int)(size / step);

    glBegin(GL_LINES);
    for (int i = -lines; i <= lines; i++) {
        float p = i * step;
        glVertex3f(p, -size, 0);
        glVertex3f(p, size, 0);
        glVertex3f(-size, p, 0);
        glVertex3f(size, p, 0);
    }
    glEnd();
}

void drawAxes() {
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // X (红)
    glColor3f(1.0f, 0.35f, 0.35f); glVertex3f(0, 0, 0); glVertex3f(50, 0, 0);
    // Y (绿)
    glColor3f(0.35f, 0.95f, 0.45f); glVertex3f(0, 0, 0); glVertex3f(0, 50, 0);
    // Z (蓝)
    glColor3f(0.35f, 0.55f, 1.0f); glVertex3f(0, 0, 0); glVertex3f(0, 0, 50);
    glEnd();
}

void drawBoundary() {
    auto& cfg = Config::SAFE_X_MIN; // use sym for brevity; actually reference each
    float xMin = (float)Config::SAFE_X_MIN, xMax = (float)Config::SAFE_X_MAX;
    float yMin = (float)Config::SAFE_Y_MIN, yMax = (float)Config::SAFE_Y_MAX;
    float zMin = (float)Config::SAFE_Z_MIN, zMax = (float)Config::SAFE_Z_MAX;

    glColor4f(1.0f, 0.78f, 0.28f, 0.40f);
    glLineWidth(1.0f);

    // 画安全边界线框
    glBegin(GL_LINE_LOOP);
    glVertex3f(xMin, yMin, zMin); glVertex3f(xMax, yMin, zMin);
    glVertex3f(xMax, yMax, zMin); glVertex3f(xMin, yMax, zMin);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(xMin, yMin, zMax); glVertex3f(xMax, yMin, zMax);
    glVertex3f(xMax, yMax, zMax); glVertex3f(xMin, yMax, zMax);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(xMin, yMin, zMin); glVertex3f(xMin, yMin, zMax);
    glVertex3f(xMax, yMin, zMin); glVertex3f(xMax, yMin, zMax);
    glVertex3f(xMax, yMax, zMin); glVertex3f(xMax, yMax, zMax);
    glVertex3f(xMin, yMax, zMin); glVertex3f(xMin, yMax, zMax);
    glEnd();
}

void drawCursor(const Vec3& pos) {
    glColor4f(1.0f, 1.0f, 1.0f, 0.9f);

    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutSolidSphere(4.0, 16, 16);
    glPopMatrix();

    // 发光光环
    glColor4f(0.25f, 0.85f, 1.0f, 0.5f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutWireSphere(8.0, 12, 12);
    glPopMatrix();
}

void drawTargetMarker(const Vec3& pos) {
    glColor4f(1.0f, 0.3f, 0.3f, 0.5f);
    glLineWidth(2.0f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutWireCube(10.0);
    glPopMatrix();
}

void drawActualMarker(const Vec3& pos) {
    glColor4f(0.3f, 0.9f, 0.4f, 0.9f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutSolidSphere(5.0, 12, 12);
    glPopMatrix();
}

void drawTrail() {
    auto& app = appState;
    EnterCriticalSection(&app.trailMutex);
    if (app.trailPoints.size() < 2) {
        LeaveCriticalSection(&app.trailMutex);
        return;
    }

    glColor4f(0.25f, 0.85f, 1.0f, 0.7f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (auto& p : app.trailPoints) {
        Vec3 v = convertTouchToRobot(p);
        glVertex3f((float)v.x, (float)v.y, (float)v.z);
    }
    glEnd();
    LeaveCriticalSection(&app.trailMutex);
}

void draw3D() {
    drawFloor();
    drawBoundary();
    drawAxes();

    // 机械臂模型
    AppState::RobotPose actual, target;
    {
        EnterCriticalSection(&appState.robotPoseMutex);
        actual = appState.robotActualPose;
        target = appState.robotTargetPose;
        LeaveCriticalSection(&appState.robotPoseMutex);
    }

    // 机械臂用实际位姿驱动
    s_robotModel.draw(actual);

    // 目标位置标记
    Vec3 targetPos(target.x, target.y, target.z);
    Vec3 actualPos(actual.x, actual.y, actual.z);
    drawTargetMarker(targetPos);
    drawActualMarker(actualPos);

    // Touch 光标
    Vec3 mapped;
    EnterCriticalSection(&appState.adjustedPosTableMutex);
    mapped = appState.adjustedPosTable;
    LeaveCriticalSection(&appState.adjustedPosTableMutex);
    drawCursor(mapped);

    // 轨迹
    drawTrail();
}

} // namespace SceneRenderer
