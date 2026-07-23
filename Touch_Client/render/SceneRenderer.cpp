#include "../config/glut_fix.h"
#include "SceneRenderer.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/RelayCore.h"

namespace SceneRenderer {

static RobotModel s_robotModel;

RobotModel& getRobotModel() { return s_robotModel; }

void init() {
    glEnable(GL_NORMALIZE);  // 自动归一化法线（STL 缩放后需要）
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
    float len = 80.0f;    // 轴线长度
    float head = 5.0f;    // 箭头大小
    float hw = 3.0f;      // 箭头半宽

    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // X 轴 (红) — 右手系: X 右
    glColor3f(1.0f, 0.35f, 0.35f); glVertex3f(0, 0, 0); glVertex3f(len, 0, 0);
    // Y 轴 (绿) — 右手系: Y 前
    glColor3f(0.35f, 0.95f, 0.45f); glVertex3f(0, 0, 0); glVertex3f(0, len, 0);
    // Z 轴 (蓝) — 右手系: Z 上
    glColor3f(0.35f, 0.55f, 1.0f); glVertex3f(0, 0, 0); glVertex3f(0, 0, len);
    glEnd();

    // X 轴箭头
    glColor3f(1.0f, 0.35f, 0.35f);
    glBegin(GL_TRIANGLES);
    glVertex3f(len, 0, 0); glVertex3f(len-head,  hw, 0); glVertex3f(len-head, -hw, 0);
    glVertex3f(len, 0, 0); glVertex3f(len-head, 0,  hw); glVertex3f(len-head, 0, -hw);
    glEnd();

    // Y 轴箭头
    glColor3f(0.35f, 0.95f, 0.45f);
    glBegin(GL_TRIANGLES);
    glVertex3f(0, len, 0); glVertex3f( hw, len-head, 0); glVertex3f(-hw, len-head, 0);
    glVertex3f(0, len, 0); glVertex3f(0, len-head,  hw); glVertex3f(0, len-head, -hw);
    glEnd();

    // Z 轴箭头
    glColor3f(0.35f, 0.55f, 1.0f);
    glBegin(GL_TRIANGLES);
    glVertex3f(0, 0, len); glVertex3f( hw, 0, len-head); glVertex3f(-hw, 0, len-head);
    glVertex3f(0, 0, len); glVertex3f(0,  hw, len-head); glVertex3f(0, -hw, len-head);
    glEnd();

    // 轴标签
    glColor3f(1.0f, 0.5f, 0.5f);
    glRasterPos3f(len + 10, 0, 0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'X');

    glColor3f(0.5f, 1.0f, 0.5f);
    glRasterPos3f(0, len + 10, 0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'Y');

    glColor3f(0.5f, 0.7f, 1.0f);
    glRasterPos3f(0, 0, len + 10);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'Z');

    // 原点小球
    glColor3f(0.8f, 0.8f, 0.8f);
    glPushMatrix();
    glutSolidSphere(3.0, 8, 8);
    glPopMatrix();
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

void drawEndEffectorLight(const Vec3& pos, bool isActive) {
    if (isActive) {
        // 按下按钮: 亮白发光球体 + 青色光环 (正在指令机械臂)
        glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
        glPushMatrix();
        glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
        glutSolidSphere(5.0, 16, 16);
        glPopMatrix();

        // 发光光环 (较大、较亮)
        glColor4f(0.25f, 0.85f, 1.0f, 0.6f);
        glPushMatrix();
        glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
        glutWireSphere(10.0, 12, 12);
        glPopMatrix();
    } else {
        // 松开按钮: 暗灰球体 + 暗淡光环 (跟随机械臂实际位置)
        glColor4f(0.55f, 0.60f, 0.65f, 0.7f);
        glPushMatrix();
        glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
        glutSolidSphere(4.0, 12, 12);
        glPopMatrix();

        // 暗淡光环
        glColor4f(0.25f, 0.85f, 1.0f, 0.25f);
        glPushMatrix();
        glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
        glutWireSphere(8.0, 10, 10);
        glPopMatrix();
    }
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

    // 机械臂可视化组: 缩小至 50% + 右移 100mm (适配视口)
    glPushMatrix();
    glTranslatef(100.0f, 0.0f, 0.0f);
    glScalef(0.5f, 0.5f, 0.5f);

    // 机械臂用实际关节角驱动
    s_robotModel.draw(actual);

    // 目标位置标记 (红色线框 — 指令目标)
    Vec3 targetPos(target.x, target.y, target.z);
    drawTargetMarker(targetPos);

    // 实际位置标记 (绿色球 — GetPose 反馈)
    Vec3 actualPos(actual.x, actual.y, actual.z);
    drawActualMarker(actualPos);

    // 末端光点: 始终跟随 targetPos，与机械臂运动逻辑一致
    // targetPos 在按下时累积增量，松开时保持原位，完美反映机械臂实际运动
    bool transmitting = RelayCore::instance().isTransmitting();
    Vec3 lightPos = targetPos;
    drawEndEffectorLight(lightPos, transmitting);

    glPopMatrix();

    // 轨迹 (仅按钮按下时记录)
    drawTrail();
}

} // namespace SceneRenderer
