#include <GL/glut.h>
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <iostream>
#include <cmath>

#include "config/Config.h"
#include "core/AppState.h"
#include "haptic/HapticDevice.h"
#include "relay/RelayCore.h"
#include "render/SceneRenderer.h"
#include "render/HudOverlay.h"

// ===== GLUT 回调 =====
float g_rotateX = 15.0f, g_rotateY = 10.0f;
float g_camDist = 2.0f;
int g_lastX = 0, g_lastY = 0;
bool g_dragging = false;

void display() {
    if (appState.isClosing) return;

    glClearColor(0.1f, 0.12f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // 3D 投影
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)Config::WINDOW_W / Config::WINDOW_H, 10.0, 2000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 相机：看向工作区中心
    double cx = (Config::SAFE_X_MIN + Config::SAFE_X_MAX) / 2.0;
    double cy = (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0;
    double cz = (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0;
    double camDist = 600.0 * g_camDist;

    gluLookAt(cx, cy - camDist * 0.5, cz + camDist,  // 相机位置
              cx, cy, cz,                               // 看向中心
              0, 0, 1);                                 // 上方向 Z

    glRotatef(g_rotateX, 1, 0, 0);
    glRotatef(g_rotateY, 0, 0, 1);

    // 3D 场景
    SceneRenderer::draw3D();

    // 2D HUD
    HudOverlay::drawAll();

    glutSwapBuffers();

    // 非阻塞反馈处理
    RelayCore::instance().pollFeedback();
}

void idle() {
    if (!appState.isClosing) { glutPostRedisplay(); Sleep(1); }
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        g_dragging = (state == GLUT_DOWN);
        if (g_dragging) { g_lastX = x; g_lastY = y; }
    }
    else if (button == 3 && state == GLUT_DOWN) { // 滚轮上
        g_camDist *= 0.9f;
        if (g_camDist < 0.3f) g_camDist = 0.3f;
    }
    else if (button == 4 && state == GLUT_DOWN) { // 滚轮下
        g_camDist *= 1.1f;
        if (g_camDist > 5.0f) g_camDist = 5.0f;
    }
}

void motion(int x, int y) {
    if (!g_dragging) return;
    g_rotateY += (x - g_lastX) * 0.5f;
    g_rotateX -= (y - g_lastY) * 0.5f;
    if (g_rotateX < -60.0f) g_rotateX = -60.0f;
    if (g_rotateX > 60.0f) g_rotateX = 60.0f;
    g_lastX = x; g_lastY = y;
}

// ===== 定时器 =====
void poseQueryTimer(int) {
    RelayCore::instance().queryPose();
    if (!appState.isClosing) {
        glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    }
}

void alarmCheckTimer(int) {
    RelayCore::instance().checkAlarm();
    if (!appState.isClosing) {
        glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 'q' || key == 'Q' || key == 27) { // q 或 ESC
        std::cout << "\nShutting down..." << std::endl;
        // 注意: 原始 GLUT 3.2 不支持 glutLeaveMainLoop()
        // 在主循环中直接做清理然后 exit
        RelayCore::instance().shutdown();
        cleanupHapticDevice();
        exit(0);
    }
}

// ===== 主函数 =====
int main(int argc, char* argv[]) {
    std::cout << "=== Touch-Dobot Digital Twin System v3.0 ===" << std::endl;
    std::cout << "Robot: " << Config::ROBOT_IP << std::endl;
    std::cout << "Safety: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 1. GLUT 初始化
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::WINDOW_W, Config::WINDOW_H);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Touch-Dobot Digital Twin");

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 2. 初始化 Touch
    std::cout << "Initializing Touch device..." << std::endl;
    if (!initHapticDevice()) {
        std::cerr << "ERROR: Touch device init failed" << std::endl;
        return -1;
    }

    // 3. 通过 Relay 层连接机械臂
    std::cout << "Initializing robot via Relay..." << std::endl;
    if (!RelayCore::instance().init()) {
        std::cerr << "ERROR: Robot init failed" << std::endl;
        cleanupHapticDevice();
        return -1;
    }

    // 4. 初始化 3D 场景
    SceneRenderer::init();

    // 5. 启动定时器
    glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);

    // 6. 进入主循环
    std::cout << "\nSystem ready." << std::endl;
    std::cout << "  Mouse drag: rotate view" << std::endl;
    std::cout << "  Mouse wheel: zoom" << std::endl;
    std::cout << "  q/ESC: quit" << std::endl;
    std::cout << "  Touch button 1: control robot\n" << std::endl;

    glutMainLoop(); // 原始 GLUT 3.2: 此函数不返回，退出通过键盘回调中的 exit(0)

    return 0; // unreachable
}
