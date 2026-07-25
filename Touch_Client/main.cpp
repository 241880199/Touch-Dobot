#include "config/glut_fix.h"
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
#include "safety/RobotDiagnostics.h"

// ===== 运行模式 =====
static bool g_noRobot = false;
static bool g_noTouch = false;

// ===== GLUT 回调 =====

void display() {
    if (appState.isClosing) return;

    // 清屏
    glClearColor(0.1f, 0.12f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ===== 获取实际窗口尺寸 =====
    int winW = glutGet(GLUT_WINDOW_WIDTH);
    int winH = glutGet(GLUT_WINDOW_HEIGHT);
    float scaleX = (float)winW / Config::WINDOW_W;
    float scaleY = (float)winH / Config::WINDOW_H;

    // ===== 3D 场景渲染到右侧视口（按比例缩放）=====
    int vpX = (int)(HudLayout::RIGHT_X * scaleX);
    int vpY3D = (int)(HudLayout::PANEL_Y * scaleY);
    int vpW = (int)(HudLayout::RIGHT_W * scaleX);
    int vpH = (int)(HudLayout::RIGHT_3D_H * scaleY);

    glViewport(vpX, vpY3D, vpW, vpH);
    glScissor(vpX, vpY3D, vpW, vpH);
    glEnable(GL_SCISSOR_TEST);

    glEnable(GL_DEPTH_TEST);

    // 等距 3D 透视视角 — 右手坐标系 (X右 Y前 Z上)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = (double)vpW / vpH;
    gluPerspective(35.0, aspect, 10.0, 5000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 等距视角：从右前上方观察，以基座附近为中心
    gluLookAt(650, -650, 450,   // eye: 右前上方角落
              150,    0, 200,   // center: 基座上方 (机器人工作空间中心)
                0,    0,   1);  // up: Z 向上

    SceneRenderer::draw3D();

    glDisable(GL_SCISSOR_TEST);

    // ===== 2D HUD 全屏渲染（缩放至实际窗口）=====
    glViewport(0, 0, winW, winH);
    HudOverlay::drawAll();

    glutSwapBuffers();

    // 非阻塞反馈处理 (无机械臂时跳过)
    if (!g_noRobot) {
        RelayCore::instance().pollFeedback();
    }
}

void idle() {
    if (!appState.isClosing) {
        glutPostRedisplay();
        // Poll force data at ~30Hz alongside feedback
        RelayCore::instance().pollForce();
        Sleep(1);
    }
}

void reshape(int w, int h) {
    // 窗口大小变化时由 display() 按比例重新计算所有视口
    glutPostRedisplay();
}

// ===== 定时器 (无机械臂模式下跳过) =====
void poseQueryTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().queryPose();
    }
    if (!appState.isClosing) {
        glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    }
}

void alarmCheckTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().checkAlarm();
    }
    if (!appState.isClosing) {
        glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    }
}

void jointAngleTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().queryJointAngles();
    }
    if (!appState.isClosing) {
        glutTimerFunc(200, jointAngleTimer, 0);
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 'q' || key == 'Q' || key == 27) { // q 或 ESC
        std::cout << "\nShutting down..." << std::endl;
        RelayCore::instance().shutdownRelayReporting();
        RobotDiagnostics::instance().shutdown();
        if (!g_noRobot) {
            std::cout << "Disabling robot..." << std::endl;
            RelayCore::instance().shutdown();  // 发送 DisableRobot() + 断开连接
        }
        if (!g_noTouch) cleanupHapticDevice();
        exit(0);
    }
    if (key == 'e' || key == 'E') {
        if (!g_noRobot) {
            std::cout << "\n[Main] 手动触发脱困..." << std::endl;
            RelayCore::instance().triggerEscape();
        }
    }
}

// ===== 主函数 =====
int main(int argc, char* argv[]) {
    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-robot") == 0) {
            g_noRobot = true;
        } else if (strcmp(argv[i], "--no-touch") == 0) {
            g_noTouch = true;
        }
    }

    std::cout << "=== Touch-Dobot Digital Twin System v3.0 ===" << std::endl;
    std::cout << "Robot: " << Config::ROBOT_IP << (g_noRobot ? " (DISABLED)" : "") << std::endl;
    std::cout << "Touch: " << (g_noTouch ? "DISABLED" : "enabled") << std::endl;
    std::cout << "Safety: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 1. GLUT 初始化 (始终执行)
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::WINDOW_W, Config::WINDOW_H);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Touch-Dobot Digital Twin");

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 2. 初始化 Touch 设备 (--no-touch 时跳过)
    if (g_noTouch) {
        std::cout << "Touch device: SKIPPED (--no-touch)" << std::endl;
    } else {
        std::cout << "Initializing Touch device..." << std::endl;
        if (!initHapticDevice()) {
            std::cerr << "ERROR: Touch device init failed" << std::endl;
            std::cerr << "  Use --no-touch to start without Touch device." << std::endl;
            return -1;
        }
    }

    // 3. 连接机械臂 (--no-robot 时跳过)
    if (g_noRobot) {
        std::cout << "Robot: SKIPPED (--no-robot)" << std::endl;
    } else {
        std::cout << "Initializing robot via Relay..." << std::endl;
        if (!RelayCore::instance().init()) {
            std::cerr << "ERROR: Robot init failed" << std::endl;
            std::cerr << "  Use --no-robot to start without robot connection." << std::endl;
            if (!g_noTouch) cleanupHapticDevice();
            return -1;
        }
    }

    // 4. 连接 MATLAB GUI (localhost:8888)
    RelayCore::instance().initRelayReporting();

    // 4.5 启动力传感器实时读取 (30004, 125Hz)
    if (!g_noRobot) {
        RelayCore::instance().initForceReader();
    }

    // 5. 初始化诊断日志
    RobotDiagnostics::instance().init(Config::DIAGNOSTIC_LOG_PATH);

    // 6. 初始化 3D 场景 (始终执行)
    SceneRenderer::init();

    // 7. 启动定时器
    glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    glutTimerFunc(500, jointAngleTimer, 0);

    // 8. 进入主循环
    std::cout << "\nSystem ready." << std::endl;
    std::cout << "  q/ESC: quit" << std::endl;
    if (!g_noTouch && !g_noRobot) {
        std::cout << "  Touch button 1: control robot" << std::endl;
    }
    std::cout << std::endl;

    glutMainLoop();

    return 0; // unreachable
}
