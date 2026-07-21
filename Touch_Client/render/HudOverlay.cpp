#include "HudOverlay.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include "../relay/RelayCore.h"
#include "../config/glut_fix.h"
#include <cstdio>
#include <cstring>

namespace HudOverlay {

// 投影到 2D 并绘制字符串
static void text2D(int x, int y, const char* text, void* font = GLUT_BITMAP_9_BY_15) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, Config::WINDOW_W, 0, Config::WINDOW_H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glRasterPos2i(x, y);
    for (const char* c = text; *c; c++) {
        glutBitmapCharacter(font, *c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawAll() {
    auto& app = appState;
    auto& relay = RelayCore::instance();
    char buf[256];

    glDisable(GL_DEPTH_TEST);

    // === 左上：坐标面板 ===
    int panelX = 15;
    int panelY = Config::WINDOW_H - 20;
    int lineH = 20;

    // 背景
    glColor4f(0.10f, 0.13f, 0.17f, 0.82f);
    glRecti(panelX - 5, panelY - 140, panelX + 240, panelY + 5);

    auto drawLine = [&](int& y, const char* label, double a, double b, double c) {
        snprintf(buf, sizeof(buf), "%s  (%6.1f, %6.1f, %6.1f)", label, a, b, c);
        glColor3f(0.92f, 0.96f, 1.00f);
        text2D(panelX, y, buf);
        y -= lineH;
    };

    int y = panelY;
    hduVector3Dd raw;
    Vec3 mapped;
    AppState::RobotPose target, actual;
    {
        EnterCriticalSection(&app.devicePosMutex);
        raw = app.devicePos;
        LeaveCriticalSection(&app.devicePosMutex);
        EnterCriticalSection(&app.adjustedPosTableMutex);
        mapped = app.adjustedPosTable;
        LeaveCriticalSection(&app.adjustedPosTableMutex);
        EnterCriticalSection(&app.robotPoseMutex);
        target = app.robotTargetPose;
        actual = app.robotActualPose;
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    drawLine(y, "Touch Raw:   ", raw[0], raw[1], raw[2]);
    drawLine(y, "Touch Mapped:", mapped.x, mapped.y, mapped.z);
    drawLine(y, "Robot Target:", target.x, target.y, target.z);
    drawLine(y, "Robot Actual:", actual.x, actual.y, actual.z);
    drawLine(y, "Delta:       ",
        actual.x - target.x, actual.y - target.y, actual.z - target.z);

    // 传输状态
    const char* state = relay.isTransmitting() ? "TX: ACTIVE" : "TX: IDLE";
    glColor3f(relay.isTransmitting() ? 0.35f : 0.62f,
              relay.isTransmitting() ? 0.90f : 0.68f,
              relay.isTransmitting() ? 0.50f : 0.78f);
    text2D(panelX, y, state);

    // === 右上：状态栏 ===
    int sx = Config::WINDOW_W - 240;
    int sy = Config::WINDOW_H - 20;

    const char* conn29999 = app.isRobotConnected ? "CONNECTED" : "DISCONNECTED";
    const char* conn30003 = app.isRobotConnected ? "CONNECTED" : "DISCONNECTED";
    const char* mode = app.isRobotInAlarm ? "ALARM" : "ENABLED";
    float lat = app.latencyMs;

    snprintf(buf, sizeof(buf), "TCP 29999: %s", conn29999);
    glColor3f(app.isRobotConnected ? 0.35f : 1.0f, app.isRobotConnected ? 0.90f : 0.35f, 0.50f);
    text2D(sx, sy, buf);

    snprintf(buf, sizeof(buf), "TCP 30003: %s", conn30003);
    text2D(sx, sy - lineH, buf);

    snprintf(buf, sizeof(buf), "Robot Mode: %s", mode);
    glColor3f(app.isRobotInAlarm ? 1.0f : 0.35f, app.isRobotInAlarm ? 0.35f : 0.90f, 0.50f);
    text2D(sx, sy - 2 * lineH, buf);

    snprintf(buf, sizeof(buf), "Latency: %.1f ms", lat);
    glColor3f(0.92f, 0.96f, 1.00f);
    text2D(sx, sy - 3 * lineH, buf);

    // === 底部：最后指令 ===
    char lastCmd[256];
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(lastCmd, app.lastCommandSent, sizeof(lastCmd) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    int cx = Config::WINDOW_W / 2 - 150;
    snprintf(buf, sizeof(buf), "Last CMD: %s", lastCmd[0] ? lastCmd : "(none)");
    glColor3f(0.62f, 0.68f, 0.78f);
    text2D(cx, 25, buf);

    glEnable(GL_DEPTH_TEST);
}

} // namespace HudOverlay
