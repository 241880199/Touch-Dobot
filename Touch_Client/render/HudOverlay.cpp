#include "HudOverlay.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <GL/glut.h>
#include <cstdio>
#include <cstring>
#include "../safety/SafetyPredictor.h"

namespace HudOverlay {

// ===== 绘制工具函数 =====

static void text2D(int x, int y, const char* text, void* font = GLUT_BITMAP_9_BY_15) {
    glRasterPos2i(x, y);
    for (const char* c = text; *c; c++) {
        glutBitmapCharacter(font, *c);
    }
}

static void drawPanelBg(int x, int y, int w, int h) {
    glColor4f(0.08f, 0.10f, 0.14f, 0.90f);
    glRecti(x, y, x + w, y + h);
    glColor4f(0.25f, 0.35f, 0.55f, 0.70f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(x, y); glVertex2i(x + w, y);
    glVertex2i(x + w, y + h); glVertex2i(x, y + h);
    glEnd();
}

static void drawPanelTitle(int x, int y, int w, const char* title) {
    // 标题背景
    glColor4f(0.12f, 0.22f, 0.42f, 0.90f);
    glRecti(x + 1, y - 20, x + w - 1, y + 1);
    // 标题文字
    glColor3f(0.90f, 0.94f, 1.00f);
    text2D(x + 6, y - 5, title, GLUT_BITMAP_9_BY_15);
}

static void drawSeparatorLine(int x1, int x2, int y) {
    glColor4f(0.25f, 0.35f, 0.55f, 0.50f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2i(x1, y); glVertex2i(x2, y);
    glEnd();
}

// ===== 顶部状态栏 =====

static void drawTopBar() {
    int w = Config::WINDOW_W;
    int h = HudLayout::TOP_BAR_H;

    // 背景
    glColor4f(0.06f, 0.08f, 0.12f, 0.95f);
    glRecti(0, 0, w, h);
    glColor4f(0.30f, 0.40f, 0.60f, 0.60f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2i(0, h); glVertex2i(w, h);
    glEnd();

    auto& app = appState;
    char buf[256];

    // 左侧: 系统标题
    glColor3f(0.50f, 0.75f, 1.00f);
    text2D(8, 8, "Touch-Dobot Relay Station", GLUT_BITMAP_HELVETICA_12);

    // 中偏左: Touch↔Relay 延迟 (本机localhost，近似0)
    int delayX = 260;
    float touchLat = 0.0f; // 单机模式：Touch 直连，延迟 ~0
    bool tcp29999 = app.isRobotConnected;
    bool tcp30003 = app.isRobotConnected;
    float robotLat = app.latencyMs;

    snprintf(buf, sizeof(buf), "Touch->Relay: %.1f ms | ", touchLat);
    glColor3f(tcp29999 ? 0.35f : 1.0f, tcp29999 ? 0.90f : 0.35f, 0.50f);
    text2D(delayX, 18, buf);
    snprintf(buf, sizeof(buf), "Relay->Robot: %.1f ms", robotLat);
    glColor3f(tcp30003 ? 0.35f : 1.0f, tcp30003 ? 0.90f : 0.35f, 0.50f);
    text2D(delayX + 160, 18, buf);

    // 中间: 机械臂 IP
    snprintf(buf, sizeof(buf), "Robot IP: %s", Config::ROBOT_IP);
    glColor3f(0.70f, 0.74f, 0.78f);
    text2D(560, 18, buf);

    // 右侧: 连接状态指示灯
    const char* robotStatus = app.isRobotConnected ? "ONLINE" : "OFFLINE";
    int rx = w - 110;
    glColor3f(app.isRobotConnected ? 0.30f : 1.0f, app.isRobotConnected ? 0.90f : 0.30f, 0.30f);
    text2D(rx, 18, "Robot:");
    glColor3f(app.isRobotConnected ? 0.35f : 1.0f, app.isRobotConnected ? 0.90f : 0.35f, 0.50f);
    text2D(rx + 50, 18, robotStatus);
}

// ===== 左栏上半：Touch→Robot 指令日志 =====

static void drawCommandPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Touch -> Robot (Commands)");

    auto& app = appState;
    int textY = y + h - 28;
    int lineH = 14;

    EnterCriticalSection(&app.commandLogMutex);
    int count = app.commandLogCount;
    int start = app.commandLogIdx;

    // 从最新往前显示 (最多显示够放的行数)
    int maxLines = (h - 30) / lineH;
    for (int i = 0; i < maxLines && i < count; i++) {
        int idx = (start - 1 - i + AppState::LOG_SIZE) % AppState::LOG_SIZE;
        if (app.commandLog[idx][0] == '\0') continue;

        // 最新条目用亮色，旧的逐渐变暗
        float alpha = 1.0f - (float)i / maxLines * 0.6f;
        glColor3f(0.30f * alpha, 0.85f * alpha, 0.50f * alpha);
        text2D(x + 6, textY, app.commandLog[idx], GLUT_BITMAP_8_BY_13);
        textY -= lineH;
    }
    LeaveCriticalSection(&app.commandLogMutex);

    if (count == 0) {
        glColor3f(0.35f, 0.38f, 0.42f);
        text2D(x + 6, textY, "(waiting for commands...)");
    }
}

// ===== 左栏下半：机械臂反馈 =====

static void drawFeedbackPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Robot -> Relay (Feedback)");

    auto& app = appState;
    int textY = y + h - 28;
    int lineH = 14;

    EnterCriticalSection(&app.feedbackLogMutex);
    int count = app.feedbackLogCount;
    int start = app.feedbackLogIdx;

    int maxLines = (h - 30) / lineH;
    for (int i = 0; i < maxLines && i < count; i++) {
        int idx = (start - 1 - i + AppState::LOG_SIZE) % AppState::LOG_SIZE;
        if (app.feedbackLog[idx][0] == '\0') continue;

        float alpha = 1.0f - (float)i / maxLines * 0.6f;
        // 报错显示红色
        bool isError = (app.feedbackLog[idx][0] != '0');
        if (isError) {
            glColor3f(1.0f * alpha, 0.30f * alpha, 0.30f * alpha);
        } else {
            glColor3f(0.60f * alpha, 0.65f * alpha, 0.70f * alpha);
        }
        text2D(x + 6, textY, app.feedbackLog[idx], GLUT_BITMAP_8_BY_13);
        textY -= lineH;
    }
    LeaveCriticalSection(&app.feedbackLogMutex);

    if (count == 0) {
        glColor3f(0.35f, 0.38f, 0.42f);
        text2D(x + 6, textY, "(waiting for feedback...)");
    }
}

// ===== 中栏上半：原始力数据 =====

static void drawForceRawPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Force Sensor (Raw)");

    auto& app = appState;
    char buf[128];

    EnterCriticalSection(&app.forceMutex);
    double fx = app.forceRaw[0];
    double fy = app.forceRaw[1];
    double fz = app.forceRaw[2];
    LeaveCriticalSection(&app.forceMutex);

    int cy = y + h / 2;
    glColor3f(0.60f, 0.65f, 0.70f);
    text2D(x + 6, cy + 12, "Force sensor data from robot end-effector");
    glColor3f(0.40f, 0.44f, 0.50f);
    text2D(x + 6, cy - 4, "(awaiting force sensor integration)");

    snprintf(buf, sizeof(buf), "Fx: %6.2f N", fx);
    glColor3f(0.70f, 0.74f, 0.78f);
    text2D(x + 6, cy - 22, buf);
    snprintf(buf, sizeof(buf), "Fy: %6.2f N", fy);
    text2D(x + 140, cy - 22, buf);
    snprintf(buf, sizeof(buf), "Fz: %6.2f N", fz);
    text2D(x + 260, cy - 22, buf);
}

// ===== 中栏下半：滤波力数据 =====

static void drawForceFilteredPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Force Output (Filtered -> Touch)");

    auto& app = appState;
    char buf[128];

    EnterCriticalSection(&app.forceMutex);
    double fx = app.forceFiltered[0];
    double fy = app.forceFiltered[1];
    double fz = app.forceFiltered[2];
    LeaveCriticalSection(&app.forceMutex);

    int cy = y + h / 2;
    glColor3f(0.60f, 0.65f, 0.70f);
    text2D(x + 6, cy + 12, "Filtered force sent to Touch device");
    glColor3f(0.40f, 0.44f, 0.50f);
    text2D(x + 6, cy - 4, "(filter pipeline not yet integrated)");

    snprintf(buf, sizeof(buf), "Fx: %6.2f N", fx);
    glColor3f(0.70f, 0.74f, 0.78f);
    text2D(x + 6, cy - 22, buf);
    snprintf(buf, sizeof(buf), "Fy: %6.2f N", fy);
    text2D(x + 140, cy - 22, buf);
    snprintf(buf, sizeof(buf), "Fz: %6.2f N", fz);
    text2D(x + 260, cy - 22, buf);
}

// ===== 右栏下半：坐标 + 力数据 =====

static void drawCoordPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Robot State");

    auto& app = appState;
    char buf[128];
    int lineH = 16;
    int ty = y + h - 28;

    // 机械臂位姿
    EnterCriticalSection(&app.robotPoseMutex);
    AppState::RobotPose pose = app.robotActualPose;
    AppState::RobotPose target = app.robotTargetPose;
    LeaveCriticalSection(&app.robotPoseMutex);

    glColor3f(0.90f, 0.94f, 1.00f);
    text2D(x + 6, ty, "Position (mm):", GLUT_BITMAP_HELVETICA_10);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  X: %8.2f  (target: %8.2f)", pose.x, target.x);
    glColor3f(0.70f, 0.85f, 0.50f);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  Y: %8.2f  (target: %8.2f)", pose.y, target.y);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  Z: %8.2f  (target: %8.2f)", pose.z, target.z);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH + 2;

    glColor3f(0.90f, 0.94f, 1.00f);
    text2D(x + 6, ty, "Orientation (deg):", GLUT_BITMAP_HELVETICA_10);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  Rx: %7.2f  Ry: %7.2f  Rz: %7.2f", pose.rx, pose.ry, pose.rz);
    glColor3f(0.70f, 0.85f, 0.50f);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH + 4;

    // 关节角度
    glColor3f(0.90f, 0.94f, 1.00f);
    text2D(x + 6, ty, "Joints (deg):", GLUT_BITMAP_HELVETICA_10);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  J1:%7.1f  J2:%7.1f  J3:%7.1f", pose.j1, pose.j2, pose.j3);
    glColor3f(0.50f, 0.80f, 0.95f);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  J4:%7.1f  J5:%7.1f  J6:%7.1f", pose.j4, pose.j5, pose.j6);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    ty -= lineH + 4;

    // 分隔线
    drawSeparatorLine(x + 4, x + w - 4, ty + 2);
    ty -= 4;

    // 力数据
    EnterCriticalSection(&app.forceMutex);
    double fx = app.forceFiltered[0];
    double fy = app.forceFiltered[1];
    double fz = app.forceFiltered[2];
    LeaveCriticalSection(&app.forceMutex);

    glColor3f(0.90f, 0.94f, 1.00f);
    text2D(x + 6, ty, "Force (N):", GLUT_BITMAP_HELVETICA_10);
    ty -= lineH;
    snprintf(buf, sizeof(buf), "  Fx: %7.2f  Fy: %7.2f  Fz: %7.2f", fx, fy, fz);
    glColor3f(0.50f, 0.80f, 0.95f);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);

    // 传输状态
    ty -= lineH + 6;
    const char* tx = (target.x != 0 || target.y != 0 || target.z != 0) ? "ACTIVE" : "IDLE";
    glColor3f(tx[0] == 'A' ? 0.35f : 0.55f, tx[0] == 'A' ? 0.90f : 0.55f, 0.55f);
    snprintf(buf, sizeof(buf), "TX: %s", tx);
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);

    // ===== SafetyPredictor 状态 =====
    ty -= 6;
    drawSeparatorLine(x + 4, x + w - 4, ty + 2);
    ty -= 4;

    SafetyVerdict v = SafetyPredictor::instance().lastVerdict();
    int alarmCount = SafetyPredictor::instance().alarmCount();

    // 安全状态灯
    ty -= lineH;
    switch (v.action) {
        case SafetyVerdict::ALLOW:
            glColor3f(0.35f, 0.90f, 0.50f);
            snprintf(buf, sizeof(buf), "Safety: OK");
            break;
        case SafetyVerdict::WARN_SLOW:
            glColor3f(1.0f, 0.78f, 0.28f);
            snprintf(buf, sizeof(buf), "Safety: WARN — %s (x%.0f%%)", v.reason, v.speedFactor * 100);
            break;
        case SafetyVerdict::REJECT:
            glColor3f(1.0f, 0.35f, 0.35f);
            snprintf(buf, sizeof(buf), "Safety: REJECT — %s", v.reason);
            break;
    }
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);

    // 报警历史计数
    ty -= lineH;
    if (alarmCount > 0) {
        glColor3f(1.0f, 0.55f, 0.25f);
        snprintf(buf, sizeof(buf), "Alarms: %d recorded", alarmCount);
    } else {
        glColor3f(0.45f, 0.50f, 0.55f);
        snprintf(buf, sizeof(buf), "Alarms: 0");
    }
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);

    // 奇异位形接近度 (末端距 Z 轴)
    ty -= lineH;
    {
        double r_xy = sqrt(pose.x * pose.x + pose.y * pose.y);
        if (r_xy < 80.0) {
            glColor3f(1.0f, 0.78f, 0.28f);
        } else {
            glColor3f(0.45f, 0.50f, 0.55f);
        }
        snprintf(buf, sizeof(buf), "Z-axis dist: %.0f mm %s", r_xy, r_xy < 30.0 ? "!!SINGULAR!!" : r_xy < 80.0 ? "(near limit)" : "");
        text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    }

    // 关节限位状态
    ty -= lineH;
    {
        static const double jlims[6][2] = {
            {-360, 360}, {-360, 360}, {-155, 155},
            {-360, 360}, {-360, 360}, {-360, 360}
        };
        double jvals[6] = { pose.j1, pose.j2, pose.j3, pose.j4, pose.j5, pose.j6 };
        double minMargin = 999;
        int worstJoint = -1;
        for (int i = 0; i < 6; i++) {
            double dLo = fabs(jvals[i] - jlims[i][0]);
            double dHi = fabs(jlims[i][1] - jvals[i]);
            double m = (dLo < dHi) ? dLo : dHi;
            if (m < minMargin) { minMargin = m; worstJoint = i; }
        }
        if (minMargin < 15.0) {
            glColor3f(1.0f, 0.55f, 0.25f);
            const char* jn[6] = {"J1","J2","J3","J4","J5","J6"};
            snprintf(buf, sizeof(buf), "%s near limit: %.1f deg", jn[worstJoint], minMargin);
        } else {
            glColor3f(0.45f, 0.50f, 0.55f);
            snprintf(buf, sizeof(buf), "Joints: OK (min margin %.0f deg)", minMargin);
        }
        text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
    }
}

// ===== 主入口 =====

void drawAll() {
    glDisable(GL_DEPTH_TEST);

    // 切换到 2D 正交投影
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, Config::WINDOW_W, 0, Config::WINDOW_H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // === 顶部状态栏 ===
    drawTopBar();

    // === 左栏 ===
    int lx = HudLayout::LEFT_X;
    int ly = HudLayout::PANEL_Y;
    int lw = HudLayout::LEFT_W;
    int lh = HudLayout::PANEL_H;
    int sh = HudLayout::SUB_H;

    drawCommandPanel(lx, ly, lw, sh);          // 上半: 指令
    drawFeedbackPanel(lx, ly + sh + 4, lw, sh); // 下半: 反馈

    // === 中栏 ===
    int cx = HudLayout::CENTER_X;
    drawForceRawPanel(cx, ly, lw, sh);           // 上半: 原始力
    drawForceFilteredPanel(cx, ly + sh + 4, lw, sh); // 下半: 滤波力

    // === 右栏下半: 坐标 + 力数据 ===
    int rx = HudLayout::RIGHT_X;
    int ry = HudLayout::RIGHT_BOTTOM_Y;
    int rw = HudLayout::RIGHT_W;
    int rh = HudLayout::RIGHT_BOTTOM_H;
    drawCoordPanel(rx, ry, rw, rh);

    // 恢复投影
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}

} // namespace HudOverlay
