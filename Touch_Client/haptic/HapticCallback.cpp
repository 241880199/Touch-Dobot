#define _USE_MATH_DEFINES
#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/RelayCore.h"
#include "../core/MathUtils.h"
#include "../config/Config.h"
#include "../safety/ConstraintForce.h"
#include "../safety/SafetyPredictor.h"
#include <HDU/hduVector.h>
#include <iostream>
#include <cmath>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData) {
    auto& app = appState;
    auto& relay = RelayCore::instance();

    if (app.isClosing) return HD_CALLBACK_DONE;

    hdBeginFrame(app.hHD);

    // ===== 1. 读取位置 =====
    hduVector3Dd newPos;
    hdGetDoublev(HD_CURRENT_POSITION, newPos);

    EnterCriticalSection(&app.devicePosMutex);
    app.devicePos = newPos;
    app.devicePos[0] = clamp(app.devicePos[0], Config::DEV_X_MIN, Config::DEV_X_MAX);
    app.devicePos[1] = clamp(app.devicePos[1], Config::DEV_Y_MIN, Config::DEV_Y_MAX);
    app.devicePos[2] = clamp(app.devicePos[2], Config::DEV_Z_MIN, Config::DEV_Z_MAX);
    hduVector3Dd localDevicePos = app.devicePos;
    LeaveCriticalSection(&app.devicePosMutex);

    // ===== 1b. 读取笔杆姿态 (HD_CURRENT_TRANSFORM → Euler ZYX) =====
    {
        hdGetDoublev(HD_CURRENT_TRANSFORM, app.transformMatrix);

        // Column-major 4x4 matrix:
        // | m0  m4  m8  m12 |   rotation 3x3:  [m0 m4 m8 ]
        // | m1  m5  m9  m13 |                   [m1 m5 m9 ]
        // | m2  m6  m10 m14 |                   [m2 m6 m10]
        // | m3  m7  m11 m15 |
        double* m = app.transformMatrix;

        // Extract ZYX intrinsic Euler angles from rotation submatrix
        // R = Rz(rz) * Ry(ry) * Rx(rx)
        double sy = -m[2];  // -R[2][0]
        if (sy > 1.0) sy = 1.0;
        if (sy < -1.0) sy = -1.0;

        double ry_rad = asin(sy);
        double rx_rad, rz_rad;

        if (fabs(cos(ry_rad)) > 1e-6) {
            // Non-gimbal-lock case
            rx_rad = atan2(m[6], m[10]);   // atan2(R[2][1], R[2][2])
            rz_rad = atan2(m[1], m[0]);    // atan2(R[1][0], R[0][0])
        } else {
            // Gimbal lock: ry ≈ ±90°, rx and rz are coupled
            rx_rad = atan2(-m[9], m[5]);   // atan2(-R[1][2], R[1][1])
            rz_rad = 0.0;
        }

        double rx_deg = rx_rad * 180.0 / M_PI;
        double ry_deg = ry_rad * 180.0 / M_PI;
        double rz_deg = rz_rad * 180.0 / M_PI;

        EnterCriticalSection(&app.stylusOrientMutex);
        app.stylusOrient[0] = rx_deg;
        app.stylusOrient[1] = ry_deg;
        app.stylusOrient[2] = rz_deg;
        LeaveCriticalSection(&app.stylusOrientMutex);
    }

    // ===== 2. 坐标转换到 robot 系 =====
    Vec3 robotPos = convertTouchToRobot(localDevicePos);

    EnterCriticalSection(&app.adjustedPosTableMutex);
    app.adjustedPosTable = robotPos;
    LeaveCriticalSection(&app.adjustedPosTableMutex);

    EnterCriticalSection(&app.adjustedPosMutex);
    app.adjustedPos = localDevicePos;
    LeaveCriticalSection(&app.adjustedPosMutex);

    // ===== 3. 按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0;
    app.button2Pressed = button2;

    // ===== 4. 轨迹 (仅按钮按下时记录) =====
    if (button1) {
        EnterCriticalSection(&app.trailMutex);
        app.trailPoints.push_back(localDevicePos);
        while ((int)app.trailPoints.size() > Config::MAX_TRAIL) {
            app.trailPoints.pop_front();
        }
        LeaveCriticalSection(&app.trailMutex);
    }

    // ===== 5. 按钮 1 状态机 -> RelayCore (位置) =====
    bool b1Changed = (button1 != app.lastButtonState);
    if (b1Changed) {
        app.lastButtonState = button1;
        if (button1) {
            relay.onButtonPress(robotPos);
        } else {
            relay.onButtonRelease();
        }
    }

    // ===== 5b. 按钮 2 状态机 -> RelayCore (姿态) =====
    static bool s_lastButton2 = false;
    bool b2Changed = (button2 != s_lastButton2);
    if (b2Changed) {
        s_lastButton2 = button2;
        if (button2) {
            // Build Vec3 from current stylus Euler angles
            double sx, sy, sz;
            EnterCriticalSection(&app.stylusOrientMutex);
            sx = app.stylusOrient[0];
            sy = app.stylusOrient[1];
            sz = app.stylusOrient[2];
            LeaveCriticalSection(&app.stylusOrientMutex);
            relay.onButton2Press(Vec3(sx, sy, sz));
        } else {
            relay.onButton2Release();
        }
    }

    // ===== 6. 持续发送（按钮保持按下） =====
    if (relay.isTransmitting()) {
        relay.sendPosition(localDevicePos);
    }

    // ===== 7. 向 MATLAB GUI 上报位置 =====
    relay.reportPosition();

    // ===== 8. 力反馈渲染 (传感器力 + 虚拟约束力) =====
    // 按钮1或按钮2按下时渲染力反馈，松开时清零
    {
        double totalForce[3] = { 0.0, 0.0, 0.0 };

        if (app.forceFeedbackEnabled && (button1 || button2)) {
            // 8a. 传感器力 (仅在非 stale 时)
            EnterCriticalSection(&app.forceDataMutex);
            if (!app.forceData.isStale) {
                totalForce[0] = app.forceData.hapticOut[0];
                totalForce[1] = app.forceData.hapticOut[1];
                totalForce[2] = app.forceData.hapticOut[2];
            }
            LeaveCriticalSection(&app.forceDataMutex);

            // 8b. 虚拟约束力 — 距离驱动: 越靠近危险区域力越大
            // 位置模式: 基于 Touch 笔尖位置
            // 姿态模式: 基于机器人实际 TCP 位置
            double constraint[3] = {0};
            {
                Vec3 forceRef = robotPos;
                if (!button1 && button2) {
                    // Orientation-only: use robot actual TCP for constraint reference
                    EnterCriticalSection(&app.robotPoseMutex);
                    forceRef = Vec3(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
                    LeaveCriticalSection(&app.robotPoseMutex);
                }
                SafetyPredictor::instance().computeConstraintForce(forceRef, constraint);
            }
            totalForce[0] += constraint[0];
            totalForce[1] += constraint[1];
            totalForce[2] += constraint[2];

            // 8c. Orient extra force (singularity avoidance constraint amplification)
            if (appState.hasOrientExtraForce) {
                EnterCriticalSection(&appState.orientForceMutex);
                totalForce[0] += appState.orientExtraForce[0];
                totalForce[1] += appState.orientExtraForce[1];
                totalForce[2] += appState.orientExtraForce[2];
                appState.hasOrientExtraForce = false;
                LeaveCriticalSection(&appState.orientForceMutex);
            }

            // 8d. Orient directional repulsion force (Phase 2: wrist alignment warning)
            if (button2 && appState.hasOrientRepulsion) {
                EnterCriticalSection(&appState.orientRepulsionMutex);
                totalForce[0] += appState.orientRepulsionForce[0];
                totalForce[1] += appState.orientRepulsionForce[1];
                totalForce[2] += appState.orientRepulsionForce[2];
                appState.hasOrientRepulsion = false;
                LeaveCriticalSection(&appState.orientRepulsionMutex);
            }

            // 8e. 总力 clamp
            double maxF = Config::FORCE_MAX_TOUCH_N;
            if (totalForce[0] > maxF) totalForce[0] = maxF;
            if (totalForce[0] < -maxF) totalForce[0] = -maxF;
            if (totalForce[1] > maxF) totalForce[1] = maxF;
            if (totalForce[1] < -maxF) totalForce[1] = -maxF;
            if (totalForce[2] > maxF) totalForce[2] = maxF;
            if (totalForce[2] < -maxF) totalForce[2] = -maxF;
        }

        if (!app.forceFeedbackEnabled) {
            // FF disabled: clear any pending one-shot orient flags so a stale
            // force doesn't fire once on re-enable.
            EnterCriticalSection(&appState.orientForceMutex);
            appState.hasOrientExtraForce = false;
            LeaveCriticalSection(&appState.orientForceMutex);

            EnterCriticalSection(&appState.orientRepulsionMutex);
            appState.hasOrientRepulsion = false;
            LeaveCriticalSection(&appState.orientRepulsionMutex);
        }

        hdSetDoublev(HD_CURRENT_FORCE, totalForce);

        // Debug: print force values every ~2s (at 1kHz callback, every 2000th call)
        static int dbgCount = 0;
        if (++dbgCount % 2000 == 0) {
            double mag = sqrt(totalForce[0]*totalForce[0] + totalForce[1]*totalForce[1] + totalForce[2]*totalForce[2]);
            if (mag > 0.01) {  // only print when there's meaningful force
                std::cerr << "[Haptic] Force applied: (" << totalForce[0] << ", "
                          << totalForce[1] << ", " << totalForce[2] << ") N  mag=" << mag
                          << "  stale=" << app.forceData.isStale
                          << "  button1=" << button1
                          << "  button2=" << button2 << std::endl;
            }
        }
    }

    hdEndFrame(app.hHD);
    return HD_CALLBACK_CONTINUE;
}
