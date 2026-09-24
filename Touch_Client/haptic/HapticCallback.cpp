#define _USE_MATH_DEFINES
#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/RelayCore.h"
#include "../core/MathUtils.h"
#include "../config/Config.h"
#include "../safety/ConstraintForce.h"
#include "../safety/SafetyPredictor.h"
#include "../force/ForcePipeline.h"   // ★ 2026-09-24: viscousDamping（环路阻尼）
#include <chrono>                   // ★ 同上：阻尼的速度基线要用 steady_clock 微秒（GetTickCount 粒度 15.6ms 不够）
#include <HDU/hduVector.h>
#include <iostream>
#include <cmath>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData) {
    auto& app = appState;
    auto& relay = RelayCore::instance();

    if (app.isClosing) return HD_CALLBACK_DONE;

    // ★★ 2026-09-22: 【触觉帧心跳 —— 无条件、每次回调都刷】。
    //   从前它写在 RelayCore::sendPosition 里，而那一行在守卫之后、且 sendPosition
    //  【只在 isTransmitting() 时才被调用】(本文件 :134) ⇒ 那个"心跳"其实是
    //   "上一次下发"的时间戳，**不是触觉线程的心跳** ⇒ 看门狗 (RelayCore.cpp :309,
    //   阈值 200*2=400ms) 在第二次按下时会读到一松手就冻住的旧值 ⇒ 把"没在下发"
    //   误判成"GLUT 死了" ⇒ EmergencyStop。现场 2026-09-22 的 `1078ms since last haptic frame`
    //   就是它，而操作员说空闲很短 —— 两边对不上正是因为那个数根本不是触觉帧的年龄。
    //   ⇒ 放在这里以后，再报 1078ms **一定**是回调真的停了 (那时查 GLUT / 控制台阻塞)。
    // ⚠ 必须在 isClosing 守卫【之后】: 关闭中不该再刷心跳，否则要退出的那一下会被当成"还活着"。
    relay.markHapticFrame();

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

            // ===== 8a-2. ★★ 2026-09-24 深夜：**粘性阻尼项**（环路阻尼）=====
            //   病与合规性论证见 Config::TOUCH_VISC_DAMPING 那一大段（一句话：重压时机械臂
            //   左右晃是"力推手柄 → 手柄动 → 位置映射忠实转发 → 臂动又产生横向力"的自持环路；
            //   而力反馈倍率有规格 1~3 ⇒ 不能靠降增益；粘性项只对运动响应、**不改静态倍率** ✓）。
            //   位置在 8e（逐轴夹 ±3.3N）【之前】⇒ 它也会被那个夹子管住 ✓
            //   ⚠ 速度必须用【≥20ms 的基线】：本回调 ~1kHz，相邻两帧只差 ~7µm（器件分辨率 ~10µm）
            //     ⇒ 逐帧差分的量化噪声就有 ~10mm/s，与真实速度（实测 12~50mm/s）同量级
            //     ⇒ 那样注入的是噪声、不是阻尼。20ms 基线 ⇒ 量化噪声 ~0.5mm/s ✓
            //   ⚠ 时间戳用 steady_clock 微秒：GetTickCount 粒度 15.6ms，配 20ms 基线会 ±78% 误差。
            {
                static bool s_dmpInit = false;
                static hduVector3Dd s_dmpPrevPos;
                static std::chrono::steady_clock::time_point s_dmpPrevT;
                static double s_dmpVel[3] = {0.0, 0.0, 0.0};
                const auto nowT = std::chrono::steady_clock::now();
                if (!s_dmpInit) {
                    s_dmpInit = true; s_dmpPrevPos = localDevicePos; s_dmpPrevT = nowT;
                } else {
                    const double dt = std::chrono::duration<double>(nowT - s_dmpPrevT).count();
                    if (dt >= 0.020) {
                        for (int i = 0; i < 3; i++) s_dmpVel[i] = (localDevicePos[i] - s_dmpPrevPos[i]) / dt;
                        s_dmpPrevPos = localDevicePos; s_dmpPrevT = nowT;
                    }
                }
                double damp[3];
                ForcePipeline::viscousDamping(s_dmpVel, Config::TOUCH_VISC_DAMPING, damp);
                for (int i = 0; i < 3; i++) totalForce[i] += damp[i];
            }

            // ===== ★★ 8b/8c/8d 虚拟约束力（触觉安全提示）—— 2026-09-21 由用户拍板【全部关闭】=====
            // 关掉的是哪三组、为什么关、以及【代价与补偿措施】, 全部写在
            //   Config::FORCE_CONSTRAINT_FORCES_ENABLED 那一大段里 —— 这里不复述。
            // 一句话: 操作员从此失去全部触觉安全提示 ⇒ 已用 client 横幅 + MATLAB 的 W| 警告补偿。
            // ⚠ 代码保留而不删: 开关翻回 true 即恢复; 两个用例测的是【计算】本身, 仍有覆盖。
            if (Config::FORCE_CONSTRAINT_FORCES_ENABLED) {
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
            } else {
                // ★ 关掉时【也要把两个"一次性"标志吃掉】—— 那两处是"写入方置位、这里消费并清零"
                //   的协议。不清零的话: 标志一直挂着, 而 appState 里的力值是【当时那一刻】的 ——
                //   万一开关翻回 true, 就会推出一股与当前状态无关的陈旧力。
                //   (本项目记过"陈旧值被当成此刻的值"的账; 关掉不等于可以不管这个协议。)
                if (appState.hasOrientExtraForce) {
                    EnterCriticalSection(&appState.orientForceMutex);
                    appState.hasOrientExtraForce = false;
                    LeaveCriticalSection(&appState.orientForceMutex);
                }
                if (appState.hasOrientRepulsion) {
                    EnterCriticalSection(&appState.orientRepulsionMutex);
                    appState.hasOrientRepulsion = false;
                    LeaveCriticalSection(&appState.orientRepulsionMutex);
                }
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
