#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../core/CoordinateTransform.h"
#include "../core/SenderQueue.h"
#include "../config/Config.h"
#include "../utils/MathUtils.h"
#include <HDU/hduVector.h>
#include <iostream>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData) {
    auto& app = appState;
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

    // ===== 2. 坐标转换 =====
    Vec3 robotPos = convertTouchToRobot(localDevicePos);

    EnterCriticalSection(&app.adjustedPosTableMutex);
    app.adjustedPosTable = robotPos;
    LeaveCriticalSection(&app.adjustedPosTableMutex);

    EnterCriticalSection(&app.adjustedPosMutex);
    app.adjustedPos = localDevicePos;
    hduVector3Dd localAdjustedPos = app.adjustedPos;
    LeaveCriticalSection(&app.adjustedPosMutex);

    // ===== 3. 预留：读取姿态 (HD_CURRENT_TRANSFORM) =====
    // hdGetDoublev(HD_CURRENT_TRANSFORM, app.transformMatrix);
    // 后续迭代：从 transformMatrix 提取 Rx/Ry/Rz 并增量映射

    // ===== 4. 轨迹记录 =====
    EnterCriticalSection(&app.trailMutex);
    app.trailPoints.push_back(localAdjustedPos);
    while ((int)app.trailPoints.size() > Config::MAX_TRAIL) {
        app.trailPoints.pop_front();
    }
    LeaveCriticalSection(&app.trailMutex);

    // ===== 5. 读取按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0; // 预留
    app.button2Pressed = button2;

    // ===== 6. 按钮 1：运动控制 =====
    static hduVector3Dd startPos;
    bool stateChanged = (button1 != app.lastButtonState);

    if (stateChanged) {
        EnterCriticalSection(&app.basePointMutex);

        app.isTransmitting = button1;
        app.lastButtonState = button1;

        if (button1) {
            // 按下：记录基准点
            app.basePoint = robotPos;
            app.isBasePointSet = true;
            startPos = localDevicePos;
        } else {
            // 松开：停止发送
            app.isBasePointSet = false;
        }

        LeaveCriticalSection(&app.basePointMutex);

        // 更新 TCP 状态显示
        EnterCriticalSection(&app.statusMutex);
        if (button1) {
            snprintf(app.transmissionState, sizeof(app.transmissionState), "STATE: transmitting");
        } else {
            snprintf(app.transmissionState, sizeof(app.transmissionState), "STATE: waiting");
        }
        LeaveCriticalSection(&app.statusMutex);
    }

    // ===== 7. 持续发送（按钮保持按下） =====
    if (app.isTransmitting && app.isBasePointSet && app.isTcpConnected) {
        Vec3 base;
        EnterCriticalSection(&app.basePointMutex);
        base = app.basePoint;
        LeaveCriticalSection(&app.basePointMutex);

        Vec3 delta = computeDelta(robotPos, base);

        // 阈值过滤
        if (delta.length() >= Config::MIN_DELTA_THRESHOLD) {
            // 入队（带背压控制）
            {
                std::lock_guard<std::mutex> lock(app.queueMutex);
                while ((int)app.sendQueue.size() >= Config::MAX_QUEUE_SIZE) {
                    app.sendQueue.pop(); // 丢弃最旧数据
                }
                app.sendQueue.push({ delta.x, delta.y, delta.z });
            }
            app.queueCV.notify_one();
        }
    }

    hdEndFrame(app.hHD);
    return HD_CALLBACK_CONTINUE;
}
