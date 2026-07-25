#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/RelayCore.h"
#include "../core/MathUtils.h"
#include "../config/Config.h"
#include "../safety/ConstraintForce.h"
#include "../safety/SafetyPredictor.h"
#include <HDU/hduVector.h>

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

    // ===== 5. 按钮 1 状态机 -> RelayCore =====
    bool stateChanged = (button1 != app.lastButtonState);
    if (stateChanged) {
        app.lastButtonState = button1;
        if (button1) {
            relay.onButtonPress(robotPos);
        } else {
            relay.onButtonRelease();
        }
    }

    // ===== 6. 持续发送（按钮保持按下） =====
    if (relay.isTransmitting()) {
        relay.sendPosition(localDevicePos);
    }

    // ===== 7. 向 MATLAB GUI 上报位置 =====
    relay.reportPosition();

    // ===== 8. 力反馈渲染 (传感器力 + 虚拟约束力) =====
    {
        double totalForce[3] = { 0.0, 0.0, 0.0 };

        // 8a. 传感器力 (仅在非 stale 时)
        EnterCriticalSection(&app.forceDataMutex);
        if (!app.forceData.isStale) {
            totalForce[0] = app.forceData.hapticOut[0];
            totalForce[1] = app.forceData.hapticOut[1];
            totalForce[2] = app.forceData.hapticOut[2];
        }
        LeaveCriticalSection(&app.forceDataMutex);

        // 8b. 虚拟约束力 (始终渲染，独立于传感器/机械臂状态)
        double constraint[3] = {0};
        SafetyPredictor::instance().computeConstraintForce(robotPos, constraint);
        totalForce[0] += constraint[0];
        totalForce[1] += constraint[1];
        totalForce[2] += constraint[2];

        // 8c. 总力 clamp
        double maxF = Config::FORCE_MAX_TOUCH_N;
        if (totalForce[0] > maxF) totalForce[0] = maxF;
        if (totalForce[0] < -maxF) totalForce[0] = -maxF;
        if (totalForce[1] > maxF) totalForce[1] = maxF;
        if (totalForce[1] < -maxF) totalForce[1] = -maxF;
        if (totalForce[2] > maxF) totalForce[2] = maxF;
        if (totalForce[2] < -maxF) totalForce[2] = -maxF;

        hdSetDoublev(HD_CURRENT_FORCE, totalForce);
    }

    hdEndFrame(app.hHD);
    return HD_CALLBACK_CONTINUE;
}
