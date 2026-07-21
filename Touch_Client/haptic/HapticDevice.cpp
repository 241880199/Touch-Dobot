#include "HapticDevice.h"
#include "../core/AppState.h"
#include <iostream>
#include <HD/hd.h>

// 前向声明：HapticCallback 在 HapticCallback.cpp 中实现
HDCallbackCode HDCALLBACK hapticCallback(void* pUserData);

bool initHapticDevice() {
    auto& app = appState;

    app.hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "Touch 设备初始化失败" << std::endl;
        return false;
    }

    hdEnable(HD_FORCE_OUTPUT);
    app.hapticCallbackHandle = hdScheduleAsynchronous(
        hapticCallback, nullptr, HD_MAX_SCHEDULER_PRIORITY);

    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "触觉回调注册失败" << std::endl;
        return false;
    }

    hdStartScheduler();
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "调度器启动失败" << std::endl;
        return false;
    }

    app.deviceInitialized = true;
    std::cout << "Touch 设备初始化成功" << std::endl;
    return true;
}

void cleanupHapticDevice() {
    auto& app = appState;
    if (app.deviceInitialized) {
        hdUnschedule(app.hapticCallbackHandle);
        hdStopScheduler();
        hdDisableDevice(app.hHD);
        app.hHD = HD_INVALID_HANDLE;
        app.deviceInitialized = false;
    }
}
