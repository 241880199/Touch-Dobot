#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <vector>
#include <atomic>
#include <windows.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include "CoordinateTransform.h"
#include "IExtension.h"

class RelayCore {
public:
    static RelayCore& instance();

    bool init();
    void shutdown();

    // Touch → Robot 正向数据流
    void sendPosition(const hduVector3Dd& devicePos);
    void onButtonPress(const Vec3& robotPos);
    void onButtonRelease();

    // Robot → Touch 反向数据流 (每帧调用)
    void pollFeedback();
    void queryPose();
    void queryJointAngles();
    void checkAlarm();

    // 力传感器数据流
    bool initForceReader();
    void pollForce();
    void shutdownForceReader();

    // 奇异脱困 (可在运行中手动触发)
    bool triggerEscape();

    // 扩展
    void registerExtension(IExtension* ext);

    // MATLAB GUI 上报
    void initRelayReporting();
    void shutdownRelayReporting();
    int  sendRelayUpdate(const char* msg);
    void reportPosition();
    void reportCommand(const char* cmd);

    // 状态查询（供 Render 层读取）
    bool isTransmitting() const { return m_transmitting; }

private:
    RelayCore();
    ~RelayCore();
    RelayCore(const RelayCore&) = delete;
    RelayCore& operator=(const RelayCore&) = delete;

    std::atomic<bool> m_transmitting{false};
    std::atomic<bool> m_basePointSet{false};
    Vec3 m_targetPos;           // 累加式机器人目标位置
    Vec3 m_lastTouchPos;        // 上一帧 Touch 位置 (robot系), 用于增量计算
    bool   m_lastTouchValid = false;
    CRITICAL_SECTION m_basePointLock;
    std::vector<IExtension*> m_extensions;

    // MATLAB relay connection
    SOCKET m_relaySocket = INVALID_SOCKET;
    CRITICAL_SECTION m_relaySocketMutex;
    DWORD m_lastRelayUpdate = 0;
    DWORD m_lastServoTime = 0;      // ServoP 发送频率控制
    HANDLE m_forceThread = NULL;
};
