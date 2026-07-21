#pragma once
#include <vector>
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
    void checkAlarm();

    // 扩展
    void registerExtension(IExtension* ext);

    // 状态查询（供 Render 层读取）
    bool isTransmitting() const { return m_transmitting; }

private:
    RelayCore() = default;
    RelayCore(const RelayCore&) = delete;
    RelayCore& operator=(const RelayCore&) = delete;

    bool m_transmitting = false;
    bool m_basePointSet = false;
    Vec3 m_basePoint;
    std::vector<IExtension*> m_extensions;
};
