#pragma once
#include <cstdio>
#include <cstdint>
#include "RobotError.h"
#include "RobotStateMachine.h"

// ===== 诊断事件 =====
struct DiagnosticEvent {
    uint64_t timestampMs = 0;
    RobotState fromState = RobotState::DISCONNECTED;
    RobotState toState = RobotState::DISCONNECTED;
    RobotErrorCode error = RobotErrorCode::OK;
    Vec3 targetPosition = {0, 0, 0};
    double jointAngles[6] = {0};
    float latencyMs = 0.0f;
    double speedFactor = 1.0;
    double constraintForceMag = 0.0;
};

// ===== 诊断日志系统 (单例) =====
class RobotDiagnostics {
public:
    static RobotDiagnostics& instance();

    void init(const char* path = "robot_diagnostics.log");
    void shutdown();

    // 记录事件
    void log(const DiagnosticEvent& e);
    void logStateChange(RobotState from, RobotState to);
    void logError(const RobotError& error, double constraintMag,
                  RobotState from = RobotState::RUNNING);

    // 统计数据
    int errorCount(RobotErrorCode code) const;

    // m_errorCounts 的槽位数。【新增错误码时必须同步改这里】—— 槽位不够时计数会被
    // 静默丢掉 (下面的 log() 有边界判断), 会话报告就会少算, 而"少算"看不出来。
    static const int ERROR_CODE_SLOTS = 25;

    // 环形缓冲区 (供 HUD 显示)
    static const int HISTORY_SIZE = 200;
    const DiagnosticEvent* history() const { return m_history; }
    int historySize() const { return m_count < HISTORY_SIZE ? m_count : HISTORY_SIZE; }
    int writeIndex() const { return m_writeIdx; }

    // 会话报告
    void writeSessionReport();

private:
    RobotDiagnostics() {}
    ~RobotDiagnostics() { shutdown(); }

    DiagnosticEvent m_history[HISTORY_SIZE];
    int m_writeIdx = 0;
    int m_count = 0;
    int m_errorCounts[ERROR_CODE_SLOTS] = {0};
    FILE* m_logFile = nullptr;
    bool m_isShutdown = false;
    uint64_t m_sessionStartMs = 0;
};
