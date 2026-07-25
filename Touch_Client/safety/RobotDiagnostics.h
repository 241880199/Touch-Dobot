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
    void logError(const RobotError& error, double constraintMag);

    // 统计数据
    int errorCount(RobotErrorCode code) const;

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
    int m_errorCounts[24] = {0};
    FILE* m_logFile = nullptr;
    uint64_t m_sessionStartMs = 0;
};
