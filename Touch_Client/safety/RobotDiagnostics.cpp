#include "RobotDiagnostics.h"
#include "../config/Config.h"
#include <windows.h>
#include <ctime>
#include <cstring>

RobotDiagnostics& RobotDiagnostics::instance() {
    static RobotDiagnostics inst;
    return inst;
}

void RobotDiagnostics::init(const char* path) {
    m_sessionStartMs = GetTickCount64();
    m_writeIdx = 0;
    m_count = 0;
    memset(m_errorCounts, 0, sizeof(m_errorCounts));

    // 打开日志文件 (追加模式)
    if (fopen_s(&m_logFile, path, "a") != 0 || !m_logFile) {
        // 静默失败 — 诊断日志不应阻塞启动
        return;
    }

    // 会话头
    time_t now = time(nullptr);
    struct tm tmInfo;
    localtime_s(&tmInfo, &now);
    char timeBuf[32];
    strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tmInfo);

    fprintf(m_logFile, "\n[%s] === SESSION START ===\n", timeBuf);
    fflush(m_logFile);
}

void RobotDiagnostics::shutdown() {
    if (m_isShutdown) return;
    m_isShutdown = true;
    writeSessionReport();
    if (m_logFile) {
        fclose(m_logFile);
        m_logFile = nullptr;
    }
}

void RobotDiagnostics::log(const DiagnosticEvent& e) {
    // 环形缓冲区
    m_history[m_writeIdx] = e;
    m_writeIdx = (m_writeIdx + 1) % HISTORY_SIZE;
    m_count++;

    // 错误计数
    if (e.error != RobotErrorCode::OK) {
        int idx = static_cast<int>(e.error);
        if (idx >= 0 && idx < 24) {
            m_errorCounts[idx]++;
        }
    }

    // 文件写入
    if (m_logFile) {
        time_t now = time(nullptr);
        struct tm tmInfo;
        localtime_s(&tmInfo, &now);
        char timeBuf[32];
        strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tmInfo);

        const char* sevStr = "INFO";
        Severity sev = getSeverity(e.error);
        if (sev == Severity::WARN) sevStr = "WARN";
        else if (sev == Severity::DEGRADE) sevStr = "DEGRADE";
        else if (sev == Severity::REJECT) sevStr = "REJECT";
        else if (sev == Severity::FATAL) sevStr = "FATAL";

        fprintf(m_logFile, "[%s] %s: %s target=(%.0f,%.0f,%.0f) joints=(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f) speed=%.2f constraint=%.1fN\n",
            timeBuf, sevStr, errorCodeName(e.error),
            e.targetPosition.x, e.targetPosition.y, e.targetPosition.z,
            e.jointAngles[0], e.jointAngles[1], e.jointAngles[2],
            e.jointAngles[3], e.jointAngles[4], e.jointAngles[5],
            e.speedFactor, e.constraintForceMag);
        fflush(m_logFile);
    }
}

void RobotDiagnostics::logStateChange(RobotState from, RobotState to) {
    DiagnosticEvent e;
    e.timestampMs = GetTickCount64();
    e.fromState = from;
    e.toState = to;
    e.error = RobotErrorCode::OK;
    log(e);
}

void RobotDiagnostics::logError(const RobotError& error, double constraintMag) {
    DiagnosticEvent e;
    e.timestampMs = error.timestampMs;
    e.fromState = RobotState::RUNNING;  // placeholder — caller should set
    e.toState = RobotState::RUNNING;
    e.error = error.code;
    e.targetPosition = error.targetPosition;
    memcpy(e.jointAngles, error.currentJoints, sizeof(e.jointAngles));
    e.latencyMs = error.latencyMs;
    e.speedFactor = error.speedFactor;
    e.constraintForceMag = constraintMag;
    log(e);
}

int RobotDiagnostics::errorCount(RobotErrorCode code) const {
    int idx = static_cast<int>(code);
    if (idx < 0 || idx >= 24) return 0;
    return m_errorCounts[idx];
}

void RobotDiagnostics::writeSessionReport() {
    if (!m_logFile) return;

    time_t now = time(nullptr);
    struct tm tmInfo;
    localtime_s(&tmInfo, &now);
    char timeBuf[32];
    strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tmInfo);

    int warns = 0, degrades = 0, rejects = 0, fatals = 0;
    for (int i = 0; i < 24; i++) {
        RobotErrorCode code = static_cast<RobotErrorCode>(i);
        Severity sev = getSeverity(code);
        switch (sev) {
            case Severity::WARN:    warns    += m_errorCounts[i]; break;
            case Severity::DEGRADE: degrades += m_errorCounts[i]; break;
            case Severity::REJECT:  rejects  += m_errorCounts[i]; break;
            case Severity::FATAL:   fatals   += m_errorCounts[i]; break;
            default: break;
        }
    }

    fprintf(m_logFile, "[%s] === SESSION END === errors: WARN=%d DEGRADE=%d REJECT=%d FATAL=%d total_events=%d\n\n",
        timeBuf, warns, degrades, rejects, fatals, m_count);
    fflush(m_logFile);
}
