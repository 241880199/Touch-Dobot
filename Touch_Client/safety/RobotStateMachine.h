#pragma once
#include <windows.h>
#include "RobotError.h"
#include "EscalationTracker.h"

// ===== 机器人状态 =====
enum class RobotState {
    DISCONNECTED,   // 未连接
    CONNECTED,      // TCP 已通，未使能
    READY,          // 空闲，等待按钮
    RUNNING,        // 运动中
    DEGRADED,       // 降级运行 (IK fail / singular warn)
    ALARM,          // 机器人报警 (mode=9)
    RECOVERING,     // 断线重连中
    FATAL           // 致命错误，需手动重启
};

inline const char* stateName(RobotState s) {
    switch (s) {
        case RobotState::DISCONNECTED: return "DISCONNECTED";
        case RobotState::CONNECTED:    return "CONNECTED";
        case RobotState::READY:        return "READY";
        case RobotState::RUNNING:      return "RUNNING";
        case RobotState::DEGRADED:     return "DEGRADED";
        case RobotState::ALARM:        return "ALARM";
        case RobotState::RECOVERING:   return "RECOVERING";
        case RobotState::FATAL:        return "FATAL";
        default:                       return "UNKNOWN";
    }
}

// ===== 状态机 =====
class RobotStateMachine {
public:
    RobotStateMachine();
    ~RobotStateMachine() { DeleteCriticalSection(&m_lock); }

    // 查询
    RobotState currentState() const;
    const char* stateStr() const;
    double speedFactor() const;
    bool canMove() const;
    bool canSendForce() const;

    // 事件驱动
    void onError(RobotError& error, const Vec3& moveDelta);
    void onRecovery();
    void onButtonPress();
    void onButtonRelease();
    void onConnect();
    void onDisconnect();
    void onEnableSuccess();
    void onEnableFail();

    // 状态转换 (供外部手动触发)
    void transitionTo(RobotState newState);

    // 错误跟踪器 (供 SafetyPredictor 读取) — 不加锁，调用者需同步
    EscalationTracker& escalation() { return m_escalation; }
    const EscalationTracker& escalation() const { return m_escalation; }

private:
    RobotState m_state = RobotState::DISCONNECTED;
    EscalationTracker m_escalation;
    RobotError m_lastError;  // 最近一次错误
    mutable CRITICAL_SECTION m_lock;
};
