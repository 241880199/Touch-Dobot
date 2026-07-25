#include "RobotStateMachine.h"
#include "../config/Config.h"
#include "RobotDiagnostics.h"
#include <algorithm>
#include <iostream>

RobotStateMachine::RobotStateMachine() {
    InitializeCriticalSection(&m_lock);
    m_state = RobotState::DISCONNECTED;
}

// ===== 查询方法 (带锁) =====

RobotState RobotStateMachine::currentState() const {
    EnterCriticalSection(&m_lock);
    RobotState result = m_state;
    LeaveCriticalSection(&m_lock);
    return result;
}

const char* RobotStateMachine::stateStr() const {
    EnterCriticalSection(&m_lock);
    const char* result = stateName(m_state);
    LeaveCriticalSection(&m_lock);
    return result;
}

double RobotStateMachine::speedFactor() const {
    EnterCriticalSection(&m_lock);
    double result;
    switch (m_state) {
        case RobotState::RUNNING:
            result = (m_lastError.code != RobotErrorCode::OK)
                ? std::max(0.1, 1.0 - m_escalation.count() * 0.15)
                : 1.0;
            break;
        case RobotState::DEGRADED:
            result = 0.3;
            break;
        case RobotState::ALARM:
        case RobotState::RECOVERING:
        case RobotState::FATAL:
            result = 0.0;
            break;
        default:
            result = 1.0;
            break;
    }
    LeaveCriticalSection(&m_lock);
    return result;
}

bool RobotStateMachine::canMove() const {
    EnterCriticalSection(&m_lock);
    bool result = (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED);
    LeaveCriticalSection(&m_lock);
    return result;
}

bool RobotStateMachine::canSendForce() const {
    EnterCriticalSection(&m_lock);
    bool result = (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED);
    LeaveCriticalSection(&m_lock);
    return result;
}

// ===== 事件驱动 (带锁) =====

void RobotStateMachine::onError(RobotError& error, const Vec3& moveDelta) {
    EnterCriticalSection(&m_lock);
    m_lastError = error;

    // 升级计数
    m_escalation.recordError(error.code, moveDelta);

    switch (m_state) {
        case RobotState::RUNNING: {
            Severity sev = error.severity;
            if (sev == Severity::FATAL) {
                std::cout << "[StateMachine] RUNNING → FATAL: " << errorCodeName(error.code) << std::endl;
                transitionTo(RobotState::FATAL);
            } else if (sev == Severity::REJECT) {
                // 拒绝该帧，状态不变
                std::cout << "[StateMachine] REJECT frame: " << errorCodeName(error.code) << std::endl;
            } else if (m_escalation.shouldEscalate()) {
                m_escalation.escalated = true;
                std::cout << "[StateMachine] RUNNING → DEGRADED ("
                          << m_escalation.count() << " consecutive "
                          << errorCodeName(error.code) << ")" << std::endl;
                transitionTo(RobotState::DEGRADED);
            }
            // WARN without escalation → stay RUNNING (speedFactor reduces)
            break;
        }
        case RobotState::DEGRADED: {
            if (error.severity == Severity::FATAL) {
                std::cout << "[StateMachine] DEGRADED → FATAL: " << errorCodeName(error.code) << std::endl;
                transitionTo(RobotState::FATAL);
            } else if (error.severity == Severity::REJECT) {
                std::cout << "[StateMachine] REJECT frame in DEGRADED: " << errorCodeName(error.code) << std::endl;
            } else if (m_escalation.shouldEscalate()) {
                // DEGRADED sustained too long → force stop
                std::cout << "[StateMachine] DEGRADED → READY (escalated to REJECT after "
                          << m_escalation.count() << " frames)" << std::endl;
                transitionTo(RobotState::READY);
            }
            break;
        }
        case RobotState::ALARM:
        case RobotState::RECOVERING:
        case RobotState::FATAL:
        case RobotState::DISCONNECTED:
        case RobotState::CONNECTED:
        case RobotState::READY:
            // No motion states — WARN/REJECT have no effect
            if (error.severity == Severity::FATAL && m_state != RobotState::FATAL) {
                transitionTo(RobotState::FATAL);
            }
            break;
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onRecovery() {
    EnterCriticalSection(&m_lock);
    switch (m_state) {
        case RobotState::ALARM:
            std::cout << "[StateMachine] ALARM → READY (recovered)" << std::endl;
            m_escalation.reset();
            transitionTo(RobotState::READY);
            break;
        case RobotState::RECOVERING:
            std::cout << "[StateMachine] RECOVERING → READY (reconnected)" << std::endl;
            m_escalation.reset();
            transitionTo(RobotState::READY);
            break;
        case RobotState::DEGRADED: {
            // Check for reverse motion de-escalation
            // This is called from sendPosition when moving away from danger
            m_escalation.reset();
            std::cout << "[StateMachine] DEGRADED → RUNNING (operator moved away)" << std::endl;
            transitionTo(RobotState::RUNNING);
            break;
        }
        default:
            break;
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onButtonPress() {
    EnterCriticalSection(&m_lock);
    if (m_state == RobotState::READY) {
        transitionTo(RobotState::RUNNING);
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onButtonRelease() {
    EnterCriticalSection(&m_lock);
    if (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED) {
        transitionTo(RobotState::READY);
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onConnect() {
    EnterCriticalSection(&m_lock);
    if (m_state == RobotState::DISCONNECTED || m_state == RobotState::RECOVERING) {
        transitionTo(RobotState::CONNECTED);
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onDisconnect() {
    EnterCriticalSection(&m_lock);
    if (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED) {
        transitionTo(RobotState::RECOVERING);
    } else {
        transitionTo(RobotState::DISCONNECTED);
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onEnableSuccess() {
    EnterCriticalSection(&m_lock);
    if (m_state == RobotState::CONNECTED) {
        transitionTo(RobotState::READY);
    }
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::onEnableFail() {
    EnterCriticalSection(&m_lock);
    transitionTo(RobotState::FATAL);
    LeaveCriticalSection(&m_lock);
}

void RobotStateMachine::transitionTo(RobotState newState) {
    if (m_state == newState) return;
    const char* from = stateName(m_state);
    const char* to = stateName(newState);
    std::cout << "[StateMachine] " << from << " → " << to << std::endl;
    RobotDiagnostics::instance().logStateChange(m_state, newState);
    m_state = newState;
}
