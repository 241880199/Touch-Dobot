#include "RobotStateMachine.h"
#include "../config/Config.h"
#include <algorithm>
#include <iostream>

RobotStateMachine::RobotStateMachine() {
    m_state = RobotState::DISCONNECTED;
}

double RobotStateMachine::speedFactor() const {
    switch (m_state) {
        case RobotState::RUNNING:
            return (m_lastError.code != RobotErrorCode::OK)
                ? std::max(0.1, 1.0 - m_escalation.count() * 0.15)
                : 1.0;
        case RobotState::DEGRADED:
            return 0.3;
        case RobotState::ALARM:
        case RobotState::RECOVERING:
        case RobotState::FATAL:
            return 0.0;
        default:
            return 1.0;
    }
}

bool RobotStateMachine::canMove() const {
    return m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED;
}

bool RobotStateMachine::canSendForce() const {
    // 传感器力仅在 RUNNING 和 DEGRADED 发送
    // 约束力始终发送 (独立于机器人状态)
    return m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED;
}

void RobotStateMachine::onError(RobotError& error, const Vec3& moveDelta) {
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
}

void RobotStateMachine::onRecovery() {
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
}

void RobotStateMachine::onButtonPress() {
    if (m_state == RobotState::READY) {
        transitionTo(RobotState::RUNNING);
    }
}

void RobotStateMachine::onButtonRelease() {
    if (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED) {
        transitionTo(RobotState::READY);
    }
}

void RobotStateMachine::onConnect() {
    if (m_state == RobotState::DISCONNECTED || m_state == RobotState::RECOVERING) {
        transitionTo(RobotState::CONNECTED);
    }
}

void RobotStateMachine::onDisconnect() {
    if (m_state == RobotState::RUNNING || m_state == RobotState::DEGRADED) {
        transitionTo(RobotState::RECOVERING);
    } else {
        transitionTo(RobotState::DISCONNECTED);
    }
}

void RobotStateMachine::onEnableSuccess() {
    if (m_state == RobotState::CONNECTED) {
        transitionTo(RobotState::READY);
    }
}

void RobotStateMachine::onEnableFail() {
    transitionTo(RobotState::FATAL);
}

void RobotStateMachine::transitionTo(RobotState newState) {
    if (m_state == newState) return;
    const char* from = stateName(m_state);
    const char* to = stateName(newState);
    std::cout << "[StateMachine] " << from << " → " << to << std::endl;
    m_state = newState;
}
