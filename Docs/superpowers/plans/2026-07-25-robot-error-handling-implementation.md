# Robot Error Handling System — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build unified robot error handling system — error taxonomy, state machine, virtual constraint forces, escalation tracking, structured diagnostics, and ServoP feedback parsing.

**Architecture:** Six new header/implementation files under `safety/` define RobotErrorCode, RobotStateMachine, EscalationTracker, ConstraintForce, and RobotDiagnostics. These feed into modified SafetyPredictor (error code + constraint force output), RelayCore (feedback parsing + health monitor), HapticCallback (force superposition), HudOverlay (state/error display), and main.cpp (diagnostics lifecycle + PING/PONG timer). All new modules are independent of robot hardware and testable offline.

**Tech Stack:** C++17 (MSVC), OpenHaptics 3.5.0, WinSock2, GLUT, Windows CRITICAL_SECTION

## Global Constraints

- IK-independent safety — all checks work when IK fails (URDF model ~408mm offset from Dobot controller)
- Touch max output: 3.3N hard clamp (constraint force + sensor force combined)
- All new files go under `Touch_Client/safety/`
- Existing SafetyVerdict struct extended (not replaced) for backward compatibility
- `--no-robot` mode: constraint force renders, state machine stays in DISCONNECTED
- `--no-touch` mode: constraint force computed but not rendered
- All new code compiles with existing MSVC build settings (cl /EHsc /std:c++17)

---

### Task 1: Config.h — Add Constraint Force + State Machine Constants

**Files:**
- Modify: `Touch_Client/config/Config.h` (append after line 58, before `// ========== 发送队列参数`)

**Interfaces:**
- Produces: `Config::CONSTRAINT_BOUNDARY_RANGE` (double, 50.0mm), `Config::CONSTRAINT_BOUNDARY_MAX_FORCE` (double, 2.0N), `Config::CONSTRAINT_SINGULAR_RANGE` (double, 80.0mm), `Config::CONSTRAINT_SINGULAR_MAX_FORCE` (double, 2.5N), `Config::CONSTRAINT_ALARM_HISTORY_RANGE` (double, 80.0mm), `Config::CONSTRAINT_ALARM_HISTORY_MAX_FORCE` (double, 1.5N), `Config::CONSTRAINT_WORKSPACE_EDGE_START` (double, 550.0mm), `Config::CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE` (double, 1.0N), `Config::HEARTBEAT_TIMEOUT_MS` (int, 500), `Config::RECONNECT_MAX_RETRIES` (int, 5), `Config::RECONNECT_BASE_DELAY_MS` (int, 1000), `Config::PING_INTERVAL_MS` (int, 500), `Config::PING_TIMEOUT_MS` (int, 500), `Config::ESCALATE_WARN_TO_DEGRADE` (int, 3), `Config::ESCALATE_DEGRADE_TO_REJECT` (int, 10), `Config::DEESCALATE_CLEAR_FRAMES` (int, 30), `Config::DIAGNOSTIC_LOG_PATH` (constexpr const char*, "robot_diagnostics.log")

- [ ] **Step 1: Add constraint force config block**

In `Touch_Client/config/Config.h`, after line 58 (`const int FORCE_RECONNECT_INTERVAL = 2000;`) and before line 60 (`// ========== 发送队列参数 ==========`), insert:

```cpp
    // ========== 虚拟约束力参数 ==========
    const double CONSTRAINT_BOUNDARY_RANGE      = 50.0;   // 安全边界感应距离 (mm)
    const double CONSTRAINT_BOUNDARY_MAX_FORCE  = 2.0;    // 安全边界最大约束力 (N)
    const double CONSTRAINT_SINGULAR_RANGE      = 80.0;   // 圆柱奇异感应距离 (mm)
    const double CONSTRAINT_SINGULAR_MAX_FORCE  = 2.5;    // 圆柱奇异最大约束力 (N)
    const double CONSTRAINT_ALARM_HISTORY_RANGE    = 80.0; // 报警历史感应距离 (mm)
    const double CONSTRAINT_ALARM_HISTORY_MAX_FORCE = 1.5; // 报警历史最大约束力 (N)
    const double CONSTRAINT_WORKSPACE_EDGE_START     = 550.0; // 工作空间边缘感应起点 (mm)
    const double CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE = 1.0;   // 工作空间边缘最大约束力 (N)

    // ========== 连接健康监控参数 ==========
    const int HEARTBEAT_TIMEOUT_MS  = 500;    // 心跳超时 (ms)
    const int RECONNECT_MAX_RETRIES = 5;      // 最大重连次数
    const int RECONNECT_BASE_DELAY_MS = 1000; // 重连基础延迟 (ms), 指数退避
    const int PING_INTERVAL_MS      = 500;    // PING 间隔 (ms)
    const int PING_TIMEOUT_MS       = 500;    // PING 超时 (ms)

    // ========== 错误升级参数 ==========
    const int ESCALATE_WARN_TO_DEGRADE   = 3;   // WARN 连续帧数 → DEGRADE
    const int ESCALATE_DEGRADE_TO_REJECT = 10;  // DEGRADE 连续帧数 → REJECT
    const int DEESCALATE_CLEAR_FRAMES    = 30;  // 清除后多少帧降级

    // ========== 诊断日志参数 ==========
    constexpr const char* DIAGNOSTIC_LOG_PATH = "robot_diagnostics.log";
```

- [ ] **Step 2: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean (constants not yet referenced by any code).

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat(config): add constraint force, health monitor, escalation, and diagnostics constants"
```

---

### Task 2: RobotError.h — Error Taxonomy

**Files:**
- Create: `Touch_Client/safety/RobotError.h`

**Interfaces:**
- Produces: `enum class RobotErrorCode` (24 error codes + OK), `enum class Severity {INFO, WARN, DEGRADE, REJECT, FATAL}`, `struct RobotError` (code, severity, timestampMs, targetPosition, currentJoints[6], speedFactor, latencyMs, consecutiveCount, message()), `Severity getSeverity(RobotErrorCode code)`, `const char* errorCodeName(RobotErrorCode code)`

- [ ] **Step 1: Create the header**

```cpp
#pragma once
#include <cstdint>
#include <cstdio>
#include "../relay/CoordinateTransform.h"

// ===== 机器人错误码 (24种) =====
enum class RobotErrorCode {
    // PRE-MOTION — 运动前预判
    ERR_WORKSPACE_RADIUS,       // 超出 620mm 工作半径
    ERR_Z_RANGE,                // Z 超出 0~795
    ERR_SAFETY_BOUNDARY,        // 超出用户安全边界
    ERR_CYLINDRICAL_SING,       // Z轴距离 < 30mm
    ERR_CYLINDRICAL_WARN,       // Z轴距离 < 80mm
    ERR_JOINTLIMIT_WARN,        // 关节距限位 < 10°
    ERR_JOINTLIMIT_EXCEED,      // 关节超出限位
    ERR_IK_NO_SOLUTION,         // IK 50次迭代不收敛
    ERR_IK_SINGULAR,            // 条件数 > 500
    ERR_IK_NEAR_SINGULAR,       // 条件数 > 100
    ERR_ALARM_HISTORY,          // 接近历史报警点 (< 80mm)
    // IN-MOTION — 运动执行反馈
    ERR_SERVOP_REJECTED,        // ServoP 被机器人拒绝
    ERR_SERVOP_TIMEOUT,         // ServoP 响应超时
    ERR_POSITION_DRIFT,         // 目标 vs 实际偏差 > 10mm
    ERR_VELOCITY_CLAMP,         // 速度被机器人钳位
    // SYSTEM — 连接/通信
    ERR_CONNECTION_LOST,        // 以太网断开
    ERR_HEARTBEAT_LOST,         // 心跳超时 500ms
    ERR_PROTOCOL_PARSE,         // 协议解析失败
    ERR_RESPONSE_INVALID,       // GetPose 返回异常值
    ERR_LATENCY_HIGH,           // RTT > 100ms
    // ALARM — 机器人主动报警
    ERR_ALARM_MODE9,            // 机器人进入 mode=9
    ERR_EMERGENCY_STOP,         // 急停被触发
    ERR_COLLISION,              // 碰撞检测触发

    OK = -1                     // 无错误
};

// ===== 严重度 =====
enum class Severity {
    INFO,       // 无影响
    WARN,       // 速度衰减 + 约束力激活
    DEGRADE,    // 强制减速 + 约束力增强
    REJECT,     // 拒绝该帧运动
    FATAL       // 停止运动 + DisableRobot
};

// ===== 错误上下文 =====
struct RobotError {
    RobotErrorCode code = RobotErrorCode::OK;
    Severity severity = Severity::INFO;
    uint64_t timestampMs = 0;
    Vec3 targetPosition = {0, 0, 0};
    double currentJoints[6] = {0};
    double speedFactor = 1.0;
    float latencyMs = 0.0f;
    int consecutiveCount = 0;

    void format(char* buf, int len) const {
        const char* name = errorCodeName(code);
        snprintf(buf, len,
            "%s target=(%.0f,%.0f,%.0f) joints=(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f) speed=%.2f",
            name,
            targetPosition.x, targetPosition.y, targetPosition.z,
            currentJoints[0], currentJoints[1], currentJoints[2],
            currentJoints[3], currentJoints[4], currentJoints[5],
            speedFactor);
    }
};

// ===== 严重度映射 =====
inline Severity getSeverity(RobotErrorCode code) {
    switch (code) {
        case RobotErrorCode::ERR_CYLINDRICAL_WARN:
        case RobotErrorCode::ERR_JOINTLIMIT_WARN:
        case RobotErrorCode::ERR_IK_NEAR_SINGULAR:
        case RobotErrorCode::ERR_ALARM_HISTORY:
        case RobotErrorCode::ERR_POSITION_DRIFT:
        case RobotErrorCode::ERR_VELOCITY_CLAMP:
        case RobotErrorCode::ERR_PROTOCOL_PARSE:
        case RobotErrorCode::ERR_RESPONSE_INVALID:
        case RobotErrorCode::ERR_LATENCY_HIGH:
            return Severity::WARN;

        case RobotErrorCode::ERR_IK_NO_SOLUTION:
            return Severity::DEGRADE;

        case RobotErrorCode::ERR_SAFETY_BOUNDARY:
        case RobotErrorCode::ERR_CYLINDRICAL_SING:
        case RobotErrorCode::ERR_JOINTLIMIT_EXCEED:
        case RobotErrorCode::ERR_IK_SINGULAR:
        case RobotErrorCode::ERR_SERVOP_REJECTED:
        case RobotErrorCode::ERR_SERVOP_TIMEOUT:
            return Severity::REJECT;

        case RobotErrorCode::ERR_WORKSPACE_RADIUS:
        case RobotErrorCode::ERR_Z_RANGE:
        case RobotErrorCode::ERR_CONNECTION_LOST:
        case RobotErrorCode::ERR_HEARTBEAT_LOST:
        case RobotErrorCode::ERR_ALARM_MODE9:
        case RobotErrorCode::ERR_EMERGENCY_STOP:
        case RobotErrorCode::ERR_COLLISION:
            return Severity::FATAL;

        default:
            return Severity::INFO;
    }
}

// ===== 错误码名称 =====
inline const char* errorCodeName(RobotErrorCode code) {
    switch (code) {
        case RobotErrorCode::ERR_WORKSPACE_RADIUS:  return "ERR_WORKSPACE_RADIUS";
        case RobotErrorCode::ERR_Z_RANGE:           return "ERR_Z_RANGE";
        case RobotErrorCode::ERR_SAFETY_BOUNDARY:   return "ERR_SAFETY_BOUNDARY";
        case RobotErrorCode::ERR_CYLINDRICAL_SING:  return "ERR_CYLINDRICAL_SING";
        case RobotErrorCode::ERR_CYLINDRICAL_WARN:  return "ERR_CYLINDRICAL_WARN";
        case RobotErrorCode::ERR_JOINTLIMIT_WARN:   return "ERR_JOINTLIMIT_WARN";
        case RobotErrorCode::ERR_JOINTLIMIT_EXCEED: return "ERR_JOINTLIMIT_EXCEED";
        case RobotErrorCode::ERR_IK_NO_SOLUTION:    return "ERR_IK_NO_SOLUTION";
        case RobotErrorCode::ERR_IK_SINGULAR:       return "ERR_IK_SINGULAR";
        case RobotErrorCode::ERR_IK_NEAR_SINGULAR:  return "ERR_IK_NEAR_SINGULAR";
        case RobotErrorCode::ERR_ALARM_HISTORY:     return "ERR_ALARM_HISTORY";
        case RobotErrorCode::ERR_SERVOP_REJECTED:   return "ERR_SERVOP_REJECTED";
        case RobotErrorCode::ERR_SERVOP_TIMEOUT:    return "ERR_SERVOP_TIMEOUT";
        case RobotErrorCode::ERR_POSITION_DRIFT:    return "ERR_POSITION_DRIFT";
        case RobotErrorCode::ERR_VELOCITY_CLAMP:    return "ERR_VELOCITY_CLAMP";
        case RobotErrorCode::ERR_CONNECTION_LOST:   return "ERR_CONNECTION_LOST";
        case RobotErrorCode::ERR_HEARTBEAT_LOST:    return "ERR_HEARTBEAT_LOST";
        case RobotErrorCode::ERR_PROTOCOL_PARSE:    return "ERR_PROTOCOL_PARSE";
        case RobotErrorCode::ERR_RESPONSE_INVALID:  return "ERR_RESPONSE_INVALID";
        case RobotErrorCode::ERR_LATENCY_HIGH:      return "ERR_LATENCY_HIGH";
        case RobotErrorCode::ERR_ALARM_MODE9:       return "ERR_ALARM_MODE9";
        case RobotErrorCode::ERR_EMERGENCY_STOP:    return "ERR_EMERGENCY_STOP";
        case RobotErrorCode::ERR_COLLISION:         return "ERR_COLLISION";
        default:                                    return "OK";
    }
}
```

- [ ] **Step 2: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean. Header is self-contained, no .cpp needed.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/safety/RobotError.h
git commit -m "feat(safety): add RobotError taxonomy — 24 error codes, severity levels, error context struct"
```

---

### Task 3: EscalationTracker.h — Error Escalation Counter

**Files:**
- Create: `Touch_Client/safety/EscalationTracker.h`

**Interfaces:**
- Produces: `struct EscalationTracker` with methods `recordError(RobotErrorCode, Vec3 delta)`, `shouldEscalate()`, `shouldDeescalate(Vec3 delta)`, `reset()`, `isEscalated()`, `consecutiveCount()`

- [ ] **Step 1: Create the header**

```cpp
#pragma once
#include <cmath>
#include "RobotError.h"
#include "../relay/CoordinateTransform.h"

// ===== 错误升级跟踪器 =====
// 持续触发同一错误 → 升级 WARN→DEGRADE→REJECT
// 反向运动 → 立即解除
struct EscalationTracker {
    RobotErrorCode currentCode = RobotErrorCode::OK;
    int consecutiveFrames = 0;
    Vec3 lastRejectDirection = {0, 0, 0};
    bool escalated = false;
    int clearFrames = 0;  // 错误清除后的帧计数

    static const int WARN_TO_DEGRADE;    // from Config
    static const int DEGRADE_TO_REJECT;
    static const int CLEAR_FRAMES;

    // 记录一帧的错误
    // delta: 操作员当前移动方向 (Touch 增量)
    void recordError(RobotErrorCode code, const Vec3& delta) {
        if (code == currentCode) {
            consecutiveFrames++;
        } else {
            currentCode = code;
            consecutiveFrames = 1;
        }
        clearFrames = 0;
        lastRejectDirection = delta;
    }

    // 检查是否应升级
    bool shouldEscalate() const {
        Severity sev = getSeverity(currentCode);
        if (sev == Severity::WARN && consecutiveFrames >= WARN_TO_DEGRADE)
            return true;
        if (sev == Severity::DEGRADE && consecutiveFrames >= DEGRADE_TO_REJECT)
            return true;
        return false;
    }

    // 检查反向运动 (操作员远离危险)
    // dangerDir: 指向危险区域的方向 (如指向Z轴、指向边界外)
    bool shouldDeescalate(const Vec3& delta, const Vec3& dangerDir) const {
        if (!escalated) return false;
        double dot = delta.x * dangerDir.x + delta.y * dangerDir.y + delta.z * dangerDir.z;
        return dot < 0;  // 点积为负 = 远离危险
    }

    // 错误已清除 (当前帧无错误)
    void onClear() {
        clearFrames++;
        if (clearFrames >= CLEAR_FRAMES) {
            reset();
        }
    }

    void reset() {
        currentCode = RobotErrorCode::OK;
        consecutiveFrames = 0;
        escalated = false;
        clearFrames = 0;
        lastRejectDirection = {0, 0, 0};
    }

    bool isEscalated() const { return escalated; }
    int count() const { return consecutiveFrames; }
};

// static const definitions (from Config)
const int EscalationTracker::WARN_TO_DEGRADE   = 3;   // Config::ESCALATE_WARN_TO_DEGRADE
const int EscalationTracker::DEGRADE_TO_REJECT = 10;  // Config::ESCALATE_DEGRADE_TO_REJECT
const int EscalationTracker::CLEAR_FRAMES      = 30;  // Config::DEESCALATE_CLEAR_FRAMES
```

- [ ] **Step 2: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/safety/EscalationTracker.h
git commit -m "feat(safety): add EscalationTracker — error escalation with reverse-motion de-escalation"
```

---

### Task 4: RobotStateMachine.h/.cpp — State-Driven Error Response

**Files:**
- Create: `Touch_Client/safety/RobotStateMachine.h`
- Create: `Touch_Client/safety/RobotStateMachine.cpp`

**Interfaces:**
- Produces (h): `enum class RobotState {DISCONNECTED, CONNECTED, READY, RUNNING, DEGRADED, ALARM, RECOVERING, FATAL}`, `class RobotStateMachine` with `currentState()`, `stateName()`, `speedFactor()`, `canMove()`, `canSendForce()`, `onError(RobotError&)`, `onRecovery()`, `onButtonPress()`, `onButtonRelease()`, `onConnect()`, `onDisconnect()`, `onEnableSuccess()`, `onEnableFail()`, `transitionTo(RobotState)`
- Consumes: RobotError.h, EscalationTracker.h

- [ ] **Step 1: Write RobotStateMachine.h**

```cpp
#pragma once
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

    // 查询
    RobotState currentState() const { return m_state; }
    const char* stateStr() const { return stateName(m_state); }
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

    // 错误跟踪器 (供 SafetyPredictor 读取)
    EscalationTracker& escalation() { return m_escalation; }
    const EscalationTracker& escalation() const { return m_escalation; }

private:
    RobotState m_state = RobotState::DISCONNECTED;
    EscalationTracker m_escalation;
    RobotError m_lastError;  // 最近一次错误
};
```

- [ ] **Step 2: Write RobotStateMachine.cpp**

```cpp
#include "RobotStateMachine.h"
#include "../config/Config.h"
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
```

- [ ] **Step 3: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean. Note: `std::max` requires `<algorithm>` — included implicitly via other headers, but add `#include <algorithm>` to .cpp if needed.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/safety/RobotStateMachine.h Touch_Client/safety/RobotStateMachine.cpp
git commit -m "feat(safety): add RobotStateMachine — 8-state FSM with error-driven transitions and escalation"
```

---

### Task 5: RobotDiagnostics.h/.cpp — Structured Diagnostic Logging

**Files:**
- Create: `Touch_Client/safety/RobotDiagnostics.h`
- Create: `Touch_Client/safety/RobotDiagnostics.cpp`

**Interfaces:**
- Produces (h): `struct DiagnosticEvent` (timestampMs, fromState, toState, error, targetPosition, jointAngles[6], latencyMs, speedFactor, constraintForceMag), `class RobotDiagnostics` as singleton with `init(path)`, `log(DiagnosticEvent&)`, `logStateChange(RobotState from, RobotState to)`, `logError(RobotError&, double constraintMag)`, `writeSessionReport()`, `shutdown()`, `errorCount(RobotErrorCode)`, `getHistory()` → pointer to ring buffer, `historySize()`
- Consumes: RobotError.h, RobotStateMachine.h

- [ ] **Step 1: Write RobotDiagnostics.h**

```cpp
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
```

- [ ] **Step 2: Write RobotDiagnostics.cpp**

```cpp
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
```

- [ ] **Step 3: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/safety/RobotDiagnostics.h Touch_Client/safety/RobotDiagnostics.cpp
git commit -m "feat(safety): add RobotDiagnostics — ring buffer, file logger, error counters, session report"
```

---

### Task 6: ConstraintForce.h/.cpp — Virtual Constraint Force Fields

**Files:**
- Create: `Touch_Client/safety/ConstraintForce.h`
- Create: `Touch_Client/safety/ConstraintForce.cpp`

**Interfaces:**
- Produces: `namespace ConstraintForce` with `computeBoundaryForce(Vec3 target, double out[3])`, `computeSingularForce(Vec3 target, double out[3])`, `computeAlarmHistoryForce(Vec3 target, const std::vector<AlarmRecord>& alarms, double out[3])`, `computeWorkspaceEdgeForce(Vec3 target, double out[3])`, `computeTotalForce(Vec3 target, const std::vector<AlarmRecord>& alarms, double out[3])` — superposition with priority order, clamped to 3.3N
- Consumes: Config.h, CoordinateTransform.h, SafetyPredictor.h (for AlarmRecord)

- [ ] **Step 1: Write ConstraintForce.h**

```cpp
#pragma once
#include <vector>
#include "../relay/CoordinateTransform.h"

struct AlarmRecord;  // forward decl from SafetyPredictor.h

namespace ConstraintForce {

    // 安全边界排斥力 — 垂直边界向内推
    // dist < 50mm → 0→2.0N 线性
    void computeBoundaryForce(const Vec3& target, double out[3]);

    // 圆柱奇异排斥力 — 径向向外推
    // r_xy < 80mm → 0→2.5N 二次增长
    void computeSingularForce(const Vec3& target, double out[3]);

    // 报警历史排斥力 — 远离报警点
    // dist < 80mm → 0→1.5N 反比
    void computeAlarmHistoryForce(const Vec3& target,
        const std::vector<AlarmRecord>& alarms, double out[3]);

    // 工作空间边缘向心力
    // dist_from_origin > 550mm → 0→1.0N 线性
    void computeWorkspaceEdgeForce(const Vec3& target, double out[3]);

    // 叠加所有力场 (优先级: 边界 > 奇异 > 报警 > 边缘)
    // 总力 clamp 到 ±3.3N
    void computeTotalForce(const Vec3& target,
        const std::vector<AlarmRecord>& alarms, double out[3]);

} // namespace ConstraintForce
```

- [ ] **Step 2: Write ConstraintForce.cpp**

```cpp
#include "ConstraintForce.h"
#include "../config/Config.h"
#include "../safety/SafetyPredictor.h"
#include <algorithm>
#include <cmath>

namespace ConstraintForce {

// ===== 辅助: 向量归一化 =====
static void normalize(const double v[3], double out[3]) {
    double mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag < 1e-12) { out[0] = out[1] = out[2] = 0.0; return; }
    double inv = 1.0 / mag;
    out[0] = v[0] * inv;
    out[1] = v[1] * inv;
    out[2] = v[2] * inv;
}

static double length(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// ===== 安全边界力 =====
void computeBoundaryForce(const Vec3& target, double out[3]) {
    out[0] = out[1] = out[2] = 0.0;

    double range = Config::CONSTRAINT_BOUNDARY_RANGE;
    double maxF  = Config::CONSTRAINT_BOUNDARY_MAX_FORCE;

    // 检查每个边界
    double dists[6] = {
        target.x - Config::SAFE_X_MIN,   // 距下界
        Config::SAFE_X_MAX - target.x,   // 距上界
        target.y - Config::SAFE_Y_MIN,
        Config::SAFE_Y_MAX - target.y,
        target.z - Config::SAFE_Z_MIN,
        Config::SAFE_Z_MAX - target.z,
    };
    double dirs[6][3] = {
        { 1, 0, 0}, {-1, 0, 0},  // X: 远离下界 / 远离上界
        { 0, 1, 0}, { 0,-1, 0},  // Y
        { 0, 0, 1}, { 0, 0,-1},  // Z
    };

    for (int i = 0; i < 6; i++) {
        if (dists[i] < range && dists[i] >= 0) {
            // 线性衰减: 边界上 = maxF, range 远处 = 0
            double ratio = 1.0 - (dists[i] / range);
            double f = ratio * maxF;
            out[0] += dirs[i][0] * f;
            out[1] += dirs[i][1] * f;
            out[2] += dirs[i][2] * f;
        }
    }
}

// ===== 圆柱奇异力 =====
void computeSingularForce(const Vec3& target, double out[3]) {
    out[0] = out[1] = out[2] = 0.0;

    double r_xy = sqrt(target.x * target.x + target.y * target.y);
    double range = Config::CONSTRAINT_SINGULAR_RANGE;
    double maxF  = Config::CONSTRAINT_SINGULAR_MAX_FORCE;

    if (r_xy >= range) return;  // 安全区域

    // 二次增长: 越接近Z轴力越大
    double ratio = 1.0 - (r_xy / range);
    double f = ratio * ratio * maxF;  // 二次

    // 方向: 径向向外 = normalize(target.x, target.y, 0)
    if (r_xy > 1e-12) {
        double inv = 1.0 / r_xy;
        out[0] = target.x * inv * f;
        out[1] = target.y * inv * f;
        out[2] = 0.0;
    }
}

// ===== 报警历史力 =====
void computeAlarmHistoryForce(const Vec3& target,
    const std::vector<AlarmRecord>& alarms, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;

    if (alarms.empty()) return;

    double range = Config::CONSTRAINT_ALARM_HISTORY_RANGE;
    double maxF  = Config::CONSTRAINT_ALARM_HISTORY_MAX_FORCE;

    // 找最近的报警点
    double minDist = 1e12;
    Vec3 nearestDir = {0, 0, 0};
    for (const auto& a : alarms) {
        double dx = target.x - a.x;
        double dy = target.y - a.y;
        double dz = target.z - a.z;
        double d = sqrt(dx*dx + dy*dy + dz*dz);
        if (d < minDist) {
            minDist = d;
            nearestDir.x = dx;
            nearestDir.y = dy;
            nearestDir.z = dz;
        }
    }

    if (minDist >= range) return;

    // 反比力: 越近力越大
    double ratio = 1.0 - (minDist / range);
    double f = ratio * maxF;

    // 方向: 远离报警点
    if (minDist > 1e-12) {
        double inv = 1.0 / minDist;
        out[0] = nearestDir.x * inv * f;
        out[1] = nearestDir.y * inv * f;
        out[2] = nearestDir.z * inv * f;
    }
}

// ===== 工作空间边缘力 =====
void computeWorkspaceEdgeForce(const Vec3& target, double out[3]) {
    out[0] = out[1] = out[2] = 0.0;

    double dist = sqrt(target.x*target.x + target.y*target.y);
    double edge = Config::CONSTRAINT_WORKSPACE_EDGE_START;
    double radius = Config::WORKSPACE_RADIUS;
    double maxF = Config::CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE;

    if (dist <= edge) return;  // 安全

    double range = radius - edge;  // 550→620 = 70mm
    if (range <= 0) return;
    double ratio = (dist - edge) / range;  // 0→1
    if (ratio > 1.0) ratio = 1.0;
    double f = ratio * maxF;

    // 方向: 向心 (指向原点), 仅XY平面
    if (dist > 1e-12) {
        double inv = 1.0 / dist;
        out[0] = -target.x * inv * f;
        out[1] = -target.y * inv * f;
        out[2] = 0.0;
    }
}

// ===== 总力叠加 =====
void computeTotalForce(const Vec3& target,
    const std::vector<AlarmRecord>& alarms, double out[3])
{
    double f[4][3] = {{0}};

    // 按优先级计算 (低优先级先算，高优先级覆盖效果由顺序无关紧要，
    // 因为所有力都叠加后统一clamp)
    computeBoundaryForce(target, f[0]);
    computeSingularForce(target, f[1]);
    computeAlarmHistoryForce(target, alarms, f[2]);
    computeWorkspaceEdgeForce(target, f[3]);

    // 叠加
    double total[3] = {0};
    for (int i = 0; i < 4; i++) {
        total[0] += f[i][0];
        total[1] += f[i][1];
        total[2] += f[i][2];
    }

    // Clamp 到 Touch 安全限制
    double mag = sqrt(total[0]*total[0] + total[1]*total[1] + total[2]*total[2]);
    double maxF = Config::FORCE_MAX_TOUCH_N;
    if (mag > maxF) {
        double scale = maxF / mag;
        total[0] *= scale;
        total[1] *= scale;
        total[2] *= scale;
    }

    out[0] = total[0];
    out[1] = total[1];
    out[2] = total[2];
}

} // namespace ConstraintForce
```

- [ ] **Step 3: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean once `#include "../safety/SafetyPredictor.h"` resolves (SafetyPredictor.h already exists). If circular include issue (SafetyPredictor.h may include ConstraintForce.h later), forward-declare AlarmRecord in ConstraintForce.h and include SafetyPredictor.h only in .cpp.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/safety/ConstraintForce.h Touch_Client/safety/ConstraintForce.cpp
git commit -m "feat(safety): add ConstraintForce — 4 virtual force fields with superposition and 3.3N clamp"
```

---

### Task 7: test_constraint_force.cpp — Constraint Force Unit Tests

**Files:**
- Create: `Touch_Client/tests/test_constraint_force.cpp`

**Interfaces:**
- Consumes: ConstraintForce.h, SafetyPredictor.h (for AlarmRecord)

- [ ] **Step 1: Write the test file**

```cpp
// Standalone test: ConstraintForce virtual force fields
// Build: cl /EHsc /std:c++17 test_constraint_force.cpp ../safety/ConstraintForce.cpp
//        ../safety/SafetyPredictor.cpp ../robot/Kinematics.cpp /Fe:test_constraint_force.exe
// Run: test_constraint_force.exe

#include <iostream>
#include <cassert>
#include <cmath>

#define M_PI 3.14159265358979323846

// Include project headers
#include "../safety/ConstraintForce.h"
#include "../safety/SafetyPredictor.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== Test 1: Boundary force = 0 at center =====
static void test_boundary_force_zero_at_center() {
    TEST(boundary_zero_at_center);
    double f[3];
    Vec3 target = {0, 0, 300};  // well within all boundaries
    ConstraintForce::computeBoundaryForce(target, f);
    CHECK(fabs(f[0]) < 0.01 && fabs(f[1]) < 0.01 && fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 2: Boundary force pushes inward near X_MAX =====
static void test_boundary_force_near_xmax() {
    TEST(boundary_near_xmax);
    double f[3];
    double xNear = Config::SAFE_X_MAX - 10.0;  // 10mm from X_MAX, well within 50mm range
    Vec3 target = { xNear, 0, 300 };
    ConstraintForce::computeBoundaryForce(target, f);
    // Force should push toward -X (away from boundary)
    CHECK(f[0] < -0.1);  // negative X force
    CHECK(fabs(f[1]) < 0.01);
    CHECK(fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 3: Boundary force pushes inward near X_MIN =====
static void test_boundary_force_near_xmin() {
    TEST(boundary_near_xmin);
    double f[3];
    double xNear = Config::SAFE_X_MIN + 10.0;
    Vec3 target = { xNear, 0, 300 };
    ConstraintForce::computeBoundaryForce(target, f);
    // Force should push toward +X (away from boundary)
    CHECK(f[0] > 0.1);  // positive X force
    PASS();
}

// ===== Test 4: Singular force points radially outward =====
static void test_singular_force_radial() {
    TEST(singular_radial);
    double f[3];
    Vec3 target = { 10, 0, 300 };  // r_xy=10mm < 80mm range
    ConstraintForce::computeSingularForce(target, f);
    // Force should point radially outward: +X direction
    CHECK(f[0] > 0.1);   // radial outward is +X
    CHECK(fabs(f[1]) < 0.01);
    CHECK(fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 5: Singular force = 0 far from Z axis =====
static void test_singular_force_zero_far() {
    TEST(singular_zero_far);
    double f[3];
    Vec3 target = { 200, 0, 300 };  // r_xy=200mm > 80mm range
    ConstraintForce::computeSingularForce(target, f);
    CHECK(fabs(f[0]) < 0.01 && fabs(f[1]) < 0.01 && fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 6: Total force clamped to 3.3N =====
static void test_total_force_clamped() {
    TEST(total_clamped);
    double f[3];
    // Position that triggers all force fields simultaneously
    Vec3 target = { 580, 0, 5 };  // near workspace edge + low Z
    ConstraintForce::computeTotalForce(target, {}, f);
    double mag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    CHECK(mag <= Config::FORCE_MAX_TOUCH_N + 0.01);
    PASS();
}

// ===== Test 7: Total force = 0 in safe zone =====
static void test_total_force_zero_safe() {
    TEST(total_zero_safe);
    double f[3];
    Vec3 target = { 0, 0, 300 };  // center of workspace
    ConstraintForce::computeTotalForce(target, {}, f);
    double mag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    CHECK(mag < 0.01);
    PASS();
}

int main() {
    std::cout << "=== ConstraintForce Unit Tests ===" << std::endl;
    test_boundary_force_zero_at_center();
    test_boundary_force_near_xmax();
    test_boundary_force_near_xmin();
    test_singular_force_radial();
    test_singular_force_zero_far();
    test_total_force_clamped();
    test_total_force_zero_safe();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
```

- [ ] **Step 2: Build and run tests**

```bash
cd Touch_Client/tests
cl /EHsc /std:c++17 test_constraint_force.cpp ../safety/ConstraintForce.cpp ../safety/SafetyPredictor.cpp ../robot/Kinematics.cpp /Fe:test_constraint_force.exe
./test_constraint_force.exe
```

Expected: 7/7 tests pass.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/tests/test_constraint_force.cpp
git commit -m "test(safety): add unit tests for ConstraintForce — boundary, singular, clamp, safe zone"
```

---

### Task 8: SafetyPredictor Upgrade — ErrorCode + Escalation + ConstraintForce

**Files:**
- Modify: `Touch_Client/safety/SafetyPredictor.h`
- Modify: `Touch_Client/safety/SafetyPredictor.cpp`

**Interfaces:**
- Extends: `SafetyVerdict` struct with `RobotErrorCode errorCode` and `double constraintForce[3]`
- Adds: `computeConstraintForce(Vec3, double out[3])` method, `EscalationTracker` member, `lastError()` accessor
- Modifies: `evaluate()` to set `verdict.errorCode` and `verdict.constraintForce`; returns `SafetyVerdict` (unchanged signature)

- [ ] **Step 1: Update SafetyVerdict struct in SafetyPredictor.h**

In `Touch_Client/safety/SafetyPredictor.h`, add `#include "RobotError.h"` at top, then modify the `SafetyVerdict` struct (lines 13-18):

```cpp
// Replace the existing SafetyVerdict struct with:
#include "RobotError.h"
#include "EscalationTracker.h"
#include "ConstraintForce.h"

struct SafetyVerdict {
    enum Action { ALLOW = 0, WARN_SLOW = 1, REJECT = 2 };
    Action action;
    RobotErrorCode errorCode;      // NEW — specific error code
    const char* reason;
    double speedFactor;            // 1.0 = full speed, 0.0 = stop
    double constraintForce[3];     // NEW — Touch坐标系虚拟约束力

    SafetyVerdict() : action(ALLOW), errorCode(RobotErrorCode::OK),
        reason(nullptr), speedFactor(1.0)
    {
        constraintForce[0] = constraintForce[1] = constraintForce[2] = 0.0;
    }
};
```

Add to SafetyPredictor class (after `lastVerdict()` line):

```cpp
    // 计算虚拟约束力 (每次 ServoP 前调用)
    void computeConstraintForce(const Vec3& target, double out[3]);

    // 获取最近一次 RobotError
    RobotError lastError() const { return m_lastError; }

    // 升级跟踪器
    EscalationTracker& escalation() { return m_escalation; }
```

Add to private members:

```cpp
    RobotError m_lastError;
    EscalationTracker m_escalation;
```

- [ ] **Step 2: Update SafetyPredictor.cpp — evaluate() to set errorCode and constraintForce**

In `Touch_Client/safety/SafetyPredictor.cpp`, at the top add:

```cpp
#include "ConstraintForce.h"
```

Then modify `evaluate()`. For each `m_lastVerdict = {SafetyVerdict::REJECT, "reason", 0.0};` line, change to include `errorCode`. The pattern for each return:

```cpp
// Example — replace line 36:
// OLD:  m_lastVerdict = {SafetyVerdict::REJECT, "exceeds workspace radius (620mm)", 0.0};
// NEW:
m_lastVerdict.action = SafetyVerdict::REJECT;
m_lastVerdict.errorCode = RobotErrorCode::ERR_WORKSPACE_RADIUS;
m_lastVerdict.reason = "exceeds workspace radius (620mm)";
m_lastVerdict.speedFactor = 0.0;
```

Apply the same pattern to ALL return statements in evaluate():

| Line | Old Reason | New errorCode |
|------|-----------|---------------|
| 36 | "exceeds workspace radius" | ERR_WORKSPACE_RADIUS |
| 41 | "Z-axis out of range" | ERR_Z_RANGE |
| 48 | "exceeds safety boundary" | ERR_SAFETY_BOUNDARY |
| 55 | "cylindrical singularity (<30mm)" | ERR_CYLINDRICAL_SING |
| 59 | "approaching cylindrical singularity (<80mm)" | ERR_CYLINDRICAL_WARN |
| 98 | "near joint limit" | ERR_JOINTLIMIT_WARN |
| 108 | "IK no solution" | ERR_IK_NO_SOLUTION |
| 114 | "joint outside limits" | ERR_JOINTLIMIT_EXCEED |
| 125 | "singular configuration (cond>500)" | ERR_IK_SINGULAR |
| 130 | "near singular region" | ERR_IK_NEAR_SINGULAR |
| 140 | "near historical alarm point (<30mm)" | ERR_ALARM_HISTORY |
| 145 | "near historical alarm zone (<80mm)" | ERR_ALARM_HISTORY |
| 152 | nullptr (ALLOW) | OK |

After all return statements, at the end of `evaluate()` (after the `m_lastVerdict = {ALLOW}` line and before `return`), compute constraint force:

```cpp
    // Compute constraint force for haptic rendering
    ConstraintForce::computeTotalForce(target, m_alarmList, m_lastVerdict.constraintForce);

    // Build RobotError for diagnostics
    m_lastError.code = m_lastVerdict.errorCode;
    m_lastError.severity = getSeverity(m_lastVerdict.errorCode);
    m_lastError.timestampMs = GetTickCount64();
    m_lastError.targetPosition = target;
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        m_lastError.currentJoints[0] = app.robotActualPose.j1;
        m_lastError.currentJoints[1] = app.robotActualPose.j2;
        m_lastError.currentJoints[2] = app.robotActualPose.j3;
        m_lastError.currentJoints[3] = app.robotActualPose.j4;
        m_lastError.currentJoints[4] = app.robotActualPose.j5;
        m_lastError.currentJoints[5] = app.robotActualPose.j6;
        LeaveCriticalSection(&app.robotPoseMutex);
    }
    m_lastError.speedFactor = m_lastVerdict.speedFactor;
```

- [ ] **Step 3: Add computeConstraintForce() method**

Add new method to SafetyPredictor.cpp:

```cpp
void SafetyPredictor::computeConstraintForce(const Vec3& target, double out[3]) {
    ConstraintForce::computeTotalForce(target, m_alarmList, out);
}
```

- [ ] **Step 4: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean. Existing callers of `SafetyPredictor::evaluate()` use `verdict.action`, `verdict.reason`, `verdict.speedFactor` — unchanged fields, no API break.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/safety/SafetyPredictor.h Touch_Client/safety/SafetyPredictor.cpp
git commit -m "feat(safety): integrate RobotErrorCode, EscalationTracker, and ConstraintForce into SafetyPredictor"
```

---

### Task 9: FeedbackParser — Error Code Extraction

**Files:**
- Modify: `Touch_Client/relay/FeedbackParser.h`
- Modify: `Touch_Client/relay/FeedbackParser.cpp`

**Interfaces:**
- Produces: `FeedbackParser::extractErrorCode(const char* feedback, int& out)` → bool, `FeedbackParser::mapRobotErrorCode(int dobotCode)` → RobotErrorCode

- [ ] **Step 1: Add declarations to FeedbackParser.h**

In `Touch_Client/relay/FeedbackParser.h`, add after the `isSuccess` declaration (line 15) and include RobotError.h:

```cpp
#include "../safety/RobotError.h"

// In namespace FeedbackParser, add after isSuccess():
    // 提取 Dobot 错误码: "-1,{0x0002},ServoP();" → 0x0002
    bool extractErrorCode(const char* feedback, int& out);

    // 映射 Dobot 错误码 → RobotErrorCode
    RobotErrorCode mapRobotErrorCode(int dobotCode);
```

- [ ] **Step 2: Add implementations to FeedbackParser.cpp**

In `Touch_Client/relay/FeedbackParser.cpp`, add:

```cpp
bool extractErrorCode(const char* feedback, int& out) {
    if (!feedback) return false;
    // Format: "-1,{0x0002},ServoP();"
    // or: "0,{},ServoP();" → success, no error
    if (feedback[0] == '0') {
        out = 0;
        return true;  // success response, error code 0
    }
    // Find "{...}" containing the error code
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;
    
    // Parse hex: "0x0002"
    const char* hex = start + 1;
    if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) {
        out = (int)strtol(hex + 2, nullptr, 16);
        return true;
    }
    // Try decimal
    out = atoi(hex);
    return true;
}

RobotErrorCode mapRobotErrorCode(int dobotCode) {
    switch (dobotCode) {
        case 0x0001: return RobotErrorCode::ERR_WORKSPACE_RADIUS;
        case 0x0002: return RobotErrorCode::ERR_JOINTLIMIT_EXCEED;
        case 0x0004: return RobotErrorCode::ERR_VELOCITY_CLAMP;
        case 0x0008: return RobotErrorCode::ERR_VELOCITY_CLAMP;
        case 0x0010: return RobotErrorCode::ERR_IK_SINGULAR;
        case 0x0020: return RobotErrorCode::ERR_COLLISION;
        default:     return RobotErrorCode::ERR_SERVOP_REJECTED;  // unknown → generic
    }
}
```

Add `#include <cstdlib>` at top if not already present (for `strtol`).

- [ ] **Step 3: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/relay/FeedbackParser.h Touch_Client/relay/FeedbackParser.cpp
git commit -m "feat(relay): add ServoP error code extraction and Dobot→RobotErrorCode mapping"
```

---

### Task 10: RelayCore Integration — State Machine + Health Monitor + ServoP Parsing + PING/PONG

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`
- Modify: `Touch_Client/relay/RelayCore.cpp`

**Interfaces:**
- Adds: `RobotStateMachine m_stateMachine` member, `HealthMonitor` nested logic, PING/PONG in `queryPose()`, ServoP error parsing in `pollFeedback()`, state machine calls in `sendPosition()`, `onButtonPress()`, `onButtonRelease()`, `init()`, `shutdown()`

- [ ] **Step 1: Update RelayCore.h**

In `Touch_Client/relay/RelayCore.h`, add include and members:

```cpp
#include "../safety/RobotStateMachine.h"

// Add public methods:
    // 状态机 (供 HUD / 外部读取)
    RobotStateMachine& stateMachine() { return m_stateMachine; }
    const RobotStateMachine& stateMachine() const { return m_stateMachine; }

    // PING/PONG 延迟测量
    void pingRobot();

// Add private member (after m_forceThread line 70):
    RobotStateMachine m_stateMachine;
    DWORD m_lastPingMs = 0;
    DWORD m_lastHeartbeatMs = 0;
```

- [ ] **Step 2: Update RelayCore::init() — integrate state machine**

In `Touch_Client/relay/RelayCore.cpp`, modify `RelayCore::init()`:

After `robotConnect()` succeeds (line 274), call:
```cpp
    m_stateMachine.onConnect();
```

After `EnableRobot()` succeeds + `GetPose` succeeds (line 358), call:
```cpp
    m_stateMachine.onEnableSuccess();
```

On any failure that returns false, call:
```cpp
    m_stateMachine.onEnableFail();
```

After `m_stateMachine.onConnect()`:
```cpp
    m_lastHeartbeatMs = GetTickCount();
```

- [ ] **Step 3: Update RelayCore::shutdown() — integrate state machine**

In `RelayCore::shutdown()`, add:
```cpp
    m_stateMachine.onDisconnect();
```

- [ ] **Step 4: Update sendPosition() — state machine + escalation**

In `RelayCore::sendPosition()`, after the `SafetyVerdict verdict = ...` line (~line 440), add state machine integration:

```cpp
    // State machine: escalate on warning
    if (verdict.errorCode != RobotErrorCode::OK) {
        Vec3 deltaVec(dx, dy, dz);
        RobotError error = SafetyPredictor::instance().lastError();
        m_stateMachine.onError(error, deltaVec);
    } else {
        m_stateMachine.escalation().onClear();
    }

    // Check if state machine allows motion
    if (!m_stateMachine.canMove()) {
        LeaveCriticalSection(&m_basePointLock);
        return;
    }

    // Apply state machine speed factor ON TOP of safety verdict
    double effectiveSpeed = s_speedMul * m_stateMachine.speedFactor();
```

Apply `effectiveSpeed` instead of `s_speedMul` in the maxStep calculation.

- [ ] **Step 5: Update onButtonPress/Release**

In `RelayCore::onButtonPress()`, add:
```cpp
    m_stateMachine.onButtonPress();
```

In `RelayCore::onButtonRelease()`, add:
```cpp
    m_stateMachine.onButtonRelease();
```

- [ ] **Step 6: Update pollFeedback() — parse ServoP errors**

In `RelayCore::pollFeedback()`, after the existing `isError` check (line 544 area), add ServoP error parsing:

```cpp
        if (fb.raw[0] != '0') {
            int dobotCode = 0;
            if (FeedbackParser::extractErrorCode(fb.raw, dobotCode) && dobotCode != 0) {
                RobotErrorCode errCode = FeedbackParser::mapRobotErrorCode(dobotCode);
                RobotError error;
                error.code = errCode;
                error.severity = getSeverity(errCode);
                error.timestampMs = GetTickCount64();
                // Get current target from state
                error.targetPosition = m_targetPos;
                error.speedFactor = m_stateMachine.speedFactor();
                
                Vec3 zeroDelta = {0, 0, 0};  // no user delta for feedback errors
                m_stateMachine.onError(error, zeroDelta);
                
                double constraintMag = 0;  // feedback error has no constraint force
                RobotDiagnostics::instance().logError(error, constraintMag);
            }
        }
```

- [ ] **Step 7: Add PING/PONG to queryPose()**

At the end of `RelayCore::queryPose()`, add PING logic:

```cpp
    // PING/PONG latency measurement
    DWORD now = GetTickCount();
    if (now - m_lastPingMs > (DWORD)Config::PING_INTERVAL_MS) {
        m_lastPingMs = now;
        pingRobot();
    }

    // Heartbeat update
    m_lastHeartbeatMs = now;
```

Add `pingRobot()` implementation:

```cpp
void RelayCore::pingRobot() {
    if (!isRobotConnected()) return;
    char pingBuf[64];
    snprintf(pingBuf, sizeof(pingBuf), "PING|%llu", GetTickCount64());
    robotSendEnable(pingBuf);
    // Response handled in pollFeedback (PONG echo)
}
```

Add heartbeat check in `queryPose()` (also add after GetAngle):

```cpp
    // Health check: if no heartbeat for HEARTBEAT_TIMEOUT_MS, trigger disconnect
    DWORD now2 = GetTickCount();
    if (now2 - m_lastHeartbeatMs > (DWORD)Config::HEARTBEAT_TIMEOUT_MS) {
        RobotError error;
        error.code = RobotErrorCode::ERR_HEARTBEAT_LOST;
        error.severity = Severity::FATAL;
        error.timestampMs = GetTickCount64();
        Vec3 zeroDelta = {0, 0, 0};
        m_stateMachine.onError(error, zeroDelta);
    }
```

- [ ] **Step 8: Update checkAlarm() — use state machine**

In `RelayCore::checkAlarm()`, when mode==9 is detected:

```cpp
    RobotError error;
    error.code = RobotErrorCode::ERR_ALARM_MODE9;
    error.severity = Severity::FATAL;
    error.timestampMs = GetTickCount64();
    Vec3 zeroDelta = {0, 0, 0};
    m_stateMachine.onError(error, zeroDelta);
```

After `escapeSingularity()` succeeds:
```cpp
    m_stateMachine.onRecovery();
```

- [ ] **Step 9: Initialize diagnostics in main via RelayCore**

In `Touch_Client/main.cpp`, add after existing init block (after line 203):

```cpp
    // 5. 初始化诊断日志
    RobotDiagnostics::instance().init(Config::DIAGNOSTIC_LOG_PATH);
```

In the `keyboard()` shutdown handler (before `exit(0)`), add:
```cpp
    RobotDiagnostics::instance().shutdown();
```

Add include at top of main.cpp:
```cpp
#include "safety/RobotDiagnostics.h"
```

- [ ] **Step 10: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles with no errors. Some warnings may appear for unused variables in the health check path — acceptable.

- [ ] **Step 11: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/main.cpp
git commit -m "feat(relay): integrate state machine, ServoP error parsing, PING/PONG, health monitor, diagnostics lifecycle"
```

---

### Task 11: HapticCallback — Constraint Force Superposition

**Files:**
- Modify: `Touch_Client/haptic/HapticCallback.cpp`

**Interfaces:**
- Consumes: `SafetyPredictor::computeConstraintForce()`, `appState.forceData`

- [ ] **Step 1: Add constraint force to haptic rendering**

In `Touch_Client/haptic/HapticCallback.cpp`, at the top add:

```cpp
#include "../safety/ConstraintForce.h"
```

In Step 8 (force rendering, currently lines 76-88), replace the force rendering block with:

```cpp
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
```

- [ ] **Step 2: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/haptic/HapticCallback.cpp
git commit -m "feat(haptic): superpose constraint force on sensor force with per-axis 3.3N clamp"
```

---

### Task 12: HudOverlay — State + Error + Diagnostics Display

**Files:**
- Modify: `Touch_Client/render/HudOverlay.cpp` (and `HudOverlay.h` if needed)

**Interfaces:**
- Consumes: `RelayCore::stateMachine()`, `RobotDiagnostics::instance()`, `SafetyPredictor::lastVerdict()`

- [ ] **Step 1: Update top bar — replace bool status with state machine**

In `Touch_Client/render/HudOverlay.cpp`, modify `drawTopBar()` (lines 50-96). Replace the robot status section (lines 89-95):

```cpp
    // 右侧: 状态机状态 + 延迟
    auto& relay = RelayCore::instance();
    const auto& sm = relay.stateMachine();
    int rx = w - 280;

    // 状态标签 (颜色根据状态)
    const char* stateStr = sm.stateStr();
    RobotState st = sm.currentState();
    switch (st) {
        case RobotState::RUNNING:
        case RobotState::READY:
            glColor3f(0.30f, 0.90f, 0.40f); break;  // green
        case RobotState::DEGRADED:
            glColor3f(1.00f, 0.60f, 0.15f); break;  // orange
        case RobotState::ALARM:
        case RobotState::FATAL:
            glColor3f(1.00f, 0.30f, 0.30f); break;  // red
        case RobotState::RECOVERING:
            glColor3f(1.00f, 0.85f, 0.20f); break;  // yellow
        default:
            glColor3f(0.50f, 0.55f, 0.60f); break;  // gray
    }
    snprintf(buf, sizeof(buf), "[%s]", stateStr);
    text2D(rx, 18, buf);

    // 延迟
    rx += 120;
    snprintf(buf, sizeof(buf), "RTT: %.1f ms", app.latencyMs);
    glColor3f(0.60f, 0.70f, 0.80f);
    text2D(rx, 18, buf);

    // Speed factor
    rx += 100;
    snprintf(buf, sizeof(buf), "Spd: %.1fx", sm.speedFactor());
    if (sm.speedFactor() < 0.5f)
        glColor3f(1.0f, 0.5f, 0.2f);
    else
        glColor3f(0.60f, 0.70f, 0.80f);
    text2D(rx, 18, buf);
```

- [ ] **Step 2: Update coord panel — add latest error display**

In `drawCoordPanel()` (around line 240), after the existing robot state display, add a section at the bottom:

```cpp
    // 最新诊断
    const auto& verdict = SafetyPredictor::instance().lastVerdict();
    if (verdict.errorCode != RobotErrorCode::OK) {
        ty -= lineH * 2;
        drawSeparatorLine(x + 4, x + w - 4, ty + lineH);
        glColor3f(1.0f, 0.85f, 0.25f);
        text2D(x + 6, ty, "Latest Warning:", GLUT_BITMAP_8_BY_13);
        ty -= lineH;
        glColor3f(1.0f, 0.60f, 0.20f);
        text2D(x + 6, ty, verdict.reason ? verdict.reason : errorCodeName(verdict.errorCode),
            GLUT_BITMAP_8_BY_13);
    }
```

- [ ] **Step 3: Add diagnostics panel (last 5 events)**

Add a new function to HudOverlay.cpp:

```cpp
// ===== 右栏底部：诊断事件 =====
static void drawDiagnosticsPanel(int x, int y, int w, int h) {
    drawPanelBg(x, y, w, h);
    drawPanelTitle(x, y + h, w, "Diagnostics");

    auto& diag = RobotDiagnostics::instance();
    int lineH = 13;
    int ty = y + h - 28;
    int maxLines = (h - 30) / lineH;
    if (maxLines > 5) maxLines = 5;

    int idx = diag.writeIndex();
    int total = diag.historySize();
    
    for (int i = 0; i < maxLines && i < total; i++) {
        int pos = (idx - 1 - i + RobotDiagnostics::HISTORY_SIZE)
                  % RobotDiagnostics::HISTORY_SIZE;
        const auto& e = diag.history()[pos];
        
        if (e.error == RobotErrorCode::OK) continue;

        char buf[128];
        snprintf(buf, sizeof(buf), "%s speed=%.1f",
            errorCodeName(e.error), e.speedFactor);
        
        Severity sev = getSeverity(e.error);
        if (sev == Severity::FATAL || sev == Severity::REJECT)
            glColor3f(1.0f, 0.35f, 0.35f);
        else if (sev == Severity::DEGRADE)
            glColor3f(1.0f, 0.60f, 0.15f);
        else
            glColor3f(0.90f, 0.85f, 0.30f);
        
        text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
        ty -= lineH;
    }

    if (total == 0) {
        glColor3f(0.35f, 0.38f, 0.42f);
        text2D(x + 6, ty, "(no errors)");
    }
}
```

- [ ] **Step 4: Register the diagnostics panel in drawAll()**

Find `drawAll()` in HudOverlay.cpp (near the end of the file). The existing panels use the layout from HudLayout. The diagnostics panel should share space or replace an existing panel. Add it as a thin strip below the coord panel or as a replacement for one of the lower-half panels.

For now, add it to the right column bottom area (sharing space with the existing coord panel). In `drawAll()`:

```cpp
    // Right column bottom: diagnostics (shares bottom portion of coord area)
    int diagH = 80;
    int diagY = HudLayout::RIGHT_BOTTOM_Y;
    drawDiagnosticsPanel(HudLayout::RIGHT_X, diagY, HudLayout::RIGHT_W, diagH);
```

And reduce the coord panel's height to make room:

```cpp
    int coordH = HudLayout::RIGHT_BOTTOM_H - diagH - 2;
    int coordY = diagY + diagH + 2;
    // Update drawCoordPanel call with new y/h
```

Actually, to minimize the change footprint, just append the diagnostics panel between the coord panel and the bottom edge:

Find the existing `drawCoordPanel` call in `drawAll()` and add after it:

```cpp
    // Diagnostics strip below coord panel
    int diagH = 70;
    drawDiagnosticsPanel(HudLayout::RIGHT_X,
        HudLayout::RIGHT_BOTTOM_Y,
        HudLayout::RIGHT_W, diagH);
```

Add includes at top of HudOverlay.cpp:
```cpp
#include "../safety/RobotDiagnostics.h"
#include "../safety/RobotError.h"
#include "../relay/RelayCore.h"
```

- [ ] **Step 5: Verify build**

```bash
cd Touch_Client && build.bat
```

Expected: compiles clean.

- [ ] **Step 6: Commit**

```bash
git add Touch_Client/render/HudOverlay.cpp
git commit -m "feat(hud): display robot state, latency, speed factor, latest error, and diagnostics panel"
```

---

### Task 13: Final Integration Build + Smoke Test

**Files:**
- (all files already modified in Tasks 1-12)
- Reference: `Touch_Client/tests/build_test.bat` (existing test batch file)

- [ ] **Step 1: Full clean build**

```bash
cd Touch_Client && build.bat
```

Expected: zero errors. Minor warnings acceptable.

- [ ] **Step 2: Run ConstraintForce unit tests**

```bash
cd Touch_Client\tests && build_test.bat
```

If `build_test.bat` doesn't cover the new test, build manually:

```bash
cd Touch_Client\tests
cl /EHsc /std:c++17 test_constraint_force.cpp ../safety/ConstraintForce.cpp ../safety/SafetyPredictor.cpp ../robot/Kinematics.cpp /Fe:test_constraint_force.exe /I../
./test_constraint_force.exe
```

Expected: 7/7 tests pass.

- [ ] **Step 3: Run --no-robot smoke test**

```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe --no-robot
```

Expected:
- Window opens normally
- Top bar shows "[DISCONNECTED]" and "Spd: 1.0x"
- Force panels show zeros
- No crash on exit (q/ESC)
- Check: `robot_diagnostics.log` file created with SESSION START/END entries

- [ ] **Step 4: Run --no-touch smoke test**

```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe --no-touch --no-robot
```

Expected: same as above, no Touch dependency.

- [ ] **Step 5: Commit final integration**

```bash
git add -A
git commit -m "feat(safety): complete robot error handling system — all 13 tasks integrated"
```

---

## Integration Verification

After all 13 tasks, verify end-to-end:

### With Touch device (no robot needed):
```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe --no-robot
```
- Touch stylus feels constraint force when moved toward virtual safety boundary
- HUD top bar shows DISCONNECTED state with green/orange/red color
- Diagnostics panel at bottom-right shows "no errors" initially
- `robot_diagnostics.log` exists with SESSION START → END
- Moving stylus rapidly toward boundary → constraint force increases progressively

### With robot:
```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe
```
- State transitions: DISCONNECTED → CONNECTED → READY
- Button 1 press → RUNNING
- Push toward safety boundary → constraint force + speed reduction
- Continuous push toward danger → RUNNING → DEGRADED → READY (escalation)
- Pull back → immediate DEGRADED → RUNNING
- Robot alarm (mode=9) → ALARM → escape sequence → READY or FATAL
- Disconnect ethernet → RECOVERING → auto-reconnect or FATAL
- All state changes logged to `robot_diagnostics.log`
- HUD shows RTT latency, speed factor, state, latest error

### Without robot:
```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe --no-robot
```
- Program starts normally
- Constraint force renders on Touch (if connected)
- Diagnostics panel shows "no errors"
- HUD shows DISCONNECTED state
- No crash

---

## File Dependency Order

```
Task 1: Config.h              (no deps)
Task 2: RobotError.h          (uses CoordinateTransform.h only)
Task 3: EscalationTracker.h   (uses RobotError.h)
Task 4: RobotStateMachine     (uses RobotError.h, EscalationTracker.h)
Task 5: RobotDiagnostics      (uses RobotError.h, RobotStateMachine.h)
Task 6: ConstraintForce       (uses Config.h, CoordinateTransform.h, SafetyPredictor.h for AlarmRecord)
Task 7: test_constraint_force (uses ConstraintForce, SafetyPredictor)
Task 8: SafetyPredictor       (uses RobotError.h, EscalationTracker.h, ConstraintForce.h)
Task 9: FeedbackParser        (uses RobotError.h)
Task 10: RelayCore            (uses RobotStateMachine, RobotDiagnostics, FeedbackParser)
Task 11: HapticCallback       (uses ConstraintForce, SafetyPredictor)
Task 12: HudOverlay           (uses RobotStateMachine, RobotDiagnostics, SafetyPredictor)
Task 13: Integration          (all tasks)
```
