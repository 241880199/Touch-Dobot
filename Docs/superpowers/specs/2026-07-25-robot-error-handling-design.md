# Robot Error Handling System — Design Specification

**Date:** 2026-07-25
**Status:** Design Draft
**Branch:** master
**Context:** 机械臂错误预防与处理机制全面升级 — 统一错误分类、状态机、触觉虚拟约束力、运行时诊断

---

## 1. Overview

### 1.1 Goal

将当前分散的、不一致的机器人错误处理（SafetyPredictor 的 4 层检查 + escapeSingularity + 散落的 cout/cerr）统一为结构化系统：

- **预防层** — IK 不可靠的情况下的纯几何安全检查 + 触觉虚拟约束力
- **检测层** — 解析机器人 ServoP 错误反馈 + 连接健康监控
- **响应层** — 状态机驱动的分级响应（WARN→DEGRADE→REJECT→FATAL）
- **记录层** — 结构化诊断日志（环形缓冲区 + 文件持久化）

### 1.2 Design Principles

- **IK-independent safety** — 所有安全检查在 IK 失败时仍然有效
- **Progressive resistance** — 操作员通过 Touch 力反馈感知危险接近（虚拟约束力）
- **Error escalation** — 持续危险行为触发升级响应；反向运动立即解除
- **Degrade, don't crash** — 任何组件失败都不导致系统崩溃
- **Offline-testable** — 虚拟约束力、状态机、诊断日志均可无设备测试

---

## 2. Architecture

```
操作员 ←→ Touch力反馈 ←→ constraintForce(±3.3N) + sensorForce
                              ↑
Touch位置 → SafetyPredictor → StateMachine → ServoP → CR3
                ↑                  ↑              ↑
          EscalationTracker    ServoP 错误    Heartbeat/PING
                ↑                  ↑              ↑
            RobotDiagnostics (环形缓冲 + 文件日志)
```

### 2.1 Module Interactions

| Producer | Data | Consumer | Rate |
|----------|------|----------|------|
| SafetyPredictor::evaluate() | SafetyVerdict + constraintForce[3] | RelayCore::sendPosition() + hapticCallback | Every ServoP (30Hz) |
| RelayCore::pollFeedback() | RobotErrorCode | RobotStateMachine::onError() | Every display frame (~60Hz) |
| ForceReader thread | heartbeat (30004 packet) | HealthMonitor | 125Hz |
| RelayCore::queryPose() | PING/PONG RTT | HealthMonitor + HUD | Every 100ms |
| RobotStateMachine | RobotState + DiagnosticEvent | HudOverlay + RobotDiagnostics::log() | On state change |

---

## 3. Unified Error Taxonomy (RobotError)

### 3.1 ErrorCode Enum

```cpp
enum class RobotErrorCode {
    // ===== PRE-MOTION (运动前预判) =====
    ERR_WORKSPACE_RADIUS,       // 超出 620mm 工作半径        FATAL
    ERR_Z_RANGE,                // Z 超出 0~795               FATAL
    ERR_SAFETY_BOUNDARY,        // 超出用户安全边界            REJECT
    ERR_CYLINDRICAL_SING,       // Z轴距离 < 30mm             REJECT
    ERR_CYLINDRICAL_WARN,       // Z轴距离 < 80mm             WARN
    ERR_JOINTLIMIT_WARN,        // 关节距限位 < 10°           WARN
    ERR_JOINTLIMIT_EXCEED,      // 关节超出限位                REJECT
    ERR_IK_NO_SOLUTION,         // IK 50次迭代不收敛          DEGRADE
    ERR_IK_SINGULAR,            // 条件数 > 500               REJECT
    ERR_IK_NEAR_SINGULAR,       // 条件数 > 100               WARN
    ERR_ALARM_HISTORY,          // 接近历史报警点 (< 80mm)     WARN

    // ===== IN-MOTION (运动执行反馈) =====
    ERR_SERVOP_REJECTED,        // ServoP 被机器人拒绝         REJECT
    ERR_SERVOP_TIMEOUT,         // ServoP 响应超时             REJECT
    ERR_POSITION_DRIFT,         // 目标 vs 实际偏差 > 10mm    WARN
    ERR_VELOCITY_CLAMP,         // 速度被机器人钳位            WARN

    // ===== SYSTEM (连接/通信) =====
    ERR_CONNECTION_LOST,        // 以太网断开                  FATAL
    ERR_HEARTBEAT_LOST,         // 心跳超时 500ms             FATAL
    ERR_PROTOCOL_PARSE,         // 协议解析失败                WARN
    ERR_RESPONSE_INVALID,       // GetPose 返回异常值          WARN
    ERR_LATENCY_HIGH,           // RTT > 100ms                WARN

    // ===== ALARM (机器人主动报警) =====
    ERR_ALARM_MODE9,            // 机器人进入 mode=9           FATAL
    ERR_EMERGENCY_STOP,         // 急停被触发                  FATAL
    ERR_COLLISION,              // 碰撞检测触发                FATAL

    // ===== OK =====
    OK = -1                     // 无错误
};
```

### 3.2 Severity Levels

| Severity | Motion Effect | Force Effect | HUD Color | Auto-recover |
|----------|--------------|-------------|-----------|--------------|
| INFO | 无影响 | 无影响 | 白色 | N/A |
| WARN | speedFactor 0.7→0.1 (渐变) | constraintForce 激活 | 黄色 | 条件解除自动恢复 |
| DEGRADE | speedFactor 0.3 (固定) | constraintForce 增强 | 橙色 | 反向运动解除 |
| REJECT | 拒绝该帧运动，保持原位 | constraintForce 最大值 | 红色闪烁 | 下帧重新评估 |
| FATAL | 停止 ServoP + DisableRobot | 传感器力清零，约束力最大 | 红色常亮 | 需手动重启 |

### 3.3 Error Context

```cpp
struct RobotError {
    RobotErrorCode code;
    Severity severity;
    uint64_t timestampMs;
    
    // 快照
    Vec3 targetPosition;
    double currentJoints[6];
    double speedFactor;
    float latencyMs;
    
    // 统计
    int consecutiveCount;       // 连续触发次数
    
    const char* message();      // "IK no solution @ target=(120,-50,350) joints=(5,-30,45,0,60,0)"
};
```

---

## 4. Virtual Constraint Force

### 4.1 Overview

在 hapticCallback 中叠加虚拟力场，使操作员在接近危险区域时感受到渐进阻力。
约束力独立于机械臂连接——`--no-robot` 模式下也能渲染，可离线测试手感。

### 4.2 Force Fields

| Zone | Trigger | Direction | Magnitude | Priority |
|------|---------|-----------|-----------|----------|
| Safety boundary | dist < 50mm | 垂直边界向内推 | 0 → 2.0N (线性) | 1 (最高) |
| Cylindrical singular | r_xy < 80mm | 径向向外 | 0 → 2.5N (二次) | 2 |
| Alarm history | dist < 80mm | 远离报警点 | 0 → 1.5N (反比) | 3 |
| Workspace edge | dist_from_origin > 550mm | 向心方向 | 0 → 1.0N (线性) | 4 |

### 4.3 Force Curves

```
安全边界:
  2.0N ┤                          ╱
  1.0N ┤                        ╱
  0.0N ┼──────────────────────┴──→ 距边界 (mm)
        0   10   20   30   40   50

圆柱奇异:
  2.5N ┤                      ╱╱
  1.5N ┤                    ╱
  0.5N ┤                  ╱
  0.0N ┼────────────────┴────→ Z轴距离 (mm)
        0   20   40   60   80

报警历史点:
  1.5N ┤              ╲
  1.0N ┤                ╲
  0.5N ┤                    ╲
  0.0N ┼──────────────────────┴→ 距离 (mm)
        0   20   40   60   80

工作空间边缘:
  1.0N ┤                          ╱
  0.5N ┤                        ╱
  0.0N ┼──────────────────────┴──→ 距原点 (mm)
        550  570  590  610  620
```

### 4.4 Superposition

```cpp
void computeConstraintForce(const Vec3& target, double out[3]) {
    double f[4][3] = {0};
    f[0] = boundaryForce(target);      // 安全边界
    f[1] = singularForce(target);      // 圆柱奇异
    f[2] = alarmHistoryForce(target);  // 报警历史
    f[3] = workspaceEdgeForce(target); // 工作空间边缘
    
    // 按优先级叠加，总力 clamp 到 3.3N
    double total[3] = {0};
    for (int i = 0; i < 4; i++) {
        total[0] += f[i][0]; total[1] += f[i][1]; total[2] += f[i][2];
    }
    double mag = sqrt(total[0]*total[0] + total[1]*total[1] + total[2]*total[2]);
    if (mag > Config::FORCE_MAX_TOUCH_N) {
        double scale = Config::FORCE_MAX_TOUCH_N / mag;
        total[0] *= scale; total[1] *= scale; total[2] *= scale;
    }
    out[0] = total[0]; out[1] = total[1]; out[2] = total[2];
}
```

### 4.5 Integration with HapticCallback

```cpp
// Step 8: 力反馈渲染
double sensorForce[3] = {0};
if (!app.forceData.isStale) {
    sensorForce = app.forceData.hapticOut;
}

double constraintForce[3];
SafetyPredictor::instance().computeConstraintForce(robotPos, constraintForce);

double totalForce[3] = {
    clamp(sensorForce[0] + constraintForce[0], ±Config::FORCE_MAX_TOUCH_N),
    clamp(sensorForce[1] + constraintForce[1], ±Config::FORCE_MAX_TOUCH_N),
    clamp(sensorForce[2] + constraintForce[2], ±Config::FORCE_MAX_TOUCH_N),
};
hdSetDoublev(HD_CURRENT_FORCE, totalForce);
```

---

## 5. Robot State Machine

### 5.1 States

```
DISCONNECTED → CONNECTED → READY → RUNNING
                            ↑         ↓
                            ├─ DEGRADED ← (IK fail / singular warn / joint limit warn)
                            ├─ ALARM ← (mode=9 / collision / estop)
                            └─ RECOVERING ← (disconnect / heartbeat lost)
                                          ↓
                                       FATAL (recovery exhausted)
```

### 5.2 State Behaviors

| State | Motion | Touch Force | HUD | Timers Active |
|-------|--------|------------|-----|---------------|
| DISCONNECTED | None | 仅虚拟约束力 | 灰 "DISCONNECTED" | None |
| CONNECTED | None | 仅虚拟约束力 | 黄 "ENABLING..." | None |
| READY | 等待按钮按下 | 仅虚拟约束力 | 绿 "READY" | GetPose, GetAngle |
| RUNNING | ServoP 30Hz | 传感器力 + 约束力 | 绿 "RUNNING" | All |
| DEGRADED | ServoP 减速 (0.3x) | 传感器力 + 约束力↑ | 橙 + reason | All |
| ALARM | 停止 ServoP | 传感器力→0，约束力最大 | 红 + alarm code | Alarm check |
| RECOVERING | None | 仅虚拟约束力 | 黄 blink "RECONNECT" | Health check |
| FATAL | DisableRobot | 仅虚拟约束力 | 红 "FATAL — restart required" | None |

### 5.3 Transitions

| From | Trigger | To |
|------|---------|----|
| DISCONNECTED | robotConnect() succeeds | CONNECTED |
| CONNECTED | EnableRobot + GetPose succeed | READY |
| CONNECTED | EnableRobot fails | FATAL |
| READY | Button 1 pressed | RUNNING |
| RUNNING | WARN (<3 consecutive) | RUNNING (speedFactor reduced) |
| RUNNING | WARN (≥3 consecutive) | DEGRADED |
| RUNNING | REJECT | RUNNING (frame rejected, no state change) |
| RUNNING | ALARM detected | ALARM |
| RUNNING | Connection lost | RECOVERING |
| DEGRADED | Moving away from danger | RUNNING |
| DEGRADED | WARN (≥10 consecutive) | READY (stop, escalate to REJECT) |
| ALARM | escapeSingularity() succeeds | READY |
| ALARM | escapeSingularity() fails | FATAL |
| RECOVERING | Reconnect succeeds | READY |
| RECOVERING | 5 reconnect attempts fail | FATAL |
| Any | `DisableRobot()` called | DISCONNECTED |

---

## 6. Error Escalation Policy

### 6.1 Escalation Tracker

```cpp
struct EscalationTracker {
    RobotErrorCode currentCode;
    int consecutiveFrames = 0;
    Vec3 lastRejectDirection;   // direction that triggered rejection
    bool isEscalated = false;
    
    static const int WARN_TO_DEGRADE = 3;    // frames
    static const int DEGRADE_TO_REJECT = 10; // frames
};
```

### 6.2 Rules

1. **Same error consecutive** → counter++ → if ≥ WARN_TO_DEGRADE, escalate WARN→DEGRADE
2. **Different error** → counter reset to 0
3. **Reverse motion** (operator moves away from danger) → immediate de-escalation to RUNNING, all counters reset
4. **DEGRADE sustained** → after DEGRADE_TO_REJECT frames, escalate to REJECT (force stop, return to READY)
5. **Error cleared for 30 frames** → auto-de-escalate one level

### 6.3 Reverse Motion Detection

```cpp
bool isMovingAwayFromDanger(Vec3 currentDelta, Vec3 dangerDirection) {
    // currentDelta: 操作员当前移动方向
    // dangerDirection: 指向危险区域的方向 (如指向Z轴、指向边界外)
    double dot = currentDelta.x * dangerDirection.x 
               + currentDelta.y * dangerDirection.y 
               + currentDelta.z * dangerDirection.z;
    return dot < 0;  // 点积为负 = 远离危险方向
}
```

---

## 7. ServoP Error Feedback Parsing

### 7.1 Robot Error Response Format

```
成功: 0,{},ServoP(...);
失败: -1,{error_code},ServoP();
```

`error_code` 与 RobotErrorCode 的映射：

| Robot Error Code | RobotErrorCode |
|-----------------|----------------|
| 0x0001 | ERR_WORKSPACE_RADIUS |
| 0x0002 | ERR_JOINTLIMIT_EXCEED |
| 0x0004 | ERR_VELOCITY_CLAMP |
| 0x0008 | ERR_VELOCITY_CLAMP |
| 0x0010 | ERR_IK_SINGULAR |
| 0x0020 | ERR_COLLISION |

### 7.2 FeedbackParser Additions

```cpp
// New declarations in FeedbackParser.h
bool extractErrorCode(const char* feedback, int& out);
RobotErrorCode mapRobotErrorCode(int dobotsErrorCode);
```

### 7.3 RelayCore::pollFeedback() Integration

```cpp
void RelayCore::pollFeedback() {
    char buf[1024];
    while (robotRecvMotionPoll(buf, sizeof(buf))) {
        if (!FeedbackParser::isSuccess(buf)) {
            int robotErrCode = 0;
            FeedbackParser::extractErrorCode(buf, robotErrCode);
            RobotErrorCode err = FeedbackParser::mapRobotErrorCode(robotErrCode);
            
            RobotError error = {err, getSeverity(err), GetTickCount64(), ...};
            m_stateMachine.onError(error);
            RobotDiagnostics::log(error);
        }
        // ... existing logging ...
    }
}
```

---

## 8. Connection Health Monitoring

### 8.1 Heartbeat Sources

| Source | Rate | Priority | Fallback |
|--------|------|----------|----------|
| 30004 realtime packets | 125Hz (8ms) | Primary | — |
| GetPose response | 10Hz (100ms) | Fallback | 30004 unavailable |
| PING/PONG | On-demand | Explicit check | Manual trigger |

### 8.2 Heartbeat Logic

```cpp
class HealthMonitor {
    DWORD m_lastHeartbeatMs = 0;
    static const DWORD HEARTBEAT_TIMEOUT = 500;  // ms
    static const int RECONNECT_MAX_RETRIES = 5;
    int m_reconnectAttempts = 0;
    
    void onHeartbeat();                           // called on every 30004 recv() or GetPose response
    bool isHealthy() const;                       // now - m_lastHeartbeatMs < HEARTBEAT_TIMEOUT
    void checkAndRecover(RobotStateMachine& sm);  // called from GLUT timer
};
```

### 8.3 PING/PONG Protocol

```
C++ → CR3 (29999):  PING|<timestamp_us>\n
CR3 → C++ (29999):  PONG|<timestamp_us>\n

RTT = (now_us - echo_timestamp_us) / 1000.0f  // ms

暴露: appState.latencyMs (HUD 实时显示)
超时: 500ms 无 PONG → ERR_LATENCY_HIGH
```

---

## 9. Diagnostic Logging

### 9.1 DiagnosticEvent

```cpp
struct DiagnosticEvent {
    uint64_t timestampMs;
    RobotState fromState;
    RobotState toState;
    RobotErrorCode error;
    
    // Context snapshot
    Vec3 targetPosition;
    double jointAngles[6];
    float latencyMs;
    double speedFactor;
    double constraintForceMag;  // total constraint force magnitude
};
```

### 9.2 Ring Buffer + File

```cpp
class RobotDiagnostics {
    static const int HISTORY_SIZE = 200;
    DiagnosticEvent m_history[HISTORY_SIZE];
    int m_writeIdx = 0;
    int m_errorCounts[24] = {0};  // indexed by RobotErrorCode
    
    FILE* m_logFile = nullptr;
    
    void init(const char* path = "robot_diagnostics.log");
    void log(const DiagnosticEvent& e);      // ring buffer + file append
    void logStateChange(RobotState from, RobotState to);
    void writeSessionReport();               // called at shutdown
    void shutdown();
};
```

### 9.3 Log Format

```
[2026-07-25 14:32:01.234] === SESSION START ===
[2026-07-25 14:32:01.456] STATE: DISCONNECTED → CONNECTED
[2026-07-25 14:32:03.789] STATE: CONNECTED → READY
[2026-07-25 14:32:15.678] WARN: ERR_CYLINDRICAL_WARN target=(12,8,350) joints=(5,-30,45,0,60,0) speed=0.6 constraint=0.8N
[2026-07-25 14:32:16.712] ESCALATE: WARN→DEGRADE (3 consecutive) ERR_CYLINDRICAL_WARN
[2026-07-25 14:32:18.045] RECOVER: DEGRADE→RUNNING (operator moved away)
[2026-07-25 14:45:30.000] === SESSION END === errors: WARN=5 DEGRADE=1 REJECT=0 FATAL=0 ALARM=0
```

---

## 10. File Impact

### 10.1 New Files (~800 lines)

```
Touch_Client/safety/RobotError.h              ~120行  错误码枚举 + Severity + RobotError结构体
Touch_Client/safety/RobotStateMachine.h       ~100行  状态机类声明 + RobotState枚举
Touch_Client/safety/RobotStateMachine.cpp     ~180行  状态转换表 + onError() + autoRecover()
Touch_Client/safety/ConstraintForce.h         ~80行   虚拟约束力接口声明
Touch_Client/safety/ConstraintForce.cpp       ~150行  4种力场实现 + 叠加 + clamp
Touch_Client/safety/RobotDiagnostics.h        ~80行   DiagnosticEvent + RobotDiagnostics类声明
Touch_Client/safety/RobotDiagnostics.cpp      ~120行  环形缓冲 + 文件I/O + sessionReport
Touch_Client/safety/EscalationTracker.h       ~60行   升级计数器 + 反向运动检测
Touch_Client/tests/test_constraint_force.cpp  ~120行  虚拟力场单元测试 (力方向/大小/叠加/clamp)
```

### 10.2 Modified Files (~200 lines)

```
Touch_Client/safety/SafetyPredictor.h         +RobotError返回 +constraintForce[3] +EscalationTracker成员
Touch_Client/safety/SafetyPredictor.cpp       +RobotErrorCode映射 +约束力计算 +升级逻辑
Touch_Client/relay/RelayCore.h                +RobotStateMachine成员 +HealthMonitor成员
Touch_Client/relay/RelayCore.cpp              +pollFeedback解析ServoP错误 +PING/PONG +心跳检测
Touch_Client/relay/FeedbackParser.h           +extractErrorCode() +mapRobotErrorCode()
Touch_Client/relay/FeedbackParser.cpp         +extractErrorCode实现 +错误码映射表
Touch_Client/haptic/HapticCallback.cpp        +约束力叠加 (3行改动)
Touch_Client/render/HudOverlay.cpp            +RobotState显示 +延迟显示 +最新错误原因
Touch_Client/main.cpp                         +RobotDiagnostics::init/shutdown +PING/PONG定时器
```

### 10.3 Unchanged Files

```
Config.h, AppState.h/.cpp, CoordinateTransform.h, Kinematics.h/.cpp,
RobotConnection.h/.cpp, ForcePipeline.h/.cpp, SceneRenderer.cpp,
RobotModel.h/.cpp, StlLoader.h/.cpp, HapticDevice.h/.cpp,
relay_gui.m, relay_config.m
```

---

## 11. Test Strategy (Offline — No Robot Required)

### 11.1 Unit Tests

| Test | Input | Verify |
|------|-------|--------|
| ConstraintForce::boundaryForce | target 5mm from X_MAX | Force points -X, magnitude > 0 |
| ConstraintForce::boundaryForce | target at center | Force magnitude = 0 |
| ConstraintForce::singularForce | r_xy = 20mm | Force points radially outward |
| ConstraintForce::singularForce | r_xy = 200mm | Force magnitude = 0 |
| ConstraintForce::superposition | all 4 zones active | Total ≤ 3.3N |
| RobotStateMachine::transition | RUNNING + WARN×3 | → DEGRADED |
| RobotStateMachine::transition | DEGRADED + reverse motion | → RUNNING |
| RobotStateMachine::transition | CONNECTED + WARN | → CONNECTED (no motion state, WARN ignored) |
| EscalationTracker | 3 consecutive same error | isEscalated = true |
| EscalationTracker | 2 consecutive + 1 different | isEscalated = false |
| EscalationTracker | reverse motion during escalation | isEscalated = false |
| FeedbackParser::extractErrorCode | "-1,{0x0002},ServoP();" | errorCode = 0x0002 |
| FeedbackParser::mapRobotErrorCode | 0x0002 | ERR_JOINTLIMIT_EXCEED |

### 11.2 Offline Integration Test

```bash
cd Touch_Client\x64\Release && .\Touch_Client.exe --no-robot
```

- Touch 设备连接，移动到虚拟安全边界 → 感到虚拟约束力递增
- 移动到 Z 轴附近 → 感到径向排斥力
- `--no-touch` 模式：HUD 正确显示所有状态和错误信息
- 日志文件 `robot_diagnostics.log` 正确写入

---

## 12. Implementation Phases

| Phase | Scope | Priority | Device Needed |
|-------|-------|----------|---------------|
| P1 — Foundation | RobotError.h + RobotStateMachine + RobotDiagnostics | P0 | No |
| P2 — Constraint Force | ConstraintForce + HapticCallback integration + unit tests | P0 | Touch only |
| P3 — SafetyPredictor Upgrade | EscalationTracker + RobotErrorCode mapping + constraintForce output | P0 | No |
| P4 — Feedback Parsing | FeedbackParser error codes + pollFeedback integration | P1 | Robot* |
| P5 — Health Monitor | Heartbeat + PING/PONG + reconnect + HealthMonitor | P1 | Robot* |
| P6 — HUD Integration | State display + latency + latest error + diagnostics panel | P1 | No |

*P4/P5 的代码框架可以无设备编写和编译，但验证联调需要机器人。

---

## 13. Acceptance Criteria

1. 操作员推动 Touch 接近安全边界时，感觉到渐进阻力（虚拟约束力），方向指向安全区域
2. 接近圆柱奇异区（Z 轴 < 80mm）时，Touch 产生径向排斥力
3. 连续 3 帧触发同一 WARN → 状态进入 DEGRADED，速度衰减，HUD 显示橙色
4. 操作员反向移动 → 立即退出 DEGRADED，恢复 RUNNING
5. IK 失败时系统进入 DEGRADED 模式而不是静默放行
6. 机器人连接断开 → 状态进入 RECOVERING，自动重连
7. 严重错误时状态进入 FATAL，自动调用 DisableRobot
8. 所有状态转换写入 `robot_diagnostics.log`，带时间戳和上下文快照
9. HUD 实时显示: RobotState, latency (ms), speedFactor, 最新错误原因
10. `--no-robot` 模式下虚拟约束力正常工作，无崩溃
11. 现有 teleoperation 功能无回归
12. 所有单元测试通过
