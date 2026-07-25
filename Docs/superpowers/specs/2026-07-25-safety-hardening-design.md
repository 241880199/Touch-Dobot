# Safety Hardening Design — 故障安全与鲁棒性增强

**Date:** 2026-07-25
**Status:** Approved
**Context:** 在 Robot Error Handling System (13 tasks, HEAD: 7613d0d) 和后审查修复 (10 findings, HEAD: e87da6f) 基础上，补齐故障安全缺口。

---

## 动机

当前安全架构在**正常运行时**完备——四层递进预判 + 四力场约束力 + 八状态机覆盖了遥操作的主要安全场景。但在**异常情况下**存在单点故障：

- 触觉线程崩溃 → 机器人停在最后一帧位置，不主动停机
- NaN 输入穿透 → 数值比较全部失效，安全门被绕过
- FATAL 状态 → 仅阻塞 ServoP，不发送 DisableRobot()
- 升级策略基于帧计数 → 1kHz 线程和 100ms 线程差 100 倍

本设计补齐这四类故障安全缺口，并为核心安全逻辑添加单元测试。

---

## 改动概览

| # | 模块 | 文件 | 代码量 |
|---|------|------|--------|
| 1 | NaN 输入防护 | `RelayCore.cpp`, `SafetyPredictor.cpp`, `AppState.h` | ~25 行 |
| 2 | 触觉线程看门狗 | `RelayCore.h/.cpp` | ~35 行 |
| 3 | FATAL→DisableRobot 回调 | `RobotStateMachine.h/.cpp`, `RelayCore.cpp` | ~20 行 |
| 4 | 时间基准升级策略 | `EscalationTracker.h`, `Config.h` | ~25 行 |
| 5 | 安全核心单元测试 | `tests/test_safety_core.cpp` (新) | ~150 行 |

---

## 第一节：NaN/Inf 输入防护

### 问题

`RelayCore::sendPosition()` 中 `dx/dy/dz` 从 Touch 设备增量计算得到，`SafetyPredictor::evaluate()` 直接用 `target` 计算 `sqrt()`。NaN 或 Inf 值会导致所有数值比较（`>` `<` `==`）返回 false，安全边界检查全部失效。

### 设计

**连续 NaN 计数器 + FATAL 升级：**

```
sendPosition() 入口:
  if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz) ||
      std::isinf(dx) || std::isinf(dy) || std::isinf(dz)) {
      m_nanFrameCount++;
      if (m_nanFrameCount >= 3) {
          // 连续 3 帧 NaN → 触发 FATAL
          RobotError error;
          error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
          error.severity = Severity::FATAL;
          error.timestampMs = GetTickCount64();
          Vec3 zeroDelta = {0, 0, 0};
          m_stateMachine.onError(error, zeroDelta);
      }
      return;  // 静默丢弃本帧
  }
  m_nanFrameCount = 0;  // 正常帧清零

SafetyPredictor::evaluate() 入口:
  if (std::isnan(target.x) || std::isnan(target.y) || std::isnan(target.z) ||
      std::isinf(target.x) || std::isinf(target.y) || std::isinf(target.z)) {
      m_lastVerdict.action = SafetyVerdict::REJECT;
      m_lastVerdict.errorCode = RobotErrorCode::ERR_EMERGENCY_STOP;
      m_lastVerdict.reason = "NaN/Inf target position";
      m_lastVerdict.speedFactor = 0.0;
      goto evaluate_done;  // 复用统一的 error building + constraint force
  }
```

### 设计理由

- **3 帧阈值**：与 EscalationTracker 的 3 帧 WARN→DEGRADE 模式一致，单次 USB 毛刺不误触发
- **正常帧清零**：不累计历史 NaN，只关心连续性
- **REJECT 而非直接 return**：走统一的 `evaluate_done` 路径，`m_lastError` 和 `constraintForce` 被正确构建，`sendPosition()` 能读到真实的错误信息
- **双重防护**：`sendPosition()` 截断 NaN 传播 + `evaluate()` 作为最后防线

### 配置

- NaN 连续帧阈值：硬编码 3（与 EscalationTracker 模式一致，不暴露为 Config 常量）

---

## 第二节：触觉线程看门狗

### 问题

如果 1kHz 触觉线程（HDAPI callback）崩溃或死锁，`sendPosition()` 不再被调用，没有 ServoP 发送，但机器人**保持使能**在最后一个位置。GLUT 主循环可能仍在渲染，系统假活。

### 设计

**双层看门狗：主检测在 GLUT idle()，兜底在 ForceReader 线程。**

### 第一层：GLUT idle() 心跳检测

```
RelayCore 新增成员:
  std::atomic<DWORD> m_lastHapticFrameMs{0};

sendPosition() 入口 (第一行):
  m_lastHapticFrameMs = GetTickCount();

idle() 回调 (main.cpp 或 RelayCore 新增方法):
  void RelayCore::checkHapticWatchdog() {
      if (!m_transmitting.load()) return;  // 没在运动时不检查
      DWORD now = GetTickCount();
      if (now - m_lastHapticFrameMs.load() > WATCHDOG_TIMEOUT_MS) {
          if (!m_watchdogTripped) {
              m_watchdogTripped = true;
              RobotError error;
              error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
              error.severity = Severity::FATAL;
              error.timestampMs = GetTickCount64();
              Vec3 zeroDelta = {0, 0, 0};
              m_stateMachine.onError(error, zeroDelta);
              // FATAL 回调会自动 DisableRobot（见第三节）
          }
      }
  }
```

### 第二层：ForceReader 线程兜底

ForceReader 是 `CreateThread` 创建的独立线程，不受 GLUT 影响。在 ForceReader 循环中加第二层检测：

```
forceReaderThread 循环内:
  static DWORD lastCheck = 0;
  DWORD now = GetTickCount();
  if (now - lastCheck > 300) {
      lastCheck = now;
      auto& relay = RelayCore::instance();
      if (relay.isTransmitting() &&
          now - relay.lastHapticFrameMs() > WATCHDOG_TIMEOUT_MS * 2) {
          // GLUT 可能已死，ForceReader 直接发送 DisableRobot
          robotSendEnable("DisableRobot()");
          Sleep(100);
      }
  }
```

### 配置常量

```
Config.h 新增:
  const int WATCHDOG_TIMEOUT_MS = 200;  // 触觉线程看门狗超时 (ms)
```

| 参数 | 值 | 理由 |
|------|-----|------|
| WATCHDOG_TIMEOUT_MS | 200 | ServoP 周期 33ms，200ms = 6 帧缺失。足够容忍 GLUT 瞬时卡顿 |
| ForceReader 兜底超时 | 400 (2x) | 给 GLUT 层充分时间反应，只在 GLUT 彻底死亡时接管 |
| ForceReader 检查间隔 | 300ms | 不频繁检查，避免影响 125Hz 数据读取性能 |

### 不检测的场景

- `m_transmitting == false`：操作员没按按钮时不检查。触觉线程可能仍在运行但不发送 ServoP，这不构成安全风险。

---

## 第三节：FATAL → DisableRobot 回调

### 问题

状态机进入 FATAL 后，`canMove()` 返回 false，后续 ServoP 被阻塞。但 `DisableRobot()` 不会自动发送——机器人保持使能状态，关节仍在伺服。

### 设计

**回调机制：状态机在 transitionTo(FATAL) 时触发注册的回调。**

```
RobotStateMachine 新增:
  using FatalCallback = void(*)();
  FatalCallback m_onFatal = nullptr;

public:
  void setFatalCallback(FatalCallback cb) { m_onFatal = cb; }

transitionTo() 中 (在 m_state = newState 之后):
  if (newState == RobotState::FATAL && m_onFatal) {
      // 在锁外调用，避免回调中的长时间操作阻塞状态机
      FatalCallback cb = m_onFatal;
      LeaveCriticalSection(&m_lock);
      cb();
      EnterCriticalSection(&m_lock);
  }
```

**RelayCore::init() 中注册：**
```cpp
m_stateMachine.setFatalCallback([]() {
    robotSendEnable("DisableRobot()");
    Sleep(100);
    std::cerr << "[Safety] FATAL: robot disabled by state machine" << std::endl;
});
```

### 设计理由

- **回调而非直接依赖**：`RobotStateMachine` 在 `safety/` 目录，不应引用 `relay/RobotConnection.h`。回调保持依赖方向正确
- **注册在 init()**：确保 `robotSendEnable` 函数指针在连接建立后才生效
- **锁外调用**：`DisableRobot()` 涉及网络 I/O，不能在 CRITICAL_SECTION 内执行。临时解锁 + 重新加锁的代价可接受
- **幂等安全**：`transitionTo()` 已有的 `if (m_state == newState) return;` 保证不会重复触发

---

## 第四节：时间基准升级策略

### 问题

`EscalationTracker` 仅用帧计数判断升级。不同调用频率下行为差异巨大：

| 线程 | 频率 | 3 帧 = | 10 帧 = |
|------|------|--------|---------|
| 触觉 (sendPosition) | ~30Hz | 100ms | 333ms |
| GLUT (queryPose) | 10Hz | 300ms | 1000ms |

### 设计

**帧计数 AND 时间阈值双重条件：**

```
EscalationTracker 改动:

  // 新增成员
  DWORD m_firstErrorMs = 0;

  // Config.h 新增
  const int MIN_WARN_MS    = 50;   // WARN 至少持续 50ms 才能升级
  const int MIN_DEGRADE_MS = 200;  // DEGRADE 至少持续 200ms 才能升级

  void recordError(RobotErrorCode code, const Vec3& delta) {
      if (code == currentCode) {
          consecutiveFrames++;
      } else {
          currentCode = code;
          consecutiveFrames = 1;
          m_firstErrorMs = GetTickCount();  // 新错误类型，重新计时
      }
      clearFrames = 0;
      lastRejectDirection = delta;
  }

  bool shouldEscalate() const {
      DWORD elapsed = GetTickCount() - m_firstErrorMs;
      Severity sev = getSeverity(currentCode);
      if (sev == Severity::WARN &&
          consecutiveFrames >= WARN_TO_DEGRADE &&
          elapsed >= Config::MIN_WARN_MS)
          return true;
      if (sev == Severity::DEGRADE &&
          consecutiveFrames >= DEGRADE_TO_REJECT &&
          elapsed >= Config::MIN_DEGRADE_MS)
          return true;
      return false;
  }

  void reset() {
      // ... existing resets ...
      m_firstErrorMs = 0;
  }
```

### 设计理由

- **AND 逻辑而非 OR**：两个条件都必须满足。帧计数保证"足够多次观察到错误"，时间保证"持续了足够长时间"
- **50ms WARN 阈值**：在 30Hz 下 ≈ 1.5 帧，在 1kHz 下 ≈ 50 帧。防止触觉线程单帧毛刺立即升级
- **200ms DEGRADE 阈值**：给操作员充足的反应时间
- **新错误类型重置计时器**：与现有 `consecutiveFrames = 1` 逻辑一致

---

## 第五节：安全核心单元测试

### 问题

现有 12 个测试覆盖 ConstraintForce (7) 和 ForcePipeline (5)，但状态机、升级策略、FATAL 回调——安全决策的核心逻辑——零测试覆盖。

### 设计

新文件 `Touch_Client/tests/test_safety_core.cpp`，8 个测试用例：

| # | 测试 | 输入 | 预期输出 |
|---|------|------|---------|
| 1 | 状态机标准转换链 | onConnect → onEnableSuccess → onButtonPress → 3x onError(WARN) → onRecovery | DISCONNECTED→CONNECTED→READY→RUNNING→DEGRADED→RUNNING |
| 2 | FATAL 回调触发 | transitionTo(FATAL) | 回调被调用 1 次 |
| 3 | canMove 各状态 | 遍历 8 个状态 | 仅 RUNNING/DEGRADED 返回 true |
| 4 | speedFactor 各状态 | 遍历关键状态 | RUNNING=1.0, DEGRADED=0.3, FATAL/ALARM/RECOVERING=0.0 |
| 5 | 升级：连续 WARN 3 帧 | 3x recordError(WARN, delta) | shouldEscalate() = true |
| 6 | 升级：不同错误重置 | recordError(ERR_A, ...) → recordError(ERR_B, ...) | consecutiveFrames = 1 |
| 7 | 降级：反向运动 | escalated=true, delta 与 dangerDir 点积 < 0 | shouldDeescalate() = true |
| 8 | 降级：30 帧清除 | 30x onClear() | escalation.reset() 被触发，isEscalated() = false |

### 不测试的内容

- IK/Jacobian 计算（数学逻辑，不属于安全模块）
- SafetyPredictor 边界值（需要 Dobot 硬件，留作集成测试）
- ConstraintForce/ForcePipeline（已有 12 个测试）
- NaN 处理（需要硬件注入，依赖运行时条件）

### 框架选择

沿用项目现有的测试模式（`test_constraint_force.cpp` 的风格：独立 `.exe`，`main()` 中 assert，返回 0 表示全部通过）。

---

## 影响分析

### 性能

| 改动 | 额外开销 | 频率 |
|------|---------|------|
| NaN 检查 (sendPosition) | 3x `std::isnan` + 3x `std::isinf` | 30Hz → 可忽略 |
| NaN 检查 (evaluate) | 同上 | 30Hz → 可忽略 |
| idle() 看门狗检查 | 1x `GetTickCount` + 1x `atomic::load` | 60Hz → 可忽略 |
| ForceReader 兜底检查 | 1x `GetTickCount` + 分支 | 每 300ms → 可忽略 |
| GetTickCount in recordError | 1x 系统调用 | 30Hz → 可忽略 |
| 时间阈值检查 (shouldEscalate) | 1x `GetTickCount` + 减法 | 30Hz → 可忽略 |

**总结：** 所有改动均为每帧 O(1) 轻量操作，零性能影响。

### 兼容性

- 无 API 变更
- 无配置格式变更
- `exit(0)` 问题（main.cpp:130）不在本次范围，属于已知遗留问题
- `m_alarmList` 线程安全问题（已知遗留）不在本次范围

---

## 风险

- **回调在锁外调用**：`transitionTo()` 临时解锁来调用 `m_onFatal`。如果回调重入（在回调内再次调用 transitionTo），状态可能不一致。缓解：`DisableRobot()` 不触发状态转换；文档要求回调不能操作状态机
- **看门狗在调试时误触发**：在断点调试时，200ms 超时会立即触发。缓解：仅在 `m_transmitting == true` 时检查，调试中一般不会按着按钮不动
- **测试覆盖不完整**：不测试 SafetyPredictor 的完整评估链。缓解：测试聚焦于纯逻辑状态转换，暂不需要硬件
