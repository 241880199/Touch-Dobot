# SafetyPredictor: 机械臂安全预判系统

**日期:** 2026-07-23
**分支:** master
**状态:** 设计中

## 1. 背景与目标

### 问题
1. 机械臂在开始运动时会立即跳跃到某个位置（根因：`pollFeedback()` 偷走 `queryPose()` 的 GetPose 响应，`robotActualPose` 永不过期）
2. 某些位置触发 CR3 报警（碰撞检测、关节限位），但没有记录机制
3. 缺乏运动学前置检查——ServoP 直接发送到 CR3，由 CR3 内部判断是否可达

### 目标
- **硬边界防护**: 工作半径 (620mm)、Z 轴范围、安全边界 —— 在 C++ 端拒绝发送 ServoP
- **运动学预判**: FK + 数值 IK + 雅可比奇异检测 —— 提前知道目标是否可达/危险
- **报警历史学习**: 记录每次 RobotMode==9 时的坐标，形成黑名单，后续靠近时减速

## 2. 架构

```
sendPosition(target)
  → SafetyPredictor::evaluate(target)
    → Layer 1: 硬边界检查 (REJECT)
        - 工作半径 > 620mm
        - Z < 0 或 Z > 795mm
        - SafetyBoundary 安全边界
    → Layer 2: 运动学检查
        - 数值 IK (DLS): 目标 Cartesian → 关节角
        - 关节限位: J1±360°, J2±360°, J3±155°, J4±360°, J5±360°, J6±360°
    → Layer 3: 奇异检测 (软边界)
        - 雅可比条件数 < 100: OK
        - 100~500: WARN + 降速至 30%
        - > 500: REJECT
    → Layer 4: 报警黑名单
        - 距离 < 30mm: WARN + 降速至 30%
        - 距离 < 80mm: WARN + 降速至 50%
  → 返回 SafetyVerdict { ALLOW, WARN_SLOW, REJECT }
```

## 3. 新增文件

### 3.1 `Touch_Client/robot/Kinematics.h`

```cpp
#pragma once
#include "../relay/CoordinateTransform.h"

namespace Kinematics {
    // URDF 参数 (mm)
    constexpr double J1_Z  = 128.3;
    constexpr double J3_X  = -274.0;
    constexpr double J4_X  = -230.0;
    constexpr double J4_Z  = 128.3;
    constexpr double J5_Y  = -116.0;
    constexpr double J6_Y  = 105.0;

    // 关节限位 (度)
    constexpr double J1_MIN = -360, J1_MAX = 360;
    constexpr double J2_MIN = -360, J2_MAX = 360;
    constexpr double J3_MIN = -155, J3_MAX = 155;
    constexpr double J4_MIN = -360, J4_MAX = 360;
    constexpr double J5_MIN = -360, J5_MAX = 360;
    constexpr double J6_MIN = -360, J6_MAX = 360;

    // FK: 关节角(度) → 末端 Cartesian 位置 (mm)
    Vec3 forwardPosition(const double joints[6]);

    // 数值 IK: 目标 → 关节角, DLS 方法
    bool inverse(const Vec3& target, const double seed[6], double out[6]);

    // 6x6 几何雅可比矩阵
    void jacobian(const double joints[6], double J[6][6]);

    // SVD 条件数
    double conditionNumber(double A[6][6]);

    // 关节限位检查
    bool isWithinJointLimits(const double joints[6]);

    // SVD 分解 (内部使用)
    bool svd(double A[6][6], double S[6], double U[6][6], double V[6][6]);
}
```

### 3.2 `Touch_Client/safety/SafetyPredictor.h`

```cpp
#pragma once
#include <vector>
#include <ctime>
#include "../relay/CoordinateTransform.h"
#include "../core/AppState.h"

struct AlarmRecord {
    double x, y, z;
    double j1, j2, j3, j4, j5, j6;
    time_t timestamp;
};

struct SafetyVerdict {
    enum Action { ALLOW, WARN_SLOW, REJECT };
    Action action;
    const char* reason;
    double speedFactor;  // 1.0 = 全速, 0.0 = 停止
};

class SafetyPredictor {
public:
    static SafetyPredictor& instance();

    // 主入口: 评估目标位姿是否安全
    SafetyVerdict evaluate(const Vec3& target);

    // 报警黑名单管理
    void addAlarmRecord(const AppState::RobotPose& pose);
    double nearestAlarmDistance(const Vec3& target) const;
    int alarmCount() const { return (int)m_alarmList.size(); }

    // 持久化
    void loadAlarmLog(const char* path);
    void saveAlarmLog(const char* path);

private:
    SafetyPredictor() = default;

    double m_lastJoints[6] = {0, 0, 0, 0, 0, 0};
    std::vector<AlarmRecord> m_alarmList;

    // 配置
    static constexpr double WORKSPACE_RADIUS   = 620.0;  // mm
    static constexpr double MAX_Z              = 795.0;  // mm
    static constexpr double SINGULARITY_WARN   = 100.0;  // 条件数
    static constexpr double SINGULARITY_REJECT = 500.0;
    static constexpr double ALARM_DANGER_R     = 30.0;   // mm
    static constexpr double ALARM_WARN_R       = 80.0;   // mm
    static constexpr double SINGULARITY_SPEED  = 0.3;
    static constexpr double ALARM_DANGER_SPEED = 0.3;
    static constexpr double ALARM_WARN_SPEED   = 0.5;
};
```

## 4. 修改文件

### 4.1 `RelayCore.cpp`

**sendPosition()**: 发送前调用 `SafetyPredictor::evaluate()`，根据 verdict 决定是否发送及速度衰减。

**checkAlarm()**: 检测到 mode==9 时立即 `queryPose()` 获取当前 Cartesian 坐标和关节角，调用 `SafetyPredictor::addAlarmRecord()` 记录。

### 4.2 `Config.h`

新增配置常量：
```cpp
constexpr double WORKSPACE_RADIUS        = 620.0;
constexpr double ROBOT_MAX_Z             = 795.0;
constexpr double SINGULARITY_COND_WARN   = 100.0;
constexpr double SINGULARITY_COND_REJECT = 500.0;
constexpr double ALARM_DANGER_RADIUS     = 30.0;
constexpr double ALARM_WARN_RADIUS       = 80.0;
```

### 4.3 `HudOverlay.cpp`

在 HUD 状态区显示：
- 报警历史计数
- 当前 SafetyVerdict 状态 (OK / WARN / REJECT)
- 警告原因文本

## 5. 算法细节

### 5.1 FK (对齐 MATLAB computeFK)

```
T = eye(4)
T = T * tr(0, 0, 128.3) * rotz(j1)
T = T * roty(π/2) * rotx(π/2) * rotz(j2)
T = T * tr(-274.0, 0, 0) * rotz(j3)
T = T * tr(-230.0, 0, 128.3) * rotz(-π/2) * rotz(j4)
T = T * tr(0, -116.0, 0) * rotx(π/2) * rotz(j5)
T = T * tr(0, 105.0, 0) * rotx(-π/2) * rotz(j6)
末端位置 = T(1:3, 4)
```

### 5.2 数值 IK (DLS / Levenberg-Marquardt)

```
Input: target_xyz (目标位置), seed_joints[6] (初始猜测)
Output: joints[6]

for i = 1 to 50:
    current_xyz = FK(joints)
    error = target_xyz - current_xyz
    if |error| < 0.1mm: converged, return true

    J = jacobian(joints)          // 6x6
    JtJ = J^T * J + λ * I        // 阻尼 λ = 0.1
    delta = J^T * error
    dq = solve(JtJ, delta)

    // 自适应阻尼
    if new_error < old_error: λ *= 0.5
    else: λ *= 2.0

    joints += dq
    clamp joints to limits

return false  // 未收敛
```

### 5.3 几何雅可比

```
对于 6-DOF 旋转关节机械臂:
J = [Jv; Jw]  (6x6)

对每个关节 i:
  z_i = 旋转轴 (世界坐标)
  p_i = 关节位置 (世界坐标, FK 已算出)
  p_ee = 末端位置

  Jv_i = z_i × (p_ee - p_i)   // 线速度分量
  Jw_i = z_i                    // 角速度分量
```

### 5.4 条件数 (SVD)

```
奇异值分解: J = U * diag(S) * V^T
条件数 = max(S) / min(S)

条件数 < 100:   正常
条件数 100~500: 接近奇异 (WARN + 减速)
条件数 > 500:   奇异构型 (REJECT)
```

## 6. 报警黑名单持久化

格式 (`alarms.log`, 每行一个报警):
```
2026-07-23 14:30:12, x=320.5, y=45.2, z=200.0, J1=10.5, J2=-30.2, J3=120.1, J4=5.0, J5=15.3, J6=-8.2
```

启动时加载，运行时追加。

## 7. 修改影响

| 文件 | 变更类型 | 说明 |
|------|----------|------|
| `robot/Kinematics.h` | 新增 | FK + IK + 雅可比 + 关节限位 |
| `robot/Kinematics.cpp` | 新增 | 实现 |
| `safety/SafetyPredictor.h` | 新增 | 安全预判入口 |
| `safety/SafetyPredictor.cpp` | 新增 | 分层校验 + 黑名单管理 |
| `relay/RelayCore.cpp` | 修改 | sendPosition 集成, checkAlarm 记录 |
| `config/Config.h` | 修改 | 新增安全常量 |
| `render/HudOverlay.cpp` | 修改 | 显示报警/安全状态 |
