# Touch ↔ MATLAB GUI 通信协议规范 v3.0

## 概述

v3.0 架构变更：C++ Touch_Client 直连机械臂（双端口 TCP），MATLAB relay_gui 仅作为可视化前端。
C++ 端通过 localhost:8888 TCP 上报实时数据到 MATLAB GUI 显示。

## 连接参数

| 参数 | 值 |
|------|-----|
| 传输协议 | TCP |
| 服务端 | MATLAB relay_gui |
| IP | 127.0.0.1 (localhost) |
| 端口 | 8888 |
| 编码 | ASCII 文本，换行符分隔 |
| 帧率 | ~20 Hz (MATLAB timer 50ms) |

## C++ → MATLAB 数据上报协议

### 位置数据

```
P|x,y,z,rx,ry,rz
```

| 字段 | 含义 | 单位 |
|------|------|------|
| x, y, z | 末端执行器位置 | mm |
| rx, ry, rz | 末端执行器姿态 (RPY) | 度 |

示例: `P|300.00,0.00,200.00,0.00,45.00,0.00`

### 力反馈数据

```
F|fx,fy,fz
```

| 字段 | 含义 | 单位 |
|------|------|------|
| fx, fy, fz | Touch 设备三轴力 | N |

示例: `F|1.23,-0.45,0.02`

### 关节角度数据

```
J|j1,j2,j3,j4,j5,j6
```

| 字段 | 含义 | 单位 |
|------|------|------|
| j1 ~ j6 | 机械臂 6 轴关节角度 | 度 |

示例: `J|0.00,15.32,-22.10,0.00,45.00,0.00`

> C++ 端通过 `GetAngle()` 查询机械臂（200ms 间隔），经 `FeedbackParser::parseAngle()` 解析后上报。

### 机器人实际位姿

```
RP|x,y,z,rx,ry,rz
```

| 字段 | 含义 | 单位 |
|------|------|------|
| x, y, z | 机器人末端执行器当前位置 | mm |
| rx, ry, rz | 机器人末端执行器当前姿态 (RPY) | 度 |

示例: `RP|300.00,0.00,200.00,0.00,45.00,0.00`

> C++ 端通过 `GetPose()` 查询机械臂（100ms 间隔），经 `FeedbackParser::parsePose()` 解析后上报到 MATLAB 3D 显示。

### 时延探测

| 方向 | 格式 | 示例 |
|------|------|------|
| C++ → MATLAB | `PING\|{序号}` | `PING\|1234` |
| MATLAB → C++ | `PONG\|{序号}` | `PONG\|1234` |

### 异常通知

| 消息 | 含义 |
|------|------|
| `ROBOT_ARM_CLOSED` | 机械臂连接断开 |

## 机械臂直连协议（C++ ↔ CR3）

C++ Touch_Client 直连机械臂双端口：

| 端口 | 用途 | 典型指令 |
|------|------|---------|
| 29999 | Dashboard 控制 | EnableRobot, DisableRobot, ClearError, RobotMode, GetPose, GetAngle, CP |
| 30003 | 实时运动 | ServoP (笛卡尔空间动态跟随) |

机械臂反馈格式: `ErrorID,{data},CommandName();`
- ErrorID=0: 成功
- ErrorID≠0: 失败

C++ 端 `FeedbackParser` 负责解析机械臂原始反馈，`RelayCore` 协调数据流。
