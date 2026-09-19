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
    // RUNTIME-GUARD — 本地补偿的运行时一致性闸门 (2026-09-19)。【两个码必须分开】:
    // 它们要做的事不同 —— 前者去按 'm'+'s' 重标模型, 后者去查负载参数有没有真的发进机械臂。
    // 合成一句话会让操作员在两个完全不同的动作之间乱猜。
    ERR_FORCE_UNCALIBRATED,     // 【没有可用模型】: 未标定 / A 全零 / A 数值退化 -> 拒绝传数据
    ERR_FORCE_INCONSISTENT,     // 有模型, 但与机械臂自报的 @576 逐通道对不上 -> 拒绝传数据

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

// 前向声明
inline const char* errorCodeName(RobotErrorCode code);

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
        // 一致性闸门: 语义就是 REJECT 的字面意思 ——「拒绝该帧运动」。
        // 【不选 FATAL】的理由 (两条, 都是实测/操作事实, 不是偏好):
        //   ① FATAL 会 DisableRobot, 而本闸门连续判决 —— 未标定/负载没发进去时它每一帧都拒,
        //      于是 FATAL 会在启动后 1 秒内把机械臂禁掉, Task 8 那个"发负载 -> 看闸门放行"的
        //      闭环就再也走不了 (闸门自己把要验证的那一步锁死)。
        //   ② 真正"不许往下传"的东西 (compensated) 已经无条件置零了, 与严重度无关 ——
        //      数据侧已经是 fail closed, FATAL 只会额外停掉【不依赖这份数据】的运动。
        // 若日后要改成 FATAL: 改这一行即可, 其余不用动。
        case RobotErrorCode::ERR_FORCE_UNCALIBRATED:
        case RobotErrorCode::ERR_FORCE_INCONSISTENT:
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
        case RobotErrorCode::ERR_FORCE_UNCALIBRATED:   return "ERR_FORCE_UNCALIBRATED";
        case RobotErrorCode::ERR_FORCE_INCONSISTENT:   return "ERR_FORCE_INCONSISTENT";
        default:                                    return "OK";
    }
}
