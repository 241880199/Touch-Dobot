#pragma once
#include <cstdint>
#include <cstdio>
#include "../relay/CoordinateTransform.h"

// ===== 机器人错误码 (25种) =====
// ⚠ 这个数字必须与 RobotDiagnostics::ERROR_CODE_SLOTS 相等 (它决定计数槽位数, 槽位不够时
//   计数被静默丢掉)。加码时两个地方一起改 —— 本注释是 2026-09-19 闸门加两个码时同步的。
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
        // ⚠ 2026-09-19 复审更正: 这里【现在根本不消费严重度】, 所以"为什么选 REJECT 而不是
        //   FATAL"这个问题在本轮【没有实际后果】, 写在前一版里的理由也就站不住 ——
        //   事实是: 报错走的是 RobotDiagnostics::logError (RobotDiagnostics.cpp:94-110),
        //   它只做两件事 —— 写日志 + RelayCore::reportDiagnostic 发一条 D| 帧给 MATLAB;
        //   它【不调用】RobotStateMachine::onError。也就是说这里填 FATAL 同样不会
        //   DisableRobot, 填 REJECT 也不会"拒绝该帧运动" —— 两条路的【效果】完全一样。
        //   (前一版写的"FATAL 会在启动后 1 秒内把机械臂禁掉"因此是错的。)
        // 本轮的【真实效果】是: compensated 全 6 个分量无条件置零 (数据侧 fail closed,
        //   见 ForceCompensation::step) + 三条咨询性消息 (stderr / robot_diagnostics.log /
        //   D| 帧)。严重度【目前只是给日志读的一个标签】。
        // 保留 REJECT 的理由只剩一条, 而且很弱: 它的字面语义与"本帧数据不可用"最贴。
        // 若日后要让严重度真的生效 (接进 onError), 那时才需要重新论证 REJECT vs FATAL。
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
