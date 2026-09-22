#pragma once
#include "../config/Config.h"
#include "CoordinateTransform.h"
#include "../core/MathUtils.h"
#include <algorithm>
#include <iostream>

namespace SafetyBoundary {

// 钳位【本身】—— 纯函数, 不受任何开关影响。
// ⚠ 生产路径【不要直接调它】: 走下面的 clampToBoundaryActive。本函数留给用例 ——
//   语义不随现场开关变, 所以"边界那条规则本身还对不对"始终有覆盖。
inline Vec3 clampToBoundary(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_X_MIN) { clamped.x = Config::SAFE_X_MIN; warned = true; }
    if (target.x > Config::SAFE_X_MAX) { clamped.x = Config::SAFE_X_MAX; warned = true; }
    if (target.y < Config::SAFE_Y_MIN) { clamped.y = Config::SAFE_Y_MIN; warned = true; }
    if (target.y > Config::SAFE_Y_MAX) { clamped.y = Config::SAFE_Y_MAX; warned = true; }
    if (target.z < Config::SAFE_Z_MIN) { clamped.z = Config::SAFE_Z_MIN; warned = true; }
    if (target.z > Config::SAFE_Z_MAX) { clamped.z = Config::SAFE_Z_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] 目标超出安全边界，已钳位。原目标: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}

// ★★ 生产路径的【唯一入口】—— 受 Config::SAFETY_BOUNDARY_CLAMP_ENABLED 控制。
// 【为什么要有它, 而不是在每个调用点各写一句 if】这条边界被用在了【两个不同的角色】上, 分在
//   RelayCore 与 SafetyPredictor 两个文件、共 6 处:
//     · 钳位 (夹住目标) —— RelayCore 的 4 处;
//     · 谓词 (clamped ≠ target ⇒ REJECT) —— SafetyPredictor 的 2 处。
//   关卡放在【函数】上, 两种用法一起关; 放在调用点上, 就得记住 6 个地方 —— 而"改了一处忘一处"
//   正是本项目反复栽的那一类 (死区那份硬门、闸门掩码那份字面量, 都是这么来的)。
// 关闭时【原样返回】—— 依赖"钳位是否改变了我"的那两处谓词随之自动不再拒绝, 它们那边不需要
//   再判断一次开关。
// ⚠ 关于那两处谓词今天的实际作用, 见 Config::SAFETY_BOUNDARY_CLAMP_ENABLED 注释里标了
//   "实测核过"的那一段 —— **它们今天都是死的**。⚠ 但死的理由【两处不同, 别当成一条】:
//   · evaluate 那条 (RelayCore.cpp:874): 拿到的就是已夹过的值 ⇒ 幂等 ⇒ 【可证】恒假;
//   · evaluatePositionOnly 那条 (RelayCore.cpp:1143): 只因开关关着才死 —— 纯姿态模式下它拿到的
//     值会被 ≤5mm 的 tcpAdj【推到夹紧之后】⇒ 开关一翻回来它就会拒。
//   逐行依据见 SafetyPredictor.cpp 里那两段 (2026-09-22)。
//   仍然一起关: 那是为了开关的语义自洽, 不是因为"否则拦得住"。
inline Vec3 clampToBoundaryActive(const Vec3& target) {
    if (!Config::SAFETY_BOUNDARY_CLAMP_ENABLED) return target;
    return clampToBoundary(target);
}

// ⚠★ 2026-09-22 核过 (全仓 + .bat + .vcxproj): 本函数【没有任何生产调用点】,
//   唯一的使用者是 tests/test_coord_safety.cpp:265/280/293/302/316。
//   ⚠ 别与两个【活着】的同名概念混淆:
//     · RobotStateMachine::speedFactor()  (safety/RobotStateMachine.cpp:28,
//       被 RelayCore.cpp:857/:1404/:1910 消费) —— 生产在用;
//     · SafetyVerdict::speedFactor 字段 (safety/SafetyPredictor.h:22) —— 生产在用。
//   ⇒ 删或留由用户定; 现状是【留着 + 标出来】, 免得下一个人以为边界降速已经接上了。
inline double computeSpeedFactor(const Vec3& target) {
    // 计算距最近边界的距离（归一化到 [0, 1]）
    double rangeX = Config::SAFE_X_MAX - Config::SAFE_X_MIN;
    double rangeY = Config::SAFE_Y_MAX - Config::SAFE_Y_MIN;
    double rangeZ = Config::SAFE_Z_MAX - Config::SAFE_Z_MIN;

    double distX = std::min(target.x - Config::SAFE_X_MIN, Config::SAFE_X_MAX - target.x) / (rangeX * 0.5);
    double distY = std::min(target.y - Config::SAFE_Y_MIN, Config::SAFE_Y_MAX - target.y) / (rangeY * 0.5);
    double distZ = std::min(target.z - Config::SAFE_Z_MIN, Config::SAFE_Z_MAX - target.z) / (rangeZ * 0.5);
    double minDist = std::min({ distX, distY, distZ });

    if (minDist >= Config::SAFE_BOUNDARY_BUFFER_RATIO) return 1.0;
    if (minDist <= 0.0) return 0.1; // 已钳位到边界，最低 10% 速度
    return minDist / Config::SAFE_BOUNDARY_BUFFER_RATIO; // 线性衰减
}

} // namespace SafetyBoundary
