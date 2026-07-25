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

    static constexpr int WARN_TO_DEGRADE   = 3;  // from Config::ESCALATE_WARN_TO_DEGRADE
    static constexpr int DEGRADE_TO_REJECT = 10; // from Config::ESCALATE_DEGRADE_TO_REJECT
    static constexpr int CLEAR_FRAMES      = 30; // from Config::DEESCALATE_CLEAR_FRAMES

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
