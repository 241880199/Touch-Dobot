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
