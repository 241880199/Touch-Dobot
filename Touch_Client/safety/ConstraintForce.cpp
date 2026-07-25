#include "ConstraintForce.h"
#include "../config/Config.h"
#include "../safety/SafetyPredictor.h"
#include <algorithm>
#include <cmath>

namespace ConstraintForce {

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
