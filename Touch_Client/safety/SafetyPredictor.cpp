#include "SafetyPredictor.h"
#include "../robot/Kinematics.h"
#include "../relay/SafetyBoundary.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

// ===== static const definitions =====
const double SafetyPredictor::WORKSPACE_RADIUS    = 620.0;
const double SafetyPredictor::MAX_Z               = 795.0;
const double SafetyPredictor::SINGULARITY_WARN    = 100.0;
const double SafetyPredictor::SINGULARITY_REJECT  = 500.0;
const double SafetyPredictor::ALARM_DANGER_R      = 30.0;
const double SafetyPredictor::ALARM_WARN_R        = 80.0;
const double SafetyPredictor::SINGULARITY_SPEED   = 0.3;
const double SafetyPredictor::ALARM_DANGER_SPEED  = 0.3;
const double SafetyPredictor::ALARM_WARN_SPEED    = 0.5;

SafetyPredictor& SafetyPredictor::instance() {
    static SafetyPredictor inst;
    return inst;
}

// ===== main entry =====

SafetyVerdict SafetyPredictor::evaluate(const Vec3& target) {
    // ===== Layer 1: hard boundaries (O(1) compute) =====

    // 1a. workspace radius
    double dist = sqrt(target.x * target.x + target.y * target.y);
    if (dist > WORKSPACE_RADIUS) {
        m_lastVerdict = {SafetyVerdict::REJECT, "exceeds workspace radius (620mm)", 0.0};
        return m_lastVerdict;
    }

    // 1b. Z-axis range
    if (target.z < 0 || target.z > MAX_Z) {
        m_lastVerdict = {SafetyVerdict::REJECT, "Z-axis out of range (0~795mm)", 0.0};
        return m_lastVerdict;
    }

    // 1c. safety boundary (reuse existing SafetyBoundary)
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    if (clamped.x != target.x || clamped.y != target.y || clamped.z != target.z) {
        m_lastVerdict = {SafetyVerdict::REJECT, "exceeds safety boundary", 0.0};
        return m_lastVerdict;
    }

    // ===== 几何奇异区域预警 (不依赖 IK 模型) =====
    double r_xy = sqrt(target.x * target.x + target.y * target.y);
    if (r_xy < 30.0) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "cylindrical singularity: too close to Z axis (<30mm)", 0.3};
        return m_lastVerdict;
    }
    if (r_xy < 80.0) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "approaching cylindrical singularity (<80mm from Z axis)", 0.6};
        return m_lastVerdict;
    }

    // ===== Layer 2: kinematics check =====
    // 用机器人当前关节角作为 IK 种子 (比从零开始收敛快得多)
    double seed[6];
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        seed[0] = app.robotActualPose.j1;
        seed[1] = app.robotActualPose.j2;
        seed[2] = app.robotActualPose.j3;
        seed[3] = app.robotActualPose.j4;
        seed[4] = app.robotActualPose.j5;
        seed[5] = app.robotActualPose.j6;
        LeaveCriticalSection(&app.robotPoseMutex);
    }
    // 如果当前关节角全为 0 (未初始化), 回退到上次成功的解
    bool seedValid = false;
    for (int i = 0; i < 6; i++) {
        if (fabs(seed[i]) > 0.001) { seedValid = true; break; }
    }
    if (!seedValid) memcpy(seed, m_lastJoints, 6 * sizeof(double));

    // 关节限位预警 (距限位 <10° 时减速)
    static const double JLIM_WARN_DEG = 10.0;
    const double jlims[6][2] = {
        {-360, 360}, {-360, 360}, {-155, 155},
        {-360, 360}, {-360, 360}, {-360, 360}
    };
    const char* jnames[6] = {"J1","J2","J3","J4","J5","J6"};
    for (int i = 0; i < 6; i++) {
        double dLo = fabs(seed[i] - jlims[i][0]);
        double dHi = fabs(jlims[i][1] - seed[i]);
        double margin = (dLo < dHi) ? dLo : dHi;
        if (margin < JLIM_WARN_DEG) {
            static char jbuf[64];
            snprintf(jbuf, sizeof(jbuf), "%s near joint limit (%.1f deg)", jnames[i], margin);
            m_lastVerdict = {SafetyVerdict::WARN_SLOW, jbuf, 0.5};
            return m_lastVerdict;
        }
    }

    // 2a. numerical IK — URDF 模型与 Dobot 控制器坐标系不匹配, 降级为辅助判断
    double joints[6];
    bool converged = Kinematics::inverse(target, seed, joints);
    if (!converged) {
        memcpy(m_lastJoints, seed, 6 * sizeof(double));
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "IK no solution", 1.0};  // 全速, 不阻塞
        return m_lastVerdict;
    }

    // 2b. joint limits
    if (!Kinematics::isWithinJointLimits(joints)) {
        m_lastVerdict = {SafetyVerdict::REJECT, "joint outside limits", 0.0};
        return m_lastVerdict;
    }

    // ===== Layer 3: singularity detection (soft boundary) =====

    double J[6][6];
    Kinematics::jacobian(joints, J);
    double cond = Kinematics::conditionNumber(J);

    if (cond > SINGULARITY_REJECT) {
        m_lastVerdict = {SafetyVerdict::REJECT, "singular configuration (cond>>500)", 0.0};
        return m_lastVerdict;
    }

    if (cond > SINGULARITY_WARN) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "near singular region", SINGULARITY_SPEED};
        // still cache joints (slow down but don't block)
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }

    // ===== Layer 4: alarm history blacklist =====

    double minDist = nearestAlarmDistance(target);
    if (minDist < ALARM_DANGER_R) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "near historical alarm point (<30mm)", ALARM_DANGER_SPEED};
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }
    if (minDist < ALARM_WARN_R) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "near historical alarm zone (<80mm)", ALARM_WARN_SPEED};
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }

    // ===== pass =====
    memcpy(m_lastJoints, joints, 6 * sizeof(double));
    m_lastVerdict = {SafetyVerdict::ALLOW, nullptr, 1.0};
    return m_lastVerdict;
}

// ===== alarm blacklist =====

void SafetyPredictor::addAlarmRecord(const AppState::RobotPose& pose) {
    AlarmRecord rec;
    rec.x = pose.x;
    rec.y = pose.y;
    rec.z = pose.z;
    rec.j1 = pose.j1;
    rec.j2 = pose.j2;
    rec.j3 = pose.j3;
    rec.j4 = pose.j4;
    rec.j5 = pose.j5;
    rec.j6 = pose.j6;
    rec.timestamp = time(nullptr);
    m_alarmList.push_back(rec);

    std::cout << "[Safety] Alarm recorded at (" << rec.x << "," << rec.y << "," << rec.z
              << ") -- total alarms: " << m_alarmList.size() << std::endl;

    // auto-save
    saveAlarmLog("alarms.log");
}

double SafetyPredictor::nearestAlarmDistance(const Vec3& target) const {
    double minDist = 1e12;
    for (const auto& a : m_alarmList) {
        double dx = target.x - a.x;
        double dy = target.y - a.y;
        double dz = target.z - a.z;
        double d = sqrt(dx*dx + dy*dy + dz*dz);
        if (d < minDist) minDist = d;
    }
    return minDist;
}

// ===== persistence =====

void SafetyPredictor::loadAlarmLog(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") != 0 || !f) {
        std::cout << "[Safety] No existing alarm log at " << path << std::endl;
        return;
    }

    m_alarmList.clear();
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        AlarmRecord rec;
        // format: YYYY-MM-DD HH:MM:SS x=... y=... z=... J1=... ...
        char date[16], tim[16];
        if (sscanf_s(line, "%s %s x=%lf y=%lf z=%lf J1=%lf J2=%lf J3=%lf J4=%lf J5=%lf J6=%lf",
            date, (unsigned)sizeof(date), tim, (unsigned)sizeof(tim),
            &rec.x, &rec.y, &rec.z, &rec.j1, &rec.j2, &rec.j3, &rec.j4, &rec.j5, &rec.j6) == 11) {
            rec.timestamp = time(nullptr);
            m_alarmList.push_back(rec);
        }
    }
    fclose(f);
    std::cout << "[Safety] Loaded " << m_alarmList.size() << " alarm records from " << path << std::endl;
}

void SafetyPredictor::saveAlarmLog(const char* path) const {
    if (m_alarmList.empty()) return;

    FILE* f = nullptr;
    if (fopen_s(&f, path, "a") != 0 || !f) return;

    const auto& a = m_alarmList.back();  // append only the last record
    char timeBuf[32];
    struct tm tmInfo;
    localtime_s(&tmInfo, &a.timestamp);
    strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tmInfo);

    fprintf(f, "%s x=%.2f y=%.2f z=%.2f J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f\n",
        timeBuf, a.x, a.y, a.z, a.j1, a.j2, a.j3, a.j4, a.j5, a.j6);
    fclose(f);
}
