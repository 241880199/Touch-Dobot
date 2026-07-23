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
    enum Action { ALLOW = 0, WARN_SLOW = 1, REJECT = 2 };
    Action action;
    const char* reason;
    double speedFactor;  // 1.0 = 全速, 0.0 = 停止
};

class SafetyPredictor {
public:
    static SafetyPredictor& instance();

    // 主入口: 评估目标位姿是否安全 (每帧 ServoP 前调用)
    SafetyVerdict evaluate(const Vec3& target);

    // 报警黑名单管理
    void addAlarmRecord(const AppState::RobotPose& pose);
    double nearestAlarmDistance(const Vec3& target) const;
    int alarmCount() const { return (int)m_alarmList.size(); }

    // 获取最近一次 verdict (给 HUD 显示)
    SafetyVerdict lastVerdict() const { return m_lastVerdict; }

    // 持久化
    void loadAlarmLog(const char* path);
    void saveAlarmLog(const char* path) const;

    // 获取 IK 种子 (上次成功求解的关节角)
    const double* lastJoints() const { return m_lastJoints; }

private:
    SafetyPredictor() {
        for (int i = 0; i < 6; i++) m_lastJoints[i] = 0.0;
    }

    double m_lastJoints[6];
    std::vector<AlarmRecord> m_alarmList;
    SafetyVerdict m_lastVerdict = {SafetyVerdict::ALLOW, nullptr, 1.0};

    // 内部阈值
    static const double WORKSPACE_RADIUS;      // 620 mm
    static const double MAX_Z;                 // 795 mm
    static const double SINGULARITY_WARN;     // 100
    static const double SINGULARITY_REJECT;   // 500
    static const double ALARM_DANGER_R;       // 30 mm
    static const double ALARM_WARN_R;         // 80 mm
    static const double SINGULARITY_SPEED;    // 0.3
    static const double ALARM_DANGER_SPEED;   // 0.3
    static const double ALARM_WARN_SPEED;     // 0.5
};
