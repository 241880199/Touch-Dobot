#pragma once
#include <vector>
#include <ctime>
#include <windows.h>
#include "../relay/CoordinateTransform.h"
#include "../core/AppState.h"
#include "RobotError.h"
#include "EscalationTracker.h"
#include "ConstraintForce.h"

struct AlarmRecord {
    double x, y, z;
    double j1, j2, j3, j4, j5, j6;
    time_t timestamp;
};

struct SafetyVerdict {
    enum Action { ALLOW = 0, WARN_SLOW = 1, REJECT = 2 };
    Action action;
    RobotErrorCode errorCode;      // 具体错误码
    const char* reason;
    double speedFactor;            // 1.0 = 全速, 0.0 = 停止
    double constraintForce[3];     // Touch坐标系虚拟约束力

    SafetyVerdict() : action(ALLOW), errorCode(RobotErrorCode::OK),
        reason(nullptr), speedFactor(1.0)
    {
        constraintForce[0] = constraintForce[1] = constraintForce[2] = 0.0;
    }
};

class SafetyPredictor {
public:
    static SafetyPredictor& instance();

    // 主入口: 评估目标位姿是否安全 (每帧 ServoP 前调用)
    SafetyVerdict evaluate(const Vec3& target);

    // 报警黑名单管理
    void addAlarmRecord(const AppState::RobotPose& pose);
    double nearestAlarmDistance(const Vec3& target) const;
    int alarmCount() const {
        EnterCriticalSection(&m_lock);
        int n = (int)m_alarmList.size();
        LeaveCriticalSection(&m_lock);
        return n;
    }

    // 获取最近一次 verdict (给 HUD 显示)
    SafetyVerdict lastVerdict() const {
        EnterCriticalSection(&m_lock);
        SafetyVerdict v = m_lastVerdict;
        LeaveCriticalSection(&m_lock);
        return v;
    }

    // 计算虚拟约束力 (每次 ServoP 前调用)
    void computeConstraintForce(const Vec3& target, double out[3]);

    // 获取最近一次 RobotError
    RobotError lastError() const {
        EnterCriticalSection(&m_lock);
        RobotError e = m_lastError;
        LeaveCriticalSection(&m_lock);
        return e;
    }

    // 升级跟踪器
    EscalationTracker& escalation() { return m_escalation; }

    // 持久化
    void loadAlarmLog(const char* path);
    void saveAlarmLog(const char* path) const;

    // 获取 IK 种子 (上次成功求解的关节角)
    const double* lastJoints() const { return m_lastJoints; }

private:
    SafetyPredictor() {
        for (int i = 0; i < 6; i++) m_lastJoints[i] = 0.0;
        InitializeCriticalSection(&m_lock);
    }
    ~SafetyPredictor() { DeleteCriticalSection(&m_lock); }
    SafetyPredictor(const SafetyPredictor&) = delete;
    SafetyPredictor& operator=(const SafetyPredictor&) = delete;

    double m_lastJoints[6];
    std::vector<AlarmRecord> m_alarmList;
    mutable CRITICAL_SECTION m_lock;  // protects m_alarmList + m_lastVerdict + m_lastError
    SafetyVerdict m_lastVerdict;
    RobotError m_lastError;
    EscalationTracker m_escalation;

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
