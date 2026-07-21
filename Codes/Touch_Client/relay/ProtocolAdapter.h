#pragma once
#include <string>
#include "CoordinateTransform.h"

namespace ProtocolAdapter {
    // 运动控制指令
    std::string buildServoP(const Vec3& pos, double rx, double ry, double rz);

    // Dashboard 指令
    std::string buildEnableRobot();
    std::string buildDisableRobot();
    std::string buildClearError();
    std::string buildRobotMode();
    std::string buildGetPose();
    std::string buildCP(unsigned int ratio);
}
