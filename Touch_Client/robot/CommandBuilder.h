#pragma once
#include <string>
#include "../relay/CoordinateTransform.h"

namespace CommandBuilder {
    inline std::string buildServoP(const Vec3& pos, double rx, double ry, double rz) {
        char buf[256];
        snprintf(buf, sizeof(buf), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
            pos.x, pos.y, pos.z, rx, ry, rz);
        return std::string(buf);
    }

    inline std::string buildEnableRobot() {
        return "EnableRobot(0.5,0,0,0)";
    }

    inline std::string buildDisableRobot() {
        return "DisableRobot()";
    }

    inline std::string buildClearError() {
        return "ClearError()";
    }

    inline std::string buildRobotMode() {
        return "RobotMode()";
    }

    inline std::string buildGetPose() {
        return "GetPose()";
    }

    inline std::string buildCP(unsigned int ratio) {
        char buf[64];
        snprintf(buf, sizeof(buf), "CP(%u)", ratio);
        return std::string(buf);
    }
}
