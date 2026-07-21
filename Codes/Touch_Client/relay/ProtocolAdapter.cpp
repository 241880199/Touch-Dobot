#include "ProtocolAdapter.h"
#include "../config/Config.h"
#include <cstdio>

namespace ProtocolAdapter {

std::string buildServoP(const Vec3& pos, double rx, double ry, double rz) {
    char buf[256];
    snprintf(buf, sizeof(buf), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        pos.x, pos.y, pos.z, rx, ry, rz);
    return std::string(buf);
}

std::string buildEnableRobot()  { return "EnableRobot(0.5,0,0,0)"; }
std::string buildDisableRobot() { return "DisableRobot()"; }
std::string buildClearError()   { return "ClearError()"; }
std::string buildRobotMode()    { return "RobotMode()"; }
std::string buildGetPose()      { return "GetPose()"; }

std::string buildCP(unsigned int ratio) {
    char buf[64];
    snprintf(buf, sizeof(buf), "CP(%u)", ratio);
    return std::string(buf);
}

} // namespace ProtocolAdapter
