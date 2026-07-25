#include "FeedbackParser.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace FeedbackParser {

bool isSuccess(const char* feedback) {
    return feedback && feedback[0] == '0';
}

bool extractData(const char* feedback, char* out, int len) {
    if (!feedback || !out) return false;
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;
    int n = (int)(end - start - 1);
    if (n <= 0 || n >= len) return false;
    memcpy(out, start + 1, n);
    out[n] = '\0';
    return true;
}

bool parsePose(const char* feedback, AppState::RobotPose& out) {
    char data[256];
    if (!extractData(feedback, data, sizeof(data))) return false;
    return sscanf_s(data, "%lf,%lf,%lf,%lf,%lf,%lf",
        &out.x, &out.y, &out.z, &out.rx, &out.ry, &out.rz) == 6;
}

bool parseAngle(const char* feedback, double out[6]) {
    char data[256];
    if (!extractData(feedback, data, sizeof(data))) return false;
    return sscanf_s(data, "%lf,%lf,%lf,%lf,%lf,%lf",
        &out[0], &out[1], &out[2], &out[3], &out[4], &out[5]) == 6;
}

bool parseMode(const char* feedback, int& out) {
    char data[64];
    if (!extractData(feedback, data, sizeof(data))) return false;
    out = atoi(data);
    return true;
}

bool extractErrorCode(const char* feedback, int& out) {
    if (!feedback) return false;
    // Format: "-1,{0x0002},ServoP();"
    // or: "0,{},ServoP();" → success, no error
    if (feedback[0] == '0') {
        out = 0;
        return true;  // success response, error code 0
    }
    // Find "{...}" containing the error code
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;

    // Parse hex: "0x0002"
    const char* hex = start + 1;
    if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) {
        out = (int)strtol(hex + 2, nullptr, 16);
        return true;
    }
    // Try decimal
    out = atoi(hex);
    return true;
}

RobotErrorCode mapRobotErrorCode(int dobotCode) {
    switch (dobotCode) {
        case 0x0001: return RobotErrorCode::ERR_WORKSPACE_RADIUS;
        case 0x0002: return RobotErrorCode::ERR_JOINTLIMIT_EXCEED;
        case 0x0004: return RobotErrorCode::ERR_VELOCITY_CLAMP;
        case 0x0008: return RobotErrorCode::ERR_VELOCITY_CLAMP;
        case 0x0010: return RobotErrorCode::ERR_IK_SINGULAR;
        case 0x0020: return RobotErrorCode::ERR_COLLISION;
        default:     return RobotErrorCode::ERR_SERVOP_REJECTED;  // unknown → generic
    }
}

} // namespace FeedbackParser
