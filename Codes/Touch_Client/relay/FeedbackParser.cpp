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

bool parseMode(const char* feedback, int& out) {
    char data[64];
    if (!extractData(feedback, data, sizeof(data))) return false;
    out = atoi(data);
    return true;
}

} // namespace FeedbackParser
