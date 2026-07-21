#include "RelayProtocol.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>

namespace RelayProtocol {

std::string buildMessage(int targetPort, const char* cmd) {
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "%d|%s", targetPort, cmd);
    return std::string(buffer);
}

bool isSuccess(const char* feedback) {
    return feedback && feedback[0] == '0';
}

bool extractData(const char* feedback, char* outData, int outLen) {
    if (!feedback || !outData) return false;
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;
    int len = int(end - start - 1);
    if (len <= 0 || len >= outLen) return false;
    strncpy(outData, start + 1, len);
    outData[len] = '\0';
    return true;
}

} // namespace RelayProtocol
