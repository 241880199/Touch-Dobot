#include "PongHandler.h"
#include "../config/Config.h"
#include <cstring>
#include <cstdio>

namespace PongHandler {

bool isPing(const char* msg) {
    return msg && strncmp(msg, Config::PING_PREFIX, 5) == 0;
}

std::string buildPong(const char* pingMsg) {
    // pingMsg 格式: "PING|1234"
    const char* seq = pingMsg + 5; // 跳过 "PING|"
    char buf[128];
    snprintf(buf, sizeof(buf), "PONG|%s", seq);
    return std::string(buf);
}

} // namespace PongHandler
