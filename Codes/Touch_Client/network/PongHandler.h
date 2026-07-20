#pragma once
#include <string>

namespace PongHandler {
    bool isPing(const char* msg);
    std::string buildPong(const char* pingMsg);
}
