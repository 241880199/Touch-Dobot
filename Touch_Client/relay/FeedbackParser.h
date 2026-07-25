#pragma once
#include "../core/AppState.h"
#include "../safety/RobotError.h"

namespace FeedbackParser {
    // 解析 GetPose() 返回: 0,{x,y,z,rx,ry,rz},GetPose();
    bool parsePose(const char* feedback, AppState::RobotPose& out);

    // 解析 GetAngle() 返回: 0,{J1,J2,J3,J4,J5,J6},GetAngle();
    bool parseAngle(const char* feedback, double out[6]);

    // 解析 RobotMode() 返回: 0,{mode},RobotMode();
    bool parseMode(const char* feedback, int& out);

    // 检查反馈是否成功 (ErrorID == 0)
    bool isSuccess(const char* feedback);

    // 提取 Dobot 错误码: "-1,{0x0002},ServoP();" → 0x0002
    bool extractErrorCode(const char* feedback, int& out);

    // 映射 Dobot 错误码 → RobotErrorCode
    RobotErrorCode mapRobotErrorCode(int dobotCode);

    // 提取 { } 中的 data 内容
    bool extractData(const char* feedback, char* out, int len);
}
