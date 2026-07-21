#pragma once
#include <string>

// 机械臂指令（在发送前可被 Extension 修改）
struct RobotCommand {
    std::string cmd;
    int targetPort; // 29999 or 30003
};

// 机械臂原始反馈（在解析后可被 Extension 处理）
struct RobotFeedback {
    int errorId = -1;
    char data[512] = {}; // {data} 部分提取结果
    char raw[1024] = {}; // 完整原始字符串
    int fromPort = 0;    // 来自哪个端口
};

// 扩展插件接口 —— 预留力滤波、夹具控制、摄像头跟踪等后续模块
class IExtension {
public:
    virtual ~IExtension() = default;

    // 指令发送前的钩子：可修改或拦截指令
    virtual void onBeforeSend(RobotCommand& cmd) {}

    // 收到机械臂反馈后的钩子
    virtual void onAfterFeedback(const RobotFeedback& fb) {}
};
