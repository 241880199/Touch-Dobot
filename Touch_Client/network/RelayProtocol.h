#pragma once
#include <string>

namespace RelayProtocol {
    // 构造发往中继站的消息: "port|command"
    std::string buildMessage(int targetPort, const char* cmd);

    // 检查反馈是否成功 (ErrorID == 0)
    bool isSuccess(const char* feedback);

    // 从原始反馈中提取 {data} 部分
    // 反馈格式: "ErrorID,{data},CommandName();"
    bool extractData(const char* feedback, char* outData, int outLen);
}
