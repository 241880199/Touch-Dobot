#pragma once

// MATLAB → C++ 反向命令解析 (单行, 以 '|' 分隔)
// 协议: "FF|0" 关力反馈, "FF|1" 开力反馈
namespace RelayCommandParser {
    enum class Command { None, ForceFeedbackOn, ForceFeedbackOff };

    // 解析一行命令。返回 Command::None 表示未知/空/非法输入。
    Command parse(const char* line);
}
