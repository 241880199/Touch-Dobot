#pragma once

// MATLAB → C++ 反向命令解析 (单行, 以 '|' 分隔)
// 协议:
//   "FF|0" 关力反馈, "FF|1" 开力反馈
//   "RG|<数值>" 设力反射增益 (范围由 ForceTuning 管, 【不在这里管】)
//   "Z|1"  力传感器调零 (与键盘 'z' 同语义: 再发一次 = 中止)
namespace RelayCommandParser {
    enum class Command {
        None,
        ForceFeedbackOn,
        ForceFeedbackOff,
        SetReflectionGain,
        ForceZero
    };

    // 解析一行命令。返回 Command::None 表示未知/空/非法输入。
    // valueOut: 仅 SetReflectionGain 时写入解析出的数值。
    //   【带默认值】—— 现有的 11 条测试一个字都不用改, 照旧编译通过并当回归网。
    Command parse(const char* line, double* valueOut = nullptr);
}
