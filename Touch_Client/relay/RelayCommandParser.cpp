#include "RelayCommandParser.h"
#include <cstring>
#include <cstdlib>

namespace RelayCommandParser {

    // 跳过前导空白, 返回第一个非空白字符。
    static const char* skipWs(const char* p) {
        while (*p == ' ' || *p == '\t') p++;
        return p;
    }

    // 值之后只允许空白或行尾。
    static bool tailIsClean(const char* p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        return *p == '\0';
    }

    // "RG|<number>" -> 数值。整个 token 必须是一个数 (不允许 "200x" / "2 00")。
    static bool parseNumberToken(const char* p, double* out) {
        if (!p || *p == '\0') return false;
        char* end = nullptr;
        const double v = strtod(p, &end);
        if (end == p) return false;
        if (!tailIsClean(end)) return false;
        if (out) *out = v;
        return true;
    }

    Command parse(const char* line, double* valueOut) {
        if (line == nullptr) return Command::None;

        // 跳过前导空白
        while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
        if (*line == '\0') return Command::None;

        // ---- FF|<0|1> ----
        if (strncmp(line, "FF|", 3) == 0) {
            const char* p = skipWs(line + 3);
            if (*p != '0' && *p != '1') return Command::None;
            const char val = *p++;
            if (!tailIsClean(p)) return Command::None;
            return (val == '1') ? Command::ForceFeedbackOn : Command::ForceFeedbackOff;
        }

        // ---- RG|<数值> ----
        // ⚠ 【范围不在这里判】—— 增益范围只有一份定义, 在 ForceTuning::GAIN_MIN/GAIN_MAX。
        //   在这里再写一遍就是"同一个规则两份实现, 改一份忘一份"(本项目有成文教训)。
        //   ⇒ 所以这里连数值都不抄 (抄了就会跟着 ForceTuning.h 一起过期), 只留符号名。
        //   本函数只保证"这是一个合法的数", 越界由 ForceTuning::setGain 拒收。
        if (strncmp(line, "RG|", 3) == 0) {
            const char* p = skipWs(line + 3);
            if (!parseNumberToken(p, valueOut)) return Command::None;
            return Command::SetReflectionGain;
        }

        // ---- Z|1 ----
        // 语义与键盘 'z' 一致 (再发一次 = 中止), 而"多次"这件事由调用方的状态决定,
        // 所以协议本身就是一条 "Z|1" —— 不搞 0/1 的开关形式。
        if (strncmp(line, "Z|", 2) == 0) {
            const char* p = skipWs(line + 2);
            if (*p != '1') return Command::None;
            if (!tailIsClean(p + 1)) return Command::None;
            return Command::ForceZero;
        }

        return Command::None;
    }
}
