#include "RelayCommandParser.h"
#include <cstring>

namespace RelayCommandParser {
    Command parse(const char* line) {
        if (line == nullptr) return Command::None;

        // 跳过前导空白
        while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
        if (*line == '\0') return Command::None;

        if (strncmp(line, "FF|", 3) != 0) return Command::None;

        const char* p = line + 3;
        while (*p == ' ' || *p == '\t') p++;
        if (*p != '0' && *p != '1') return Command::None;

        char val = *p++;
        // 值后只允许空白或行尾
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p != '\0') return Command::None;

        return (val == '1') ? Command::ForceFeedbackOn : Command::ForceFeedbackOff;
    }
}
