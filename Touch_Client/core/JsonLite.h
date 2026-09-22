#pragma once
#include <cstring>

// 极简 JSON 取值助手 —— ForceCalibration 与 ForceTuning 共用的那一份。
// 从前 ForceCalibration.cpp 里有一份 static jsonFind; ForceTuning 又要一个同形状的,
// 于是抽到这里, 两边共用。**新的调用方一律用这里, 别再写一份。**
//
// ⚠ 尚未合并的一份 (2026-09-22 核过, 别以为下面这句已经不成立):
//   `force/PayloadCalibration.cpp:1830` 里还有一个【函数内 static】的逐字相同实现
//   (7 个调用点, 读 payload_calib.json)。**本次刻意不动它** ——
//   `test_payload_calibration` 【不在测试床里】(它有刻意留红的断言, 接进来会让整套永远红),
//   所以改它是【不可验证】的改动。要做就单开一个小任务, 并把那个套件先弄成可跑的。
//
// 语义: 找到 key (含引号, 如 "\"version\"") 之后的第一个非空字符, 跳过 ':'。
//   返回指向 '[' 或第一个数字/'-' 的指针 (供 strtod / 数组解析接着读);
//   找不到 key 时返回 nullptr。
// ⚠ 这不是一个合规的 JSON 解析器: 它不认转义、不认嵌套、不管 key 出现在字符串值里
//   的情况。只用于读【本程序自己写出来的】那种极简文件。
namespace JsonLite {
    inline const char* find(const char* buf, const char* key) {
        if (!buf || !key) return nullptr;
        const char* p = strstr(buf, key);
        if (!p) return nullptr;
        p += strlen(key);
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        if (*p == ':') p++;
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        return p;
    }
}
