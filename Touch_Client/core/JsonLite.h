#pragma once
#include <cstring>

// 极简 JSON 取值助手 —— 【全程序唯一一份】。
// 从前 ForceCalibration.cpp 里有一份 static jsonFind; ForceTuning 又要一个同形状的,
// 于是抽到这里, 两边共用。别在别处再写第三份。
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
