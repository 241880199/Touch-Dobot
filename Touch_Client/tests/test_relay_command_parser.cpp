// Standalone test: RelayCommandParser — MATLAB 反向命令解析
// Build: build_relay_command_test.bat
// Run: test_relay_command_parser.exe

#include <iostream>
#include <cmath>        // fabs —— 别靠 <iostream> 的传递包含 (本项目同目录其余用例都显式写这一行)
#include "../relay/RelayCommandParser.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

using R = RelayCommandParser::Command;

static void test_ff_on() {
    TEST(ff_on);
    CHECK(RelayCommandParser::parse("FF|1") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_off() {
    TEST(ff_off);
    CHECK(RelayCommandParser::parse("FF|0") == R::ForceFeedbackOff);
    PASS();
}

static void test_ff_on_with_newline() {
    TEST(ff_on_with_newline);
    CHECK(RelayCommandParser::parse("FF|1\n") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_on_with_crlf() {
    TEST(ff_on_with_crlf);
    CHECK(RelayCommandParser::parse("FF|1\r\n") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_leading_whitespace() {
    TEST(ff_leading_whitespace);
    CHECK(RelayCommandParser::parse("  FF|0") == R::ForceFeedbackOff);
    PASS();
}

static void test_ff_invalid_digit() {
    TEST(ff_invalid_digit);
    CHECK(RelayCommandParser::parse("FF|2") == R::None);
    PASS();
}

static void test_ff_trailing_garbage() {
    TEST(ff_trailing_garbage);
    CHECK(RelayCommandParser::parse("FF|10") == R::None);
    PASS();
}

static void test_ff_missing_value() {
    TEST(ff_missing_value);
    CHECK(RelayCommandParser::parse("FF|") == R::None);
    PASS();
}

static void test_empty() {
    TEST(empty);
    CHECK(RelayCommandParser::parse("") == R::None);
    PASS();
}

static void test_null() {
    TEST(null);
    CHECK(RelayCommandParser::parse(nullptr) == R::None);
    PASS();
}

static void test_unknown_command() {
    TEST(unknown_command);
    CHECK(RelayCommandParser::parse("P|1,2,3") == R::None);
    PASS();
}

// ===== 2026-09-22 新增: 增益与调零 =====

static void test_rg_valid() {
    TEST(rg_valid);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|200.5", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 200.5) < 1e-9);
    PASS();
}

static void test_rg_range_not_checked_here() {
    TEST(rg_range_not_checked_here);
    // 【解析器不管范围】—— 范围是 ForceTuning 的事 (单一定义)。解析器只负责"这是个合法的数"。
    // 这里钉住这条分工, 免得将来有人把增益范围 (ForceTuning::GAIN_MIN/GAIN_MAX) 塞进解析器、
    // 于是范围有了第二份实现 (数值也不抄进来, 抄了会过期 —— 只用符号名)。
    // 注意这条用例【不引用 ForceTuning.h】: 解析器与它无依赖, 引用反而把两者绑在一起。
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|5000", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 5000.0) < 1e-9);
    PASS();
}

static void test_rg_missing_value_out_param() {
    TEST(rg_missing_value_out_param);
    // 没给 out 参数也不能崩
    CHECK(RelayCommandParser::parse("RG|200") == R::SetReflectionGain);
    PASS();
}

static void test_rg_invalid() {
    TEST(rg_invalid);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|abc", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|200x", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|2 00", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|-", &v) == R::None);
    PASS();
}

static void test_zero_command() {
    TEST(zero_command);
    CHECK(RelayCommandParser::parse("Z|1") == R::ForceZero);
    CHECK(RelayCommandParser::parse("Z|1\n") == R::ForceZero);
    CHECK(RelayCommandParser::parse("Z|0") == R::None);   // 只有 1, 没有 0
    PASS();
}

static void test_rg_not_confused_with_ff() {
    TEST(rg_not_confused_with_ff);
    // FF| 与 RG| 是两条不同命令, 前缀不能互相吞
    CHECK(RelayCommandParser::parse("FF|1") == R::ForceFeedbackOn);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|1", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 1.0) < 1e-9);   // 是 1.0, 【不是】被当成 FF|1
    PASS();
}

int main() {
    std::cout << "=== RelayCommandParser Tests ===" << std::endl;
    test_ff_on();
    test_ff_off();
    test_ff_on_with_newline();
    test_ff_on_with_crlf();
    test_ff_leading_whitespace();
    test_ff_invalid_digit();
    test_ff_trailing_garbage();
    test_ff_missing_value();
    test_empty();
    test_null();
    test_unknown_command();
    test_rg_valid();
    test_rg_range_not_checked_here();
    test_rg_missing_value_out_param();
    test_rg_invalid();
    test_zero_command();
    test_rg_not_confused_with_ff();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
