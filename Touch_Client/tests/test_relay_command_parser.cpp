// Standalone test: RelayCommandParser — MATLAB 反向命令解析
// Build: build_relay_command_test.bat
// Run: test_relay_command_parser.exe

#include <iostream>
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
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
