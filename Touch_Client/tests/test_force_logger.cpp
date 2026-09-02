// Standalone test: ForceLogger — CSV 行格式化 (纯函数)
// Build: build_force_logger_test.bat
// Run: test_force_logger.exe

#include <iostream>
#include <cstring>
#include "../force/ForceLogger.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static void test_format_basic() {
    TEST(format_basic);
    char buf[256];
    double f[6] = {1.0, -2.0, 3.5, 0.1, 0.2, 0.3};
    double p[6] = {100.0, 200.0, 300.0, 1.0, 2.0, 3.0};
    int n = ForceLogger::formatLine(buf, sizeof(buf), 1234, f, p, 1);
    CHECK(n > 0);
    CHECK(strcmp(buf, "1234,1.000,-2.000,3.500,0.100,0.200,0.300,100.000,200.000,300.000,1.000,2.000,3.000,1") == 0);
    PASS();
}

static void test_format_ff_disabled() {
    TEST(format_ff_disabled);
    char buf[256];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    ForceLogger::formatLine(buf, sizeof(buf), 0, f, p, 0);
    // 最后一个字段 (ff_enabled) 为 0
    CHECK(strstr(buf, ",0") != nullptr);
    CHECK(buf[strlen(buf)-1] == '0');
    PASS();
}

static void test_format_negative() {
    TEST(format_negative);
    char buf[256];
    double f[6] = {-5.5, -6.25, -7.125, -0.5, -0.75, -1.0};
    double p[6] = {-1.0, -2.0, -3.0, -4.0, -5.0, -6.0};
    ForceLogger::formatLine(buf, sizeof(buf), 999, f, p, 1);
    CHECK(strstr(buf, "-5.500,-6.250,-7.125") != nullptr);
    PASS();
}

static void test_format_truncation_returns_needed() {
    TEST(format_truncation_returns_needed);
    char small[16];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    int needed = ForceLogger::formatLine(small, sizeof(small), 123456789, f, p, 1);
    CHECK(needed >= (int)sizeof(small));  // 返回所需总长度 >= 缓冲, 说明会截断
    PASS();
}

int main() {
    std::cout << "=== ForceLogger Tests ===" << std::endl;
    test_format_basic();
    test_format_ff_disabled();
    test_format_negative();
    test_format_truncation_returns_needed();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
