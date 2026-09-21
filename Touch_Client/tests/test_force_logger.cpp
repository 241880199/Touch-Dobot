// Standalone test: ForceLogger — CSV 行格式化 (纯函数)
// Build: build_force_logger_test.bat
// Run: test_force_logger.exe
//
// ★ 2026-09-21: 列布局从 14 列扩到 18 列 (行末追加 acc_x,acc_y,acc_z,is_still)。
//   ⚠ 本次改动【悄悄改义】了旧用例里一处断言: 从前 `buf[strlen(buf)-1]` 是 ff_enabled,
//     现在是 is_still。断言没有报错、但它量的是【另一个字段】—— 那正是本项目最怕的
//     "安静地不一致"。本文件把那处改成显式检查【字段位置】, 并单列一个用例把两个末尾整数
//     分开验证 (否则改列序也测不出来)。

#include <iostream>
#include <cstring>
#include "../force/ForceLogger.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// 数一行里有几个逗号 ⇒ 列数 = 逗号数 + 1。用来把"列布局"本身钉住 ——
// 光测某个子串出现过, 测不出"少了一列/多了一列"。
static int countCommas(const char* s) {
    int n = 0;
    for (; *s; ++s) if (*s == ',') n++;
    return n;
}

static void test_format_basic() {
    TEST(format_basic);
    char buf[320];
    double f[6] = {1.0, -2.0, 3.5, 0.1, 0.2, 0.3};
    double p[6] = {100.0, 200.0, 300.0, 1.0, 2.0, 3.0};
    double a[3] = {0.25, -1.5, 3.0};
    int n = ForceLogger::formatLine(buf, sizeof(buf), 1234, f, p, 1, a, 0);
    CHECK(n > 0);
    CHECK(strcmp(buf,
        "1234,1.000,-2.000,3.500,0.100,0.200,0.300,100.000,200.000,300.000,1.000,2.000,3.000,1,"
        "0.2500,-1.5000,3.0000,0") == 0);
    CHECK(countCommas(buf) == 17);   // 18 列
    PASS();
}

static void test_ff_enabled_is_column_14_not_last() {
    TEST(ff_enabled_is_column_14_not_last);
    // ★ 这一条替掉旧版那个 `buf[strlen(buf)-1] == '0'`: 那个断言在列扩展之后
    //   量的是 is_still 而不是 ff_enabled。改为【显式定位第 14 个字段】。
    char buf[320];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    double a[3] = {0,0,0};
    ForceLogger::formatLine(buf, sizeof(buf), 0, f, p, 0, a, 1);   // ff=0, still=1
    // 第 14 个字段: 从第 13 个逗号之后开始
    const char* q = buf;
    for (int i = 0; i < 13; i++) { q = strchr(q, ','); CHECK(q != nullptr); q++; }
    CHECK(*q == '0');                 // ff_enabled = 0
    // 最后一个字段是 is_still
    CHECK(buf[strlen(buf) - 1] == '1');
    PASS();
}

static void test_is_still_matches_input() {
    TEST(is_still_matches_input);
    // 两个末尾整数必须【各自】对应入参 —— 否则改列序也测不出来。
    char bufA[320], bufB[320];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    double a[3] = {0,0,0};
    ForceLogger::formatLine(bufA, sizeof(bufA), 1, f, p, 1, a, 0);
    ForceLogger::formatLine(bufB, sizeof(bufB), 1, f, p, 0, a, 1);
    CHECK(strstr(bufA, ",1,0.0000,0.0000,0.0000,0") != nullptr);
    CHECK(strstr(bufB, ",0,0.0000,0.0000,0.0000,1") != nullptr);
    PASS();
}

static void test_acc_precision() {
    TEST(acc_precision);
    // acc 是 %.4f, 而力是 %.3f —— 精度不同是【有意的】: Fi 的误差尺度 ~0.5N ⇒ acc ~1.2m/s²,
    // 四位小数够分辨; 而力的 %.3f 是传感器自己的量化台阶。
    char buf[320];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    double a[3] = {1.23456, -5.67894, 0.0};
    ForceLogger::formatLine(buf, sizeof(buf), 7, f, p, 1, a, 0);
    CHECK(strstr(buf, ",1.2346,-5.6789,0.0000,0") != nullptr);
    PASS();
}

static void test_format_negative() {
    TEST(format_negative);
    char buf[320];
    double f[6] = {-5.5, -6.25, -7.125, -0.5, -0.75, -1.0};
    double p[6] = {-1.0, -2.0, -3.0, -4.0, -5.0, -6.0};
    double a[3] = {-0.5, 0.0, 0.5};
    ForceLogger::formatLine(buf, sizeof(buf), 999, f, p, 1, a, 0);
    CHECK(strstr(buf, "-5.500,-6.250,-7.125") != nullptr);
    PASS();
}

static void test_format_truncation_returns_needed() {
    TEST(format_truncation_returns_needed);
    char small[16];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    double a[3] = {0,0,0};
    int needed = ForceLogger::formatLine(small, sizeof(small), 123456789, f, p, 1, a, 0);
    CHECK(needed >= (int)sizeof(small));  // 返回所需总长度 >= 缓冲, 说明会截断
    PASS();
}

static void test_full_line_fits_default_buffer() {
    TEST(full_line_fits_default_buffer);
    // log() 里用的是 char buf[320] —— 钉住"最长的一行也装得下", 免得将来加列时静默截断。
    char buf[320];
    double f[6] = {-123.456, -123.456, -123.456, -123.456, -123.456, -123.456};
    double p[6] = {-123456.789, -123456.789, -123456.789, -179.999, -179.999, -179.999};
    double a[3] = {-123.4567, -123.4567, -123.4567};
    int needed = ForceLogger::formatLine(buf, sizeof(buf), 9999999999UL, f, p, 1, a, 1);
    CHECK(needed > 0);
    CHECK(needed < (int)sizeof(buf));     // 未被截断
    PASS();
}

int main() {
    std::cout << "=== ForceLogger Tests ===" << std::endl;
    test_format_basic();
    test_ff_enabled_is_column_14_not_last();
    test_is_still_matches_input();
    test_acc_precision();
    test_format_negative();
    test_format_truncation_returns_needed();
    test_full_line_fits_default_buffer();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
