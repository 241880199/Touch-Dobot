// CalibStore 单测: 路径推导 + 有效期判定。
// 不碰真实文件系统 —— 这两个都是纯函数。
#include "../core/CalibStore.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>
#include <cmath>

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::printf("  " #name "... "); } while(0)
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL: %s\n", #cond); g_failed++; return; } } while(0)
#define PASS() do { std::printf("PASS\n"); g_passed++; } while(0)

// 正常布局: exe 在 <repo>\Touch_Client\x64\Release\ 下, 上溯两级 (x64\Release -> Touch_Client)
static void test_derive_dir_normal() {
    TEST(derive_dir_normal);
    char out[512];
    CHECK(CalibStore::deriveDir(
        "D:\\Projects\\Touch\\Touch_Client\\x64\\Release\\Touch_Client.exe", out, sizeof(out)));
    CHECK(strcmp(out, "D:\\Projects\\Touch\\Touch_Client\\calib\\") == 0);
    PASS();
}

// 只有一级: 不能再往上溯 (上溯不足时保持原样并仍以反斜杠结尾)
static void test_derive_dir_shallow() {
    TEST(derive_dir_shallow);
    char out[512];
    CHECK(CalibStore::deriveDir("C:\\a\\b.exe", out, sizeof(out)));
    CHECK(strcmp(out, "C:\\a\\calib\\") == 0);
    PASS();
}

// 反斜杠数量不足 -> 返回 false, 不越界
static void test_derive_dir_bad_input() {
    TEST(derive_dir_bad_input);
    char out[512];
    CHECK(!CalibStore::deriveDir("no_separators.exe", out, sizeof(out)));
    CHECK(!CalibStore::deriveDir(nullptr, out, sizeof(out)));
    PASS();
}

static void test_is_fresh_boundaries() {
    TEST(is_fresh_boundaries);
    const long DAY = 86400;
    CHECK( CalibStore::isFresh(1000, 1000 + DAY,      DAY));  // 刚好 24h -> 仍有效
    CHECK(!CalibStore::isFresh(1000, 1000 + DAY + 1,  DAY));  // 超 1s -> 过期
    CHECK( CalibStore::isFresh(1000, 1005,            DAY));
    CHECK(!CalibStore::isFresh(0,    1000,            DAY));  // 缺字段 -> 过期
    CHECK(!CalibStore::isFresh(-5,   1000,            DAY));  // 负值 -> 过期
    CHECK( CalibStore::isFresh(2000, 1000,            DAY));  // 时钟回拨 -> 当作刚保存
    PASS();
}

int main() {
    std::printf("=== CalibStore Tests ===\n");
    test_derive_dir_normal();
    test_derive_dir_shallow();
    test_derive_dir_bad_input();
    test_is_fresh_boundaries();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
