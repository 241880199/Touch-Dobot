// CalibStore 单测: 路径推导 + 有效期判定。
// 不碰真实文件系统 —— 这两个都是纯函数。
#include "../core/CalibStore.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <ctime>
#include <windows.h>

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

// 造一个样本标定文件; savedAt == 0 表示【不写】saved_at_unix 字段。
static void writeFixture(const char* dir, const char* name, long savedAt) {
    char path[512];
    snprintf(path, sizeof(path), "%s%s", dir, name);
    FILE* f = fopen(path, "w");
    if (!f) return;
    if (savedAt == 0) fprintf(f, "{ \"mass_kg\": 1.0 }\n");
    else fprintf(f, "{ \"version\": 2, \"saved_at_unix\": %ld, \"mass_kg\": 1.0 }\n", savedAt);
    fclose(f);
}

static bool fixtureExists(const char* dir, const char* name) {
    char path[512];
    snprintf(path, sizeof(path), "%s%s", dir, name);
    FILE* f = fopen(path, "r");
    if (f) { fclose(f); return true; }
    return false;
}

// resolve() 的两条承重策略: 无时间戳即作废、超期作废, 且都要改名 .expired。
// (注: 本用例会打印模块自己的 [Calib] !! 提示行 —— 那是被测的用户可见行为, 属预期。)
static void test_resolve_policy() {
    TEST(resolve_policy);
    const char* d = "test_calibstore_tmp\\";
    CreateDirectoryA("test_calibstore_tmp", NULL);
    const long now = (long)time(NULL);

    // 新鲜: 带时间戳且在有效期内 -> 可用, 不改名
    writeFixture(d, "fresh.json", now);
    CHECK(CalibStore::resolveIn(d, "fresh.json") != nullptr);
    CHECK(fixtureExists(d, "fresh.json"));

    // 缺 saved_at_unix -> 作废并改名
    writeFixture(d, "nostamp.json", 0);
    CHECK(CalibStore::resolveIn(d, "nostamp.json") == nullptr);
    CHECK(!fixtureExists(d, "nostamp.json"));
    CHECK(fixtureExists(d, "nostamp.json.expired"));

    // 超期 -> 作废并改名
    writeFixture(d, "stale.json", now - Config::CALIB_MAX_AGE_SEC - 60);
    CHECK(CalibStore::resolveIn(d, "stale.json") == nullptr);
    CHECK(!fixtureExists(d, "stale.json"));
    CHECK(fixtureExists(d, "stale.json.expired"));

    // 不存在 -> 静默返回 nullptr (无提示)
    CHECK(CalibStore::resolveIn(d, "missing.json") == nullptr);

    // 顺手钉住被 Config.h 规定的值 (reviewer Minor #4)
    CHECK(Config::CALIB_MAX_AGE_SEC == 86400);

    remove("test_calibstore_tmp\\fresh.json");
    remove("test_calibstore_tmp\\nostamp.json.expired");
    remove("test_calibstore_tmp\\stale.json.expired");
    remove("test_calibstore_tmp");
    PASS();
}

int main() {
    std::printf("=== CalibStore Tests ===\n");
    test_derive_dir_normal();
    test_derive_dir_shallow();
    test_derive_dir_bad_input();
    test_is_fresh_boundaries();
    test_resolve_policy();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
