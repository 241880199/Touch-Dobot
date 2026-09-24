// Standalone test: ForceTuning — MATLAB 可调增益的模块
// Build: build_force_tuning_test.bat
// Run:   test_force_tuning.exe
//
// !! 本测试【不调】ForceTuning::loadOnStartup() !!
//    它读的是 CalibStore 的真实路径 (calib/force_tuning.json), 会被【现场调参】污染 ——
//    昨天在 MATLAB 上拖到 300, 今天的测试就跟着变。测试必须只看静态初值 (Config 的 120)。
//    读写用例一律用【显式路径的临时文件】, 不用模块级的那条路径。

#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <windows.h>              // GetTickCount: tick() 那两条用例拿它当【基准时刻 t0】
#include "../force/ForceTuning.h"
#include "../config/Config.h"     // 用例要直接核 Config::FORCE_REFLECTION_GAIN

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static const char* kTmp = "_tuning_test_tmp.json";

// ===== 纯解析 =====

static void test_parse_ok() {
    TEST(parse_ok);
    double v = 0.0;
    CHECK(ForceTuning::parseGainJson("{\n \"version\": 1,\n \"reflection_gain\": 250.0\n}\n", &v));
    CHECK(fabs(v - 250.0) < 1e-9);
    PASS();
}

static void test_parse_wrong_version() {
    TEST(parse_wrong_version);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("{\n \"version\": 2,\n \"reflection_gain\": 250.0\n}\n", &v));
    PASS();
}

static void test_parse_missing_version() {
    TEST(parse_missing_version);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("{\n \"reflection_gain\": 250.0\n}\n", &v));
    PASS();
}

static void test_parse_out_of_range() {
    TEST(parse_out_of_range);
    double v = 0.0;
    // 低于下限
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":99.9}", &v));
    // 高于上限
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":300.1}", &v));
    PASS();
}

static void test_parse_nan_inf() {
    TEST(parse_nan_inf);
    double v = 0.0;
    // strtod 会把 "nan" / "inf" 都解析成功 ⇒ 挡住它们的责任在 parseGainJson 里。
    // ⚠ 本用例【锁的是行为, 不是 inRange 里那句 isfinite】: 实测把 isfinite 删掉, 本用例
    //   照样绿 (NaN 的任何比较都为假, ±inf 必有一侧越界 ⇒ 区间比较已经挡住了)。
    //   它真正防的是【把区间判断改写成 "!(v < GAIN_MIN || v > GAIN_MAX)" 那种形状】——
    //   那个改写会让 NaN 通过。所以留着它有价值, 但别以为它覆盖了 isfinite。
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":nan}", &v));
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":inf}", &v));
    PASS();
}

static void test_parse_garbage() {
    TEST(parse_garbage);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("not json at all", &v));
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":}", &v));
    CHECK(!ForceTuning::parseGainJson("", &v));
    CHECK(!ForceTuning::parseGainJson(nullptr, &v));
    PASS();
}

// ===== setGain 与范围 =====

static void test_set_gain_bounds() {
    TEST(set_gain_bounds);
    ForceTuning::setGain(250.0);
    CHECK(fabs(ForceTuning::gain() - 250.0) < 1e-9);

    CHECK(ForceTuning::setGain(ForceTuning::GAIN_MIN));      // 下限本身接受
    CHECK(fabs(ForceTuning::gain() - 100.0) < 1e-9);
    CHECK(ForceTuning::setGain(ForceTuning::GAIN_MAX));      // 上限本身接受
    CHECK(fabs(ForceTuning::gain() - 300.0) < 1e-9);

    ForceTuning::setGain(200.0);
    CHECK(!ForceTuning::setGain(99.9));                       // 拒收
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);          // 且【值不变】
    CHECK(!ForceTuning::setGain(300.1));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    CHECK(!ForceTuning::setGain(std::nan("")));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    CHECK(!ForceTuning::setGain(INFINITY));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    PASS();
}

static void test_static_initial_value_is_legal() {
    TEST(static_initial_value_is_legal);
    // 防默认值悄悄漂走。
    CHECK(fabs(ForceTuning::defaultGain() - Config::FORCE_REFLECTION_GAIN) < 1e-9);
    // 防"恢复默认"按钮发出一个被 setGain 自己拒收的值 —— 那在界面上表现为"按了没反应"。
    CHECK(ForceTuning::defaultGain() >= ForceTuning::GAIN_MIN);
    CHECK(ForceTuning::defaultGain() <= ForceTuning::GAIN_MAX);
    PASS();
}

// ===== 落盘 / 读回 (显式路径, 不碰现场文件) =====

static void test_save_load_roundtrip() {
    TEST(save_load_roundtrip);
    CHECK(ForceTuning::saveToFile(kTmp, 275.5));
    double v = 0.0;
    CHECK(ForceTuning::loadFromFile(kTmp, &v));
    CHECK(fabs(v - 275.5) < 1e-6);
    remove(kTmp);
    PASS();
}

static void test_missing_file_is_quiet_false() {
    TEST(missing_file_is_quiet_false);
    double v = 0.0;
    // 文件不存在 = 还没调过, 正常路径 ⇒ 返回 false 但不出声 (出声的是"文件在、内容不合规")
    CHECK(!ForceTuning::loadFromFile("_definitely_not_here_12345.json", &v));
    PASS();
}

static void test_corrupt_file_rejected() {
    TEST(corrupt_file_rejected);
    FILE* f = fopen(kTmp, "w");
    CHECK(f != nullptr);
    fprintf(f, "{\n \"version\": 1,\n \"reflection_gain\": 9999.0\n}\n");   // 越界
    fclose(f);
    double v = 0.0;
    CHECK(!ForceTuning::loadFromFile(kTmp, &v));
    remove(kTmp);
    PASS();
}

// ===== tick(): 防抖落盘的【生产路径】—— 时间作为输入 (2026-09-24) =====
//
// 【为什么要有它】: `TUNING_DEBOUNCE_MS` 那 1 秒防抖是"跨重启保留"的【唯一】实现 ——
//   `tick()` 是唯一的落盘路径 (RelayCore::pollRelayCommands 每帧调它)。
//   而本文件此前【11】条用例没有一条碰过 tick() (实测: 加这两条之前跑出 "11 passed, 0 failed";
//   计划/规格里写的 "12 条" 是笔误): 规格的验收行「落盘 → 读回」只经由
//   `saveToFile` / `loadFromFile` 那对显式函数验证过 ⇒ 生产走的那条路一条断言都没有。
// 【时间怎么进来】: `tickAt(nowMs)` 是接缝, `tick()` 在它外面包一层 GetTickCount()。
//   基准 `t0` 取 `GetTickCount()`: `setGain` 刚刚返回, 它的 `s_dirtyMs` 与 t0 只差几微秒
//   ⇒ t0 之后的偏移就是 s_dirtyMs 之后的偏移 (窗口 1000ms 对几微秒的误差免疫)。
// ⚠ 两条用例都【自己建立前置状态】(先 setGain 把静默期起点钉在 t0) —— 不依赖 main() 里的
//   调用顺序, 也不依赖 s_dirty 在进入本用例时是什么值 (它是 file-static, 会跨用例残留)。
static bool fileExistsAt(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    fclose(f);
    return true;
}

static void test_tick_at_debounces_persistence() {
    TEST(tick_at_debounces_persistence);

    remove(kTmp);
    // ⚠ tick() 写的是 CalibStore::fileFor("force_tuning.json") —— 一条【由 exe 位置推出的】
    //   绝对路径, 不是 kTmp。不打这一针, 下面全部 fileExistsAt(kTmp) 都在断言一个
    //   永远不存在的文件 (永远"绿", 也就永远测不出防抖)。见 setStorePathForTest 的说明。
    ForceTuning::setStorePathForTest(kTmp);
    const unsigned long t0 = GetTickCount();
    CHECK(ForceTuning::setGain(150.0));       // 标脏, 并把静默期起点钉在 t0

    ForceTuning::tickAt(t0);                  // 立刻: 还在静默期
    CHECK(!fileExistsAt(kTmp));
    ForceTuning::tickAt(t0 + 999);            // 差 1ms: 仍然不写
    CHECK(!fileExistsAt(kTmp));
    ForceTuning::tickAt(t0 + 1000);           // 到点: 写
    CHECK(fileExistsAt(kTmp));

    double v = 0.0;                           // 写的必须是【目标值】
    CHECK(ForceTuning::loadFromFile(kTmp, &v));
    CHECK(fabs(v - 150.0) < 1e-9);

    remove(kTmp);
    PASS();
}

static void test_tick_at_does_not_rewrite_when_clean() {
    TEST(tick_at_does_not_rewrite_when_clean);

    remove(kTmp);
    ForceTuning::setStorePathForTest(kTmp);   // 同上: 把落盘目标钉到本用例的临时文件
    const unsigned long t0 = GetTickCount();
    CHECK(ForceTuning::setGain(160.0));
    ForceTuning::tickAt(t0 + 1000);           // 写第一次
    CHECK(fileExistsAt(kTmp));

    remove(kTmp);                             // 删掉: 若下面又写, 文件会重新出现
    ForceTuning::tickAt(t0 + 5000);           // 已经不脏了 ⇒ 什么都不做
    CHECK(!fileExistsAt(kTmp));               // ★ "不是每次都重写"的判据

    PASS();
}

int main() {
    std::cout << "=== ForceTuning Tests ===" << std::endl;
    test_parse_ok();
    test_parse_wrong_version();
    test_parse_missing_version();
    test_parse_out_of_range();
    test_parse_nan_inf();
    test_parse_garbage();
    test_set_gain_bounds();
    test_static_initial_value_is_legal();
    test_save_load_roundtrip();
    test_missing_file_is_quiet_false();
    test_corrupt_file_rejected();
    test_tick_at_debounces_persistence();
    test_tick_at_does_not_rewrite_when_clean();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
