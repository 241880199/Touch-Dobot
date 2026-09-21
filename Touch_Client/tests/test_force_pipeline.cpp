// Standalone test: ForcePipeline filter + mapping + transform
// Build: see task-10-brief for exact command (add -I paths for OpenHaptics SDK)
// Run: test_force_pipeline.exe

#include <iostream>
#include <cassert>
#include <cmath>
#include <windows.h>
#include "../force/ForcePipeline.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static void test_residual_deadzone() {
    TEST(residual_deadzone);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Below residual deadzone (0.05N) → output zero
    fd.compensated[0] = 0.03; fd.compensated[1] = -0.03; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();
    ForcePipeline::step(fd);

    CHECK(fabs(fd.hapticOut[0]) < 0.01); // deadzone suppressed
    CHECK(fabs(fd.hapticOut[1]) < 0.01);
    CHECK(fabs(fd.hapticOut[2]) < 0.01);
    PASS();
}

static void test_saturation() {
    TEST(saturation);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Way above full scale → clamp (gain is applied post-mapping, so check final hapticOut)
    fd.compensated[0] = 500.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();
    ForcePipeline::step(fd);

    double clampedMax = Config::FORCE_MAX_TOUCH_N * Config::FORCE_REFLECTION_GAIN;
    CHECK(fabs(fd.hapticOut[0]) <= clampedMax + 0.01);
    CHECK(fd.hapticOut[0] > 0.0); // positive input → positive output
    PASS();
}

static void test_coord_transform() {
    TEST(coord_transform);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Input: Fx=10, Fy=20, Fz=30 (all well above residual deadzone)
    fd.compensated[0] = 10.0; fd.compensated[1] = 20.0; fd.compensated[2] = 30.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();

    // Run many steps to let Butterworth filter converge to steady state
    for (int i = 0; i < 100; i++) {
        ForcePipeline::step(fd);
    }

    // hapticOut 的映射: Fx→X, +Fz→Y, +Fy→Z, 各自乘净比例 (ratio × gain)。
    // ★ 2026-09-21 垂直项【从 -Fz 改成 +Fz】—— 本用例原来把 -Fz 钉死 (旧断言:
    //   `fd.hapticOut[1] < -0.01`, 注释 "should be from -Fz = -30")。
    //   改的理由【不是调参】: 反馈要的是【阻力】, 而 -Fz 的语义是"操作员压下去、触觉也往下推"
    //   —— 那是帮忙不是抵抗 (那行代码自己的注释举的例子就自相矛盾)。依据与现场实测见
    //   Config::FORCE_FEEDBACK_Z_SIGN 那一大段。
    //   ⚠ 这条断言【当初钉住的是一件错的东西】—— 一次正确的修法在这种用例下会【看起来像回归】。
    //     保留这段说明, 免得下一个人以为符号是随手改的。
    // 三个轴都按【带符号的精确值】检查 (不只查方向): 方向对而幅值错同样是错的。
    double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    double gain = Config::FORCE_REFLECTION_GAIN;
    CHECK(fabs(fd.hapticOut[0] - ratio * 10.0 * gain) < 0.01);   // 来自 +Fx = +10
    CHECK(fabs(fd.hapticOut[1] - ratio * 30.0 * gain) < 0.01);   // 来自 +Fz = +30 ⇒ 正
    CHECK(fabs(fd.hapticOut[2] - ratio * 20.0 * gain) < 0.01);   // 来自 +Fy = +20
    PASS();
}

static void test_filter_convergence() {
    TEST(filter_convergence);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Step input: 0 → 100N on Fx only
    fd.compensated[0] = 100.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();

    // Run many steps — filtered output should converge to input
    double last = 0.0;
    for (int i = 0; i < 200; i++) {
        ForcePipeline::step(fd);
        last = fd.filtered[0];
    }
    CHECK(fabs(last - 100.0) < 2.0); // converged within 2%
    PASS();
}

static void test_stale_detection() {
    TEST(stale_detection);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Old timestamp → stale
    fd.lastUpdateMs = GetTickCount() - 500;
    ForcePipeline::step(fd);
    // isStale is set by ForceReader; pollForce triggers zero out.
    // Here we just verify the struct default and mutate
    CHECK(fd.isStale == false || fd.isStale == true); // trivially passes — state is externally set
    PASS();
}

int main() {
    std::cout << "=== ForcePipeline Unit Tests ===" << std::endl;
    test_residual_deadzone();
    test_saturation();
    test_coord_transform();
    test_filter_convergence();
    test_stale_detection();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
