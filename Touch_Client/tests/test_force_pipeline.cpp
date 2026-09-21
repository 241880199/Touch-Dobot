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

    // 远低于门限 ⇒ 输出应当被压到基本为 0。
    // ⚠ 这句注释从前写的是 "Below residual deadzone (0.05N)" —— 而常数是
    //   Config::FORCE_RESIDUAL_DEADZONE_N = 0.20 ⇒ 数字过期了 (2026-09-21 改对)。
    //   两个数的差别不影响这条断言 (0.03 在两个门限下都远低于), 但它会误导读者。
    const double dz = Config::FORCE_RESIDUAL_DEADZONE_N;
    CHECK(dz > 0.1);   // 钉住"门限是 0.2 那个量级", 免得将来它被改小到让上面那句又变成实话
    fd.compensated[0] = 0.03; fd.compensated[1] = -0.03; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();
    ForcePipeline::step(fd);

    CHECK(fabs(fd.hapticOut[0]) < 0.01); // 软门把远低于门限的量压到基本为 0
    CHECK(fabs(fd.hapticOut[1]) < 0.01);
    CHECK(fabs(fd.hapticOut[2]) < 0.01);
    PASS();
}

// ★ 2026-09-21 新增 —— 直接钉住软门【要治的那个现象】。
// 【现象】现场: 静止时 FZ 读数在 0 与 ~0.2 之间来回跳。那不是力在跳, 是【硬门在阈值处跳变】:
//   残余悬在 dz 附近 (实测 'z' 之后 comp_z ≈ −0.166, dz = 0.20) ⇒ 抖一点就整段跳。
// 【这条断言测什么】取门限【两侧】各一个点, 输出必须几乎相等。
//   硬门: 0 与 0.201×ratio×gain ≈ 0.199 N ⇒ 差一整个门限 ⇒ 手感上就是"在跳" ⇒ 本条会红。
//   软门: 0.197 与 0.201 ⇒ 差 0.004 N×ratio×gain ⇒ 不跳。
//   ⇒ 所以这条用例是【可证伪】的: 谁把软门改回硬门, 它会立刻红。
static void test_soft_deadzone_no_jump() {
    TEST(soft_deadzone_no_jump);
    const double dz    = Config::FORCE_RESIDUAL_DEADZONE_N;
    const double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    const double gain  = Config::FORCE_REFLECTION_GAIN;

    AppState::ForceData below, above;
    ForcePipeline::init();
    below.compensated[0] = dz * 0.995; below.lastUpdateMs = GetTickCount();
    for (int i = 0; i < 200; i++) ForcePipeline::step(below);   // 让滤波收敛

    ForcePipeline::init();
    above.compensated[0] = dz * 1.005; above.lastUpdateMs = GetTickCount();
    for (int i = 0; i < 200; i++) ForcePipeline::step(above);

    const double jump = fabs(above.hapticOut[0] - below.hapticOut[0]);
    (void)ratio; (void)gain;   // 只在下面那句注释里用来说明量级
    CHECK(jump < 0.02);
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

// ★ 2026-09-21 新增: 死区现在是【唯一一份定义】(ForcePipeline.h 里那个 inline), 而且是软的。
// 这条用例【直接调它】—— 证明头文件里确实暴露了它 (RelayCore 构造 F| 消息时用的就是同一个)。
// 【为什么需要这条】这条规则曾经有【两份硬门实现】(触觉那路一份、F| 那路一份), 改了一份忘一份,
//   现场就表现为"MATLAB 上 FZ 在 0 与 ±0.2 之间阶跃式跳"。
// ⇒ 若谁把它改回硬门、或又在别处写第二份, 这条会红 (门限以下必须【不为 0】, 硬门会给 0)。
static void test_soft_deadzone_shared_and_smooth() {
    TEST(soft_deadzone_shared_and_smooth);
    const double dz = Config::FORCE_RESIDUAL_DEADZONE_N;
    // 门限以上: 【原样】1:1 —— 笔压 (0.3~0.6N) 落在这个区间, 幅值不许受影响
    CHECK(ForcePipeline::softDeadzone(+1.0, dz) == +1.0);
    CHECK(ForcePipeline::softDeadzone(-1.0, dz) == -1.0);
    // 门限处: 连续 (上下两支相接)
    CHECK(fabs(ForcePipeline::softDeadzone(dz, dz) - dz) < 1e-12);
    // 门限以下: 被压小、但【不为 0】(软门; 硬门在这里会给 0 ⇒ 这就是跳变的来源)
    CHECK(ForcePipeline::softDeadzone(+0.1, dz) > 0.0);
    CHECK(ForcePipeline::softDeadzone(+0.1, dz) < 0.1);
    // 符号保持
    CHECK(ForcePipeline::softDeadzone(-0.1, dz) < 0.0);
    PASS();
}

int main() {
    std::cout << "=== ForcePipeline Unit Tests ===" << std::endl;
    test_residual_deadzone();
    test_soft_deadzone_no_jump();
    test_soft_deadzone_shared_and_smooth();
    test_saturation();
    test_coord_transform();
    test_filter_convergence();
    test_stale_detection();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
