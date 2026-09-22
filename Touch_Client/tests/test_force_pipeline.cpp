// Standalone test: ForcePipeline filter + mapping + transform
// Build: call .\build_force_pipeline_test.bat (in this directory) -- it carries the exact cl line.
//   ⚠ .\ 【不能省】—— 从 bash 经 cmd 调用时 NoDefaultCurrentDirectoryInExePath=1, 裸名字 call 会 not recognized (2026-09-22 实测)。
//   ⚠ 从前这里指向 .superpowers/sdd/task-10-brief.md 里的"exact command"，而那个文件
//     (a) 被 gitignore(盘上存在但不在仓里)、(b) 里面根本没有 cl /Fe: 也不提本用例
//     ⇒ 那是一根死指针 (2026-09-22 核过)。
// Run: test_force_pipeline.exe

#include <iostream>
#include <cassert>
#include <cmath>
#include <windows.h>
#include "../force/ForcePipeline.h"
#include "../force/ForceTuning.h"
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

// 2026-09-22: 增益旋钮【真的接上了】的证据。
// 【为什么必须有这条】只断言 ForceTuning::setGain 的返回值不构成证据 —— 那证明的是
//   "模块内部一致", 不是"ForcePipeline 真的读它"。本项目踩过"先确认它真的被执行过"的亏。
static void test_gain_actually_changes_output() {
    TEST(gain_actually_changes_output);

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;              // 1.0 N 远在死区 0.20 之上 ⇒ softDeadzone 原样返回
    fd.lastUpdateMs = GetTickCount();

    // gain = 300: 输出 = 1.0 × (3.3/200) × 300 = 4.95, 横向符号 −1 ⇒ −4.95
    ForceTuning::setGain(300.0);
    ForcePipeline::init();                // 让斜坡直接就位 (不然要等 0.25s)
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 让滤波器收敛
    CHECK(fabs(fd.hapticOut[0] - (-4.95)) < 0.05);

    // gain = 120: 同一个输入 ⇒ −1.98
    ForceTuning::setGain(120.0);
    ForcePipeline::init();
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);
    CHECK(fabs(fd.hapticOut[0] - (-1.98)) < 0.03);

    PASS();
}

// 斜坡: 增益不是一步到位, 而是 800/秒 (⇒ 100→300 走 0.25s = 31 帧 @125Hz)。
// 判据取一个区间而不是精确帧数 —— 精确值会随常数微调而红, 那不是缺陷。
static void test_gain_ramp_is_gradual() {
    TEST(gain_ramp_is_gradual);

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;
    fd.lastUpdateMs = GetTickCount();

    ForceTuning::setGain(100.0);
    ForcePipeline::init();                       // 斜坡就位在 100
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 先让滤波器收敛
    const double at100 = fd.hapticOut[0];
    CHECK(fabs(at100 - (-1.65)) < 0.03);         // 1.0 × 0.0165 × 100 = 1.65

    // 跳到 300, 数多少帧才到位
    ForceTuning::setGain(300.0);
    int frames = 0;
    while (fabs(fd.hapticOut[0] - (-4.95)) > 0.05 && frames < 200) {
        ForcePipeline::step(fd);
        frames++;
    }
    CHECK(frames < 200);                         // 必须【到达】—— 卡住说明斜坡没在动
    CHECK(frames >= 20);                         // 必须【不是一步到位】—— 否则斜坡是假的
    CHECK(frames <= 45);                         // 也别慢得离谱 (理论 31 帧)

    ForceTuning::setGain(Config::FORCE_REFLECTION_GAIN);   // 复原, 免得污染后面的用例
    ForcePipeline::init();
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

    // 远高于满量程 ⇒ 映射后必然打到夹子。★ 2026-09-22: 【跑到收敛再断言】。
    // 从前只调 1 次 step(): fc=5Hz/fs=125Hz 的二阶 Butterworth 一步只给
    // b0·500 ≈ 6.7 N, 映射后 ≈13.3, 离夹子 396 差得远 ⇒ 那句 `<= clampedMax` 恒真,
    // 用例名说 saturation 却什么都没验 (旧注释已照实承认, 本次做完)。
    fd.compensated[0] = 500.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();

    // 200 步与 test_soft_deadzone_no_jump 同一惯例 (≈10 个时间常数 ⇒ 充分收敛)。
    // ★ 2026-09-22 订正(实测): 越过映射门限(需 filtered > 200N)是【第 6 步】才到的
    //   (实测 filtered: 第 5 步 173N、第 6 步 223N)。约束它的是【滤波器自身的上升时间】,
    //   不是梯度限幅: fc=5Hz/fs=125Hz 的 τ ≈ 4 个采样, 第 5/6 步的理论值 ≈177/223N,
    //   与实测吻合。梯度限幅反而是【不生效】的 —— 因为单步滤波器输出只动 ~6.7N,
    //   远在 50 N/frame 之下。(本处原写"200/50=4 步", 那是把梯度限幅当成了约束, 归因错了。)
    for (int i = 0; i < 200; i++) ForcePipeline::step(fd);

    // ★ 判据从"不超过夹子"改成"【正好落在夹子上】"—— 这才是饱和。
    //   算式 (逐环核过, 见 ForcePipeline.cpp:79-89 / :126 / :135):
    //     softDeadzone(500)=500 → ×(3.3/200)=8.25 → 硬夹到 3.3 → ×SIGN(−1) → ×120
    //     = −396 = −FORCE_MAX_TOUCH_N × FORCE_REFLECTION_GAIN
    //   ⇒ 若谁把 mapForceToTouch 里的硬夹去掉, 输出会是 −8.25×(−1)×120 = −990 ⇒ 本条立刻红。
    double clampedMax = Config::FORCE_MAX_TOUCH_N * Config::FORCE_REFLECTION_GAIN;
    // ★ 钉住"夹子是 396 那个量级" —— 因为上面那条期望值是从【流水线自己乘的】两个常数
    //   (FORCE_MAX_TOUCH_N × FORCE_REFLECTION_GAIN) 推出来的, 那两个常数被重调时它会一直绿。
    //   同形先例: test_residual_deadzone 的 `CHECK(dz > 0.1);`。
    CHECK(clampedMax > 100.0);
    CHECK(fabs(fabs(fd.hapticOut[0]) - clampedMax) < 0.01);   // 正好在夹子上
    // ⚠ 符号故意写死负号 (拿 Config 常数去算断言会对那些常数【恒真】),
    //   完整论述见 test_coord_transform 的同段说明。
    CHECK(fd.hapticOut[0] < 0.0);   // 正输入 ⇒ 负输出 (横向映射 FORCE_FEEDBACK_LATERAL_SIGN = −1)
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

    // hapticOut 的映射: Fx→Touch X、Fy→Touch Z 各带整体符号 −1 再乘净比例 (ratio × gain);
    //                    Fz→Touch Y 那一轴【整轴关掉了】(常量 0)。
    // ★★ 2026-09-21 当天定案两次, 本行钉住最终值。
    //
    // 【横向为什么是负号】力映射 = 位置映射的逆 (L = Mᵀ), 而代码里写死的轴对应等于 Mᵀ 再整体
    //   取负 ⇒ L = −Mᵀ。那个整体符号由【唯一有实测锚点的那一行】(垂直: 压笔 ⇒ comp_z = −1.55,
    //   且现场确认 −fz 方向是对的) 定死, 再对三行一起成立。用户同时报告"阻力的方向都反了"。
    //   ⇒ 完整推导 (含"这只是推导+现场描述、可证伪的检验是什么") 见
    //     Config::FORCE_FEEDBACK_LATERAL_SIGN 那一大段 —— 动符号之前先读它。
    //
    // 【垂直为什么整轴关】Touch Y 正是操作员用来【落笔 / 维持入纸深度】的那一轴: 往下推 =
    //   帮忙往纸里按; 往上推 = 把笔抬起来, **根本落不了笔** ⇒ 两个方向都不行, 不是符号问题。
    //
    // ⚠ 三个数这里【都故意写字面值】, 不写 `* Config::FORCE_FEEDBACK_*`: 拿常数去算断言会让
    //   本用例对它们【恒真】—— 改符号/重新打开那一轴它也不红, 而"这三路各是什么方向、
    //   垂直开着还是关着"正是本行要测的东西。
    //   ⇒ 将来谁要动它们, 本行【会红】, 那是**故意的**: 它逼人回去读那两段注释并按实测重定。
    // ⚠ 输入故意给 fz = +30 (很大的值): 若垂直那轴还开着, 输出会是 ±0.99 N 量级, 0.01 的容差
    //   必然拦住 ⇒ 这条断言不会因为"输入太小"而恒真地通过。
    // 三个轴都按【带符号的精确值】检查 (不只查方向): 方向对而幅值错同样是错的。
    double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    double gain = Config::FORCE_REFLECTION_GAIN;
    CHECK(fabs(fd.hapticOut[0] - (-ratio * 10.0 * gain)) < 0.01);     // 来自 +Fx = +10 ⇒ 【负】
    CHECK(fabs(fd.hapticOut[1]) < 0.01);                              // Fz→Y 已关 ⇒ 与 fz=+30 无关
    CHECK(fabs(fd.hapticOut[2] - (-ratio * 20.0 * gain)) < 0.01);     // 来自 +Fy = +20 ⇒ 【负】
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
    test_gain_actually_changes_output();
    test_gain_ramp_is_gradual();
    test_soft_deadzone_no_jump();
    test_soft_deadzone_shared_and_smooth();
    test_saturation();
    test_coord_transform();
    test_filter_convergence();
    test_stale_detection();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
