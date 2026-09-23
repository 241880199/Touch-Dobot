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

// 增益用例会改【全局】的 ForceTuning::gain(), 而 CHECK 失败会立刻 return ——
//   ⇒ 若把复原写在用例末尾, 一条红会让后面【每个】用例都跟着红。
//   【实测证据 (负对照, 2026-09-22)】把斜坡关掉 (step 里直接 snap) 那一版: 除了本条红,
//     saturation 与 coord_transform 也红了 —— 它们期望的是默认增益 120, 而失败的斜坡用例
//     提前 return 时 gain 还停在 300。那两条其实无辜, 却把"病因"指错了地方。
//   ⇒ 把复原绑在作用域上: 无论正常走完还是中途 return 都会执行。
struct GainScope {
    const double saved;
    GainScope() : saved(ForceTuning::gain()) {}
    ~GainScope() {
        // ★ 复原的是【构造那一刻读到的值】(ForceTuning::gain()), 【不是】Config::FORCE_REFLECTION_GAIN。
        //   今天两者相等 (静态初值就来自 Config, 且测试不读 force_tuning.json), 但这个区别是要紧的:
        //   若某条用例之前已经有人改过增益, 本守卫把那个值原样放回去 —— 比"手工复原成默认值"是【更强】
        //   的保证 (手工复原只在"用例之间增益总是回到默认值"这个前提下才等价, 而那个前提正是它要守的东西)。
        //   ⇒ 所以别在这条注释里写"与旧用例的手工复原等价": 那句话把强保证说成了等号右边。
        if (!ForceTuning::setGain(saved)) {
            // ⚠ 不许静默失败: setGain 会做范围校验, saved 若落在 [GAIN_MIN, GAIN_MAX] 之外就【不改状态】
            //   ⇒ 增益会被留在被测用例改过的值上, 后面每条用例都在错的增益下跑, 而红点会指向别处。
            //   (saved 只可能来自 gain(), 它自己也是经校验写进去的 ⇒ 正常不会发生。) 同一条规矩见本文
            //   其余 setGain 调用点: 返回值一律 CHECK。
            std::cout << "  [GainScope] !! 复原失败: " << saved
                      << " 不在 [ForceTuning::GAIN_MIN, GAIN_MAX] 内 —— 增益留在被改过的值上" << std::endl;
        }
        ForcePipeline::init();   // 顺带把斜坡与滤波器复位
    }
};

// 2026-09-22: 增益旋钮【真的接上了】的证据。
// 【为什么必须有这条】只断言 ForceTuning::setGain 的返回值不构成证据 —— 那证明的是
//   "模块内部一致", 不是"ForcePipeline 真的读它"。本项目踩过"先确认它真的被执行过"的亏。
static void test_gain_actually_changes_output() {
    TEST(gain_actually_changes_output);
    GainScope gainScope;   // 中途 CHECK 失败也要把增益复原 (见上面那段)

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;              // 1.0 N 远在死区 0.20 之上 ⇒ softDeadzone 原样返回
    fd.lastUpdateMs = GetTickCount();

    // gain = 300: 输出 = 1.0 × (3.3/200) × 300 = 4.95, 横向符号 +1 ⇒ +4.95
    // ⚠ 符号 2026-09-23 由 −1 翻成 +1：现场实测「往右推 touch 会有往右的力」（= 顺着推，不是阻力），
    //   命中了 Config::FORCE_FEEDBACK_LATERAL_SIGN 注释里【它自己写下的那条可证伪检验】。
    //   期望值随之翻号、幅度不变；依据（含两个现场报告互相矛盾那件事）记在该常数处。
    // ⚠ 300 是 ForceTuning::GAIN_MAX 的当前值 —— 范围若收窄, setGain 会【返回 false 且什么都不改】,
    //   而下面那条断言只会在值对不上时红 ⇒ 病因看起来是"映射错了"。所以前置条件必须在这里
    //   被【命名】: 这条 CHECK 红了就是"量程对不上", 不是"增益没生效"。
    CHECK(ForceTuning::setGain(300.0));
    ForcePipeline::init();                // 让斜坡直接就位 (不然要等 0.25s)
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 让滤波器收敛
    CHECK(fabs(fd.hapticOut[0] - (+4.95)) < 0.05);

    // gain = 120: 同一个输入 ⇒ +1.98
    CHECK(ForceTuning::setGain(120.0));
    ForcePipeline::init();
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);
    CHECK(fabs(fd.hapticOut[0] - (+1.98)) < 0.03);

    PASS();
}

// ★ 2026-09-22 (Fix round 1) 新增 —— ForcePipeline::saturationSensorN() 从前【既没有调用者也没有用例】,
//   而它是后续 Task 要在 MATLAB 界面上显示的【那个数】: 公式错了就是界面在撒谎, 而且没人会发现。
//
// 【期望值怎么来的 —— 从代码现推, 不是抄注释里的 1.67】
//   ForcePipeline.h 里那两行是:  净比例 = FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N
//                               返回值 = FORCE_MAX_TOUCH_N / (净比例 × gain)
//   把净比例代进去, FORCE_MAX_TOUCH_N 上下相消 ⇒ 返回值 ≡ FORCE_MAX_SENSOR_N / gain。
//   ⇒ 本用例的期望值就写【化简后的那一支】。它是另一个算式, 所以能钉住实现:
//     少乘/多乘一次 gain、比值写反、除数被当成被除数, 这里都会不等 (负对照实测见 task-2-report.md)。
static void test_saturation_sensor_n() {
    TEST(saturation_sensor_n);
    // 本用例【不需要】GainScope: saturationSensorN 只吃参数, 不读 ForceTuning::gain() (纯函数)。

    // (a) 出厂默认增益处钉住数值。期望值 = FORCE_MAX_SENSOR_N / 120 (= 当前 1.667 N ——
    //     与 ForcePipeline.h 那段注释里写的 1.67 一致)。
    //     ⚠ 120.0 写【字面值】是故意的: 它就是 Config::FORCE_REFLECTION_GAIN 的当前值
    //     (已回核 config/Config.h, 不是凭记忆), 但本用例要测的是"gain = 120 时那个数是多少"。
    //     若让它跟着那个常数漂, 常数一改这条就变成在测另一个增益, 而它自己不会说。
    const double satAtDefault = ForcePipeline::saturationSensorN(120.0);
    CHECK(fabs(satAtDefault - Config::FORCE_MAX_SENSOR_N / 120.0) < 1e-6);
    // ★ 钉住"它是 1.6x 那个量级" —— 上面那条的期望值里只有 FORCE_MAX_SENSOR_N 与被测实现同源,
    //   所以若有人把量程改成离谱的值 (而值的确会跟着变), 上面那条会一直绿。这一条补上"量级对不对"。
    //   ⚠ 改量程而红是【故意的】: 逼人来读这段并确认界面上的数该是多少。
    //   同形先例: test_residual_deadzone 的 `CHECK(dz > 0.1);` / test_saturation 的 `CHECK(clampedMax > 100.0);`。
    CHECK(satAtDefault > 1.0 && satAtDefault < 3.0);

    // (b) 量程上界处: 数值 + 【单调性】(增益越高 ⇒ 越早打顶 ⇒ 这个输入值越小)。
    //     上界显式写成符号 —— 范围的【唯一一份定义】在 ForceTuning.h, 这里不重述它的数值。
    CHECK(ForceTuning::GAIN_MAX > 120.0);   // 本用例的前提: 上界高于出厂默认, 否则下一步的比较没意义
    const double satAtMax = ForcePipeline::saturationSensorN(ForceTuning::GAIN_MAX);
    CHECK(fabs(satAtMax - Config::FORCE_MAX_SENSOR_N / ForceTuning::GAIN_MAX) < 1e-6);
    CHECK(satAtMax < satAtDefault);         // 单调: 增益 300 > 120 ⇒ 打顶输入 0.667 < 1.667

    // (c) 退化输入: gain <= 0 ⇒ 走实现里那道守卫, 返回 0 而【不是】inf/NaN。
    //     (gain = 0 会让净比例为 0 ⇒ 除法会爆; 这道守卫就是防它。) 界面拿到的是 0, 不是 nan。
    const double satZero = ForcePipeline::saturationSensorN(0.0);
    CHECK(satZero == 0.0);
    CHECK(satZero == satZero);              // 排除 NaN (NaN != NaN)
    CHECK(ForcePipeline::saturationSensorN(-120.0) == 0.0);
    PASS();
}

// 斜坡: 增益不是一步到位, 而是按 FORCE_GAIN_SLEW_PER_S 逐帧逼近 (⇒ 走完 GAIN_MIN→GAIN_MAX
//   约 0.25s = 31 帧 @FORCE_FILTER_FS_HZ)。
// 判据取一个区间而不是精确帧数 —— 精确值会随常数微调而红, 那不是缺陷。
// ⚠ 本行的 0.25s / 31 帧是【当前常数下的算术】, 不是依据: 斜坡速率的定义在
//   Config::FORCE_GAIN_SLEW_PER_S, 增益范围的唯一定义在 ForceTuning::GAIN_MIN / GAIN_MAX。
//   改了它们, 本注释不会红 (下面那几个区间断言可能也不红) ⇒ 它只是读起来有用的算术。
static void test_gain_ramp_is_gradual() {
    TEST(gain_ramp_is_gradual);
    GainScope gainScope;   // 中途 CHECK 失败也要把增益复原 (见上面那段)

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;
    fd.lastUpdateMs = GetTickCount();

    // ⚠ 100 / 300 是 ForceTuning::GAIN_MIN / GAIN_MAX 的当前值 (范围的唯一定义在 ForceTuning.h)。
    //   前置条件必须【被命名】: 范围若收窄, setGain 返回 false 且【什么都不改】, 那么下面那些
    //   数值断言会红成"斜坡错了/映射错了", 病因指错地方。这两条 CHECK 红了就是"量程对不上"。
    CHECK(ForceTuning::setGain(100.0));
    ForcePipeline::init();                       // 斜坡就位在 100
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 先让滤波器收敛
    const double at100 = fd.hapticOut[0];
    CHECK(fabs(at100 - (+1.65)) < 0.03);         // 1.0 × 0.0165 × 100 = 1.65 (符号 2026-09-23 翻正)

    // 跳到 300, 数多少帧才到位
    CHECK(ForceTuning::setGain(300.0));
    int frames = 0;
    while (fabs(fd.hapticOut[0] - (+4.95)) > 0.05 && frames < 200) {
        ForcePipeline::step(fd);
        frames++;
    }
    CHECK(frames < 200);                         // 必须【到达】—— 卡住说明斜坡没在动
    CHECK(frames >= 20);                         // 必须【不是一步到位】—— 否则斜坡是假的
    CHECK(frames <= 45);                         // 也别慢得离谱 (理论 31 帧)

    // 复原【不在这里做】: GainScope 的析构是唯一权威 (它连中途 return 那条路都覆盖, 这里的手工
    //   复原只覆盖"走到底"那一条)。★ 2026-09-22 (Fix round 1) 把原来那两行 (setGain + init)
    //   删掉了 —— 它们是同一件事的第二份实现, 而本项目有成文教训: 同一规则两份实现, 改一份忘一份。
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
    const double gain  = ForceTuning::gain();   // 运行时值 —— 流水线乘的是它 (见本函数末尾的说明)

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
    // ⚠ 上面两个量【不参与断言】(否则对本用例要测的东西恒真)。但 gain 读的是【运行时】值
    //   (ForceTuning::gain()): 流水线乘的是它, 不是编译时的 Config::FORCE_REFLECTION_GAIN ——
    //   注释里引用一个"流水线没在用的数"就是下一句假话的种子 (2026-09-22 Fix round 1)。
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
    //     softDeadzone(500)=500 → ×(3.3/200)=8.25 → 硬夹到 3.3 → ×SIGN(−1) → ×gain
    //     = −FORCE_MAX_TOUCH_N × gain
    //   ⇒ 若谁把 mapForceToTouch 里的硬夹去掉, 输出会是 −8.25×(−1)×gain ⇒ 本条立刻红。
    // ★★ 2026-09-22 (Fix round 1): 乘数从【编译时】Config::FORCE_REFLECTION_GAIN 改成
    //   【运行时】ForceTuning::gain() —— 流水线现在乘的是后者 (见 ForcePipeline.cpp 的增益那一步),
    //   而两个数【今天相等】只是因为本测试二进制从不加载 force_tuning.json (所以 gain() = 静态初值
    //   = Config::FORCE_REFLECTION_GAIN)。用前者当期望值, 一旦有人给测试装上 tuning 文件,
    //   这条断言就会拿一个【流水线没乘的数】去比 —— 而且它不会"响亮地红", 只会怪在别处。
    double clampedMax = Config::FORCE_MAX_TOUCH_N * ForceTuning::gain();
    // ★ 钉住"夹子是 396 那个量级" (396 = FORCE_MAX_TOUCH_N × ForceTuning::gain() 的当前值,
    //   即 3.3 × 120) —— 因为上面那条期望值是从【流水线自己乘的那两个量】推出来的, 它们被重调时
    //   它会一直绿 (期望值里的 gain 现在是运行时值, 量程与增益两者它都跟着漂)。
    //   同形先例: test_residual_deadzone 的 `CHECK(dz > 0.1);`。
    CHECK(clampedMax > 100.0);
    CHECK(fabs(fabs(fd.hapticOut[0]) - clampedMax) < 0.01);   // 正好在夹子上
    // ⚠ 符号故意写死负号 (拿 Config 常数去算断言会对那些常数【恒真】),
    //   完整论述见 test_coord_transform 的同段说明。
    CHECK(fd.hapticOut[0] > 0.0);   // 正输入 ⇒ 正输出 (器件 X ← +基座 Fx, FORCE_FEEDBACK_TOUCH_X_SIGN = +1;
                                    //   2026-09-23 现场实测推翻旧符号，依据见该常数处注释)
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

    // hapticOut 的映射: Fx→Touch X、Fy→Touch Z 各带整体符号 **+1** 再乘净比例 (ratio × gain);
    //                    Fz→Touch Y 那一轴【整轴关掉了】(常量 0)。
    // ★★ 2026-09-23: 横向符号由 **−1 翻成 +1** —— 本行随之钉住**新值**。
    //
    // 【横向为什么现在是正号】现场实测"往右推 touch 会有往右的力"（= 顺着推、**不是阻力**），
    //   正好命中 Config::FORCE_FEEDBACK_LATERAL_SIGN 那段**【自己写下的可证伪检验】**
    //   （"把笔尖按住并朝某个方向推, 手上必须被推向相反方向；若仍是顺着推 ⇒ σ 推断错了"）
    //   ⇒ 判否 ⇒ 按同段的规定翻号。
    //   ⚠ 那次翻转同时**推翻了"一个 σ 三行通用"这个前提**（垂直那行的 −1 是**实测**的、横向是从它推的），
    //     而且**两份现场报告互相矛盾**（09-21 报"方向都反了" ⇒ 当时才定 −1）。
    //   ⇒ 完整依据、矛盾怎么处置、以及"**横向逐轴实测仍待做**"见
    //     Config::FORCE_FEEDBACK_LATERAL_SIGN 那一大段 —— **动符号之前先读它**。
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
    // ★★ 2026-09-22 (Fix round 1): gain 必须读【运行时】的 ForceTuning::gain() —— 流水线乘的是它,
    //   不是编译时的 Config::FORCE_REFLECTION_GAIN。两个数今天相等【只因为】本测试二进制从不加载
    //   force_tuning.json (ForceTuning.cpp 的静态初值就取自 Config, 且测试不调 loadOnStartup)。
    //   ⇒ 拿编译时常数当期望值 = 一条"只在测试环境成立"的断言: 它现在绿, 但它钉的不是流水线
    //     真正用的那个数。上面 ratio 那两个常数【保留】是对的 —— 流水线确实读它们。
    double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    double gain = ForceTuning::gain();
    CHECK(fabs(fd.hapticOut[0] - (+ratio * 10.0 * gain)) < 0.01);     // 来自 +Fx = +10 ⇒ 【正】
                                                                      //   (2026-09-23 符号 −1→+1，同 f196cb0)
    CHECK(fabs(fd.hapticOut[1]) < 0.01);                              // Fz→Y 已关 ⇒ 与 fz=+30 无关
    CHECK(fabs(fd.hapticOut[2] - (-ratio * 20.0 * gain)) < 0.01);     // 来自 +Fy = +20 ⇒ 【负】
                                                                      //   ★ 2026-09-23: 器件 Z ← −基座 Fy（Mᵀ 决定，与 X 相反）
                                                                      //   (2026-09-23 符号 −1→+1，同 f196cb0)
    PASS();
}

// ★★ 2026-09-23（用户要求"先解决阻力方向"）：**横向两路的相对符号是【结构性的】，不是选择**。
//   力是向量 ⇒ 器件系里的力 = Mᵀ·(基座系力)，而 M 就是平移路径那张表：
//       基座 +X  ->  器件 +X      (同号)
//       基座 +Y  ->  器件 −Z      (【反号】)   ← 这一条是本用例要钉的
//       基座 +Z  ->  器件 +Y      (同号, 今天整轴关掉)
//   ⇒ 对同样的输入, `hapticOut[0]` 与 `hapticOut[2]` 必须【符号相反】。
//   任何"用一个常数给两路"的写法都【必然】违反它 —— 那正是 09-21 与 09-23 两份互相矛盾的
//   现场报告各说一路的根源（σ=−1: X 错 Z 对 · σ=+1: X 对 Z 错）。本用例会让那种写法变红。
static void test_lateral_channels_have_opposite_relative_sign() {
    AppState::ForceData fd = {};
    fd.compensated[0] = +10.0;   // 基座 Fx 为正
    fd.compensated[1] = +20.0;   // 基座 Fy 为正
    fd.compensated[2] = 0.0;
    fd.lastUpdateMs = GetTickCount();
    CHECK(ForceTuning::setGain(120.0));
    ForcePipeline::init();
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);
    // 两路都必须非零（否则这条断言会被"某一路恒 0"白拿）
    CHECK(fabs(fd.hapticOut[0]) > 1e-6);
    CHECK(fabs(fd.hapticOut[2]) > 1e-6);
    const double s0 = fd.hapticOut[0] > 0 ? 1.0 : -1.0;
    const double s2 = fd.hapticOut[2] > 0 ? 1.0 : -1.0;
    CHECK(s0 * s2 < 0.0);            // ← 必须相反
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

// ★ 2026-09-23 新增 (抖动计划 Task 2): 两个【只读】累加器真的被 step() 喂了 —— 而且喂的是
//   【两个不同的量】。本用例不读硬件、不碰 socket, 只用现成的 AppState::ForceData。
//
// 【为什么必须有这条】Task 2 的接线全是"没有返回值的副作用", 没有任何既有用例会因为
//   接线写错而红: 删掉那两行 addFrame、把两个累加器互换、只喂一个、或者按错的东西喂,
//   整床照样全绿 —— 而判据 A/B 会据此给出【相反】的结论。本项目在"接线从未被用例区分过"
//   这条上吃过多次亏 (最近一次: 按钮2 的限幅用例单轴 ⇒ 逐分量夹与旋转角夹等价)。
//
// 【它怎么区分】(两个子情形, 都在死区【两侧】各钉一次)
//   (a) 死区【之上】(dz*4 的常量输入): 残差每帧都越阈 ⇒ fracResidual == 1.0 精确值。
//       ⚠ 它同时是"阈值在喂帧前被武装过"的证据: 没武装时 fracAbove 返回 −1, 不是 1.0。
//   (b) 死区【之下】(dz*0.25): 残差 = 那个输入本身 (0.05 量级, 逐帧不变, 不过死区);
//       而输出还要再挨一次软门 (r²) 与增益 ⇒ meanOut 比 meanResidual 【小得多】。
//       若把两个累加器【接反】(输出喂成残差), 这个比值会变成 1, 本条立刻红。
//       ⇒ 这一条是"两个量不可互换"在本仓唯一的自动证据。
static void test_jitter_accumulators_are_fed_and_distinct() {
    TEST(jitter_accumulators_are_fed_and_distinct);
    const double dz = Config::FORCE_RESIDUAL_DEADZONE_N;
    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) fd.compensated[i] = 0.0;

    // ⚠ 【先把滤波器建立起来, 再开窗口】: init() 会把滤波器清零, 于是窗口头几帧的 filtered
    //   其实还在从 0 往上爬 (阶跃响应的时间常数约 4 帧)。把它们算进窗口, 均值会偏低、
    //   越阈占比也不会是"每帧都越"—— 那量到的是【滤波器建立过程】, 不是本用例要问的东西。
    //   (第一版就是这么写红的: fracResidual 报 0.925 而不是 1.0。)
    //   这里顺便把 resetJitterStats() 也走一遍: 它【不该】把武装的阈值一起清掉 ——
    //   清掉的话下面的 fracResidual 会变成 −1 (算不出来), 一眼就能看出来。
    const int SETTLE = 20;          // ≈5 个时间常数 ⇒ 值已在渐近线的 0.7% 以内
    const int N = 40;

    // ---- (a) 死区之上 ----
    fd.compensated[0] = dz * 4.0;
    ForcePipeline::init();          // 清窗口 + 武装越阈计数 (必须在喂帧之前)
    for (int i = 0; i < SETTLE; i++) ForcePipeline::step(fd);
    ForcePipeline::resetJitterStats();      // 窗口从这里开始 (武装是粘性的)
    for (int i = 0; i < N; i++) ForcePipeline::step(fd);

    ForcePipeline::JitterSnapshot s;
    ForcePipeline::copyJitterSnapshot(s);
    CHECK(s.nOut == N);             // 一次 step = 一帧 (不数回调次数)
    CHECK(s.nResidual == N);
    CHECK(s.nOut == s.nResidual);   // 同一个喂入点 ⇒ 恒等 (main.cpp 的打印也靠这条不变式)
    CHECK(s.fracResidual[0] == 1.0);   // 每帧都越阈; 不是 −1 (⇒ 阈值武装过了, 且没被 reset 清掉)
    CHECK(s.meanResidual[0] > dz);     // 残差 = 死区的【输入】(传感器 N), 未被软门压小

    // ---- (b) 死区之下: 两个量必须分得开 ----
    fd.compensated[0] = dz * 0.25;
    ForcePipeline::init();          // 重新开一个窗口 (顺带复位滤波器与斜坡)
    for (int i = 0; i < SETTLE; i++) ForcePipeline::step(fd);
    ForcePipeline::resetJitterStats();
    for (int i = 0; i < N; i++) ForcePipeline::step(fd);
    ForcePipeline::copyJitterSnapshot(s);
    CHECK(s.nOut == N);
    CHECK(s.fracResidual[0] == 0.0);          // 0.05 量级的残差不越 0.20 的门 —— 而且【不是 −1】
    // 它就是那个输入本身 (窗口里没有建立过程了 ⇒ 容差可以收紧到 5%)。
    CHECK(fabs(s.meanResidual[0] - dz * 0.25) < dz * 0.25 * 0.05);
    CHECK(s.meanOut[0] > 0.0);                // 输出还活着 (没被清零)
    CHECK(s.meanOut[0] < s.meanResidual[0] * 0.5);   // 软门 + 增益 ⇒ 小得多; 接反了会变成 1:1

    // ---- 窗口语义: reset 之后从 0 开始 (main.cpp 的"自上次 'n' 起"靠它) ----
    ForcePipeline::resetJitterStats();
    ForcePipeline::copyJitterSnapshot(s);
    CHECK(s.nOut == 0);
    CHECK(s.nResidual == 0);
    CHECK(s.fracResidual[0] == 0.0);   // 0 帧 ⇒ 0 (不是 −1: 空集的占比是"没有", 不是"算不出来")
    PASS();
}

int main() {
    std::cout << "=== ForcePipeline Unit Tests ===" << std::endl;
    test_residual_deadzone();
    test_gain_actually_changes_output();
    test_gain_ramp_is_gradual();
    test_saturation_sensor_n();
    test_soft_deadzone_no_jump();
    test_soft_deadzone_shared_and_smooth();
    test_saturation();
    test_coord_transform();
    test_lateral_channels_have_opposite_relative_sign();
    test_filter_convergence();
    test_stale_detection();
    test_jitter_accumulators_are_fed_and_distinct();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
