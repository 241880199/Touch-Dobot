// 抖动诊断用的【纯统计累加器】—— 只累加, 不判断。头文件内联, 无 .cpp 可链。
//
// 为什么要有它 (2026-09-23, 抖动诊断计划 Task 1):
//   现场报"阻力的方向一直在抖动"。要判定这是不是"噪声 1.27σ 对 0.20 N 死区"这个机制,
//   需要两个数, 而且必须是【同一个仪器】给出的:
//     ① hapticOut (= 真正被触觉映射消费的那个量, 不是 filtered、不是 F| 帧) 在【帧率】上的逐轴 sd;
//     ② 残差越过死区 (Config::FORCE_RESIDUAL_DEADZONE_N = 0.20 N) 的【帧占比】。
//   判据写在 Docs/superpowers/plans/2026-09-23-haptic-output-jitter-instrument.md:
//     · 占比 ≈ 15~25% ⇒ 机制成立;
//     · 占比远低于 15% ⇒ 机制【不成立】, 别按机制去改死区。
//   所以这个累加器不是在"给一行比较加个测试": 它算错, 结论会【反过来】。
//
// ⚠ 喂数据的接口约定 —— 【必须按新帧喂, 不能按 1 kHz 回调节拍喂】:
//     HapticCallback 的回调是 1 kHz, 而补偿/滤波/映射是【按帧】(约 123 Hz) 跑的。
//     按回调节拍喂, 同一帧的值会被重复累加几十次 (采样保持) ⇒ sd 被【系统性压低】,
//     越死区占比也被压低, 于是"机制不成立"这个结论会凭空出现。
//     本项目在这一点上栽过一次, 是成文教训 (见 Config.h 里 FORCE_FILTER_CUTOFF 附近那段:
//     滤波器按 fs=120 算、流水线却跑在 ~11 Hz ⇒ 死区只有 1.7σ ⇒ 6.9% 的帧穿过去)。
//     ⇒ 调用侧每帧调 addFrame 一次, 判据用【帧新鲜度】而不是回调次数。
//
// ★★ 数值算法: 【Welford 在线式】(2026-09-23 Task 2 换掉原来的"两遍"):
//     每轴只留 (n, mean, M2) 三个数 —— O(1) 内存、一次遍历、无分配、无 push_back。
//     逐帧:  d = v − mean;  mean += d/n;  M2 += d·(v − mean)
//     样本 sd = sqrt(M2 / (n−1))。
//   【为什么必须换】本累加器要接在 ForcePipeline::step() 的【控制路径】上, 而"两遍"必须
//     保留全部样本 ⇒ 123 Hz 下约 30 MB/小时, 而且帧循环里的 push_back 【可能重新分配】——
//     那是没有人量过的时序特征变化。
//   【为什么不用一遍的 Σx² − n·mean²】那两个量在常量输入时几乎相等, 相减会掉有效位,
//     连"静止"都可能得到一个非 0 的 sd —— 而静止时的 sd 正是判据的分母。
//     Welford 对常量输入给【精确的 0】: 第一帧之后 d 恒为 0, M2 一步都不再变
//     (见用例 constant_sequence_sd_is_exactly_zero)。
//   ⚠【没有】加"样本数上限": 那会静默改变 sd 的含义 (窗口一满就变成"最近 N 帧的 sd")。
//     要省内存只能换算法, 不能在上限上做文章。
//
// ⚠★ 【越阈计数必须在喂帧【之前】武装】—— 这是换成 Welford 带来的接口后果, 见 setCountThreshold:
//     "两遍"能回答【任意】阈值 (调用时扫一遍留下的样本); Welford 不留样本, 占比只能
//     【边喂边数】⇒ 阈值必须在喂第一帧之前就知道。而 fracAbove(axis, thr) 是【调用时】
//     才把阈值传进来的 (那时窗口早喂完了), 于是补不回来。
//     ⇒ 所以有 setCountThreshold(thr): 调用侧在【开始喂之前】把阈值告诉它一次
//       (粘性, 跨 reset 保留 —— 阈值是"那个量"的属性, 不是"这个窗口"的属性)。
//     ⇒ 算不出来时 fracAbove 返回【−1】而不是 0 (三种情形见那个函数):
//       0 看起来像一个结论 ("没有帧越阈"), 而真相是"这个数我没法算"。
//       本项目最怕的正是"数照样印出来, 只是它是错的"。
#pragma once

#include <cmath>

struct JitterStats {                       // 只累加, 不判断
    // 清空到"一帧都没喂过"的状态。'n' 探针每次打印【之后】调用 (窗口 = 自上次 'n' 起)。
    // ⚠ 必须把三个轴的所有累加量一起清: 只把 n_ 置 0 而留着 mean_/M2_ 的话, 下一次的
    //   mean/sd 会掺进上一次的数据, 而且【从输出上看不出来】。
    // ⚠ 【不清】计数的阈值 (见 setCountThreshold): 它是"这个量"的属性, 每个窗口都一样。
    void reset() {
        n_ = 0;
        countedN_ = 0;
        for (int a = 0; a < 3; a++) {
            mean_[a] = 0.0;
            m2_[a] = 0.0;
            countAbove_[a] = 0;
        }
    }

    // 武装"越阈计数": 此后每喂一帧, 就记一下各轴 |v| >= thr 的有几帧, 供 fracAbove 用。
    // ⚠ 必须在【喂第一帧之前】调用 —— Welford 不留样本, 事后补不回来 (见文件顶上那段)。
    //   【武装晚了不会被当成 0】: 下面用 countedN_ 记账 —— 只要有帧是"武装之前"喂进来的,
    //   fracAbove 就返回 −1 (算不出来), 不会把一个假的 0% 印成结论。
    // ⚠ 阈值按【精确比较】匹配: 调用侧与 fracAbove 要用【同一个来源】的那个数
    //   (本项目是 Config::FORCE_RESIDUAL_DEADZONE_N 这一处定义), 别两处各写一个字面量。
    // 粘性: 跨 reset() 保留 (阈值不随窗口变); 重复武装会清掉已累计的计数, 因为那些计数
    //   是按旧阈值数的 —— 留着就是两次口径混在一格里。
    void setCountThreshold(double thr) {
        armed_ = true;
        countThr_ = thr;
        countedN_ = 0;
        for (int a = 0; a < 3; a++) countAbove_[a] = 0;
    }

    // 每个【新帧】调一次 (不是每次回调 —— 见文件顶上那段接口约定)。
    // v 是三轴读数, 顺序由调用侧定义 (本项目是 [0]=X, [1]=Y, [2]=Z)。
    // ⚠ O(1): 只有加减乘除与三个 double 的比较, 【没有分配、没有 push_back】——
    //   它跑在控制路径上 (ForcePipeline::step), 见文件顶上"为什么必须换 Welford"。
    void addFrame(const double v[3]) {
        n_++;
        const double inv = 1.0 / (double)n_;
        for (int a = 0; a < 3; a++) {
            const double d = v[a] - mean_[a];
            mean_[a] += d * inv;
            m2_[a] += d * (v[a] - mean_[a]);      // 注意: 用【更新后】的 mean
            if (armed_ && std::fabs(v[a]) >= countThr_) countAbove_[a]++;
        }
        if (armed_) countedN_++;
    }

    // 已累计的【帧】数。打印 sd 时必须一并打印它: 没有 N 的 sd 是半截信息
    // (本项目有成文教训), 而且 N 太小的时候 sd 本身不可信。
    int n() const { return n_; }

    double mean(int axis) const {
        if (n_ <= 0 || !axisValid(axis)) return 0.0;
        return mean_[axis];
    }

    // 样本 sd (÷ n-1)。n < 2 ⇒ 0。
    //   为什么 n=1 返回 0 而不是 NaN/Inf: 单个样本"散布"无定义, 而 0 是调用侧能安全打印的
    //   值; 但同时【n 只有 1 时别把它当结论】—— 判据要靠 n() 一起看。
    double sd(int axis) const {
        if (n_ < 2 || !axisValid(axis)) return 0.0;
        const double v = m2_[axis] / (double)(n_ - 1);
        return (v > 0.0) ? std::sqrt(v) : 0.0;   // 负的 M2 只可能是 -0.0 那一档, 别开根号
    }

    // |v[axis]| >= thr 的【帧占比】。
    //   取绝对值: 死区是两侧对称的, 方向不分正负 (现场报的是"方向在抖动", 正负两侧同样算越)。
    //   边界【算越】(用的是 >=): 与"越过死区"这个说法的约定一致, 并与 Config.h 里
    //   FORCE_RESIDUAL_DEADZONE_N 那条注释记下的实测口径对齐。
    //   thr 由调用侧传入而【不在这里写死】: 死区只有 Config.h 一处定义, 而且阈值一变,
    //   这里若写死就会静默地换含义 (本项目有成文教训)。
    // ⚠ 返回值 −1 【不是】"0%": 它表示"这个阈值我没法算" —— 三种情形:
    //   · 没武装过阈值 (调用侧没调 setCountThreshold);
    //   · 问了一个与已武装的不同的阈值 (阈值按精确比较匹配);
    //   · 有帧是【武装之前】就喂进来的 (countedN_ != n_: 那些帧没人替它们数)。
    //   这些必须与 0% 长得不一样, 否则"没量到"会被读成"机制不成立" (见文件顶上那段)。
    //   n()==0 ⇒ 0 (空集, 什么都没喂过, 出不了任何结论; 越界轴同理, 只为避免 UB)。
    double fracAbove(int axis, double thr) const {
        if (!axisValid(axis)) return 0.0;
        if (n_ <= 0) return 0.0;
        if (!armed_ || thr != countThr_) return -1.0;
        if (countedN_ != n_) return -1.0;
        return (double)countAbove_[axis] / (double)n_;
    }

private:
    // 越界轴【只为避免 UB】: 调用侧是 for (a = 0; a < 3; a++) 的循环, 正常不会传越界值。
    // 越界读在诊断仪器里表现为随机数或崩溃, 比返回 0 难查得多。
    static bool axisValid(int axis) { return axis >= 0 && axis < 3; }

    int n_ = 0;
    // Welford 的三件套, 逐轴一份: 均值、Σ(x−mean)² 的在线累积。见文件顶上那段。
    double mean_[3] = {0.0, 0.0, 0.0};
    double m2_[3] = {0.0, 0.0, 0.0};
    // 越阈计数 —— 只在 armed_ 为真时逐帧累加 (阈值见 countThr_)。
    int countAbove_[3] = {0, 0, 0};
    int countedN_ = 0;          // 计数【真的看着】的帧数: 与 n_ 不等 ⇒ 占比算不出来 (见 fracAbove)
    double countThr_ = 0.0;
    bool armed_ = false;
};
