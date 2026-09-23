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
// 数值算法: 两遍 (先求均值, 再求偏差平方和), 样本 sd = sqrt(Σ(x-mean)² / (n-1))。
//   不用 Σx² - n·mean² 那个一遍写法: 那两个量在常量输入时几乎相等, 相减会掉有效位,
//   连"静止"都可能得到一个非 0 的 sd —— 而静止时的 sd 正是判据的分母。
//   两遍写法对常量输入给【精确的 0】(见用例 constant_sequence_sd_is_exactly_zero)。
//
// ⚠ 代价 (Task 2 接线前要知情): 两遍算法要留下原始样本, 所以 addFrame 会增长内存
//   (每帧 3 个 double) 且可能重新分配。'n' 探针按一下 reset 一次, 正常使用是"按分钟"的窗口;
//   但若不按 'n' 就一直喂, 内存【无上界】。这是"两遍"换来的, 不是疏忽 ——
//   若要长时间无人值守地跑, 该换的是算法 (Welford 在线式, O(1) 内存), 而不是在这里加个上限。
#pragma once

#include <cmath>
#include <vector>

struct JitterStats {                       // 只累加, 不判断
    // 清空到"一帧都没喂过"的状态。'n' 探针每次打印前调用。
    // ⚠ 必须连 [0, n) 的样本一起清: 只把 n_ 置 0 而留着样本的话, 下一次的 mean/sd
    //   会掺进上一次的数据, 而且【从输出上看不出来】。
    void reset() {
        n_ = 0;
        for (int a = 0; a < 3; a++) samples_[a].clear();
    }

    // 每个【新帧】调一次 (不是每次回调 —— 见文件顶上那段接口约定)。
    // v 是三轴读数, 顺序由调用侧定义 (本项目是 [0]=X, [1]=Y, [2]=Z)。
    void addFrame(const double v[3]) {
        for (int a = 0; a < 3; a++) samples_[a].push_back(v[a]);
        n_++;
    }

    // 已累计的【帧】数。打印 sd 时必须一并打印它: 没有 N 的 sd 是半截信息
    // (本项目有成文教训), 而且 N 太小的时候 sd 本身不可信。
    int n() const { return n_; }

    double mean(int axis) const {
        if (n_ <= 0 || !axisValid(axis)) return 0.0;
        double sum = 0.0;
        const std::vector<double>& s = samples_[axis];
        for (size_t i = 0; i < s.size(); i++) sum += s[i];
        return sum / (double)n_;
    }

    // 样本 sd (÷ n-1)。n < 2 ⇒ 0。
    //   为什么 n=1 返回 0 而不是 NaN/Inf: 单个样本"散布"无定义, 而 0 是调用侧能安全打印的
    //   值; 但同时【n 只有 1 时别把它当结论】—— 判据要靠 n() 一起看。
    double sd(int axis) const {
        if (n_ < 2 || !axisValid(axis)) return 0.0;
        const double m = mean(axis);              // 第一遍: 均值
        double ss = 0.0;                          // 第二遍: 偏差平方和
        const std::vector<double>& s = samples_[axis];
        for (size_t i = 0; i < s.size(); i++) {
            const double d = s[i] - m;
            ss += d * d;
        }
        return std::sqrt(ss / (double)(n_ - 1));
    }

    // |v[axis]| >= thr 的【帧占比】。0 帧 ⇒ 0 (不除零)。
    //   取绝对值: 死区是两侧对称的, 方向不分正负 (现场报的是"方向在抖动", 正负两侧同样算越)。
    //   边界【算越】(用的是 >=): 与"越过死区"这个说法的约定一致, 并与 Config.h 里
    //   FORCE_RESIDUAL_DEADZONE_N 那条注释记下的实测口径对齐。
    //   thr 由调用侧传入而【不在这里写死】: 死区只有 Config.h 一处定义, 而且阈值一变,
    //   这里若写死就会静默地换含义 (本项目有成文教训)。
    double fracAbove(int axis, double thr) const {
        if (n_ <= 0 || !axisValid(axis)) return 0.0;
        int count = 0;
        const std::vector<double>& s = samples_[axis];
        for (size_t i = 0; i < s.size(); i++) {
            if (std::fabs(s[i]) >= thr) count++;
        }
        return (double)count / (double)n_;
    }

private:
    // 越界轴【只为避免 UB】: 调用侧是 for (a = 0; a < 3; a++) 的循环, 正常不会传越界值。
    // 直接 samples_[axis] 越界读在诊断仪器里表现为随机数或崩溃, 比返回 0 难查得多。
    static bool axisValid(int axis) { return axis >= 0 && axis < 3; }

    int n_ = 0;
    // 三轴各留一份原始样本 —— "两遍"的前提。见文件顶上关于内存代价的那段。
    std::vector<double> samples_[3];
};
