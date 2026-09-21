// 启动零偏漂移检查的【判定】部分 —— 纯函数, 头文件内联。
//
// 为什么抽出来 (2026-09-21, Task 5):
//   这段判定原先整个长在 main.cpp 的 runZeroDriftCheck() 里, 是 file-static, 由主循环用
//   【真实时间】(GetTickCount) 驱动, 并且直接读 appState.forceData 与
//   ForceCompensation::guardState() —— 没有可注入点, 所以它【一个分支都测不到】。
//   偏偏它又是"等一致性闸门放行才做事"的检查, 而闸门此前一直拒绝 ⇒ 它至今跑的全是
//   【未做】那一支。闸门修好之后, 「放行 -> 正常」与「放行 -> 超阈」这两条路会【首次】
//   在现场执行 —— 一条从未跑过、又没有用例的路径, 风险不该只靠现场。
//
// 边界划在哪 (为什么这样切):
//   · 本文件只管【拿现有状态判定该说什么】。不读时钟、不读任何全局、不写盘、不打印。
//   · 时间 (GetTickCount) 与全部 module-static 累计量留在调用侧 main.cpp。
//   · "这检查该不该跑 / 这帧该不该采样"也留在调用侧 (有没有存储零偏、是不是 --no-robot、
//     读数是否 stale、是否已定稿) —— 那些是【状态】不是【判定】, 且要在调用侧副作用里做。
//   · 三个旋钮 (阈值 / 等待期 / 最少样本数) 由调用侧传入, 不在这里写死: 否则阈值一变,
//     用例里走哪一支就跟着变, 断言会静默地换含义。阈值本身仍只有 Config.h 一处定义。
// 于是同一份实现既被 main.cpp 编译, 也被 tests/test_force_compensation.cpp 直接调用
// (头文件内联的【唯一一份定义】, 不是测试里的复制品)。
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

#include "ForceCompensation.h"

namespace ZeroDriftCheck {

enum class Outcome {
    // 闸门还在拒绝、但没等满等待期: 【不是结论】, 继续等 (调用侧重开累计窗口)。
    Waiting = 0,
    // 闸门放行, 三轴均值的模 <= 阈值。
    Normal = 1,
    // 闸门放行, 三轴均值的模 > 阈值。(严格大于 —— 与抽出前的判据一致。)
    OverThreshold = 2,
    // 闸门从第一次拒绝起满等待期仍不放行 -> 【未做】: 明说本次没查, 不给任何漂移数。
    NotDone = 3,
    // 闸门放行了, 但能用的样本太少 -> 【样本不足】: 明说本次不作结论。
    InsufficientSamples = 4
};

struct Input {
    // 闸门当前状态。非 OK 时 mean 与 sampleCount 没有意义 (读数是被闸门置零的)。
    ForceCompensation::GuardState guard = ForceCompensation::GuardState::UNCALIBRATED;
    // 补偿后读数 (fd.filtered) 的三轴【均值】, 不是累计和。
    double mean[3] = {0.0, 0.0, 0.0};
    // 从【第一次被拒】那一刻起算的时长 (ms)。用累计窗口起点当等待起点的话, 每次拒绝都重开
    // 窗口, 这个截止就永远到不了 —— 所以调用侧维护的是两个时钟, 这里只要一个数。
    unsigned long refuseElapsedMs = 0;
    int sampleCount = 0;
    double thresholdN = 0.0;    // 告警阈值 (N)
    unsigned long waitMs = 0;   // 闸门拒绝时的最长等待 (ms)
    int minSamples = 0;         // 出结论所需的最少样本数
};

struct Decision {
    Outcome outcome = Outcome::Waiting;
    // 本函数【自己算出来】的三轴模 (N)。只在放行且样本够的分支上有值, 其余分支恒为 0 ——
    // 拒绝期间 compensated 全是闸门置的 0, 拿它算出来的"漂移"恒为 0, 那个数不能当结论报。
    double driftN = 0.0;
    // 该说的话 (可含换行)。Waiting 时为空: 还在等不是结论, 一声不吭是旧行为, 保持不变。
    std::string text;
};

// 三轴模。与 main.cpp 抽出前那句 sqrt(x²+y²+z²) 逐位等价。
inline double norm3(const double v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 数值 -> 文字, 用 %g。旧的 std::cout << double 走的是默认格式 (精度 6、%g 的定点/科学
// 计数法切换点), %g 与它一致 ⇒ 抽出来之后同一组数印出来的字不变。
inline std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return std::string(buf);
}

inline std::string num(int v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d", v);
    return std::string(buf);
}

inline Decision decide(const Input& in) {
    Decision d;

    // ---- 闸门在拒绝: 读数被置零, 此刻量不到零偏。【不装作查过】。----
    if (in.guard != ForceCompensation::GuardState::OK) {
        if (in.refuseElapsedMs < in.waitMs) {
            // 还没等满: 继续等, 这一次不出声 (调用侧重开累计窗口)。
            d.outcome = Outcome::Waiting;
            return d;
        }
        d.outcome = Outcome::NotDone;
        const unsigned long secs = in.refuseElapsedMs / 1000;
        // ★ 2026-09-21 (Task 7): 闸门拒绝的原因【不止两种】—— 第三种是"参考量不可用"。
        //   这一段从前说的是"那一段分得开'没有可用模型'与'有模型但对不上'", 而下面那句
        //   ⚠ 又只讲"对不上"的一种来源 —— 参考量不可用时那两句都成了误导: 那一刻
        //   【根本没有第二个读数】, 拿"对不上"的处置 (查下发 / 查零偏) 去忙是白忙。
        //   ⇒ 原因不同, 该说的话就不同 (判据各分支的处置本来就不一样, 见 ForceCompensation.h
        //     的三种拒绝原因)。三种状态【共用一个 outcome】: 结论都是"本次没查"。
        const char* causeLine = nullptr;
        switch (in.guard) {
            case ForceCompensation::GuardState::REFERENCE_UNAVAILABLE:
                causeLine =
                    "[Force]   ⚠ 本次\"没查\"的原因【不是】模型、也【不是】负载参数:"
                    " 判据的参考量这一路【没有数据】(六维力在线状态不是在线 / 帧已陈旧)\n"
                    "[Force]     ⇒ 两边【没有比过】, 逐通道表也没有。要去查的是这一路的数据"
                    "(30004 帧、六维力在线状态), 不是去重标、也不是去查下发。\n";
                break;
            case ForceCompensation::GuardState::UNCALIBRATED:
                causeLine =
                    "[Force]   ⚠ 本次\"没查\"的原因【不是】负载参数: 【没有可用模型】"
                    "(未标定 / A 全零 / A 数值退化) ⇒ 连要比的模型都没有。\n"
                    "[Force]     先按 'm' 采多姿态 -> 's' 解出 A, 再按 'z' 调零。\n";
                break;
            default:   // INCONSISTENT —— 只有这一支是"两边都读到了数, 但对不上"
                causeLine =
                    "[Force]   ⚠ '对不上'不止'负载参数没发进机械臂'一种来源: 零偏漂到"
                    "超出容差同样会让两边对不上 (上面那段里的逐通道表\n"
                    "[Force]     写着是哪些通道超了限)。别只查下发那一处。\n";
                break;
        }
        d.text =
            "[Force] 零偏漂移检查: 【未做】—— 一致性闸门从第一次拒绝起已 "
            + num((double)secs) +
            " s 一直在拒绝 (原因见上面 \"[Force] !!\" 那一段; 那一段把【没有可用模型】与"
            "【参考量不可用】与【有模型但对不上】三种原因分开报, 处置各不相同)。\n"
            "[Force]   闸门拒绝时 compensated (以及由它推出来的 filtered) 是全 0,"
            " 0 不是零偏 —— 拿它算出来的\"漂移\"恒为 0, 所以本检查在拒绝期间"
            " 给不出任何结论。\n"
            + causeLine +
            "[Force]   闸门放行之后重启本程序即可 (本检查是启动时的一次性检查)。";
        return d;
    }

    // ---- 闸门放行, 但样本太少: ★ 必须出声 ----
    // 这里曾经是【静默】的 (设完 done 就 return, 既不报结论、也不报"没查"), 于是
    // "查了、没发现问题" 与 "根本没查" 在输出上分不开 —— 与本项目最怕的"安静地错"同类。
    // 用户指令 (2026-09-21): 改成明说。这一支也【不】给漂移数 (样本不够, 给不出)。
    if (in.sampleCount < in.minSamples) {
        d.outcome = Outcome::InsufficientSamples;
        d.text =
            "[Force] 零偏漂移检查: 【样本不足】—— 闸门放行后只累计到 " + num(in.sampleCount) +
            " 个样本 (至少需要 " + num(in.minSamples) +
            " 个), 样本不足, 本次不作结论。\n"
            "[Force]   这里【不】打印\"正常\": 样本不够时,\"查了没发现问题\"与\"根本没查\""
            "在输出上必须分得开。闸门是刚放行的, 能用的样本还没攒够 —— 下次启动会再查一次。";
        return d;
    }

    // ---- 闸门放行且样本够: 出结论 ----
    d.driftN = norm3(in.mean);
    if (d.driftN > in.thresholdN) {
        d.outcome = Outcome::OverThreshold;
        d.text =
            "\n[Force] ⚠ 零偏漂移检查: 补偿后读数 " + num(d.driftN) +
            " N, 超过阈值 " + num(in.thresholdN) + " N\n"
            "[Force]   两种可能:\n"
            "[Force]     · 零偏漂了 (温度/时间)  -> 按 'z' 重新调零\n"
            "[Force]     · 硬件有变化 (加装/拆装) -> 按 'm' 采多姿态后 's' 重标负载\n"
            "[Force]   两种都不影响继续操作, 但建议尽快处理。";
    } else {
        d.outcome = Outcome::Normal;
        d.text =
            "[Force] 零偏漂移检查: 补偿后读数 " + num(d.driftN) +
            " N, 正常 (< " + num(in.thresholdN) + " N)";
    }
    return d;
}

}   // namespace ZeroDriftCheck
