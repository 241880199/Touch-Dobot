#pragma once
#include <cmath>

// ===== 帧率噪声探针的【统计部分】(2026-09-21) =====
//
// 【它要回答的问题】(一句话)
//   这条流水线【当时】跑在 ~11 Hz 上 (实测 pollForce 间隔均值 92 ms), 而 30004 帧是 8 ms/125 Hz
//   (厂商文档 Docs/机械臂资料/TCP_IP远程控制接口文档.md:63)。滤波器的系数又是按 fs=120 Hz 算的。
//   (★ 2026-09-21 当天就按本探针的读数把流水线挪到了帧率上 —— 见 RelayCore::forceReaderThread
//    与 Config::FORCE_FILTER_CUTOFF。本模块的用途不变: 它是那件事的【判据来源】。)
//   ⇒ 噪声只能靠【过采样 + 平均】压下去, 而我们现在根本没有过采样。
//   【判决就一个数】把连续 k 个原始样本取平均, 噪声的 sd 掉多少?
//     · 掉成 1/sqrt(k)  ⇒ 样本间独立 (宽带噪声) ⇒ 把流水线挪到帧率上跑就能【白赚】√N;
//     · 基本不掉       ⇒ 噪声落在比 k 个采样更慢的频带上 (机械抖动/带内) ⇒ 换思路, 别指望平均。
//
// 【为什么这些统计量要单独抽出来 + 必须带单测】
//   这类函数的错法【很安静】: 一个写错的块平均会给出"平均之后噪声不掉"的结论 —— 而那正好
//   会把人引到错误的修法上 (放弃平均、去改机械, 或反过来白改一通滤波)。
//   ⚠ 本项目刚刚栽过一次"仪器本身出错": 2026-09-21 的 comp 窗口均值被"采样保持"污染
//     (comp 由 pollForce 每 92 ms 刷新, 而我的去重键是每帧盖的), 我据此报了一个【不存在的
//     隐藏项】。仪器也要有判据。
//   ⇒ 单测拿【构造信号】钉死: 白噪声必须按 1/sqrt(k) 掉, 慢带限信号必须【不】掉。
//     谁把公式改错, 这两条会立刻红。
namespace NoiseProbe {

// 均值与【样本】标准差 (除以 n-1, 与别处的 sampleVar 同一口径)。n < 2 ⇒ sd = 0 (无方差可言)。
// 两遍算法 (先求均值再求偏差平方): 这里的量级不需要差式的速度, 而两遍法没有相消问题。
inline void meanSd(const double* x, int n, double* mean, double* sd) {
    *mean = 0.0;
    *sd   = 0.0;
    if (n <= 0) return;
    double s = 0.0;
    for (int i = 0; i < n; i++) s += x[i];
    *mean = s / (double)n;
    if (n < 2) return;
    double v = 0.0;
    for (int i = 0; i < n; i++) { const double d = x[i] - *mean; v += d * d; }
    *sd = std::sqrt(v / (double)(n - 1));
}

// 块平均: 把 x 按【连续 k 个】分块, 每块取平均, 返回这些块均值的 sd 与块数。
// 【这就是探针要的那个判决】: 样本间独立时块均值的 sd = 原 sd / sqrt(k);
//   噪声相关时间 ≫ k 个采样时几乎不掉。
// ⚠ 只用【完整的块】: 尾部不足 k 个的一律丢弃。用不完整块会把尾块的方差算得比别的块小,
//   把 sd 往下拉 —— 那正是"仪器自己造假", 而这个数正是要被判读的那个数。
// ⚠ 先减去全局均值再累加: 原始 @1304 的 z 在 +4.6 N 上下, 而要看的变化是 0.1 N 量级。
//   不中心化的话 Σm² − nb·mean² 这种差式会白扔约 4 位有效数字。
inline void blockSd(const double* x, int n, int k, double* sd, int* blocks) {
    *sd     = 0.0;
    *blocks = 0;
    if (n <= 0 || k <= 0) return;
    const int nb = n / k;
    *blocks = nb;
    if (k == 1) {                      // 退化成"单个样本", 由调用方走 meanSd 更省事
        double m;
        meanSd(x, n, &m, sd);
        return;
    }
    if (nb < 2) return;                // 块数 < 2 ⇒ 无方差可言
    double g = 0.0;
    for (int i = 0; i < n; i++) g += x[i];
    g /= (double)n;
    double s = 0.0, sq = 0.0;
    for (int b = 0; b < nb; b++) {
        double t = 0.0;
        for (int i = 0; i < k; i++) t += x[b * k + i] - g;
        const double m = t / (double)k;
        s  += m;
        sq += m * m;
    }
    const double mean = s / (double)nb;
    double v = sq - (double)nb * mean * mean;
    v /= (double)(nb - 1);
    if (v > 0.0) *sd = std::sqrt(v);
}

// 自相关 (归一化到 lag=0 的总平方和)。
// 【为什么要报它】它给出"噪声的相关时间" = 平均几个采样才真正独立 —— 没有它, 块平均那条
//   曲线陡不陡只能猜。k 个【独立】样本才谈得上 1/sqrt(k); 若 lag=1 就已经掉到 0, 说明
//   一个采样就是一个独立样本, 过采样立刻有收益。
// ⚠ 分母用【全部 n 个】的总平方和、分子只累加 (n−lag) 项 (未做 (n−lag) 偏差修正): 这是刻意的,
//   本量只作【定性判读】(高/低/相关时间量级), 不参与任何判决门限。别拿它当严格统计量用。
inline double autocorr(const double* x, int n, int lag) {
    if (n <= 1 || lag <= 0 || lag >= n) return 0.0;
    double g = 0.0;
    for (int i = 0; i < n; i++) g += x[i];
    g /= (double)n;
    double c = 0.0, v = 0.0;
    for (int i = 0; i + lag < n; i++) c += (x[i] - g) * (x[i + lag] - g);
    for (int i = 0; i < n; i++) v += (x[i] - g) * (x[i] - g);
    return (v > 0.0) ? c / v : 0.0;
}

// 理论下降倍数 1/sqrt(k) —— 打印时与实测并排, 一眼看出掉没掉。
inline double idealBlockRatio(int k) {
    return (k > 0) ? 1.0 / std::sqrt((double)k) : 1.0;
}

} // namespace NoiseProbe
