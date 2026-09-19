#pragma once

// 重复姿态对的【登记规则】—— 采集侧 (main.cpp 的 BiasCheck) 里唯一一处有逻辑的判断,
// 单独抽成纯函数好单测 (test_payload_calibration.cpp 的 test_repeat_pair_registration)。
//
// 【协议: 原地复采】(2026-09-19 修订)
//   摆姿态 → SPACE 采样 → ★【保持不动】→ 按 'r' → 再按 SPACE 采一次。
//   'r' 把【紧接着的下一次采样】登记为【上一次采样那个姿态】的重复访问, 即一对 =
//   (上一笔的行号, 这一笔的行号) —— 两次采样之间机械臂不移动。
//
// 【为什么不是"回到第 1 个姿态再采一次"】(旧协议, 已废)
//   姿态是【手拖】出来的, 拖不出两次一样的位姿, 旧协议在实机上根本执行不了。而它坏在
//   危险的那一侧: 一对的两次访问落在不同位姿上时, 差值里混进两个位姿之间的重力差
//   (零点几 N, 对着 0.0224 N 的残差), σ_rep 被抬到 ~0.5 N, χ²/dof 落到 ~0.002, 门限
//   随之放宽到【无条件放行】—— 一把永远通过的尺子, 正是本模块要消灭的"绿色但不携带信息"。
//   原地复采与"拖不回去"无关: 实机日志里保持不动的区间 0/16 段发生姿态漂移。
//
// 纯函数: 不碰机械臂状态、不碰传感器、不碰任何全局量。
namespace RepeatPairRegistry {

    enum Status {
        OK = 0,        // 登记成功, first / second 有效
        NO_PREVIOUS,   // 没有"上一笔"可复采 (还没采过, 或刚被 reset 掉了整批)
        LIMIT          // 已达对数上限 —— 再多也不会更准
    };

    // prev   = 'r' 之前那一笔的行号 (没有则 < 0)
    // cur    = 紧接着这一笔的行号
    // n      = 已登记的对数
    // maxPairs = 对数上限
    // 成功时写出 *first = prev, *second = cur; 失败时两个指针都不动 (调用方不必先清)。
    inline Status registerPair(int prev, int cur, int n, int maxPairs,
                               int* first, int* second) {
        // prev < 0 是实况 (还没采过 / 刚开新一批); prev >= cur 在采集流程里走不到,
        // 但它同样是"没有一个【在先的】样本可配", 归到同一类拒绝, 不另立名目。
        if (prev < 0 || prev >= cur) return NO_PREVIOUS;
        if (n >= maxPairs) return LIMIT;
        if (first)  *first  = prev;
        if (second) *second = cur;
        return OK;
    }

} // namespace RepeatPairRegistry
