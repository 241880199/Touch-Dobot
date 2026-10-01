# -*- coding: utf-8 -*-
"""力-频率扫描（'r' 键回放）的【分段】分析。

用途：`force_wave.csv` 里混着多个块，本脚本
  ① 按 t_us 把所有【有运动】的块拼成一条时间轴；
  ② 用 `ff` 列的 1->0 翻转当锚点定出扫描的时刻起点 T0（翻转 = 第 5 段开始 = T0+40s）；
  ③ 按 10 秒一段把帧归到 8 个段（0.5/1/2/4 × FF 开/关）；
  ④ 每段算：实测主频（去趋势 + 加窗 DFT 峰）、幅度、加速度、力的波动、归一化力；
  ⑤ 打印 FF 开/关逐段对照，并对 F ∝ A^p · f^q 做最小二乘。

⚠ 判据是【先写死】的（见 evidence/2026-09-29-on-machine-session.md:621-642）：
   归一化力（= F / (A·(2πf)²)）恒定 ⇒ 惯性；随频率下降 ⇒ 结构振动；与频率无关 ⇒ 噪声源。

⚠ 只读，不改任何东西。用法: python _analyze_sweep_segments.py [csv]
"""
import math
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave.csv"

I_ACT_X, I_ACT_Y = 13, 14
I_FILT = (4, 5, 6)
I_FF = 22
SEG_SEC = 10.0
# ⚠★ 档数是**入参**（第 2 个命令行参数，默认 5）。2026-10-01 踩过：档数 4→5（8 段→10 段）时
#   下面的锚点仍写 40 ⇒ **整轮错位一段**（每段实测频率都成了名义值的 2 倍）。
#   ⇒ 现在全部由 NSPEED 推出来，并且**表打完之后会自检**（实测/名义若系统性对不上就出声）。
ALL_SPEEDS = (0.5, 1.0, 2.0, 4.0, 6.0)       # SweepPlan::kSpeed（历史顺序，别改）
NSPEED = int(sys.argv[2]) if len(sys.argv) > 2 else 5
SPEEDS = ALL_SPEEDS[:NSPEED]
SEG_COUNT = len(SPEEDS) * 2                  # SweepPlan::kSegCount（每档 FF 开/关各一次）
# 锚点 = 「FF 从 1 变 0」那一刻 = 后半的第一段开始 = 前半的【总时长】。
FF_HALF_SEC = (SEG_COUNT // 2) * SEG_SEC
F0_HZ = 0.365          # SweepWaveform::kF0Hz（名义值，只用于对照）


def read_blocks(path):
    out, cur = [], None
    for line in open(path, "r", encoding="utf-8", errors="replace"):
        line = line.rstrip("\r\n")
        if line.startswith("# wave"):
            cur = []
            out.append((line, cur))
        elif line.startswith("#") or not line or cur is None:
            continue
        else:
            cur.append([float(x) for x in line.split(",")])
    return out


def sd(v):
    n = len(v)
    if n < 2:
        return 0.0
    m = sum(v) / n
    return math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))


def peak_freq(y, dt, lo=0.05, hi=10.0):
    """去线性趋势 + Hann 窗后的 DFT 峰（Hz）。dt = 采样间隔(秒)。"""
    n = len(y)
    if n < 32:
        return 0.0
    xm = (n - 1) / 2.0
    ym = sum(y) / n
    sxy = sum((i - xm) * (y[i] - ym) for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    b = sxy / sxx if sxx else 0.0
    a = ym - b * xm
    d = [(y[i] - (a + b * i)) * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)))
         for i in range(n)]
    best, f = None, lo
    while f <= hi:
        w = 2 * math.pi * f * dt
        re = im = 0.0
        for i in range(n):
            re += d[i] * math.cos(w * i)
            im += d[i] * math.sin(w * i)
        p = re * re + im * im
        if best is None or p > best[1]:
            best = (f, p)
        f += 0.005
    return best[0]


def main():
    blocks = read_blocks(CSV)
    moving = []
    print(f"文件: {CSV}")
    print(f"块数: {len(blocks)}")
    for head, rows in blocks:
        if not rows:
            continue
        ax = [r[I_ACT_X] for r in rows]
        s = sd(ax)
        ff = sum(r[I_FF] for r in rows) / len(rows)
        tag = "运动" if s >= 1.0 else "静止"
        print(f"   {head.strip()[:62]:<64} sd(act_x)={s:7.3f}  mean(ff)={ff:4.2f}  [{tag}]")
        if s >= 1.0:
            moving += [(r[0], r) for r in rows]
    if not moving:
        print("!! 没有找到运动块 —— 是不是没跑扫描？")
        return
    moving.sort(key=lambda x: x[0])
    # ⚠★ 按 t_us 去重：同一段里按两次 'n' 时，两个窗口【会重叠】⇒ 同一帧出现两次
    #   ⇒ 重复计数会把 n、sd、主频全都带偏。帧带绝对微秒时间戳，按它去重是可靠的。
    _seen = set()
    _dedup = []
    for t, r in moving:
        if t in _seen:
            continue
        _seen.add(t)
        _dedup.append((t, r))
    if len(_dedup) != len(moving):
        print(f"（按 t_us 去重: {len(moving)} → {len(_dedup)} 帧，丢掉了 {len(moving)-len(_dedup)} 个重复）")
    moving = _dedup
    span = (moving[-1][0] - moving[0][0]) / 1e6
    print(f"\n运动帧 {len(moving)}，跨度 {span:.1f}s")

    # 找【所有】ff 1->0 翻转 —— 每个 = 那一轮的「第 5 段开始」= T0+40s。
    # ⚠ 两轮之间至少隔 60s，避免把同一轮里的抖动当成新轮。
    anchors = []
    for i in range(1, len(moving)):
        if moving[i - 1][1][I_FF] == 1.0 and moving[i][1][I_FF] == 0.0:
            t = moving[i][0]
            if not anchors or t - anchors[-1] > int(60e6):
                anchors.append(t)
    if not anchors:
        print("!! 没找到 ff 的 1->0 翻转 —— 无法定出段的时间轴。")
        print("   可能原因: 扫描没跑完 / 只跑了前四段 / FF 没被切换。如实记，别猜。")
        return
    print(f"锚点（ff 1->0）: {len(anchors)} 个 ⇒ 判定为 {len(anchors)} 轮")

    for rd, at in enumerate(anchors, 1):
        t0 = at - int(FF_HALF_SEC * 1e6)
        segs = {}
        for t, r in moving:
            s = int((t - t0) // int(SEG_SEC * 1e6))
            if 0 <= s < SEG_COUNT:
                segs.setdefault(s, []).append(r)
        if segs:
            analyze_round(rd, t0, segs)


def analyze_round(rd, t0, segs):
    print()
    print(f"########## 第 {rd} 轮（T0={t0:.0f}）##########")
    hdr = (f"{'段':>2} {'倍速':>5} {'FF':>4} {'n':>5} {'实测f':>7} {'名义f':>7} "
           f"{'A(mm)':>7} {'a(m/s2)':>8} {'sd|F|':>7} {'归一化':>9}")
    print(hdr)
    print("-" * len(hdr))
    res = []
    for s in range(SEG_COUNT):
        rs = segs.get(s, [])
        if not rs:
            print(f"{s+1:>2}  (无帧)")
            continue
        sp = SPEEDS[s % len(SPEEDS)]
        on = s < SEG_COUNT // 2
        dt = 1.0 / 122.7
        f = peak_freq([r[I_ACT_X] for r in rs], dt)
        A = math.sqrt(sd([r[I_ACT_X] for r in rs]) ** 2 +
                      sd([r[I_ACT_Y] for r in rs]) ** 2) / 1000.0
        acc = A * (2 * math.pi * f) ** 2
        # ⚠ F 的口径: 力的【模长】的 sd（补偿后的落盘口 filt_*）
        F = sd([math.sqrt(sum(r[i] ** 2 for i in I_FILT)) for r in rs])
        norm = F / acc if acc > 1e-9 else float("nan")
        res.append(dict(seg=s + 1, sp=sp, on=on, f=f, A=A * 1000, acc=acc, F=F, norm=norm))
        # 机器可读的一行（供跨轮合并拟合用；不同轮可以有不同的幅度）
        print(f"POINT,{rd},{s+1},{sp},{'ON' if on else 'OFF'},{f:.4f},{A*1000:.3f},{F:.5f}")
        print(f"{s+1:>2} {sp:>5} {'ON' if on else 'OFF':>4} {len(rs):>5} "
              f"{f:>7.3f} {F0_HZ*sp:>7.3f} {A*1000:>7.2f} {acc:>8.3f} {F:>7.4f} {norm:>9.4f}")

    # ★★ 自检：实测频率应当 ≈ 名义频率。若系统性对不上（例如整体差 2 倍），
    #    说明段的时间轴对错了（最常见的成因是【档数变了而锚点没跟着变】）⇒ 出声。
    ratios = [p["f"] / (F0_HZ * p["sp"]) for p in res if p["f"] > 0 and p["sp"] > 0]
    if ratios:
        ratios.sort()
        med = ratios[len(ratios) // 2]
        print(f"\n自检: 实测/名义 频率比的中位数 = {med:.3f}"
              f"（范围 {ratios[0]:.2f}~{ratios[-1]:.2f}）")
        if not (0.7 <= med <= 1.4):
            print("  ⛔⛔ 系统性对不上 ⇒ **段的时间轴几乎肯定对错了**（整轮错位）!")
            print("     ⇒ 先怀疑【档数】：本脚本第 2 个参数 = 本轮的倍速档数"
                  f"（本次按 {NSPEED} 档算）。8 段时代是 4 档, 10 段是 5 档。")
            print("     ⚠ 下面这张表**不可用**，别照着读。")

    print("\n=== FF 开/关 逐段对照（同倍速的两段【动作应逐帧相同】）===")
    print("   ★ 这一段回答的是“关掉力反馈，这个力还在不在”")
    for i in range(len(SPEEDS)):
        a = next((r for r in res if r["seg"] == i + 1), None)
        b = next((r for r in res if r["seg"] == i + 1 + len(SPEEDS)), None)
        if not a or not b:
            continue
        print(f"   {a['sp']:>4}x  f: {a['f']:.3f} vs {b['f']:.3f}   "
              f"A: {a['A']:.2f} vs {b['A']:.2f} mm   "
              f"sd|F|: {a['F']:.4f} vs {b['F']:.4f}  ({b['F']/a['F']:.2f}x)")

    print("\n=== 判据：归一化力 = F/(A·(2πf)²) ===")
    print("   恒定⇒惯性 · 随频率下降⇒结构振动 · 与频率无关⇒噪声源")
    pts = [r for r in res if r["F"] > 0 and r["f"] > 0 and r["A"] > 0]
    if len(pts) >= 3:
        n = len(pts)
        X = [[1.0, math.log(p["f"]), math.log(p["A"] * 1e-3)] for p in pts]
        Y = [math.log(p["F"]) for p in pts]
        M = [[sum(X[i][a] * X[i][b] for i in range(n)) for b in range(3)] for a in range(3)]
        V = [sum(X[i][a] * Y[i] for i in range(n)) for a in range(3)]
        ok = True
        for i in range(3):
            p = max(range(i, 3), key=lambda r: abs(M[r][i]))
            M[i], M[p] = M[p], M[i]
            V[i], V[p] = V[p], V[i]
            if abs(M[i][i]) < 1e-12:
                ok = False
                break
            for r in range(i + 1, 3):
                k = M[r][i] / M[i][i]
                for c in range(i, 3):
                    M[r][c] -= k * M[i][c]
                V[r] -= k * V[i]
        if ok:
            sol = [0.0] * 3
            for i in (2, 1, 0):
                sol[i] = (V[i] - sum(M[i][c] * sol[c] for c in range(i + 1, 3))) / M[i][i]
            print(f"   F ∝ A^{sol[2]:.2f} · f^{sol[1]:.2f}     (纯惯性要求 f^2, A^1)")
            amin = min(p["A"] for p in pts)
            amax = max(p["A"] for p in pts)
            if amax / amin < 1.2:
                print(f"   ⚠ 幅度几乎不变（{amin:.2f}~{amax:.2f} mm, 仅 {amax/amin:.2f}x）"
                      f" ⇒ **A 的指数不可辨识**，只有 f 的指数有意义。")
            else:
                print(f"   （幅度跨 {amax/amin:.2f}x，A 的指数可辨识）")
    fmin = min(p["f"] for p in pts)
    print(f"   ⚠ 最低频 {fmin:.3f} Hz 的窗口里只有约 "
          f"{fmin*8.3:.1f} 个周期 ⇒ 该点的 f 不确定度约 ±{1/8.3:.2f} Hz"
          f"（判据按 (2πf)² 归一 ⇒ 低端最不可靠）。")


if __name__ == "__main__":
    main()
