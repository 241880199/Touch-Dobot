# -*- coding: utf-8 -*-
"""力-频率扫描：把力拆成【相干分量】（锁在驱动频率上）与【宽带分量】。

【为什么需要它】总力 `sd|F|` 是两个东西的叠加：
  · 相干分量 —— 跟着驱动频率走的那个单频分量；
  · 宽带分量 —— 只要机械臂在动就有的噪声（静止时 ~0.009 N，运动时 ~0.05~0.07）。
把两者混在一个 sd 里拟合，会得到一条【假的弱幂律】（2026-10-01 实测：混着算是 A^0.32·f^0.18，
只取相干分量是 A^0.94·f^0.31 —— **幅度指数差了三倍**）。

判据仍是先写死的那个（见 evidence/2026-09-29-on-machine-session.md:621-642）：
  归一化力恒定 ⇒ 惯性（要求 A^1·f^2）· 随频率降 ⇒ 结构振动 · 与频率无关 ⇒ 噪声源。

⚠ 只读。用法: python _analyze_coherent.py <csv> [档数=5]
⚠★ 锚点/段数**全部由档数算出**，不写死 —— 2026-10-01 因为写死 40s（8 段时代的值）而
   让整轮错位一段，连踩两次。改档数时这里不需要动。
"""
import math
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave.csv"
ALL_SPEEDS = (0.5, 1.0, 2.0, 4.0, 6.0)      # SweepPlan::kSpeed（历史顺序，别改）
NSPEED = int(sys.argv[2]) if len(sys.argv) > 2 else 5
SPEEDS = ALL_SPEEDS[:NSPEED]

# ⚠★ 必须与 `SweepPlan::kSegSec` 【一致】。2026-10-01 由 10 改成 30（为把 0.18Hz 的谐波间隔分开）。
#   这是一个只能靠人记住的耦合 —— 改了档数/段长，脚本这边也要改。
SEG_SEC = 30.0                               # SweepPlan::kSegSec
SEG_COUNT = len(SPEEDS) * 2                  # SweepPlan::kSegCount
FF_HALF_SEC = (SEG_COUNT // 2) * SEG_SEC     # 锚点：FF 由 1 变 0 = 后半开始 = 前半总时长
F0_HZ = 0.365                                # SweepWaveform::kF0Hz
FS = 122.7                                   # 名义采样率（用于 DFT 的相位步长）
I_ACT_X, I_ACT_Y = 13, 14
I_FILT = (4, 5, 6)
I_FF = 22
BAND_MIN_HZ = 0.12                           # 频带半宽下限（≈ 8.3s 窗的 DFT 分辨率）


def read_blocks(path):
    out, cur = [], None
    for line in open(path, "r", encoding="utf-8", errors="replace"):
        line = line.rstrip("\r\n")
        # ⚠ 2026-10-01：连续录制（`'w'`）的块头是 `# rec`，不是 `# wave` —— 两种都要认。
        if line.startswith("# wave") or line.startswith("# rec"):
            cur = []
            out.append(cur)
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


def detrend_window(y):
    n = len(y)
    xm = (n - 1) / 2.0
    ym = sum(y) / n
    sxy = sum((i - xm) * (y[i] - ym) for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    b = sxy / sxx if sxx else 0.0
    a = ym - b * xm
    return [(y[i] - (a + b * i)) * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)))
            for i in range(n)]


def band_fraction(y, f0, half):
    """去趋势 + 加窗后，[f0-half, f0+half] 内的能量占总能量的比例。"""
    d = detrend_window(y)
    n = len(d)
    tot = band = 0.0
    f = 0.05
    while f <= 8.0:
        w = 2 * math.pi * f / FS
        re = im = 0.0
        for i in range(n):
            re += d[i] * math.cos(w * i)
            im += d[i] * math.sin(w * i)
        p = re * re + im * im
        tot += p
        if abs(f - f0) <= half:
            band += p
        f += 0.01
    return band / tot if tot > 0 else 0.0


def main():
    moving = []
    for rows in read_blocks(CSV):
        if not rows:
            continue
        ax = [r[I_ACT_X] for r in rows]
        m = sum(ax) / len(ax)
        if math.sqrt(sum((v - m) ** 2 for v in ax) / len(ax)) < 1.0:
            continue                      # 静止块
        moving += [(r[0], r) for r in rows]
    if not moving:
        print("!! 没有运动块")
        return
    moving.sort(key=lambda x: x[0])
    seen, ded = set(), []
    for t, r in moving:
        if t in seen:
            continue
        seen.add(t)
        ded.append((t, r))
    if len(ded) != len(moving):
        print(f"（按 t_us 去重: {len(moving)} → {len(ded)}）")
    moving = ded

    anchor = None
    for i in range(1, len(moving)):
        if moving[i - 1][1][I_FF] == 1.0 and moving[i][1][I_FF] == 0.0:
            anchor = moving[i][0]
            break
    if anchor is None:
        print("!! 没找到 ff 1->0 翻转")
        return
    t0 = anchor - int(FF_HALF_SEC * 1e6)
    print(f"档数 {NSPEED} ⇒ 段数 {SEG_COUNT}、前半 {FF_HALF_SEC:.0f}s；T0={t0:.0f}")

    segs = {}
    for t, r in moving:
        s = int((t - t0) // int(SEG_SEC * 1e6))
        if 0 <= s < SEG_COUNT:
            segs.setdefault(s, []).append(r)

    print()
    hdr = (f"{'段':>2} {'倍速':>5} {'FF':>4} {'n':>5} {'名义f':>6} {'实测f':>6} "
           f"{'A(mm)':>6} {'sd|F|':>7} {'带内%':>7} {'相干':>7} {'宽带':>7}")
    print(hdr)
    print("-" * len(hdr))
    pts = []
    for s in range(SEG_COUNT):
        rs = segs.get(s, [])
        if len(rs) < 300:
            print(f"{s+1:>2}  (帧太少 {len(rs)})")
            continue
        sp = SPEEDS[s % len(SPEEDS)]
        on = s < SEG_COUNT // 2
        f0 = F0_HZ * sp
        half = max(0.20 * f0, BAND_MIN_HZ)
        A = math.sqrt(sd([r[I_ACT_X] for r in rs]) ** 2 +
                      sd([r[I_ACT_Y] for r in rs]) ** 2) / 1000.0
        # ★★ 三样必须在【同一个基底】上：
        #   [错的] 拿"模长的 sd"当总量、却用逐轴算相干 ⇒ 模长有交叉项 ⇒ 不满足 总²=相干²+宽带²
        #          （2026-10-01 实测后果: 高频段的"宽带"算出来是 0.0000，拟合被带偏）
        #   [对的] 三样都用【逐轴 rms 的平方和开方】⇒ 恒等式精确成立。
        tot2 = 0.0
        coh2 = 0.0
        bfs = []
        for i in I_FILT:
            y = [r[i] for r in rs]
            s2 = sd(y) ** 2
            bf = band_fraction(y, f0, half)
            bfs.append(bf)
            tot2 += s2
            coh2 += s2 * bf
        F = math.sqrt(tot2)                 # 三轴 rms
        Fc = math.sqrt(coh2)                # 相干分量
        Fb = math.sqrt(max(tot2 - coh2, 0.0))   # 宽带分量（恒等式精确）
        pts.append(dict(s=s + 1, sp=sp, on=on, f=f0, A=A * 1000, F=F, Fc=Fc, Fb=Fb,
                        bf=sum(bfs) / len(bfs)))
        print(f"{s+1:>2} {sp:>5} {'ON' if on else 'OFF':>4} {len(rs):>5} {f0:>6.3f} "
              f"{f0:>6.3f} {A*1000:>6.2f} {F:>7.4f} {sum(bfs)/len(bfs)*100:>6.1f}% "
              f"{Fc:>7.4f} {Fb:>7.4f}")
        print(f"POINT,{s+1},{sp},{'ON' if on else 'OFF'},{f0:.4f},{A*1000:.3f},"
              f"{F:.5f},{Fc:.5f},{Fb:.5f}")

    def fit(key, label):
        d = [p for p in pts if p[key] > 0]
        n = len(d)
        if n < 4:
            print(f"  {label}: 点太少")
            return
        X = [[1.0, math.log(p["A"]), math.log(p["f"])] for p in d]
        Y = [math.log(p[key]) for p in d]
        M = [[sum(X[i][a] * X[i][b] for i in range(n)) for b in range(3)] for a in range(3)]
        V = [sum(X[i][a] * Y[i] for i in range(n)) for a in range(3)]
        for i in range(3):
            q = max(range(i, 3), key=lambda r: abs(M[r][i]))
            M[i], M[q] = M[q], M[i]
            V[i], V[q] = V[q], V[i]
            if abs(M[i][i]) < 1e-12:
                print(f"  {label}: 退化")
                return
            for r in range(i + 1, 3):
                k = M[r][i] / M[i][i]
                for c in range(i, 3):
                    M[r][c] -= k * M[i][c]
                V[r] -= k * V[i]
        sol = [0.0] * 3
        for i in (2, 1, 0):
            sol[i] = (V[i] - sum(M[i][c] * sol[c] for c in range(i + 1, 3))) / M[i][i]
        print(f"  {label:22s}: F ∝ A^{sol[2]:+.2f} · f^{sol[1]:+.2f}   (n={n})")

    print("\n=== 幂律拟合 ===  （纯惯性要求 A^+1 · f^+2）")
    fit("F", "总力 sd|F|")
    fit("Fc", "相干分量")
    fit("Fb", "宽带分量")

    bf_all = [p["bf"] for p in pts]
    print(f"\n带内占比: 最低 {min(bf_all)*100:.1f}%  最高 {max(bf_all)*100:.1f}%")
    print("  低占比 ⇒ 该段的力主要是【宽带】的，不锁在驱动频率上。")


if __name__ == "__main__":
    main()
