# -*- coding: utf-8 -*-
"""从 force_wave.csv 抽一段【单频】轨迹，生成 relay/SweepWaveformData.h。

取法(B)（见设计 §3.1）：取一块臂确实跟上的 tgt_x/y/z -> 去线性趋势 -> 稳健主频 f0
-> 取恰好一个周期 -> 只保留 f0 分量 -> 归一化到单位峰值 -> 输出 128x3 表。

源块: 2026-10-01 13:29:00（主频 ~0.36 Hz; tgt sd 28.80 ~ act sd 28.76 => 臂跟上了）
"""
import math, sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave.csv"
WANT = "2026-10-01 13:29:00"
N = 128

def read_blocks(path):
    out, cur = [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.rstrip("\r\n")
        if line.startswith("# wave"):
            cur = {"head": line, "rows": []}
            out.append(cur)
        elif line.startswith("#") or not line:
            continue
        elif cur is not None:
            cur["rows"].append([float(x) for x in line.split(",")])
    return out

def detrend(y):
    n = len(y); xm = (n - 1) / 2.0; ym = sum(y) / n
    sxy = sum((i - xm) * (y[i] - ym) for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    b = sxy / sxx if sxx else 0.0
    a = ym - b * xm
    return [y[i] - (a + b * i) for i in range(n)]

def dom_freq(y, fs, lo=0.15, hi=10.0):
    d = detrend(y); n = len(d)
    w = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    d = [d[i] * w[i] for i in range(n)]
    best, f = None, lo
    while f <= hi:
        om = 2 * math.pi * f / fs
        re = sum(d[i] * math.cos(om * i) for i in range(n))
        im = sum(d[i] * math.sin(om * i) for i in range(n))
        p = re * re + im * im
        if best is None or p > best[1]:
            best = (f, p)
        f += 0.005
    return best[0]

def main():
    blk = [b for b in read_blocks(CSV) if WANT in b["head"]]
    if len(blk) != 1:
        sys.exit(f"!! 期望恰好 1 块匹配 {WANT}, 实际 {len(blk)}")
    rows = blk[0]["rows"]
    # 列序（26 列）: 0 t_us, 7..9 = tgt_x/y/z
    t = [(r[0] - rows[0][0]) / 1e6 for r in rows]
    fs = (len(rows) - 1) / (t[-1] - t[0])
    f0 = dom_freq([r[7] for r in rows], fs)
    print(f"块 {blk[0]['head'].strip()}  fs={fs:.2f}  f0={f0:.4f} Hz  周期={1.0/f0:.3f}s")
    # 取恰好一个周期 -> 重采样到 N 点 -> 只保留 f0 分量（单点 DFT 的 2/N 幅度）= 纯正弦
    harm = []
    for k in range(3):
        y = detrend([r[7 + k] for r in rows])   # ★ 先去趋势, 否则线性漂移会漏进 f0 分量
        re = im = 0.0
        for i in range(len(y)):
            om = 2 * math.pi * f0 * t[i]
            re += y[i] * math.cos(om)
            im += y[i] * math.sin(om)
        amp = 2.0 * math.hypot(re, im) / len(y)
        ph = math.atan2(im, re)
        harm.append((amp, ph))
        print(f"  轴{k}: f0 分量 幅度={amp:.4f} mm  相位={math.degrees(ph):.1f} deg")
    tab = []
    for j in range(N):
        th = 2 * math.pi * j / N
        tab.append([harm[k][0] * math.cos(th + harm[k][1]) for k in range(3)])
    peak = max(math.sqrt(sum(v * v for v in row)) for row in tab)
    if peak <= 0:
        sys.exit("!! 峰值 0 —— 源块没有周期成分")
    tab = [[v / peak for v in row] for row in tab]
    # 自检
    pk = max(math.sqrt(sum(v * v for v in row)) for row in tab)
    assert abs(pk - 1.0) < 1e-12, pk
    assert abs(math.dist(tab[0], tab[-1])) > 0, "首末点不该相同(环是按 N 点闭合)"
    print(f"自检: 归一化峰值={pk:.12f}  平面性={max(abs(v[2]-tab[0][2]) for v in tab):.2e}")
    with open("Touch_Client/relay/SweepWaveformData.h", "w", encoding="utf-8") as fh:
        fh.write("// 本文件由 _extract_sweep_waveform.py 生成 —— 不要手改。\n")
        fh.write("// 源: force_wave.csv 的 %s 块（主频 %.4f Hz）。\n" % (WANT, f0))
        fh.write("#pragma once\n\nnamespace SweepWaveform {\n")
        fh.write("constexpr double kF0Hz = %.6f;\n" % f0)
        fh.write("constexpr int kTableN = %d;\n" % N)
        fh.write("constexpr double kTable[kTableN][3] = {\n")
        for row in tab:
            fh.write("    {%.9f, %.9f, %.9f},\n" % tuple(row))
        fh.write("};\n}  // namespace SweepWaveform\n")
    print("已写 Touch_Client/relay/SweepWaveformData.h")

main()
