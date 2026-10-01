# -*- coding: utf-8 -*-
"""
2026-10-01 上机第二站：抖动【频率扫描】离线分析。

判据（先写死在 Docs/superpowers/evidence/2026-09-29-on-machine-session.md:621-642）：
  把力的波动【归一化成"单位加速度的力"】后对频率作图 ——
    基本恒定   ⇒ 惯性
    随频率下降 ⇒ 结构性振动(带通)
    与频率无关 ⇒ 与运动无关的噪声源

做法：每块独立统计
  · 采样率与时长（用 t_us 真实时基，不用名义 123 Hz）
  · 运动幅度 = sd(act_x/y/(z)) 的 mm，运动主频 = act_x 去均值后的 DFT 峰
  · 力的波动 = sd(filt_x/y/z)（落盘口，即补偿后的力）
  · 归一化力 = sd(|filt|) / (A·(2πf)²)   —— A 用实测运动幅度(m)，f 用实测主频(Hz)

⚠ 只读，不改任何东西。
"""
import math
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave.csv"

COLS = ["t_us", "raw_x", "raw_y", "raw_z", "filt_x", "filt_y", "filt_z",
        "tgt_x_mm", "tgt_y_mm", "tgt_z_mm", "tgt_rx_deg", "tgt_ry_deg", "tgt_rz_deg",
        "act_x_mm", "act_y_mm", "act_z_mm", "act_rx_deg", "act_ry_deg", "act_rz_deg",
        "dev_x_mm", "dev_y_mm", "dev_z_mm", "ff",
        "tcpV_x", "tcpV_y", "tcpV_z"]


def read_blocks(path):
    blocks, cur = [], None
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\r\n")
            if not line:
                continue
            if line.startswith("# wave"):
                cur = {"head": line, "rows": [], "cols": None}
                blocks.append(cur)
                continue
            if line.startswith("#"):
                if cur is not None and cur["cols"] is None:
                    cur["cols"] = [c.strip() for c in line.lstrip("#").split(",")]
                continue
            if cur is None:
                continue
            cur["rows"].append([float(x) for x in line.split(",")])
    return blocks


def sd(v):
    n = len(v)
    if n < 2:
        return 0.0
    m = sum(v) / n
    return math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))


def mean(v):
    return sum(v) / len(v) if v else 0.0


def dom_freq(y, ts):
    """去均值后 DFT 峰频率（Hz）。y 与 ts 等长；ts 秒。"""
    n = len(y)
    if n < 16:
        return 0.0
    m = mean(y)
    y = [v - m for v in y]
    T = ts[-1] - ts[0]
    if T <= 0:
        return 0.0
    best_f, best_p = 0.0, -1.0
    f = 0.1
    while f <= 20.0:
        w = 2.0 * math.pi * f
        re = im = 0.0
        for i in range(n):
            a = w * (ts[i] - ts[0])
            re += y[i] * math.cos(a)
            im += y[i] * math.sin(a)
        p = re * re + im * im
        if p > best_p:
            best_p, best_f = p, f
        f += 0.05
    return best_f


def col(rows, name):
    i = COLS.index(name)
    return [r[i] for r in rows]


def main():
    blocks = read_blocks(PATH)
    print(f"文件: {PATH}")
    print(f"块数: {len(blocks)}")
    print()
    hdr = (f"{'#':>2} {'块头时刻':>23} {'n':>5} {'时长s':>6} {'fs':>6} "
           f"{'sd(act_x)':>9} {'sd(act_y)':>9} {'主频Hz':>7} "
           f"{'sd(filt_x)':>10} {'sd(filt_y)':>10} {'sd(filt_z)':>10} "
           f"{'mean(ff)':>8} {'mean|raw_z|':>11}")
    print(hdr)
    print("-" * len(hdr))
    for k, b in enumerate(blocks):
        rows = b["rows"]
        if not rows:
            continue
        colok = b["cols"] and len(b["cols"]) == len(COLS)
        if not colok:
            print(f"  !! 块 {k} 列数 {len(b['cols']) if b['cols'] else '?'} != {len(COLS)}")
            continue
        ts = [(r[0] - rows[0][0]) / 1e6 for r in rows]
        dur = ts[-1] if ts else 0.0
        fs = (len(rows) - 1) / dur if dur > 0 else 0.0
        ax, ay = col(rows, "act_x_mm"), col(rows, "act_y_mm")
        fx, fy, fz = col(rows, "filt_x"), col(rows, "filt_y"), col(rows, "filt_z")
        ff = col(rows, "ff")
        rz = col(rows, "raw_z")
        f0 = dom_freq(ax, ts)
        stamp = b["head"].split("frames=")[0].replace("# wave", "").strip()
        print(f"{k:>2} {stamp:>23} {len(rows):>5} {dur:>6.2f} {fs:>6.1f} "
              f"{sd(ax):>9.4f} {sd(ay):>9.4f} {f0:>7.2f} "
              f"{sd(fx):>10.4f} {sd(fy):>10.4f} {sd(fz):>10.4f} "
              f"{mean(ff):>8.2f} {mean([abs(v) for v in rz]):>11.3f}")

    print()
    print("=== 归一化：单位加速度的力 ===")
    print("   加速度幅度 a = A·(2πf)²  [A 用 sd(act) 当幅度代理, m; f 用实测主频]")
    print(f"{'#':>2} {'f(Hz)':>7} {'A(mm)':>8} {'a(m/s²)':>10} "
          f"{'sd|filt|':>10} {'N/(m/s²)':>11}")
    for k, b in enumerate(blocks):
        rows = b["rows"]
        if not rows or not (b["cols"] and len(b["cols"]) == len(COLS)):
            continue
        ts = [(r[0] - rows[0][0]) / 1e6 for r in rows]
        ax, ay = col(rows, "act_x_mm"), col(rows, "act_y_mm")
        A = math.sqrt(sd(ax) ** 2 + sd(ay) ** 2) / 1000.0     # m
        f0 = dom_freq(ax, ts)
        a = A * (2 * math.pi * f0) ** 2 if f0 > 0 else 0.0
        fmag = [math.sqrt(x * x + y * y + z * z) for x, y, z in
                zip(col(rows, "filt_x"), col(rows, "filt_y"), col(rows, "filt_z"))]
        s = sd(fmag)
        ratio = s / a if a > 1e-9 else float("nan")
        print(f"{k:>2} {f0:>7.2f} {A*1000:>8.3f} {a:>10.3f} {s:>10.4f} {ratio:>11.5f}")


if __name__ == "__main__":
    main()
