# -*- coding: utf-8 -*-
"""静止位置力：力是不是【位置的函数】？

【配套的上机手法】用 `'w'` 连续录制（`force_wave_cont.csv`）：
  按一次 'w' 开始 → 把臂挪一小步、停住几秒、再挪一小步 …… → 按一次 'w' 停。
  ⇒ **全程不用按 'n'**，而且 `tgt_*`/`act_*` 都在落盘列里 ⇒ **位置不必人工记**。

【本脚本干什么】从连续记录里**自己认出"静止段"**（位置在几秒内基本不动），
对每一段算：**位置的三轴均值** 与 **力的三轴均值/标准差**。
⇒ 然后看：**力随位置变不变**（那就是"位置力"），还是各段都一样（那它必须运动才出现）。

⚠ 判据（先写死，见 specs/2026-10-01-static-position-force-run-sheet.md）：
  · 各静止段的 **filt 均值** 随位置显著变化（量级 ~0.1 N / 5 mm） ⇒ 纯弹性位置力
  · 各段几乎一样（差异 ≪ 块间差异）                          ⇒ 必须运动才出现
  ⚠ 先看【均值】不要只看 sd（sd 混进静止噪声）。

用法: python _analyze_static_positions.py [csv] [静止判定阈值mm=0.02] [最短静止秒=2.0]
"""
import math
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave_cont.csv"
TOL_MM = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02
MIN_SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

I_TGT = (7, 8, 9)
I_ACT = (13, 14, 15)
I_RAW = (1, 2, 3)
I_FILT = (4, 5, 6)
I_FF = 22


def read_blocks(path):
    out, cur = [], None
    for line in open(path, "r", encoding="utf-8", errors="replace"):
        line = line.rstrip("\r\n")
        if line.startswith("# rec"):
            cur = []
            out.append((line, cur))
        elif line.startswith("#") or not line or cur is None:
            continue
        else:
            cur.append([float(x) for x in line.split(",")])
    return out


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def sd(v):
    n = len(v)
    if n < 2:
        return 0.0
    m = sum(v) / n
    return math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))


def main():
    blocks = read_blocks(CSV)
    print(f"文件: {CSV}")
    print(f"录制块: {len(blocks)}")
    rows = []
    for head, rs in blocks:
        if rs:
            print(f"  {head.strip()[:44]:<46} {len(rs)} 帧")
            rows += rs
    if not rows:
        print("!! 没有数据 —— 是不是忘了按 'w' 停？（连续录制是【停时才落盘】）")
        return

    # 按时间排序（块内已有序，跨块可能有重叠 -> 去重）
    rows.sort(key=lambda r: r[0])
    seen, ded = set(), []
    for r in rows:
        if r[0] in seen:
            continue
        seen.add(r[0])
        ded.append(r)
    rows = ded
    dur = (rows[-1][0] - rows[0][0]) / 1e6
    print(f"\n总帧 {len(rows)}  跨度 {dur:.1f}s\n")

    # 找"静止段": act 在滑动窗内基本不动
    W = max(3, int(MIN_SEC * 122.7))
    static, i = [], 0
    while i + W < len(rows):
        seg = rows[i:i + W]
        ok = True
        for k, idx in enumerate(I_ACT):
            v = [r[idx] for r in seg]
            if max(v) - min(v) > TOL_MM:
                ok = False
                break
        if ok:
            # 尽量往后延伸
            j = i + W
            while j < len(rows):
                ext = rows[j]
                v = [ext[idx] for idx in I_ACT]
                base = [seg[0][idx] for idx in I_ACT]
                if max(abs(v[k] - base[k]) for k in range(3)) > TOL_MM:
                    break
                j += 1
            static.append((i, j))
            i = j
        else:
            i += 1

    if not static:
        print(f"!! 没找到静止段（阈值 {TOL_MM} mm / 最短 {MIN_SEC} s）")
        print("   可能: 全程都在动, 或者位置噪声超过阈值。**如实在报告里记, 别调阈值去凑。**")
        return

    print(f"找到 {len(static)} 个静止段（阈值 {TOL_MM} mm，最短 {MIN_SEC} s）\n")
    hdr = (f"{'#':>2} {'起s':>7} {'长s':>6} "
           f"{'x_mm':>9} {'y_mm':>9} {'z_mm':>9} "
           f"{'|raw|均值':>10} {'raw_x均':>9} {'|filt|均':>9} {'filt_x均':>9} {'sd(filt_x)':>10}")
    print(hdr)
    print("-" * len(hdr))
    out = []
    t0 = rows[0][0]
    for k, (a, b) in enumerate(static, 1):
        seg = rows[a:b]
        px = [mean([r[i] for r in seg]) for i in I_ACT]
        rx = [mean([r[i] for r in seg]) for i in I_RAW]
        fxv = [r[4] for r in seg]
        fmag = mean([math.sqrt(sum(r[i] ** 2 for i in I_FILT)) for r in seg])
        # 逐轴的 filt 均值（力 vs 位置时各轴配各轴）
        fcomp = [mean([r[i] for r in seg]) for i in I_FILT]
        out.append(dict(k=k, t=(seg[0][0] - t0) / 1e6, dur=len(seg) / 122.7,
                        p=px, raw=rx, fmag=fmag, fx=mean(fxv), sdfx=sd(fxv),
                        fcomp=fcomp))
        print(f"{k:>2} {out[-1]['t']:>7.1f} {out[-1]['dur']:>6.1f} "
              f"{px[0]:>9.3f} {px[1]:>9.3f} {px[2]:>9.3f} "
              f"{math.sqrt(sum(v*v for v in rx)):>10.4f} {rx[0]:>9.4f} "
              f"{fmag:>9.4f} {out[-1]['fx']:>9.4f} {out[-1]['sdfx']:>10.4f}")

    print("\n=== 力 vs 位置（各轴配各轴的力）===")
    print("   ⚠ 斜率 = 力的均值 / 该轴位置 的最小二乘；同轴重复测的段也在里面，")
    print("     若各段力几乎不随位置动 ⇒ 斜率会很小、且散布就是噪声底。")
    for ax, fi, name in ((0, 4, 'x'), (1, 5, 'y'), (2, 6, 'z')):
        ps = [o['p'][ax] for o in out]
        fs = [o['fcomp'][fi - 4] for o in out]
        if max(ps) - min(ps) < 1e-9:
            print(f"  {name} 轴: 位置没变（本组没沿这个轴动）")
            continue
        mp = mean(ps); mf = mean(fs)
        sxy = sum((p - mp) * (f - mf) for p, f in zip(ps, fs))
        sxx = sum((p - mp) ** 2 for p in ps)
        slope = sxy / sxx if sxx > 1e-12 else float('nan')
        print(f"  {name} 轴: 位置跨 {max(ps)-min(ps):7.3f} mm   力 filt_{name} 跨 {max(fs)-min(fs):7.4f} N"
              f"   斜率 {slope*1000:+9.3f} N/m")
    # 同位置重复的散布（噪声底）—— 按位置分组的残差
    print("\n=== 噪声底（同一位置的重复段之间的散布）===")
    for fi, name in ((4, 'x'), (5, 'y'), (6, 'z')):
        g = {}
        for o in out:
            key = (round(o['p'][0]), round(o['p'][1]), round(o['p'][2]))
            g.setdefault(key, []).append(o['fcomp'][fi - 4])
        reps = [v for v in g.values() if len(v) >= 2]
        if not reps:
            print(f"  filt_{name}: 没有重复位置（每组都只测了一次）⇒ 噪声底要靠【同一位置连测几块】")
            continue
        w = mean([max(v) - min(v) for v in reps])
        print(f"  filt_{name}: {len(reps)} 个位置有重复, 组内极差平均 {w:.4f} N")
    print("\n⚠ 同一位置重复测的那几段，它们之间的【散布】就是噪声底；")
    print("  位置间差异要【明显大于】它才算数。")


if __name__ == "__main__":
    main()
