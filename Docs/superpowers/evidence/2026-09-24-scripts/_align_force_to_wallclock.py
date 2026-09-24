# -*- coding: utf-8 -*-
"""把 force_demo_log.csv（t_ms = 开机毫秒）锚到墙钟：用 pose 去 MATLAB 日志的 RP| 序列里找同一时刻。"""
import csv, re, math
from datetime import datetime
from bisect import bisect_left

CSV = r"D:\Projects\Touch\Touch_Client\force_demo_log.csv"
LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")

rows = []
with open(CSV, newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k: float(v) for k, v in r.items()})
        except (TypeError, ValueError):
            continue
print(f"CSV {len(rows)} 行, t_ms {rows[0]['t_ms']:.0f} .. {rows[-1]['t_ms']:.0f}")

rp = []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    payload = m.group(3)
    i, k = -1, -1
    while True:
        k = payload.find(b"RP|", k + 1)
        if k < 0:
            break
        i = k
    if i < 0:
        continue
    tail = payload[i + 3:].strip()
    if num6.fullmatch(tail):
        rp.append((m.group(1).decode(), [float(x) for x in tail.split(b",")]))
print(f"RP| {len(rp)} 条, {rp[0][0]} .. {rp[-1][0]}")

# 用每第 200 行 CSV 去 RP| 里找最近位姿
print("\n=== CSV 行 -> 最接近的 RP| 时刻（位姿欧氏距离 < 3mm 才算命中）===")
hits = []
for idx in range(0, len(rows), 200):
    r = rows[idx]
    p = (r["pose_x"], r["pose_y"], r["pose_z"])
    best = min(rp, key=lambda x: math.dist(p, x[1][:3]))
    d = math.dist(p, best[1][:3])
    hits.append((r["t_ms"], d, best[0]))
    print(f"  t_ms={r['t_ms']:9.0f}  pose=({p[0]:8.1f},{p[1]:8.1f},{p[2]:8.1f})  最近 RP| {best[0]}  距离 {d:6.2f} mm")

ok = [(t, w) for t, d, w in hits if d < 3.0]
print(f"\n命中 {len(ok)}/{len(hits)}")
if len(ok) >= 4:
    def sec(s):
        h, mi, rest = s.split(":")
        return int(h) * 3600 + int(mi) * 60 + float(rest)
    offs = [sec(w) * 1000 - t for t, w in ok]
    offs.sort()
    med = offs[len(offs) // 2]
    print(f"推定偏移 中位数 = {med:.0f} ms  (墙钟 = t_ms + {med:.0f}ms, 取当天 0 点起算)")
    a = sec(rows[0]["t_ms"] * 0 + "0:00:00")  # 占位
    def wall(t_ms):
        s = (t_ms + med) / 1000.0
        return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:06.3f}"
    print(f"\n⇒ CSV 覆盖墙钟 {wall(rows[0]['t_ms'])} .. {wall(rows[-1]['t_ms'])}")
    print("   关节模式那两段： 23:11:15.8-23:11:34.9  与  23:11:43.3-23:11:58.3")
    # 把力最大的几行换算成墙钟
    rows2 = [r for r in rows]
    for r in rows2:
        r["F"] = math.sqrt(r["Fx"] ** 2 + r["Fy"] ** 2 + r["Fz"] ** 2)
    print("\n=== |F| 最大的 10 行 -> 墙钟 ===")
    for r in sorted(rows2, key=lambda r: -r["F"])[:10]:
        print(f"  {wall(r['t_ms'])}  |F| {r['F']:.3f}  (Fx {r['Fx']:+.3f} Fy {r['Fy']:+.3f} Fz {r['Fz']:+.3f})")
