# -*- coding: utf-8 -*-
"""用整段序列（不是单点姿势）把 force_demo_log.csv 锚到墙钟。
做法：对每个候选偏移，取一批 RP| 样本，看 CSV 在同一时刻的位姿是否对得上，评分取中位距离。"""
import csv, re, math
from datetime import datetime
from bisect import bisect_left

CSV = r"D:\Projects\Touch\Touch_Client\force_demo_log.csv"
LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")


def sec(s):
    h, mi, rest = s.split(":")
    return int(h) * 3600 + int(mi) * 60 + float(rest)


rows = []
with open(CSV, newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k: float(v) for k, v in r.items()})
        except (TypeError, ValueError):
            continue
ts = [r["t_ms"] for r in rows]

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
        rp.append((sec(m.group(1).decode()) * 1000.0, [float(x) for x in tail.split(b",")]))

# 取一批"位姿有变化"的 RP| 样本当指纹（静止的不具区分度）
anchors = []
prev = None
for t, v in rp:
    if prev is not None and math.dist(v[:3], prev[:3]) > 0.5:
        anchors.append((t, v))
    prev = v
# 再掺入若干静止样本，压制错位
still = [x for x in rp if not any(abs(x[0] - a[0]) < 1.0 for a in anchors)][::40]
anchors = sorted(anchors + still)
# 只留晚段（23:05–23:20）—— 其余几小时不可能与这 674 s 的 CSV 重叠
anchors = [a for a in anchors if (23 * 3600 + 5 * 60) * 1000 <= a[0] <= (23 * 3600 + 20 * 60) * 1000]
print(f"指纹样本 {len(anchors)} 个（晚段 23:05–23:20）")

best = None
lo, hi = 4.7e7, 6.6e7   # 偏移范围（ms）：由 RP| 与 CSV 各自的时间跨度夹出
step = 50.0
n = int((hi - lo) / step)
for it in range(n):
    off = lo + it * step
    ds = []
    for t, v in anchors:
        ct = t - off
        ## 与 CSV 时间轴对齐（t = t_ms + off ⇒ t_ms = t - off）
        if ct < ts[0] or ct > ts[-1]:
            continue
        j = bisect_left(ts, ct)
        if j >= len(rows):
            j = len(rows) - 1
        ds.append(math.dist(v[:3], (rows[j]["pose_x"], rows[j]["pose_y"], rows[j]["pose_z"])))
    if len(ds) < max(20, len(anchors) * 0.5):
        continue
    ds.sort()
    score = ds[len(ds) // 2]          # 中位距离
    if best is None or score < best[0]:
        best = (score, off, len(ds))
print(f"最佳偏移 {best[1]:.0f} ms  中位位姿偏差 {best[0]:.2f} mm  （参与 {best[2]} 个指纹）")

off = best[1]


def wall(t_ms):
    s = (t_ms + off) / 1000.0
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:06.3f}"


print(f"\n⇒ CSV 覆盖 {wall(ts[0])} .. {wall(ts[-1])}")
print("   关节模式两段：23:11:15.8-23:11:34.9 与 23:11:43.3-23:11:58.3")

for r in rows:
    r["F"] = math.sqrt(r["Fx"] ** 2 + r["Fy"] ** 2 + r["Fz"] ** 2)
prev = None
for r in rows:
    r["dpose"] = 0.0 if prev is None else math.dist(
        (r["pose_x"], r["pose_y"], r["pose_z"]), (prev["pose_x"], prev["pose_y"], prev["pose_z"]))
    prev = r

print("\n=== |F| 最大的 12 行 -> 墙钟 ===")
for r in sorted(rows, key=lambda r: -r["F"])[:12]:
    print(f"  {wall(r['t_ms'])}  |F| {r['F']:6.3f}  Fx {r['Fx']:+6.3f} Fy {r['Fy']:+6.3f} Fz {r['Fz']:+6.3f}  Δpose {r['dpose']:6.2f}mm")

print("\n=== 运动段（Δpose>0.5mm）成簇 ===")
clusters = []
cur = None
for r in rows:
    if r["dpose"] > 0.5:
        if cur is None:
            cur = [r, r]
        else:
            cur[1] = r
    else:
        if cur is not None and (cur[1]["t_ms"] - cur[0]["t_ms"]) > 3000:
            clusters.append(cur)
        cur = None
if cur:
    clusters.append(cur)
for a, b in clusters:
    seg = [r for r in rows if a["t_ms"] <= r["t_ms"] <= b["t_ms"]]
    fs = [r["F"] for r in seg]
    print(f"  {wall(a['t_ms'])} .. {wall(b['t_ms'])}  时长 {(b['t_ms']-a['t_ms'])/1000:5.1f}s  n={len(seg):4d}  "
          f"|F| 均值 {sum(fs)/len(fs):.3f}  sd {math.sqrt(sum((x-sum(fs)/len(fs))**2 for x in fs)/max(len(fs)-1,1)):.3f}  最大 {max(fs):.3f}")
