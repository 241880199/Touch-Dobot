# -*- coding: utf-8 -*-
"""落盘力（force_demo_log.csv）在【运动 vs 静止】下的量级。
用途：判 ③「空载运动时较大且抖的力」是不是传感器力反馈链（8a）的残差，而不是被关掉的斥力。"""
import csv, math
from collections import defaultdict

CSV = r"D:\Projects\Touch\Touch_Client\force_demo_log.csv"
rows = []
with open(CSV, newline="") as f:
    for r in csv.DictReader(f):
        try:
            if any(v is None or v == "" for v in r.values()):
                raise ValueError("缺字段")
            rows.append({k: float(v) for k, v in r.items()})
        except ValueError:
            continue
print(f"行数 {len(rows)}")
t0, t1 = rows[0]["t_ms"], rows[-1]["t_ms"]
print(f"时间跨度 {(t1-t0)/1000:.1f} s  ({t0:.0f} -> {t1:.0f} ms)")

for r in rows:
    r["F"] = math.sqrt(r["Fx"] ** 2 + r["Fy"] ** 2 + r["Fz"] ** 2)
    r["M"] = math.sqrt(r["Mx"] ** 2 + r["My"] ** 2 + r["Mz"] ** 2)

# 运动判据：与上一行的姿态位移（mm）
prev = None
for r in rows:
    if prev is None:
        r["dpose"] = 0.0
    else:
        r["dpose"] = math.dist((r["pose_x"], r["pose_y"], r["pose_z"]),
                               (prev["pose_x"], prev["pose_y"], prev["pose_z"]))
    prev = r

still = [r for r in rows if r["is_still"] == 1]
mov = [r for r in rows if r["is_still"] != 1]


def stats(name, xs):
    if not xs:
        print(f"  {name}: 无")
        return
    n = len(xs)
    mean = sum(xs) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / max(n - 1, 1))
    print(f"  {name:28s} n={n:5d}  |F| 均值 {mean:7.3f}  sd {sd:6.3f}  最大 {max(xs):7.3f} N")


print("\n=== |F| 按 is_still 分组 ===")
stats("静止 (is_still=1)", [r["F"] for r in still])
stats("运动 (is_still=0)", [r["F"] for r in mov])
stats("全部", [r["F"] for r in rows])

print("\n=== 按 |Δpose| 分桶（mm/采样，11Hz ⇒ 10mm/s ≈ 0.9mm/采样）===")
buckets = defaultdict(list)
for r in rows:
    d = r["dpose"]
    k = ("0 静止" if d < 0.05 else "1 慢 <0.5" if d < 0.5 else
         "2 中 0.5-2" if d < 2 else "3 快 2-5" if d < 5 else "4 很快 >5")
    buckets[k].append(r["F"])
for k in sorted(buckets):
    xs = buckets[k]
    print(f"  {k:12s} n={len(xs):5d}  |F| 均值 {sum(xs)/len(xs):7.3f}  sd "
          f"{math.sqrt(sum((x-sum(xs)/len(xs))**2 for x in xs)/max(len(xs)-1,1)):6.3f}  最大 {max(xs):7.3f}")

print("\n=== |F| 最大的 15 行 ===")
top = sorted(rows, key=lambda r: -r["F"])[:15]
print("     t_ms     |F|     Fx      Fy      Fz     |M|   Δpose  is_still  acc")
for r in top:
    print(f"  {r['t_ms']:9.0f} {r['F']:7.3f} {r['Fx']:+7.3f} {r['Fy']:+7.3f} {r['Fz']:+7.3f}"
          f" {r['M']:6.3f} {r['dpose']:7.2f}   {r['is_still']:.0f}     "
          f"{math.sqrt(r['acc_x']**2+r['acc_y']**2+r['acc_z']**2):6.3f}")
