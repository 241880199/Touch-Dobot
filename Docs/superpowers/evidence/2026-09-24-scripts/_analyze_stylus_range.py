# -*- coding: utf-8 -*-
"""笔杆姿态的取值域体检：rx/rz 有没有真的走到 ±180（跨接缝），ry 有没有靠近 ±90（万向锁）。"""
import re

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")

rec = []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    payload = m.group(3)
    i, k = -1, -1
    while True:
        k = payload.find(b"P|", k + 1)
        if k < 0:
            break
        if k > 0 and (65 <= payload[k - 1] <= 90 or 97 <= payload[k - 1] <= 122):
            continue
        i = k
    if i < 0:
        continue
    tail = payload[i + 2:].strip()
    if num6.fullmatch(tail):
        rec.append((m.group(1).decode(), [float(x) for x in tail.split(b",")]))

n = len(rec)
print(f"样本 {n} 条")
names = ["x", "y", "z", "Rx", "Ry", "Rz"]
for j in range(6):
    vals = [v[j] for _, v in rec]
    print(f"  {names[j]:3s}: min {min(vals):9.2f}  max {max(vals):9.2f}")

print("\n=== 危险域计数 ===")
for j, label, thr in ((3, "|Rx| > 170 (接缝附近)", 170), (5, "|Rz| > 170 (接缝附近)", 170), (4, "|Ry| > 80 (万向锁附近)", 80)):
    c = sum(1 for _, v in rec if abs(v[j]) > thr)
    print(f"  {label:26s}: {c:6d} / {n}  ({100.0*c/max(n,1):.2f}%)")

# 相邻样本的跳变（同一帧跨 ±360 的特征）
print("\n=== 相邻样本姿态跳变（|Δ|>90° 视作疑似跨接缝）===")
hits = 0
for (t0, a), (t1, b) in zip(rec, rec[1:]):
    for j, nm in ((3, "Rx"), (4, "Ry"), (5, "Rz")):
        d = b[j] - a[j]
        if abs(d) > 90:
            hits += 1
            if hits <= 12:
                print(f"  {t0} -> {t1}  {nm}: {a[j]:+9.2f} -> {b[j]:+9.2f}   Δ={d:+9.2f}")
print(f"  共 {hits} 处")
