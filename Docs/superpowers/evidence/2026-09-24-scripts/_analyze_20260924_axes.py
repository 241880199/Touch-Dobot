# -*- coding: utf-8 -*-
"""2026-09-24：按【固定时间基线的角速度】切段并求每段的笔杆旋转轴（器件系）。
慢动作也要抓得住 ⇒ 基线 0.30s（26Hz 下约 8 个样本），阈值按角速度给（°/s）。"""
import re, math
from datetime import datetime

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260924.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty, jnt = [], []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    t, p = m.group(1).decode(), m.group(3)
    for tag, sink in ((b"P|", sty), (b"J|", jnt)):
        i, k = -1, -1
        while True:
            k = p.find(tag, k + 1)
            if k < 0:
                break
            if k > 0 and (65 <= p[k - 1] <= 90 or 97 <= p[k - 1] <= 122):
                continue
            i = k
        if i < 0:
            continue
        tail = p[i + 2:].strip()
        if num6.fullmatch(tail):
            sink.append((t, [float(x) for x in tail.split(b",")]))
            break

mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
tr = lambda A: [[A[j][i] for j in range(3)] for i in range(3)]


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    return mul(Rz, mul(Ry, Rx))


def rotvec(A):
    t = A[0][0] + A[1][1] + A[2][2]
    ang = math.acos(max(-1.0, min(1.0, (t - 1) / 2)))
    if ang < 1e-9:
        return None, 0.0
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    n = math.sqrt(sum(v * v for v in ax)) or 1.0
    return tuple(v / n for v in ax), math.degrees(ang)


BASE = 0.30    # 秒
samples = []
for i, (t, v) in enumerate(sty):
    # 找到时间上 >=BASE 之前最近的一条
    j = i - 1
    while j >= 0 and (T(t) - T(sty[j][0])).total_seconds() < BASE:
        j -= 1
    if j < 0:
        continue
    ta, va = sty[j]
    dt = (T(t) - T(ta)).total_seconds()
    if dt <= 0:
        continue
    ax, ang = rotvec(mul(tr(R_of(*va[3:])), R_of(*v[3:])))
    if ax is None:
        continue
    rate = ang / dt
    if rate < 4.0:          # °/s：低于它当静止（噪声底）
        continue
    samples.append((t, ax, ang, rate))

print(f"P| {len(sty)} 条 ⇒ 有效运动样本 {len(samples)} 条（角速度 ≥ 4°/s）")
if not samples:
    raise SystemExit

# 按 2 秒桶汇总：主轴 + 累计转角
from collections import defaultdict
buck = defaultdict(list)
for t, ax, ang, rate in samples:
    key = t[:5] + ":" + ("%02d" % (int(t[6:8]) // 2 * 2))
    buck[key].append((ax, ang))

print("\n=== 每 2 秒桶（主轴 / 累计转角）===")
for k in sorted(buck):
    vs = buck[k]
    # 符号对齐后取中位
    ref = vs[0][0]
    al = [v if sum(a * b for a, b in zip(ref, v)) >= 0 else tuple(-x for x in v) for v, _ in vs]
    med = [sorted(a[i] for a in al)[len(al) // 2] for i in range(3)]
    n = math.sqrt(sum(x * x for x in med)) or 1.0
    med = tuple(x / n for x in med)
    dom = max(range(3), key=lambda i: abs(med[i]))
    tot = sum(a for _, a in vs)
    print(f"  {k}  n={len(vs):3d}  转角 {tot:6.1f}°  轴 ({med[0]:+.2f},{med[1]:+.2f},{med[2]:+.2f})  "
          f"主 {('XYZ'[dom])}  偏名义轴 "
          f"{math.degrees(math.acos(min(1,abs(med[dom])))):.0f}°")
