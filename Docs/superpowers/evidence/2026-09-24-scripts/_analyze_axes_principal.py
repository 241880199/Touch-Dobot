# -*- coding: utf-8 -*-
"""三条动作段的【主旋转轴】(器件系)：段内每步增量的旋转向量按角度加权求和后归一。
用途：判"器件 X/Y/Z"能不能当三根轴用（即轴之间是否近似正交）。"""
import re, math
from datetime import datetime

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")

SEGS = [("自转(判据:J6)", "23:09:09.500", "23:09:23.500"),
        ("自转(第二段)", "23:11:16.000", "23:11:33.500"),
        ("前后摆(判据:J4)", "23:11:43.900", "23:11:51.200"),
        ("左右摆(判据:J5)", "23:11:51.300", "23:11:58.500")]


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty = []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    p = m.group(3)
    i, k = -1, -1
    while True:
        k = p.find(b"P|", k + 1)
        if k < 0:
            break
        if k > 0 and (65 <= p[k - 1] <= 90 or 97 <= p[k - 1] <= 122):
            continue
        i = k
    if i < 0:
        continue
    tail = p[i + 2:].strip()
    if num6.fullmatch(tail):
        sty.append((m.group(1).decode(), [float(x) for x in tail.split(b",")]))


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    mul = lambda P, Q: [[sum(P[i][k] * Q[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
tr = lambda A: [[A[j][i] for j in range(3)] for i in range(3)]


def rotvec_vec(A):
    """返回 角度(deg)×单位轴 的三维向量（含方向）"""
    t = A[0][0] + A[1][1] + A[2][2]
    ang = math.acos(max(-1.0, min(1.0, (t - 1) / 2)))
    if ang < 1e-9:
        return (0.0, 0.0, 0.0), 0.0
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    n = math.sqrt(sum(v * v for v in ax)) or 1.0
    d = math.degrees(ang)
    return tuple(d * v / n for v in ax), d


def norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / n for x in v)


axes = {}
for name, s0, s1 in SEGS:
    seg = [(t, v) for t, v in sty if s0 <= t <= s1]
    S = [0.0, 0.0, 0.0]
    tot = 0.0
    n = 0
    prev = None
    for t, v in seg:
        if prev:
            dt = (T(t) - T(prev[0])).total_seconds()
            if 0.15 < dt < 4.0:
                rv, d = rotvec_vec(mul(tr(R_of(*prev[1][3:])), R_of(*v[3:])))
                if d > 1.5:
                    S = [S[i] + rv[i] for i in range(3)]
                    tot += d
                    n += 1
        prev = (t, v)
    if n == 0:
        print(f"{name}: 无有效增量")
        continue
    ax = norm(S)
    axes[name] = ax
    print(f"{name:16s} 样本 {len(seg):3d}  有效步 {n:3d}  总转角 {tot:6.1f}°  "
          f"主方向 ({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f})")
    for i, nm in enumerate("XYZ"):
        pass
    print(f"                 |与器件X夹角| {math.degrees(math.acos(max(-1,min(1,abs(ax[0]))))):5.1f}°"
          f"  |Y| {math.degrees(math.acos(max(-1,min(1,abs(ax[1]))))):5.1f}°"
          f"  |Z| {math.degrees(math.acos(max(-1,min(1,abs(ax[2]))))):5.1f}°")

print("\n=== 三条轴两两夹角（绝对值；90° = 正交）===")
ks = list(axes)
for i in range(len(ks)):
    for j in range(i + 1, len(ks)):
        d = sum(axes[ks[i]][k] * axes[ks[j]][k] for k in range(3))
        print(f"  {ks[i]:16s} vs {ks[j]:16s}  {math.degrees(math.acos(max(-1,min(1,abs(d))))):6.1f}°")
