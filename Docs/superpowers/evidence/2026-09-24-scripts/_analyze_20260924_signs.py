# -*- coding: utf-8 -*-
"""把"笔杆相对【按下时参照】转了多少"(真 ΔR 的器件系旋转向量)与"命令关节走了多少"并排，
逐段抽 ② 的符号关系。参照 = 按住那一刻（= burst 起点附近）的笔杆姿态。"""
import re, math
from datetime import datetime
from bisect import bisect_left

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260924.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")
servoj_re = re.compile(rb"ServoJ\((-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){5})")


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty, cmd = [], []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    t, p = m.group(1).decode(), m.group(3)
    ms = list(servoj_re.finditer(p))
    if ms:
        cmd.append((t, [float(x) for x in ms[-1].group(1).split(b",")]))
        continue
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
        sty.append((t, [float(x) for x in tail.split(b",")]))

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
        return (0.0, 0.0, 0.0)
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    n = math.sqrt(sum(v * v for v in ax)) or 1.0
    return tuple(math.degrees(ang) * v / n for v in ax)


T0 = "18:37:29.384"      # 按住那一刻（burst 起点）
ref = min(sty, key=lambda x: abs((T(x[0]) - T(T0)).total_seconds()))[1]
Rref = R_of(*ref[3:])
jref = min(cmd, key=lambda x: abs((T(x[0]) - T(T0)).total_seconds()))[1]
print(f"参照: 笔杆 ({ref[3]:.2f},{ref[4]:.2f},{ref[5]:.2f})°   关节 ({['%.2f'%v for v in jref]})")

SEGS = [("A 18:37:30-44 (主X)", "18:37:30.0", "18:37:44.5"),
        ("B 18:37:46-52 (主Y?)", "18:37:46.0", "18:37:52.0"),
        ("C 18:37:54-18:38:26 (主Z)", "18:37:54.0", "18:38:26.5"),
        ("D 18:38:28-32 (X/Y)", "18:38:28.0", "18:38:32.0")]

print(f"\n{'段':26s} {'Δ笔杆器件系旋转向量(X,Y,Z)':34s} {'Δ命令 dJ4,dJ5,dJ6':30s}")
for name, s0, s1 in SEGS:
    a = min(sty, key=lambda x: abs((T(x[0]) - T(s0)).total_seconds()))
    b = min(sty, key=lambda x: abs((T(x[0]) - T(s1)).total_seconds()))
    rv = rotvec(mul(tr(Rref), R_of(*b[1][3:])))
    c0 = min(cmd, key=lambda x: abs((T(x[0]) - T(s0)).total_seconds()))[1]
    c1 = min(cmd, key=lambda x: abs((T(x[0]) - T(s1)).total_seconds()))[1]
    dj = [c1[i] - c0[i] for i in range(6)]
    print(f"{name:26s} ({rv[0]:+7.2f},{rv[1]:+7.2f},{rv[2]:+7.2f})   "
          f"({dj[3]:+7.2f},{dj[4]:+7.2f},{dj[5]:+7.2f})")

print("\n注: 现行映射是 dJ4←ΔRx(欧拉) · dJ5←ΔRz(欧拉) · dJ6←ΔRy(欧拉)，三者 SIGN 均 +1；")
print("    上表第一列是【真 ΔR 的器件系旋转向量】，与欧拉差不是同一个量 —— 这一栏是给修法用的。")
