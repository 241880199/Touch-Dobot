# -*- coding: utf-8 -*-
"""纯度检验：把录下来的笔杆 ZYX Euler 还原成 R，取相邻样本的增量旋转，
看它是不是"绕单一一根轴"（纯自转）—— 以及那根轴在【器件系】里是否稳定。"""
import re, math
from datetime import datetime

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")

WINDOWS = [("23:08:57", "23:09:27"), ("23:11:13", "23:12:01")]


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty = []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    t, payload = m.group(1).decode(), m.group(3)
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
        sty.append((t, [float(x) for x in tail.split(b",")]))

print(f"P| 共 {len(sty)} 条\n")


def R_of(rx, ry, rz):
    """R = Rz(rz)·Ry(ry)·Rx(rx)，角度制"""
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    # Rx
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    def mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def T3(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def rotvec(A):
    """3x3 旋转矩阵 -> (角(deg), 单位轴)"""
    tr = A[0][0] + A[1][1] + A[2][2]
    cosang = max(-1.0, min(1.0, (tr - 1) / 2))
    ang = math.acos(cosang)
    if ang < 1e-9:
        return 0.0, (0, 0, 0)
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    n = math.sqrt(sum(v * v for v in ax)) or 1.0
    return math.degrees(ang), tuple(v / n for v in ax)


for w0, w1 in WINDOWS:
    print(f"########## 窗口 {w0} .. {w1} ##########")
    seg = [(t, v) for t, v in sty if w0 <= t <= w1]
    print(f"  样本 {len(seg)} 条")
    print("  t           Rx      Ry      Rz   |  Δ(deg)  轴(器件系)                | 轴(世界系)")
    prev = None
    for t, v in seg:
        line = f"  {t} {v[3]:8.2f}{v[4]:8.2f}{v[5]:8.2f}"
        if prev:
            dt = (T(t) - T(prev[0])).total_seconds()
            if dt < 6:
                R0, R1 = R_of(*prev[1][3:]), R_of(*v[3:])
                ang_b, ax_b = rotvec(mul(T3(R0), R1))     # 体系（器件自身系）
                _, ax_w = rotvec(mul(R1, T3(R0)))         # 世界系
                line += (f"  |  {ang_b:6.2f}  ({ax_b[0]:+.2f},{ax_b[1]:+.2f},{ax_b[2]:+.2f})"
                         f"   ({ax_w[0]:+.2f},{ax_w[1]:+.2f},{ax_w[2]:+.2f})")
        prev = (t, v)
        print(line)
    print()
