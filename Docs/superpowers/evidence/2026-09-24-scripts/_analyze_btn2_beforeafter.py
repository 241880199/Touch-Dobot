# -*- coding: utf-8 -*-
"""三段实测动作，各自对比：
   现行实现（逐欧拉角作差 -> dJ4<-Rx, dJ5<-Rz, dJ6<-Ry）
   vs
   候选修法（真 ΔR 在器件系取旋转向量 -> X=>J4, Y=>J5, Z=>J6）
参考量一律取该段【第一帧】的笔杆姿态（近似"按下那一刻的快照"）。"""
import re, math
from datetime import datetime

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")

SEGS = [("自转(器件Z)", "23:09:12.000", "23:09:23.500"),
        ("前后摆(器件X)", "23:11:43.900", "23:11:51.200"),
        ("左右摆(器件Y)", "23:11:51.300", "23:11:58.500")]


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

mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
transp = lambda A: [[A[j][i] for j in range(3)] for i in range(3)]


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    return mul(Rz, mul(Ry, Rx))


def rotvec(A):
    tr = A[0][0] + A[1][1] + A[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    if ang < 1e-9:
        return (0.0, 0.0, 0.0)
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    return tuple(math.degrees(ang) * v for v in ax)   # 旋转向量（方向×角）


for name, s0, s1 in SEGS:
    seg = [(t, v) for t, v in sty if s0 <= t <= s1]
    if not seg:
        print(f"### {name}: 段内无样本\n")
        continue
    ref = seg[0][1]
    end = seg[-1][1]
    dEuler = [end[i] - ref[i] for i in range(3, 6)]
    dRvec = rotvec(mul(transp(R_of(*ref[3:])), R_of(*end[3:])))   # 器件系
    dRvec_w = rotvec(mul(R_of(*end[3:]), transp(R_of(*ref[3:]))))  # 世界系（对照）
    print(f"### {name}   【{seg[0][0]} .. {seg[-1][0]}】  样本 {len(seg)} 条")
    print(f"    参考笔杆姿态 Rx,Ry,Rz = ({ref[3]:.2f}, {ref[4]:.2f}, {ref[5]:.2f})")
    print(f"    欧拉角差  ΔRx,ΔRy,ΔRz = ({dEuler[0]:+7.2f}, {dEuler[1]:+7.2f}, {dEuler[2]:+7.2f})")
    print(f"    └ 现行映射 ⇒  dJ4={dEuler[0]:+7.2f}  dJ5={dEuler[2]:+7.2f}  dJ6={dEuler[1]:+7.2f}   (Rx,Rz,Ry 的顺序)")
    print(f"    真 ΔR 器件系旋转向量 = ({dRvec[0]:+7.2f}, {dRvec[1]:+7.2f}, {dRvec[2]:+7.2f})")
    print(f"    └ 候选映射 ⇒  dJ4={dRvec[0]:+7.2f}  dJ5={dRvec[1]:+7.2f}  dJ6={dRvec[2]:+7.2f}")
    print(f"    (对照) 世界系旋转向量 = ({dRvec_w[0]:+7.2f}, {dRvec_w[1]:+7.2f}, {dRvec_w[2]:+7.2f})")
    # 串扰比：本轮"该动的那个"之外的量 / 该动的量
    def crosstalk(v, idx):
        main = abs(v[idx]) or 1e-9
        return (abs(v[(idx + 1) % 3]) + abs(v[(idx + 2) % 3])) / main
    print(f"    串扰比（另两轴之和 / 主轴）：欧拉角 {crosstalk([dEuler[0], dEuler[1], dEuler[2]], [0,2,1][{'自转(器件Z)':2,'前后摆(器件X)':0,'左右摆(器件Y)':1}[name]]):.2f}"
          f"   旋转向量 {crosstalk(dRvec, {'自转(器件Z)':2,'前后摆(器件X)':0,'左右摆(器件Y)':1}[name]):.2f}")
    print(f"    → 该动的量 欧拉 {abs([dEuler[0],dEuler[1],dEuler[2]][[0,2,1][{'自转(器件Z)':2,'前后摆(器件X)':0,'左右摆(器件Y)':1}[name]]]):.2f}°  旋转向量 {abs(dRvec[{'自转(器件Z)':2,'前后摆(器件X)':0,'左右摆(器件Y)':1}[name]]):.2f}°")
    print()
