# -*- coding: utf-8 -*-
"""在【按下时的姿态】下，算"绕器件某根轴转 δ ⇒ 欧拉角三个分量各变多少"。
现行映射 dJ4←ΔRx · dJ5←ΔRz · dJ6←ΔRy（SIGN 全 +1）⇒ 这张灵敏度矩阵就是
"同样转 10°，现任实现会给每个关节发多少度"。用来解释现场那句
【摆动时臂动得少、自转一点就大幅转动】。"""
import math

REF = (-58.93, 11.83, -6.41)   # 按下那一刻的笔杆欧拉角（取自今晚日志）


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    mul = lambda P, Q: [[sum(P[i][k] * Q[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


def euler_of(R):
    """R = Rz·Ry·Rx 的 ZYX 反解（与 HapticCallback 的提取式同构）"""
    sy = max(-1.0, min(1.0, -R[2][0]))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:
        rx = math.atan2(-R[1][2], R[1][1])
        rz = 0.0
    return tuple(math.degrees(v) for v in (rx, ry, rz))


def axis_rot(u, deg):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    if u == 0:
        return [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    if u == 1:
        return [[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]]
    return [[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]]


mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

Rref = R_of(*REF)
e0 = euler_of(Rref)
print(f"参照姿态 欧拉 = ({e0[0]:.2f},{e0[1]:.2f},{e0[2]:.2f})°")
print(f"\n{'绕器件轴转 +10°':18s} {'Δ欧拉 (Rx,Ry,Rz)':30s} {'⇒ 现任发给':22s}")
print(f"{'':18s} {'':30s} dJ4=ΔRx dJ5=ΔRz dJ6=ΔRy")
rows = []
for u, nm in enumerate("XYZ"):
    Rn = mul(Rref, axis_rot(u, 10.0))     # 【体系】旋转 = 绕笔杆自身轴
    e1 = euler_of(Rn)
    d = [e1[i] - e0[i] for i in range(3)]
    rows.append((nm, d))
    print(f"{'器件 ' + nm + ' 转 +10°':18s} ({d[0]:+7.2f},{d[1]:+7.2f},{d[2]:+7.2f})   "
          f"({d[0]:+7.2f}, {d[2]:+7.2f}, {d[1]:+7.2f})")

print("\n=== 同样的物理动作，现任实现给每个关节的【增益】（度/度）===")
for nm, d in rows:
    print(f"  绕器件 {nm} 转 10° ⇒ J4 {d[0]/10:+.2f} · J5 {d[2]/10:+.2f} · J6 {d[1]/10:+.2f}")
print("\n（若修成'器件轴旋转向量分解'，这三行应当变成 10° 只喂一个关节、增益 1.00。）")
