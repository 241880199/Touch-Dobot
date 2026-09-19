#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计算笔夹工具链的质量、质心与【惯量张量】(相对机器人法兰), 供
EnableRobot(load,cx,cy,cz) 与 PayLoad(weight,inertia) 使用。

坐标系 = pen_clamp.scad 的装配坐标系:
    +z 朝上(指向机械臂), z=0 是打印法兰的底面, z=FLANGE_T(6) 是法兰顶面/传感器贴合面。
    机器人法兰面在 z = ROBOT_FACE_Z (传感器本体顶面 = 6 + 24.5 = 30.5)。

【惯量张量的坐标系与参考点】—— 与上面同向, 只平移:
    - 轴向: 与装配系同向。+x/+y 同装配系; +z 指向机械臂(法兰内侧), 工具本体伸向 -z。
    - 原点: 机器人法兰面中心, 即装配系 (0, 0, 30.5)。张量【绕法兰面中心】, 不是绕质心
      (机械臂要的是绕法兰工具系原点的载荷惯量)。
    - 单位 kg·m^2 (内部按 g·mm^2 累加, 打印时 ×1e-9)。
    若厂家把法兰工具系的 +z 定义为【法兰向外】(与本文相反), 则 Ixz/Iyz 变号; 本装配的近轴
    对称性使总质心的 x,y < 0.05 mm, 这两个分量 ~0, 两种约定可互换。Ixx/Iyy/Izz 不受影响。

几何来源:
    - 打印件 (flange/block/collet_s/ring): 读 Hardware/stl/*.stl, 逐四面体累加算体积 + 质心 +
      绕网格原点的二阶矩 —— 对【三角网格本身】是精确积分, 不是近似 (Mirtich 1996)
    - 传感器 / 光轴 / 笔: 解析实心圆柱 (绕自身质心的闭式解 + 平行轴)
    - 紧固件: 解析【质点】, 即忽略其自身尺寸只有平行轴项 (见 FASTENER 注释里的量级估计)

用法:
    python Hardware/tools/compute_payload.py [--block-z -150] [--pen-mass-g 9.0] [--selftest]

改动硬件后重跑本脚本, 把结果填进 Touch_Client/config/Config.h 的
ROBOT_PAYLOAD_KG / ROBOT_PAYLOAD_CX_MM / ROBOT_PAYLOAD_CY_MM / ROBOT_PAYLOAD_CZ_MM;
惯量候选值 (见输出末段) 发给 PayLoad(weight, inertia)。
"""

import argparse
import math
import os
import struct
import sys

# ===== 材料密度 (g/mm^3) =====
RHO_PETG = 1.27e-3   # 打印件 (PETG)
RHO_STEEL = 7.85e-3  # 45# 钢 (光轴)
RHO_BAMBOO = 0.8e-3  # 毛笔杆 (竹/木) — 建议用秤实测后覆盖

# ===== 传感器 KWR75B (力传感器外形参数.md + 用户手册) =====
SENSOR_MASS_G = 280.0   # 用户手册: 重量 0.28 kg
SENSOR_BODY_D = 66.0    # 法兰外径 Φ66 (主导体积段)
SENSOR_BODY_H = 24.5    # 法兰安装高度 (自贴合面向上)
SENSOR_BOSS_D = 31.5    # 顶部凸台 Φ31.5
SENSOR_BOSS_H = 7.0     # 凸台高 = 31.5(总高) - 24.5

# ===== 打印件/装配几何 (pen_clamp.scad 参数) =====
FLANGE_T = 6.0        # 法兰板厚; 也是传感器贴合面的 z
ROD_D, ROD_L = 8.0, 190.0
ROD_R = 31.0          # 光轴心距中心 (scad ROD_R)
ROD_ANG = -45.0       # 光轴相对 +X 轴的角度 deg (scad ROD_ANG) —— 两根在 -45°/135°
PEN_D, PEN_L = 8.0, 220.0
COLLET_HEAD_L, RING_L = 20.0, 10.0
# 装配里块/夹头/环的 z 偏移(相对块自身原点): 环顶贴块底
RING_OFFSET_FROM_BLOCK = -(COLLET_HEAD_L + RING_L)   # 环在块坐标系下的偏移 (-30)

# 紧固件(按 README 最终 BOM 估算) — 分两处, 别都堆在法兰附近:
#   法兰侧: M6x16 x4 (~22g) + Φ6 销 x2 (~5g) + 耳座顶丝/螺母 x4 (~5g)
#   块/环侧: M4x35 x2 (~9g) + M4x12 顶丝 x3 (~4.5g) + 锁高顶丝/螺母 x3 (~3.5g)
#            + 环径向螺母 x3 (~3.3g)  ≈ 20g, 位置随 --block-z 移动
# 惯量上按【质点】处理 (只用平行轴项)。自身尺寸的贡献 ~ m·(d²+h²)/12:
#   M6x16 单颗 ~ m·(6²+16²)/12 ≈ 25 g·mm², 8 颗加起来 ~0.2 kg·mm²,
#   比它们的平行轴项 (法兰侧 32g × 28.5² ≈ 26000 g·mm²) 小两个数量级, 可忽略。
FLANGE_FASTENER_MASS_G = 32.0
FLANGE_FASTENER_Z = 2.0
BLOCK_FASTENER_MASS_G = 20.0
BLOCK_FASTENER_DZ = -15.0   # 相对块底 (块顶螺母 / 锁紧环 / 锁高螺母的大致重心)

STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl")


# ===== 3x3 张量小工具 (全部 g·mm^2 量纲) =====

def zeros3():
    return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]


def add3(A, B):
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]


def paxis(m, r):
    """平行轴增量 m·(|r|^2·δij − ri·rj)。r 是【参考点指向质心】的向量 (mm)。"""
    r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    return [[m * ((r2 if i == j else 0.0) - r[i] * r[j]) for j in range(3)] for i in range(3)]


def cylinder_icm(m, radius, height):
    """实心圆柱绕【自身质心】的惯量, 轴向 = z: Izz = m r²/2, Ixx = Iyy = m(3r²+h²)/12。"""
    izz = 0.5 * m * radius * radius
    ixx = m * (3.0 * radius * radius + height * height) / 12.0
    return [[ixx, 0.0, 0.0], [0.0, ixx, 0.0], [0.0, 0.0, izz]]


def jacobi_eig(A):
    """对称 3x3 的 Jacobi 特征分解。返回 (特征值升序, 每个特征值对应的单位特征向量)。"""
    a = [row[:] for row in A]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(64):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        diag = abs(a[0][0]) + abs(a[1][1]) + abs(a[2][2])
        if off <= 1e-16 * (diag + 1e-300):
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            apq = a[p][q]
            if abs(apq) < 1e-300:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * apq)
            t = (1.0 if theta >= 0.0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):                      # 列旋转
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            for k in range(3):                      # 行旋转
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
            for k in range(3):                      # 特征向量同步
                vkp, vkq = v[k][p], v[k][q]
                v[k][p] = c * vkp - s * vkq
                v[k][q] = s * vkp + c * vkq
    idx = sorted(range(3), key=lambda i: a[i][i])
    return [a[i][i] for i in idx], [[v[r][i] for r in range(3)] for i in idx]


# ===== STL =====

def read_stl(path):
    """返回三角形列表 [(v0,v1,v2), ...], 每个顶点是 (x,y,z)。支持二进制与 ASCII。"""
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84:
            return []
        # 二进制 STL: 80 字节头 + uint32 面数; 面数校验决定格式
        n = struct.unpack("<I", head[80:84])[0]
        if 84 + n * 50 == os.path.getsize(path):
            tris = []
            f.seek(84)
            data = f.read()
            for i in range(n):
                off = i * 50 + 12  # 跳过法向
                vals = struct.unpack_from("<9f", data, off)
                tris.append((vals[0:3], vals[3:6], vals[6:9]))
            return tris
    # ASCII 回退
    tris, cur = [], []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if s.startswith("vertex"):
                p = s.split()
                cur.append((float(p[1]), float(p[2]), float(p[3])))
                if len(cur) == 3:
                    tris.append(tuple(cur))
                    cur = []
    return tris


def volume_centroid_cov(tris):
    """闭合三角网格: 有符号体积 (mm^3)、质心 (mm)、绕网格原点的二阶矩协方差 C (mm^5)。

    C_ij = ∫ x_i x_j dV。逐四面体累加 (Mirtich 1996): 四面体 (0, a, b, c) 的有符号体积
    V = det(a,b,c)/6, 而 (在单位单纯形上 ∫u² = 1/60, ∫uv = 1/120, 雅可比 = 6V)
        ∫ x_i x_j dV = (V/20)·[ 2(aᵢaⱼ+bᵢbⱼ+cᵢcⱼ) + (aᵢbⱼ+bᵢaⱼ) + (aᵢcⱼ+cᵢaⱼ) + (bᵢcⱼ+cᵢbⱼ) ]
    (系数 1/20 由 --selftest 用立方体解析解 + 蒙特卡洛核对过; 写成 1/4 会让惯量正好大 5 倍。)
    对【任意】参考原点成立 (网格闭合, 散度定理把手原点那项消掉), 这里原点取网格自身原点,
    坐标小, 无大数相消。

    实心体绕原点的惯量 = ρ·(tr(C)·δ − C), 其中 ρ = 质量/体积。
    """
    vol6 = 0.0
    cx = cy = cz = 0.0
    C = zeros3()
    for a, b, c in tris:
        d = (a[0] * (b[1] * c[2] - b[2] * c[1])
             - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0]))
        vol6 += d
        w = d / 4.0  # 四面体质心 = (0+a+b+c)/4
        cx += w * (a[0] + b[0] + c[0])
        cy += w * (a[1] + b[1] + c[1])
        cz += w * (a[2] + b[2] + c[2])
        v6 = d / 6.0  # 有符号体积
        for i in range(3):
            for j in range(i, 3):
                q = (2.0 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j])
                     + (a[i] * b[j] + b[i] * a[j])
                     + (a[i] * c[j] + c[i] * a[j])
                     + (b[i] * c[j] + c[i] * b[j]))
                C[i][j] += v6 * q / 20.0
    for i in range(3):
        for j in range(i):
            C[i][j] = C[j][i]
    vol = vol6 / 6.0
    if abs(vol) < 1e-9:
        return 0.0, (0.0, 0.0, 0.0), C
    return vol, (cx / vol6, cy / vol6, cz / vol6), C


def stl_part(tris, rho, dz):
    """打印件: 返回 (质量 g, 装配系质心 (x,y,z), 绕【自身质心】的惯量张量 g·mm^2)。

    STL 的 x/y 已是装配系坐标, 只有 z 需要加装配偏移 dz。
    """
    vol, (cx, cy, cz), C = volume_centroid_cov(tris)
    if vol < 0.0:  # 绕序反了 -> 有符号体积与二阶矩同时反号
        C = [[-C[i][j] for j in range(3)] for i in range(3)]
        vol = -vol
    m = vol * rho
    tr = C[0][0] + C[1][1] + C[2][2]
    I_mesh = [[rho * ((tr if i == j else 0.0) - C[i][j]) for j in range(3)] for i in range(3)]
    # 网格原点 -> 零件质心 (减掉平行轴项)
    I_cm = add3(I_mesh, [[-x for x in row] for row in paxis(m, (cx, cy, cz))])
    return m, (cx, cy, cz + dz), I_cm


def bbox(tris):
    pts = [v for t in tris for v in t]
    return (min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts),
            min(p[2] for p in pts), max(p[2] for p in pts))


def selftest():
    """用解析解校验二阶矩累加与平行轴搬移 (惯量算错是最可能的失效点)。"""
    def box_tris(sx, sy, sz):
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        v = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
             (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
        quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                 (2, 3, 7, 6), (1, 2, 6, 5), (0, 4, 7, 3)]
        out = []
        for a, b, c, d in quads:
            out.append((v[a], v[b], v[c]))
            out.append((v[a], v[c], v[d]))
        return out

    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  [%s] %-34s got=%.6e want=%.6e" % ("ok" if good else "FAIL", name, got, want))

    sx, sy, sz, rho = 10.0, 20.0, 30.0, 1.0
    tris = box_tris(sx, sy, sz)
    m, c, I = stl_part(tris, rho, 0.0)
    print("自检 (解析解比对):")
    check("box 体积", m / rho, sx * sy * sz, 1e-9)
    check("box 质心 x", c[0], 0.0, 1e-12)
    check("box Ixx_cm (m(sy²+sz²)/12)", I[0][0], m * (sy * sy + sz * sz) / 12.0, 1e-9)
    check("box Iyy_cm", I[1][1], m * (sx * sx + sz * sz) / 12.0, 1e-9)
    check("box Izz_cm", I[2][2], m * (sx * sx + sy * sy) / 12.0, 1e-9)
    check("box Ixy_cm = 0", I[0][1], 0.0, 1e-12)

    # 平行轴: 沿 +z 搬 d, Ixx/Iyy += m d², Izz 不变
    d = 37.0
    J = add3(I, paxis(m, (0.0, 0.0, d)))
    check("paxis Ixx (+m d²)", J[0][0], I[0][0] + m * d * d, 1e-9)
    check("paxis Izz (不变)", J[2][2], I[2][2], 1e-12)

    # 解析圆柱: 细长杆 Izz << Ixx
    mc, r, h = 75.0, 4.0, 190.0
    Ic = cylinder_icm(mc, r, h)
    check("rod Izz (m r²/2)", Ic[2][2], mc * r * r / 2.0, 1e-12)
    check("rod Ixx (m(3r²+h²)/12)", Ic[0][0], mc * (3 * r * r + h * h) / 12.0, 1e-12)
    check("rod Izz/Ixx 比 (=6r²/(3r²+h²))", Ic[2][2] / Ic[0][0],
          6.0 * r * r / (3.0 * r * r + h * h), 1e-12)

    # Jacobi: diag(3,1,2) 的特征值应为 1,2,3; 旋转过的也要对
    lam, vec = jacobi_eig([[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]])
    check("jacobi λ1", lam[0], 1.0, 1e-12)
    check("jacobi λ3", lam[2], 3.0, 1e-12)
    ang = 0.7
    ca, sa = math.cos(ang), math.sin(ang)
    R = [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]]
    A = [[sum(R[i][k] * [5.0, 0, 0][k] * R[j][k] for k in range(3))
          + sum(R[i][k] * [0, 2.0, 0][k] * R[j][k] for k in range(3))
          + sum(R[i][k] * [0, 0, 9.0][k] * R[j][k] for k in range(3)) for j in range(3)]
         for i in range(3)]
    lam2, _ = jacobi_eig(A)
    check("jacobi 旋转阵 λ1", lam2[0], 2.0, 1e-9)
    check("jacobi 旋转阵 λ3", lam2[2], 9.0, 1e-9)
    check("jacobi 迹守恒", sum(lam2), 16.0, 1e-9)

    print("自检%s" % ("" if ok else " ** 失败 **"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--block-z", type=float, default=-150.0,
                    help="滑动块底面 z (装配默认 -150; 实际按夹持点位置)")
    ap.add_argument("--pen-mass-g", type=float, default=None,
                    help="毛笔实测质量 (g); 不给则按竹木密度估算。同时进入质量/质心/惯量")
    ap.add_argument("--petg", type=float, default=RHO_PETG, help="PETG 密度 g/mm^3")
    ap.add_argument("--selftest", action="store_true", help="跑解析解自检后退出")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)

    parts = []  # (名称, 质量g, (质心x, 质心y, 质心z) 装配系, 绕自身质心的惯量 g·mm²)

    # --- 打印件: 直接读 STL ---
    offsets = {
        "flange": 0.0,
        "block": args.block_z,
        "collet_s": args.block_z,
        "ring": args.block_z + RING_OFFSET_FROM_BLOCK,
    }
    for name, dz in offsets.items():
        path = os.path.normpath(os.path.join(STL_DIR, name + ".stl"))
        tris = read_stl(path)
        if not tris:
            # 缺件会让总量静默偏小, 直接失败而不是给个错数
            print("!! 读不到或解析失败: %s" % path, file=sys.stderr)
            sys.exit(1)
        mass, (cx, cy, cz), I_cm = stl_part(tris, args.petg, dz)
        parts.append(("%-9s (STL)" % name, mass, (cx, cy, cz), I_cm))
        bb = bbox(tris)
        print("# %-9s vol=%8.0f mm^3  mass=%6.1f g  bbox z=[%.1f, %.1f]  -> z=%.1f"
              % (name, mass / args.petg, mass, bb[4], bb[5], cz))

    # --- 传感器: 本体 + 凸台, 总质量固定 0.28kg, 按体积分配 ---
    # 惯量按【实心圆柱】估。KWR75B 的弹性体是环状, 质量比实心盘更靠外, 故 Izz 是真值的下界:
    #   实心盘 m r²/2 (=143 g·mm²·10³) … 薄环 m r² (=286)。Ixx/Iyy 受此影响 <1%。
    v_body = math.pi * (SENSOR_BODY_D / 2) ** 2 * SENSOR_BODY_H
    v_boss = math.pi * (SENSOR_BOSS_D / 2) ** 2 * SENSOR_BOSS_H
    m_body = SENSOR_MASS_G * v_body / (v_body + v_boss)
    m_boss = SENSOR_MASS_G * v_boss / (v_body + v_boss)
    z_body = FLANGE_T + SENSOR_BODY_H / 2.0            # 本体: z ∈ [6, 30.5]
    z_boss = FLANGE_T - SENSOR_BOSS_H / 2.0            # 凸台: z ∈ [-1, 6]
    parts.append(("sensor body", m_body, (0.0, 0.0, z_body),
                  cylinder_icm(m_body, SENSOR_BODY_D / 2, SENSOR_BODY_H)))
    parts.append(("sensor boss", m_boss, (0.0, 0.0, z_boss),
                  cylinder_icm(m_boss, SENSOR_BOSS_D / 2, SENSOR_BOSS_H)))

    # --- 光轴 2×: x/y 上抵消(总质心), 但【惯量不抵消】—— 两根各在心距 ROD_R 处,
    #     绕法兰 z 轴的 Izz 主要由这个半径贡献。合成张量在自身质心处保留 Ixy 项。---
    rod_m = math.pi * (ROD_D / 2) ** 2 * ROD_L * RHO_STEEL
    px = ROD_R * math.cos(math.radians(ROD_ANG))
    py = ROD_R * math.sin(math.radians(ROD_ANG))
    rod_own = cylinder_icm(rod_m, ROD_D / 2, ROD_L)
    I_rods = zeros3()
    for sx, sy in ((px, py), (-px, -py)):
        I_rods = add3(I_rods, add3(rod_own, paxis(rod_m, (sx, sy, 0.0))))
    parts.append(("guide rod x2", 2 * rod_m, (0.0, 0.0, -ROD_L / 2.0), I_rods))

    # --- 笔: 实心圆柱 Φ8×220 (竹杆, 质量可用 --pen-mass-g 覆盖) ---
    pen_geo_m = math.pi * (PEN_D / 2) ** 2 * PEN_L * RHO_BAMBOO
    pen_m = args.pen_mass_g if args.pen_mass_g else pen_geo_m
    parts.append(("pen Φ8x220", pen_m, (0.0, 0.0, -PEN_L / 2.0 - 1.0),
                  cylinder_icm(pen_m, PEN_D / 2, PEN_L)))

    # --- 紧固件 (分法兰侧 / 块环侧两处, 后者随 block_z 走; 质点模型) ---
    parts.append(("fasteners@flange", FLANGE_FASTENER_MASS_G, (0.0, 0.0, FLANGE_FASTENER_Z),
                  zeros3()))
    parts.append(("fasteners@block", BLOCK_FASTENER_MASS_G,
                  (0.0, 0.0, args.block_z + BLOCK_FASTENER_DZ), zeros3()))

    # --- 合成 ---
    M = sum(p[1] for p in parts)
    CX = sum(p[1] * p[2][0] for p in parts) / M
    CY = sum(p[1] * p[2][1] for p in parts) / M
    CZ = sum(p[1] * p[2][2] for p in parts) / M

    robot_face_z = FLANGE_T + SENSOR_BODY_H  # 传感器顶面 = 机器人法兰面

    # --- 惯量: 各零件绕【法兰面中心】的贡献 = I_cm + 平行轴(法兰面中心 -> 零件质心) ---
    I_flange = zeros3()   # 绕法兰面中心 (发给机械臂的就是它)
    I_com = zeros3()      # 绕总质心 (参考量, 与参考点选取无关的部分)
    part_izz = []         # 每个零件对法兰系 Izz 的贡献, 便于手算核对
    for name, m, (x, y, z), I_cm in parts:
        I_flange = add3(I_flange, add3(I_cm, paxis(m, (x, y, z - robot_face_z))))
        I_com = add3(I_com, add3(I_cm, paxis(m, (x - CX, y - CY, z - CZ))))
        d2 = x * x + y * y
        part_izz.append(I_cm[2][2] + m * d2)

    print("\n%-14s %10s %10s %14s" % ("part", "mass(g)", "z(mm)", "Izz(kg·m²)"))
    for (name, m, (x, y, z), _), izz in zip(parts, part_izz):
        print("%-14s %10.1f %10.1f %14.3e" % (name, m, z, izz * 1e-9))
    print("-" * 52)
    print("%-14s %10.1f %10.1f %14.3e" % ("TOTAL", M, CZ, I_flange[2][2] * 1e-9))
    print("\n质心 (scad 坐标系): x=%.2f y=%.2f z=%.2f mm" % (CX, CY, CZ))
    print("机器人法兰面 z = %.1f mm" % robot_face_z)
    d = robot_face_z - CZ

    # --- 惯量张量打印 ---
    print("\n=== 惯量张量 (kg·m²), 参考点 = 机器人法兰面中心 ===")
    print("轴向同装配系: +z 指向机械臂(法兰内侧), 工具本体伸向 -z; 原点在装配系 z=%.1f 处" % robot_face_z)
    print("%12s %12s %12s" % ("", "x", "y"))
    for i, rn in enumerate("xyz"):
        print("%12s %12.6e %12.6e %12.6e" % (rn, I_flange[i][0] * 1e-9,
                                             I_flange[i][1] * 1e-9, I_flange[i][2] * 1e-9))
    print("对称性检查 max|Iij-Iji| = %.3e kg·m²"
          % (max(abs(I_flange[i][j] - I_flange[j][i]) for i in range(3) for j in range(3)) * 1e-9))

    lam, axes = jacobi_eig(I_flange)
    lam_c, _ = jacobi_eig(I_com)
    print("\n绕法兰面: Ixx=%.6e  Iyy=%.6e  Izz=%.6e kg·m²"
          % (I_flange[0][0] * 1e-9, I_flange[1][1] * 1e-9, I_flange[2][2] * 1e-9))
    print("绕总质心: Ixx=%.6e  Iyy=%.6e  Izz=%.6e kg·m²"
          % (I_com[0][0] * 1e-9, I_com[1][1] * 1e-9, I_com[2][2] * 1e-9))
    print("\n主惯量 (绕法兰面, 升序) 与主轴 (法兰系单位向量; 特征向量的正负号任意):")
    for k in range(3):
        print("  λ%d = %.6e kg·m²   轴 = (%+.4f, %+.4f, %+.4f)"
              % (k + 1, lam[k] * 1e-9, axes[k][0], axes[k][1], axes[k][2]))
    print("  迹 = %.6e kg·m² (三轴和, 应 = Ixx+Iyy+Izz = %.6e)"
          % (sum(lam) * 1e-9, (I_flange[0][0] + I_flange[1][1] + I_flange[2][2]) * 1e-9))
    print("  绕质心的主惯量 = %.6e / %.6e / %.6e kg·m²" % tuple(x * 1e-9 for x in lam_c))

    # --- PayLoad 候选 ---
    print("\n=== PayLoad(weight, inertia) 的候选 (inertia 是单标量 kg·m², 厂家文档未定义取哪个) ===")
    print("  绕法兰面  Ixx = %.6e" % (I_flange[0][0] * 1e-9))
    print("  绕法兰面  Iyy = %.6e" % (I_flange[1][1] * 1e-9))
    print("  绕法兰面  Izz = %.6e   <- 与绕质心的 Izz 几乎相同 (质心只在 z 轴上, x,y < 0.05 mm)"
          % (I_flange[2][2] * 1e-9))
    print("  绕质心    Ixx = %.6e" % (I_com[0][0] * 1e-9))
    print("  绕质心    Iyy = %.6e" % (I_com[1][1] * 1e-9))
    print("  主惯量    λ1/λ2/λ3 = %.6e / %.6e / %.6e" % tuple(x * 1e-9 for x in lam))
    print("  迹/3 (各向同性等效) = %.6e" % (sum(lam) / 3.0 * 1e-9))
    print("实机上以 P 塌到 0 为判据确定厂家取的是哪一个 (或查厂家确认), 本脚本不猜。")
    print("不确定度: 传感器按实心圆柱 (Izz 下界, 薄环模型为其 2 倍, 差 ~1.4e-4 kg·m²);")
    print("          打印件为网格精确值; 紧固件按质点 (自身尺寸项 <0.3%)。")

    print("\n=== 填入 Config.h ===")
    print("ROBOT_PAYLOAD_KG        = %.2f" % (M / 1000.0))
    print("ROBOT_PAYLOAD_CX_MM     = %.1f" % CX)
    print("ROBOT_PAYLOAD_CY_MM     = %.1f" % CY)
    print("ROBOT_PAYLOAD_CZ_MM     = %.1f   # 沿工具轴(法兰向外)为正" % d)
    print("ROBOT_PAYLOAD_CZ_MM_ALT = %.1f   # 若约定相反则用此值" % (-d))
    print("\n(质心在法兰下方 %.1f mm; 若 EnableRobot 的 Z 约定是法兰向内为正, 取负值)" % d)


if __name__ == "__main__":
    main()
