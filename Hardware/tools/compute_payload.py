#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计算笔夹工具链的质量与质心 (相对机器人法兰), 供 EnableRobot(load,cx,cy,cz) 使用。

坐标系 = pen_clamp.scad 的装配坐标系:
    +z 朝上(指向机械臂), z=0 是打印法兰的底面, z=FLANGE_T(6) 是法兰顶面/传感器贴合面。
    机器人法兰面在 z = ROBOT_FACE_Z (传感器本体顶面)。

几何来源:
    - 打印件 (flange/block/collet_s/ring): 直接读 Hardware/stl/*.stl, 按体积分算体积+质心
    - 传感器 / 光轴 / 笔 / 紧固件: 按解析圆柱 + 已知质量

用法:
    python Hardware/tools/compute_payload.py [--block-z -150] [--pen-mass-g 9.0]

改动硬件后重跑本脚本, 把结果填进 Touch_Client/config/Config.h 的
ROBOT_PAYLOAD_KG / ROBOT_PAYLOAD_CX_MM / ROBOT_PAYLOAD_CY_MM / ROBOT_PAYLOAD_CZ_MM。
"""

import argparse
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
PEN_D, PEN_L = 8.0, 220.0
COLLET_HEAD_L, RING_L = 20.0, 10.0
# 装配里块/夹头/环的 z 偏移(相对块自身原点): 环顶贴块底
RING_OFFSET_FROM_BLOCK = -(COLLET_HEAD_L + RING_L)   # 环在块坐标系下的偏移 (-30)

# 紧固件(按 README 最终 BOM 估算) — 分两处, 别都堆在法兰附近:
#   法兰侧: M6x16 x4 (~22g) + Φ6 销 x2 (~5g) + 耳座顶丝/螺母 x4 (~5g)
#   块/环侧: M4x35 x2 (~9g) + M4x12 顶丝 x3 (~4.5g) + 锁高顶丝/螺母 x3 (~3.5g)
#            + 环径向螺母 x3 (~3.3g)  ≈ 20g, 位置随 --block-z 移动
FLANGE_FASTENER_MASS_G = 32.0
FLANGE_FASTENER_Z = 2.0
BLOCK_FASTENER_MASS_G = 20.0
BLOCK_FASTENER_DZ = -15.0   # 相对块底 (块顶螺母 / 锁紧环 / 锁高螺母的大致重心)

STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl")


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


def volume_centroid(tris):
    """闭合三角网格的有符号体积 (mm^3) 与质心 (mm)。散度定理。"""
    vol6 = 0.0
    cx = cy = cz = 0.0
    for a, b, c in tris:
        d = (a[0] * (b[1] * c[2] - b[2] * c[1])
             - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0]))
        vol6 += d
        w = d / 4.0  # 四面体质心 = (0+a+b+c)/4
        cx += w * (a[0] + b[0] + c[0])
        cy += w * (a[1] + b[1] + c[1])
        cz += w * (a[2] + b[2] + c[2])
    vol = vol6 / 6.0
    if abs(vol) < 1e-9:
        return 0.0, (0.0, 0.0, 0.0)
    return vol, (cx / vol6, cy / vol6, cz / vol6)


def bbox(tris):
    pts = [v for t in tris for v in t]
    return (min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts),
            min(p[2] for p in pts), max(p[2] for p in pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--block-z", type=float, default=-150.0,
                    help="滑动块底面 z (装配默认 -150; 实际按夹持点位置)")
    ap.add_argument("--pen-mass-g", type=float, default=None,
                    help="毛笔实测质量 (g); 不给则按竹木密度估算")
    ap.add_argument("--petg", type=float, default=RHO_PETG, help="PETG 密度 g/mm^3")
    args = ap.parse_args()

    parts = []  # (名称, 质量g, 质心z, 质心x, 质心y)

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
        vol, (cx, cy, cz) = volume_centroid(tris)
        mass = abs(vol) * args.petg
        parts.append(("%-9s (STL)" % name, mass, cz + dz, cx, cy))
        bb = bbox(tris)
        print("# %-9s vol=%8.0f mm^3  mass=%6.1f g  bbox z=[%.1f, %.1f]  -> z=%.1f"
              % (name, vol, mass, bb[4], bb[5], cz + dz))

    # --- 传感器: 本体 + 凸台, 总质量固定 0.28kg, 按体积分配 ---
    v_body = 3.141592653589793 * (SENSOR_BODY_D / 2) ** 2 * SENSOR_BODY_H
    v_boss = 3.141592653589793 * (SENSOR_BOSS_D / 2) ** 2 * SENSOR_BOSS_H
    m_body = SENSOR_MASS_G * v_body / (v_body + v_boss)
    m_boss = SENSOR_MASS_G * v_boss / (v_body + v_boss)
    z_body = FLANGE_T + SENSOR_BODY_H / 2.0            # 本体: z ∈ [6, 30.5]
    z_boss = FLANGE_T - SENSOR_BOSS_H / 2.0            # 凸台: z ∈ [-1, 6]
    parts.append(("sensor body", m_body, z_body, 0.0, 0.0))
    parts.append(("sensor boss", m_boss, z_boss, 0.0, 0.0))

    # --- 光轴 2× (对称, x/y 抵消) ---
    rod_m = 3.141592653589793 * (ROD_D / 2) ** 2 * ROD_L * RHO_STEEL
    parts.append(("guide rod x2", 2 * rod_m, -ROD_L / 2.0, 0.0, 0.0))

    # --- 笔 ---
    pen_geo_m = 3.141592653589793 * (PEN_D / 2) ** 2 * PEN_L * RHO_BAMBOO
    pen_m = args.pen_mass_g if args.pen_mass_g else pen_geo_m
    parts.append(("pen Φ8x220", pen_m, -PEN_L / 2.0 - 1.0, 0.0, 0.0))

    # --- 紧固件 (分法兰侧 / 块环侧两处, 后者随 block_z 走) ---
    parts.append(("fasteners@flange", FLANGE_FASTENER_MASS_G, FLANGE_FASTENER_Z, 0.0, 0.0))
    parts.append(("fasteners@block", BLOCK_FASTENER_MASS_G,
                  args.block_z + BLOCK_FASTENER_DZ, 0.0, 0.0))

    # --- 合成 ---
    M = sum(p[1] for p in parts)
    CX = sum(p[1] * p[3] for p in parts) / M
    CY = sum(p[1] * p[4] for p in parts) / M
    CZ = sum(p[1] * p[2] for p in parts) / M

    robot_face_z = FLANGE_T + SENSOR_BODY_H  # 传感器顶面 = 机器人法兰面

    print("\n%-14s %10s %10s" % ("part", "mass(g)", "z(mm)"))
    for name, m, z, x, y in parts:
        print("%-14s %10.1f %10.1f" % (name, m, z))
    print("-" * 36)
    print("%-14s %10.1f" % ("TOTAL", M))
    print("\n质心 (scad 坐标系): x=%.2f y=%.2f z=%.2f mm" % (CX, CY, CZ))
    print("机器人法兰面 z = %.1f mm" % robot_face_z)
    d = robot_face_z - CZ
    print("\n=== 填入 Config.h ===")
    print("ROBOT_PAYLOAD_KG        = %.2f" % (M / 1000.0))
    print("ROBOT_PAYLOAD_CX_MM     = %.1f" % CX)
    print("ROBOT_PAYLOAD_CY_MM     = %.1f" % CY)
    print("ROBOT_PAYLOAD_CZ_MM     = %.1f   # 沿工具轴(法兰向外)为正" % d)
    print("ROBOT_PAYLOAD_CZ_MM_ALT = %.1f   # 若约定相反则用此值" % (-d))
    print("\n(质心在法兰下方 %.1f mm; 若 EnableRobot 的 Z 约定是法兰向内为正, 取负值)" % d)


if __name__ == "__main__":
    main()
