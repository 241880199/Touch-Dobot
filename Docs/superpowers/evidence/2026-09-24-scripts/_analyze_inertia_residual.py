# -*- coding: utf-8 -*-
"""残差里还剩多少"未补偿的 m·a"？
把 `filtered`（补偿后的力）对 `acc`（运动估计器的加速度，先按 R^T 映到传感器系）做最小二乘，
看斜率 k 是多少 —— k≈0 表示惯量项确实被抵掉；k≈工具质量(0.66kg) 表示【一点没抵】。"""
import csv, math

CSV = r"D:\Projects\Touch\Touch_Client\force_demo_log.csv"
rows = []
for r in csv.DictReader(open(CSV, newline="")):
    try:
        rows.append({k: float(v) for k, v in r.items()})
    except (TypeError, ValueError):
        continue

rows = [r for r in rows if r["ff_enabled"] == 1]
print(f"样本 {len(rows)} 条（ff_enabled=1）")

# 姿态阶梯上的真实速度（Δpose / 该段实际经过的时间）
prev = None
for r in rows:
    if prev is None:
        r["v"] = 0.0
    else:
        dt = (r["t_ms"] - prev["t_ms"]) / 1000.0
        d = math.dist((r["pose_x"], r["pose_y"], r["pose_z"]),
                      (prev["pose_x"], prev["pose_y"], prev["pose_z"]))
        r["v"] = d / dt if dt > 0.02 else r["v"] if "v" in r else 0.0
    prev = r


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


for r in rows:
    r["amag"] = math.sqrt(r["acc_x"] ** 2 + r["acc_y"] ** 2 + r["acc_z"] ** 2)

print("\n=== 按 |acc| 分档（m/s²）===")
BANDS = [(0, 0.01), (0.01, 0.05), (0.05, 0.15), (0.15, 0.3), (0.3, 1e9)]
for lo, hi in BANDS:
    seg = [r for r in rows if lo <= r["amag"] < hi]
    if not seg:
        print(f"  |acc| [{lo},{hi})  无")
        continue
    F = [math.sqrt(r["Fx"] ** 2 + r["Fy"] ** 2 + r["Fz"] ** 2) for r in seg]
    m = sum(F) / len(F)
    sd = math.sqrt(sum((x - m) ** 2 for x in F) / max(len(F) - 1, 1))
    print(f"  |acc| [{lo},{hi:<5}) n={len(seg):5d}  |F| 均值 {m:6.3f}  sd {sd:6.3f}  最大 {max(F):6.3f}")

# 最小二乘：F_i ≈ k * aS_i（传感器系），只用有明显运动的样本
sel = [r for r in rows if r["amag"] > 0.05]
print(f"\n=== 最小二乘（|acc|>0.05 的 {len(sel)} 条）===")
for axis, col in enumerate(["Fx", "Fy", "Fz"]):
    num = den = 0.0
    for r in sel:
        Rm = R_of(r["pose_rx"], r["pose_ry"], r["pose_rz"])
        # R 行主序；传感器系分量 aS = R^T · acc
        acc = (r["acc_x"], r["acc_y"], r["acc_z"])
        aS = [Rm[0][axis] * acc[0] + Rm[1][axis] * acc[1] + Rm[2][axis] * acc[2]]
        num += aS[0] * r[col]
        den += aS[0] * aS[0]
    k = num / den if den > 1e-12 else float("nan")
    # R²
    ss_tot = sum((r[col] - sum(x[col] for x in sel) / len(sel)) ** 2 for r in sel)
    ss_res = 0.0
    for r in sel:
        Rm = R_of(r["pose_rx"], r["pose_ry"], r["pose_rz"])
        acc = (r["acc_x"], r["acc_y"], r["acc_z"])
        aS = Rm[0][axis] * acc[0] + Rm[1][axis] * acc[1] + Rm[2][axis] * acc[2]
        ss_res += (r[col] - k * aS) ** 2
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    print(f"  {col} ≈ k·(R^T·acc)_{axis}:  k = {k:+7.3f} kg   R² = {r2:5.3f}")

print("\n参考：工具链质量估计 Config::ROBOT_PAYLOAD_SEED_KG = 0.66 kg")
print("     若 k ≈ ±0.66 ⇒ 惯量项【完全没被抵掉】；k ≈ 0 ⇒ 已抵掉，残差另有来源。")
