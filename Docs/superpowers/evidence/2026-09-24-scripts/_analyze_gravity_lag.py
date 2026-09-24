# -*- coding: utf-8 -*-
"""重力补偿滞后假说：姿态是 100ms 阶梯、而关节模式在快速转腕
⇒ 一步台阶内 `Fg = A·g(θ_k)` 用的是【旧姿态】⇒ 残差 = A·(g(θ_now) − g(θ_stale))。
离线算：每一步台阶的 |ΔFg|，再看它与观测到的 |F| 是否对得上。"""
import csv, math, json

CSV = r"D:\Projects\Touch\Touch_Client\force_demo_log.csv"
CAL = r"D:\Projects\Touch\Touch_Client\calib\force_calib.json"
G = 9.81
OFF = 64513200.0   # 墙钟偏移（上次对齐得到的）

cal = json.load(open(CAL))
A = cal["a_matrix"]
det = (A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6])
       + A[2] * (A[3] * A[7] - A[4] * A[6]))
print(f"A 的 det = {det:.5f}   massScale = |det|^(1/3) = {abs(det) ** (1/3):.3f} kg")

rows = []
for r in csv.DictReader(open(CSV, newline="")):
    try:
        rows.append({k: float(v) for k, v in r.items()})
    except (TypeError, ValueError):
        continue
for r in rows:
    r["F"] = math.sqrt(r["Fx"] ** 2 + r["Fy"] ** 2 + r["Fz"] ** 2)
    r["wall"] = (r["t_ms"] + OFF) / 1000.0


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    mul = lambda P, Q: [[sum(P[i][k] * Q[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


def Fg_of(r):
    R = R_of(r["pose_rx"], r["pose_ry"], r["pose_rz"])
    g = (R[2][0] * G, R[2][1] * G, R[2][2] * G)     # 第三【行】= R·(0,0,G) 的分量（行主序 R[6..8]）
    return [A[3 * i] * g[0] + A[3 * i + 1] * g[1] + A[3 * i + 2] * g[2] for i in range(3)]


# 台阶感知：只在姿态真的变了的地方算 ΔFg
prev = None
for r in rows:
    r["dFg"] = 0.0
    r["step"] = 0.0
    if prev is not None:
        same = (r["pose_rx"] == prev["pose_rx"] and r["pose_ry"] == prev["pose_ry"]
                and r["pose_rz"] == prev["pose_rz"])
        if not same:
            f0, f1 = Fg_of(prev), Fg_of(r)
            r["dFg"] = math.dist(f0, f1)
            r["step"] = max(abs(r["pose_rx"] - prev["pose_rx"]), abs(r["pose_ry"] - prev["pose_ry"]),
                            abs(r["pose_rz"] - prev["pose_rz"]))
    prev = r

steps = [r for r in rows if r["dFg"] > 0]
print(f"\n姿态台阶 {len(steps)} 步；|ΔFg| 中位 {sorted(x['dFg'] for x in steps)[len(steps)//2]:.3f} N，"
      f"最大 {max(x['dFg'] for x in steps):.3f} N")
print(f"|ΔFg| > 0.5N 的步数 {sum(1 for x in steps if x['dFg'] > 0.5)}；> 1.0N 的步数 {sum(1 for x in steps if x['dFg'] > 1.0)}")


def wall(r):
    s = r["wall"]
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:06.3f}"


print("\n=== |ΔFg| 最大的 12 步 ===")
for r in sorted(steps, key=lambda r: -r["dFg"])[:12]:
    print(f"  {wall(r)}   |ΔFg| {r['dFg']:6.3f} N   姿态步长 {r['step']:6.2f}°   "
          f"同期 |F| {r['F']:6.3f}")

print("\n=== |F| 最大的 12 行，看同行的 |ΔFg| ===")
for r in sorted(rows, key=lambda r: -r["F"])[:12]:
    print(f"  {wall(r)}   |F| {r['F']:6.3f}   |ΔFg| {r['dFg']:6.3f}   姿态步长 {r['step']:6.2f}°")

# 相关性
xs = [r["dFg"] for r in steps]
ys = [r["F"] for r in steps]
mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
sy = math.sqrt(sum((y - my) ** 2 for y in ys))
print(f"\ncorr(|ΔFg|, |F|) = {cov / (sx * sy):+.3f}   （{len(steps)} 步）")
