# -*- coding: utf-8 -*-
"""2026-09-24 上机数据：分段求【笔杆三根真实旋转轴】（器件系）+ 命令/实际关节增量。
估计器：逐步增量的旋转向量 → 单位轴 → 相对参考轴对齐符号 → 取【逐分量中位】。
（用中位而不是求和：自转是来回拧的，求和会让轴向分量自相抵消、只剩抖动。）"""
import re, math
from datetime import datetime

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260924.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")
servoj_re = re.compile(rb"ServoJ\((-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){5})")


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty, jnt, cmd = [], [], []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    t, p = m.group(1).decode(), m.group(3)
    ms = list(servoj_re.finditer(p))
    if ms:
        cmd.append((t, [float(x) for x in ms[-1].group(1).split(b",")]))
        continue
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

print(f"P| {len(sty)}  J| {len(jnt)}  ServoJ {len(cmd)}")

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


def median_axis(vecs):
    """符号对齐后逐分量取中位（参考 = 第一个向量）"""
    if not vecs:
        return None, 0, 0.0
    ref = vecs[0]
    S = [0.0, 0.0, 0.0]
    tot = 0.0
    aligned = []
    for v in vecs:
        d = sum(a * b for a, b in zip(ref, v))
        vv = v if d >= 0 else tuple(-x for x in v)
        aligned.append(vv)
        for i in range(3):
            S[i] += vv[i]
        tot += math.sqrt(sum(x * x for x in vv))
    med = [sorted(a[i] for a in aligned)[len(aligned) // 2] for i in range(3)]
    n = math.sqrt(sum(x * x for x in med)) or 1.0
    return tuple(x / n for x in med), len(vecs), tot


# ServoJ 连续段
bursts, cur = [], [cmd[0]]
for a, b in zip(cmd, cmd[1:]):
    if (T(b[0]) - T(a[0])).total_seconds() > 1.0:
        bursts.append(cur)
        cur = []
    cur.append(b)
bursts.append(cur)
bursts = [b for b in bursts if len(b) >= 10]
print(f"\nServoJ 连续段 {len(bursts)} 段")


def near(seq, t, tol=2.0):
    best = None
    for ts, v in seq:
        d = abs((T(ts) - T(t)).total_seconds())
        if d <= tol and (best is None or d < best[0]):
            best = (d, ts, v)
    return best


for k, b in enumerate(bursts, 1):
    t0, t1 = b[0][0], b[-1][0]
    dj = [y - x for x, y in zip(b[0][1], b[-1][1])]
    print(f"\n  [{k}] {t0} .. {t1}  {(T(t1)-T(t0)).total_seconds():5.1f}s  {len(b):4d} 帧")
    print(f"      命令 dJ1..6 = {['%+6.2f' % v for v in dj]}")
    a0, a1 = near(jnt, t0), near(jnt, t1)
    if a0 and a1:
        da = [y - x for x, y in zip(a0[2], a1[2])]
        print(f"      实际 dJ1..6 = {['%+6.2f' % v for v in da]}")
    # 段内逐步轴（器件系）
    seg = [(t, v) for t, v in sty if t0 <= t <= t1]
    vecs = []
    for (ta, va), (tb, vb) in zip(seg, seg[1:]):
        dt = (T(tb) - T(ta)).total_seconds()
        if not (0.01 < dt < 0.5):
            continue
        rv = rotvec(mul(tr(R_of(*va[3:])), R_of(*vb[3:])))
        if math.sqrt(sum(x * x for x in rv)) > 1.0:
            vecs.append(rv)
    ax, n, tot = median_axis(vecs)
    if ax:
        dom = max(range(3), key=lambda i: abs(ax[i]))
        print(f"      P| {len(seg)} 条, 有效步 {n}, 总转角 {tot:.0f}°")
        print(f"      逐步轴(器件系) = ({ax[0]:+.3f},{ax[1]:+.3f},{ax[2]:+.3f})  ⇒ 主导 {'XYZ'[dom]}"
              f"  与名义轴夹角: X {math.degrees(math.acos(min(1,abs(ax[0])))):.0f}°"
              f" / Y {math.degrees(math.acos(min(1,abs(ax[1])))):.0f}°"
              f" / Z {math.degrees(math.acos(min(1,abs(ax[2])))):.0f}°")
    else:
        print("      P| 段内无有效步")
