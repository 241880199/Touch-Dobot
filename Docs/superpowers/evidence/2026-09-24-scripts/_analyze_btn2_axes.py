# -*- coding: utf-8 -*-
"""把按钮2 关节模式那几段，按"增量旋转在器件系里的主导轴"切成时间线。
用途：看操作员这一趟到底依次做了哪几个动作，各自的真旋转轴是哪一根。"""
import re, math
from datetime import datetime
from bisect import bisect_left

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")
servoj_re = re.compile(rb"ServoJ\(")


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


sty, cmd_t = [], []
for raw in open(LOG, "rb"):
    m = line_re.match(raw)
    if not m:
        continue
    t, payload = m.group(1).decode(), m.group(3)
    if servoj_re.search(payload):
        cmd_t.append(t)
        continue
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


def R_of(rx, ry, rz):
    a, b, c = (math.radians(v) for v in (rx, ry, rz))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    Rx = [[1, 0, 0], [0, ca, -sa], [0, sa, ca]]
    Ry = [[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]]
    Rz = [[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]]
    mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(Rz, mul(Ry, Rx))


def transp(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotvec(A):
    tr = A[0][0] + A[1][1] + A[2][2]
    cosang = max(-1.0, min(1.0, (tr - 1) / 2))
    ang = math.acos(cosang)
    if ang < 1e-9:
        return 0.0, (0.0, 0.0, 0.0)
    s = 2 * math.sin(ang)
    ax = ((A[2][1] - A[1][2]) / s, (A[0][2] - A[2][0]) / s, (A[1][0] - A[0][1]) / s)
    n = math.sqrt(sum(v * v for v in ax)) or 1.0
    return math.degrees(ang), tuple(v / n for v in ax)


def sec(s):
    """一天内的秒数（Windows 上 1900 年的 datetime.timestamp() 会报错）"""
    return (T(s) - T("00:00:00.000")).total_seconds()


if not cmd_t:
    raise SystemExit("没有 ServoJ")
lo, hi = sec(cmd_t[0]) - 5, sec(cmd_t[-1]) + 5
seg = [(t, v) for t, v in sty if lo <= sec(t) <= hi]
print(f"按钮2 关节模式时间窗 {cmd_t[0]} .. {cmd_t[-1]}，笔杆样本 {len(seg)} 条\n")
print("  t             Δ(deg)  体系轴 (X,Y,Z)        主导轴   |Δ|>1.5° 才计数")
from collections import defaultdict
buckets = defaultdict(float)
prev = None
for t, v in seg:
    if prev:
        dt = (T(t) - T(prev[0])).total_seconds()
        if 0.15 < dt < 4.0:
            ang, ax = rotvec(mul(transp(R_of(*prev[1][3:])), R_of(*v[3:])))
            if ang > 1.5:
                dom = max(range(3), key=lambda i: abs(ax[i]))
                name = "XYZ"[dom]
                val = ax[dom]
                print(f"  {t}  {ang:6.2f}   ({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f})    {name}{'+' if val > 0 else '-'}")
                # 按 30 秒桶统计
                key = (sec(t) - lo) // 60
                buckets[(key, name + ("+" if val > 0 else "-"))] += ang
    prev = (t, v)

print("\n=== 按分钟桶 × 主导轴 的累计转角（度）===")
for (b, k), v in sorted(buckets.items()):
    print(f"  +{int(b)} 分钟桶  {k:3s}  {v:8.1f}°")
