# -*- coding: utf-8 -*-
"""定位 ServoJ 连续段，并配对：笔杆 P| / 实际关节 J| / 机械臂实际位姿 RP| / 命令增量。

解析要点：消息可能粘行 ⇒ 取【最后一个】消息；且 "RP|" 内含 "P|" ⇒ 必须排除前面是字母的匹配。
"""
import re
from datetime import datetime
from bisect import bisect_left

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")
num6 = re.compile(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}")
servoj_re = re.compile(rb"ServoJ\((-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){5})")
servop_re = re.compile(rb"ServoP\((-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){5})")


def T(s):
    return datetime.strptime(s, "%H:%M:%S.%f")


cmd, sty, jnt, rp, cmdp = [], [], [], [], []
with open(LOG, "rb") as f:
    for raw in f:
        m = line_re.match(raw)
        if not m:
            continue
        t, payload = m.group(1).decode(), m.group(3)
        ms = list(servoj_re.finditer(payload))
        mp = list(servop_re.finditer(payload))
        if ms:
            cmd.append((t, [float(x) for x in ms[-1].group(1).split(b",")]))
            continue
        if mp:
            cmdp.append((t, [float(x) for x in mp[-1].group(1).split(b",")]))
            continue
        picked = False
        for tag, sink in ((b"P|", sty), (b"J|", jnt), (b"RP|", rp)):
            found, k = -1, -1
            while True:
                k = payload.find(tag, k + 1)
                if k < 0:
                    break
                # 前面是字母 ⇒ 这是 "RP|" 里的 "P|"，跳过（下标照常前进，别再原地打转）
                if k > 0 and (65 <= payload[k - 1] <= 90 or 97 <= payload[k - 1] <= 122):
                    continue
                found = k
            if found < 0:
                continue
            tail = payload[found + len(tag):].strip()
            if not num6.fullmatch(tail):
                continue
            sink.append((t, [float(x) for x in tail.split(b",")]))
            picked = True
            break
        if not picked:
            continue

print(f"ServoJ 命令 {len(cmd)} 条 | ServoP 命令 {len(cmdp)} 条 | 笔杆 P| {len(sty)} | 关节 J| {len(jnt)} | 位姿 RP| {len(rp)}")
if not cmd:
    raise SystemExit

bursts, cur = [], [cmd[0]]
for a, b in zip(cmd, cmd[1:]):
    if (T(b[0]) - T(a[0])).total_seconds() > 1.0:
        bursts.append(cur)
        cur = []
    cur.append(b)
bursts.append(cur)
bursts = [b for b in bursts if len(b) >= 10]
print(f"ServoJ 连续段 {len(bursts)} 段（≥10 帧）\n")


def near(seq, t, tol=2.5):
    """seq 已按时间有序；取 t 最近的一条，须在 tol 秒内"""
    if not seq:
        return None
    ts = [x[0] for x in seq]
    k = bisect_left(ts, t)
    best = None
    for idx in (k - 1, k):
        if 0 <= idx < len(seq):
            d = abs((T(seq[idx][0]) - T(t)).total_seconds())
            if d <= tol and (best is None or d < best[0]):
                best = (d, seq[idx][0], seq[idx][1])
    return best


def show(label, s0, s1, names, sl=(0, 3), scale=1.0):
    if not (s0 and s1):
        print(f"  {label}: 无采样")
        return
    a, b = sl
    d = [round((y - x) * scale, 2) for x, y in zip(s0[2][a:b], s1[2][a:b])]
    print(f"  {label}: {dict(zip(names, d))}   [{s0[1]} .. {s1[1]}]")


for b in bursts:
    t0, t1 = b[0][0], b[-1][0]
    dur = (T(t1) - T(t0)).total_seconds()
    j0, j1 = b[0][1], b[-1][1]
    dj = [y - x for x, y in zip(j0, j1)]
    print(f"--- {t0} .. {t1}  ({dur:.1f}s, {len(b)} 帧) ---")
    print(f"  命令 dJ1..dJ6 = {['%+7.2f' % v for v in dj]}")
    show("实际 dJ1..dJ6", near(jnt, t0), near(jnt, t1), ["J1", "J2", "J3", "J4", "J5", "J6"], (0, 6))
    show("笔杆 dPx,dPy,dPz(mm)", near(sty, t0), near(sty, t1), ["x", "y", "z"], (0, 3))
    show("笔杆 dRx,dRy,dRz(deg)", near(sty, t0), near(sty, t1), ["Rx", "Ry", "Rz"], (3, 6))
    show("机械臂 dRPxyz(mm)", near(rp, t0), near(rp, t1), ["X", "Y", "Z"], (0, 3))
    show("机械臂 dRP rpy(deg)", near(rp, t0), near(rp, t1), ["rx", "ry", "rz"], (3, 6))
    print()
