# -*- coding: utf-8 -*-
"""从 MATLAB 旁路日志里挖 P|(笔杆) 与 J|(关节角) 序列，找关节运动段落并配对笔杆姿态变化。"""
import re, sys

LOG = r"D:\Projects\Touch\Relay_Station\_matlab_session_20260923.log"
line_re = re.compile(rb"^(\d\d:\d\d:\d\d\.\d\d\d) (\S+)\s+(.*)$")

P, J = [], []   # (t, [floats])
with open(LOG, "rb") as f:
    for raw in f:
        m = line_re.match(raw)
        if not m:
            continue
        t, kind, payload = m.group(1).decode(), m.group(2).decode(errors="replace"), m.group(3)
        # payload 可能前面粘着别的消息尾巴，取最后一个 "X|"
        for tag, sink in (("P|", P), ("J|", J)):
            i = payload.rfind(tag.encode())
            if i < 0:
                continue
            tail = payload[i + 2:]
            # 只接受"整条都是数字、逗号分隔"的尾巴（避免抓到粘行）
            if not re.fullmatch(rb"-?\d+(\.\d+)?(,-?\d+(\.\d+)?){5}", tail.strip()):
                continue
            try:
                vals = [float(x) for x in tail.split(b",")]
            except ValueError:
                continue
            sink.append((t, vals))
            break

def span(name, seq):
    if not seq:
        print(f"{name}: 0 条")
        return
    print(f"{name}: {len(seq)} 条, 时间 {seq[0][0]} .. {seq[-1][0]}")

span("P| 笔杆", P)
span("J| 关节", J)

if not J:
    sys.exit(0)

# 关节运动段落：相邻采样任一关节变化 > 3°
print("\n=== 关节运动段落（相邻 J| 采样间任一关节变化 > 3°）===")
eps = []
for a, b in zip(J, J[1:]):
    d = [y - x for x, y in zip(a[1], b[1])]
    if max(abs(v) for v in d) > 3.0:
        eps.append((a[0], b[0], a[1], b[1], d))
print(f"共 {len(eps)} 段")
for t0, t1, j0, j1, d in eps[:40]:
    print(f"{t0} -> {t1}")
    print(f"   J: {['%.2f' % v for v in j0]} -> {['%.2f' % v for v in j1]}")
    print(f"   dJ d1..d6: {['%+.2f' % v for v in d]}")

# 每段配对笔杆：取时间段内首个/末个 P|
print("\n=== 每段配对的笔杆 P|（段内首末）===")
for t0, t1, j0, j1, d in eps[:40]:
    inside = [v for t, v in P if t0 <= t <= t1]
    if not inside:
        print(f"{t0}..{t1}: 段内无 P|")
        continue
    p0, p1 = inside[0], inside[-1]
    dp = [y - x for x, y in zip(p0, p1)]
    print(f"{t0}..{t1}  P: {['%.2f' % v for v in p0]} -> {['%.2f' % v for v in p1]}")
    print(f"    dP xyz: {['%+.2f' % v for v in dp[:3]]}   dP Rx/Ry/Rz: {['%+.2f' % v for v in dp[3:]]}")
