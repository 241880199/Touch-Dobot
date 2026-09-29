# -*- coding: utf-8 -*-
"""
判两件事（都只用现成数据，不碰机械臂、不改代码）：

A) `act`(@624 ToolVectorActual) 到底是【真反馈】还是【控制器回显的命令】？
   A1 高频含量：真测量有传感器/量化噪声，回显会把 tgt 的谱原样搬过来（甚至更平滑）。
   A2 静止段：tgt 逐位不变的那几帧里，act 是否仍在动。
   A3 阶跃响应：tgt 单帧大跳时，act 是【同样幅度跟着跳】还是【斜坡爬上去】—— 后者才是真伺服。

B) 那个 ~2 Hz 的力谱峰是不是只在【按压】时才有（= 载荷下的粘滑），还是空载也在（= 环路自激）。
"""
import numpy as np

PATH = 'Docs/superpowers/evidence/2026-09-24-force-wave-loop-confirmed-dev.csv'
FS_FALLBACK = 122.9


def load():
    blocks, cur = [], None
    for line in open(PATH):
        s = line.strip()
        if s.startswith('# wave'):
            cur = {'time': s.split()[3], 'rows': [], 'hdr': None}
            blocks.append(cur)
        elif s.startswith('# t_us'):
            cur['hdr'] = s[2:].split(',')
        elif cur is not None and cur['hdr'] and s:
            cur['rows'].append([float(x) for x in s.split(',')])
    return [b for b in blocks if len(b['hdr']) == 22]


def welch1(x, nperseg=512):
    nperseg = int(min(nperseg, max(64, len(x))))
    if nperseg % 2:
        nperseg -= 1
    x = np.asarray(x, dtype=float)
    step = nperseg // 2
    win = np.hanning(nperseg)
    segs = []
    for i in range(0, len(x) - nperseg + 1, step):
        seg = x[i:i + nperseg]
        segs.append(np.fft.rfft((seg - seg.mean()) * win))
    if not segs:
        segs = [np.fft.rfft((x - x.mean()) * np.hanning(len(x)))]
    S = np.array(segs)
    return np.fft.rfftfreq(nperseg, 1.0 / FS_FALLBACK), (np.abs(S) ** 2).mean(axis=0)


def main():
    blocks = load()
    print('=' * 72)
    print('A) act 的性质')
    print('=' * 72)
    for b in blocks:
        h = b['hdr']; a = np.array(b['rows']); ix = {n: i for i, n in enumerate(h)}
        t, g = a[:, ix['tgt_x_mm']], a[:, ix['act_x_mm']]
        dtg = np.diff(t); dag = np.diff(g)
        sd_t, sd_g = t.std(), g.std()

        # A1 高频能量比：30~60 Hz 的功率 / 0.3~5 Hz 的功率
        f, Pt = welch1(t); _, Pg = welch1(g)
        lo = (f >= 0.3) & (f <= 5.0); hi = (f >= 30.0) & (f <= 60.0)
        rt = Pt[hi].mean() / max(Pt[lo].mean(), 1e-30)
        rg = Pg[hi].mean() / max(Pg[lo].mean(), 1e-30)

        # A2 静止段：tgt 保持不变、长度 >= 6 的连续片段里 act 的逐帧变化
        runs, i = [], 0
        while i < len(t):
            j = i
            while j + 1 < len(t) and t[j + 1] == t[i]:
                j += 1
            if j - i + 1 >= 6:
                runs.append((i, j))
            i = j + 1
        mv = np.concatenate([np.abs(np.diff(g[i:j + 1])) for i, j in runs]) if runs else np.array([0.0])

        # A3 阶跃：tgt 单帧跳变最大的 5 次，看 act 在 ±12 帧内怎么跟
        idx = np.argsort(-np.abs(dtg))[:5]
        step_dv, step_resp = [], []
        for k in range(len(dtg)):
            pass
        for k in [int(v) for v in idx]:
            if k < 12 or k + 13 > len(g):
                continue
            d = float(dtg[k])
            if d == 0.0:
                continue
            resp = [float(v) for v in (g[k + 1:k + 13] - g[k - 12]) / d]
            step_dv.append(d)
            step_resp.append(resp)

        print('--- %s  sd(tgt)=%.3f sd(act)=%.3f' % (b['time'], sd_t, sd_g))
        print('    A1 高频/低频能量比    tgt=%.3e   act=%.3e   (act/tgt=%.2f)' % (rt, rg, rg / max(rt, 1e-30)))
        print('    A2 静止段 %2d 段：tgt 帧间变化 = 0，act 帧间变化 sd=%.5f mm（占 sd(act) 的 %.2f%%）'
              % (len(runs), mv.std(), 100 * mv.std() / max(sd_g, 1e-9)))
        if step_dv:
            print('    A3 最大 tgt 单帧跳变: ' + ', '.join('%+.3f' % d for d in step_dv) + ' mm')
            for dtv, resp in list(zip(step_dv, step_resp))[:3]:
                print('       跳变 %+.3f mm ⇒ act 随后 12 帧的完成比例: [%s]'
                      % (dtv, ' '.join('%.2f' % r for r in resp)))

    print()
    print('=' * 72)
    print('B) ~2 Hz 力谱峰：只在按压时才有吗')
    print('=' * 72)
    for b in blocks:
        h = b['hdr']; a = np.array(b['rows']); ix = {n: i for i, n in enumerate(h)}
        rawz = a[:, ix['raw_z']]
        thr = np.percentile(np.abs(rawz), 50)
        for lbl, m in (('低压 |raw_z|<中位', np.abs(rawz) <= thr), ('高压 |raw_z|>中位', np.abs(rawz) > thr)):
            # 用掩码把样本连成段再拼（只取连续长度 >= 128 的段）
            segs, i = [], 0
            while i < len(m):
                if m[i]:
                    j = i
                    while j + 1 < len(m) and m[j + 1]:
                        j += 1
                    if j - i + 1 >= 256:
                        segs.append(a[i:j + 1])
                    i = j + 1
                else:
                    i += 1
            if not segs:
                print('--- %s  %s : 无 >=256 帧的连续段' % (b['time'], lbl)); continue
            cat = np.concatenate(segs)
            fs, P = welch1(cat[:, ix['raw_x']])
            band = (fs >= 0.3) & (fs <= 20.0)
            k = np.argmax(np.where(band, P, -1))
            print('--- %s  %-16s n=%4d  raw_x 谱峰 f=%.2f Hz  信噪比=%.1f'
                  % (b['time'], lbl, len(cat), fs[k], P[k] / max(np.median(P[band]), 1e-30)))


if __name__ == '__main__':
    main()
