# -*- coding: utf-8 -*-
"""
离线互谱：用现成的 22 列波形数据量【环路频率 + 各环节幅度比/相位】。
只读，不改任何东西。临时脚本（未提交），先例 _analyze_btn2_*.py。

用途：证据报告说"环路频率已量到 1~5 Hz"，记忆说"拿不到频率" —— 这两条是同一个
      数据集的相反结论，本脚本去判。
"""
import sys, math
import numpy as np

PATH = 'Docs/superpowers/evidence/2026-09-24-force-wave-loop-confirmed-dev.csv'


def load(path):
    blocks, cur = [], None
    for line in open(path):
        s = line.strip()
        if s.startswith('# wave'):
            f = s.split()
            cur = {'date': f[2], 'time': f[3], 'rows': [], 'hdr': None}
            blocks.append(cur)
        elif s.startswith('# t_us'):
            cur['hdr'] = s[2:].split(',')
        elif cur is not None and cur['hdr'] and s:
            cur['rows'].append([float(x) for x in s.split(',')])
    return [b for b in blocks if b['rows']]


def welch(x, fs, nperseg=512):
    """段平均周期图（Hann 窗、50% 重叠）。返回 (f, Pxx, 复数谱段列表)。"""
    step = nperseg // 2
    win = np.hanning(nperseg)
    segs = []
    for i in range(0, len(x) - nperseg + 1, step):
        seg = x[i:i + nperseg]
        seg = (seg - seg.mean()) * win
        segs.append(np.fft.rfft(seg))
    if not segs:
        seg = (x - x.mean()) * np.hanning(len(x))
        segs = [np.fft.rfft(seg)]
    S = np.array(segs)
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    Pxx = (np.abs(S) ** 2).mean(axis=0)
    return f, Pxx, S


def coherence(Sa, Sb):
    """多段平均的 |Sxy|^2/(Sxx Syy)。"""
    Sxy = (Sa * np.conj(Sb)).mean(axis=0)
    Sxx = (np.abs(Sa) ** 2).mean(axis=0)
    Syy = (np.abs(Sb) ** 2).mean(axis=0)
    den = Sxx * Syy
    coh = np.where(den > 0, np.abs(Sxy) ** 2 / np.where(den > 0, den, 1.0), 0.0)
    return coh, Sxy


def main():
    blocks = [b for b in load(PATH) if len(b['hdr']) == 22]
    print('22 列块数 =', len(blocks))
    if not blocks:
        print('没有 22 列的数据块'); return 1

    hdr = blocks[0]['hdr']
    idx = {n: i for i, n in enumerate(hdr)}

    for b in blocks:
        arr = np.array(b['rows'])
        t = arr[:, 0] * 1e-6                       # s
        dt = np.median(np.diff(t))
        fs = 1.0 / dt
        def c(name): return arr[:, idx[name]]

        devx, tgtx, actx = c('dev_x_mm'), c('tgt_x_mm'), c('act_x_mm')
        rawx, filt_x = c('raw_x'), c('filt_x')

        f, Pdev, Sdev = welch(devx, fs)
        _, Ptgt, Stgt = welch(tgtx, fs)
        _, Pact, Sact = welch(actx, fs)
        _, Praw, Sraw = welch(rawx, fs)

        band = (f >= 0.3) & (f <= 30.0)
        if not band.any():
            print('%s 频带为空' % b['time']); continue
        kpk = np.argmax(np.where(band, Pdev, -1))     # 以手柄位移的谱峰为环路频率候选

        coh_dt, _ = coherence(Sdev, Stgt)
        coh_ta, _ = coherence(Stgt, Sact)
        coh_dr, Sdr = coherence(Sdev, Sraw)

        def ph(Sxy):
            return math.degrees(math.atan2(Sxy[kpk].imag, Sxy[kpk].real))

        def amp_ratio(P1, P2):
            return math.sqrt(P2[kpk] / P1[kpk]) if P1[kpk] > 0 else float('nan')

        print('--- %s %s  fs=%.1f Hz  n=%d' % (b['date'], b['time'], fs, len(arr)))
        print('    谱峰 f=%.2f Hz   sd(dev_x)=%.3f  sd(tgt_x)=%.3f  sd(raw_x)=%.4f' %
              (f[kpk], devx.std(), tgtx.std(), rawx.std()))
        print('    dev->tgt : coh=%.3f  phase=%+7.1f deg  |tgt|/|dev|=%.3f' %
              (coh_dt[kpk], ph(coh_dt), amp_ratio(Pdev, Ptgt)))
        print('    tgt->act : coh=%.3f  phase=%+7.1f deg  |act|/|tgt|=%.3f' %
              (coh_ta[kpk], ph(coh_ta), amp_ratio(Ptgt, Pact)))
        print('    dev->raw : coh=%.3f  phase=%+7.1f deg  |raw|/|dev|=%.2e' %
              (coh_dr[kpk], ph(Sdr), amp_ratio(Pdev, Praw)))

        # 只看 raw_x 谱峰在哪（力的"抖"在哪个频率上最强）
        braw = (f >= 0.3) & (f <= 30.0)
        kr = np.argmax(np.where(braw, Praw, -1))
        print('    raw_x 谱峰 f=%.2f Hz（信噪比 P/Pmed=%.1f）' %
              (f[kr], Praw[kr] / np.median(Praw[braw])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
