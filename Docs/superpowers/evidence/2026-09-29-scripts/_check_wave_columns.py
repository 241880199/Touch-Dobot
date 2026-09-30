# -*- coding: utf-8 -*-
"""核对 force_wave.csv 的三件事是否一致：header 列数 / 格式说明符数 / 实参数。
本仓记过账：检查器本身会假绿（例如把跨行拼接的字符串截断），所以这里用一个小状态机
逐字符走字符串字面量，不依赖脆的正则。"""
import re
import sys

BS = chr(92)          # 反斜杠，避开各层转义
QUOTE = chr(34)       # 双引号


def lits(s):
    """抽出连续的 C 字符串字面量并拼接（剥掉引号），遇到非字面量就停。"""
    out, i, n = [], 0, len(s)
    while i < n:
        while i < n and s[i] in ' \t\r\n':
            i += 1
        if i >= n or s[i] != QUOTE:
            break
        i += 1
        buf = []
        while i < n and s[i] != QUOTE:
            if s[i] == BS:
                buf.append(s[i:i + 2]); i += 2
            else:
                buf.append(s[i]); i += 1
        i += 1
        out.append(''.join(buf))
    return ''.join(out)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'Touch_Client/main.cpp'
    src = open(path, encoding='utf-8').read()
    k = src.index('26 列')
    blk = src[k - 3000:k + 2500]

    # ⚠ 锚点必须是【唯一的】那个：本函数里有【两条】`fprintf(wf, "..."#` 开头的调用
    #   （先 `# wave <时间>  frames=N`，后 `# t_us,...`）。只写 `fprintf(wf, "` 会命中前一条，
    #   于是"检查器"去数了时间戳那行 —— 报出来的数是假的。第一次就踩了这个坑。
    hseg = blk[blk.index('fprintf(wf, ' + QUOTE + '# t_us'):]
    hstr = lits(hseg[hseg.index(QUOTE):])
    hstr = hstr.split(chr(92) + 'n')[0]
    hdrs = [x for x in hstr.lstrip('# ').split(',') if x]

    fseg = blk[blk.index('fprintf(wf, ' + QUOTE + '%llu'):]
    fseg = fseg[:fseg.index('buf[i].tickUs')]
    fstr = lits(fseg[fseg.index(QUOTE):])

    aseg = blk[blk.index('buf[i].tickUs'):]
    aseg = aseg[:aseg.index(');')]

    spec = re.findall(r'%[-0-9.]*(?:llu|d|f)', fstr)
    nargs = aseg.count('buf[i].')

    print('header 列数     =', len(hdrs))
    print('format 说明符数 =', len(spec))
    print('实参数         =', nargs)
    print()
    print('列名尾部   =', hdrs[-6:])
    print('说明符尾部 =', spec[-6:])
    print()
    ok = len(hdrs) == len(spec) == nargs
    print('三者一致:', ok)
    if not ok:
        for i in range(max(len(hdrs), len(spec))):
            a = hdrs[i] if i < len(hdrs) else '---'
            b = spec[i] if i < len(spec) else '---'
            print('  %2d  %-12s %s' % (i, a, b))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
