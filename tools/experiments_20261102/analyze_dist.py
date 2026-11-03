# -*- coding: utf-8 -*-
"""analyze_dist.py -- 从 raw.csv 里找"干扰事件"，算 峰值 -> 回正时间

本体稳态本身就在 ±2° 里持续晃动（实测 std≈1.0°，摩擦/回差导致），
所以"回正"不能定义成 <0.8°，否则永远等不到安静时刻。这里用 2.5° 带 + 0.5 s 保持。
"""
import sys, io

path = sys.argv[1] if len(sys.argv) > 1 else r'D:\DeepSeek_Harness\对话文件\work\raw.csv'
rows = []
with io.open(path, encoding='utf-8', errors='replace') as f:
    f.readline()
    for line in f:
        p = line.strip().split(',')
        if len(p) != 4:
            continue
        try:
            rows.append((int(p[0]), int(p[1]), int(p[2]), int(p[3])))
        except ValueError:
            continue
if not rows:
    print('NO DATA')
    sys.exit()

fps = 100.0
segs = []
cur = []
for r in rows:
    if r[3] == 5:
        cur.append(r)
    else:
        if len(cur) > 20:
            segs.append(cur)
        cur = []
if len(cur) > 20:
    segs.append(cur)

print('frames=%d  balance_frames=%d  segments=%d' % (
    len(rows), sum(len(s) for s in segs), len(segs)))

abs_all = sorted(abs(r[1]) for s in segs for r in s)
if abs_all:
    p50 = abs_all[int(0.50 * (len(abs_all) - 1))] / 10.0
    p95 = abs_all[int(0.95 * (len(abs_all) - 1))] / 10.0
    print('steady |theta|: p50=%.2f deg  p95=%.2f deg  max=%.2f deg' % (
        p50, p95, abs_all[-1] / 10.0))

TRIG = 40      # 4.0 deg 才算一次干扰
BAND = 25      # 2.5 deg 回正带
HOLD = 50      # 0.5 s 保持

events = []
for s in segs:
    n = len(s)
    i = 0
    while i < n:
        if abs(s[i][1]) > TRIG:
            start = i
            peak = abs(s[i][1]); pk = i
            j = i
            rec = None
            while j < n:
                a = abs(s[j][1])
                if a > peak:
                    peak = a; pk = j
                if j > i and a < BAND:
                    ok = True
                    for m in range(1, HOLD + 1):
                        if j + m >= n or abs(s[j + m][1]) >= BAND:
                            ok = False; break
                    if ok:
                        rec = s[j][0]; break
                j += 1
            if rec is None:
                rec = s[min(j, n - 1)][0]
            rebound = 0
            k = j
            while k < n and s[k][0] - rec < 80:
                rebound = max(rebound, abs(s[k][1])); k += 1
            events.append(dict(
                t0=s[start][0] / fps, peak=peak / 10.0,
                tpk=s[pk][0] / fps,
                rec_ms=(rec - s[pk][0]) / fps * 1000.0,
                total_ms=(rec - s[start][0]) / fps * 1000.0,
                duty=max(abs(x[2]) for x in s[start:min(j + 1, n)]),
                rebound=rebound / 10.0))
            i = j
            while i < n and abs(s[i][1]) > TRIG:
                i += 1
        else:
            i += 1

print('\nevents: %d' % len(events))
if events:
    print('  #   t0[s]   peak[deg]   peak->2.5deg[ms]   total[ms]   |duty|max   rebound[deg]')
    for k, e in enumerate(events, 1):
        print('  %2d %7.2f   %7.1f        %8.0f        %7.0f      %5d        %5.1f' % (
            k, e['t0'], e['peak'], e['rec_ms'], e['total_ms'], e['duty'], e['rebound']))
    print('\nbuckets:')
    for lo, hi in [(4, 6), (6, 9), (9, 13), (13, 20), (20, 90)]:
        es = [e for e in events if lo <= e['peak'] < hi]
        if es:
            avg = sum(e['rec_ms'] for e in es) / len(es)
            print('  %2d-%2d deg : n=%2d  avg_rec=%6.0f ms  max=%6.0f ms' % (
                lo, hi, len(es), avg, max(e['rec_ms'] for e in es)))
