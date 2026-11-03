# -*- coding: utf-8 -*-
"""recompute_all.py -- 用 docs/data/ 里的原始 CSV 重算报告里的每一个数字。

这是"数字可复算"的唯一入口：报告 / 测试清单里出现的实测值，都应该能在这里找到出处。
只依赖 numpy + pandas。运行：

    python tools/recompute_all.py

输出各项指标与判据口径。任何数字对不上，说明文档与数据脱节了。
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "docs", "data")
FS = 100.0            # 遥测帧率 100 Hz
COUNTS_PER_REV = 824.5
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV      # ≈ 0.4366 °/count
DUTY_FULL = 2500                            # PWM 满量程 counts


def load(name):
    d = pd.read_csv(os.path.join(DATA, name))
    d["sec"] = d["t"] / FS
    d["th"] = d["theta"] / 10.0
    return d


def runs_of(state, target):
    """返回 [(起点下标, 长度)]，按长度降序"""
    s = state.values
    out, cur, start = [], 0, 0
    for i, v in enumerate(s):
        if v == target:
            if cur == 0:
                start = i
            cur += 1
        else:
            if cur:
                out.append((start, cur))
            cur = 0
    if cur:
        out.append((start, cur))
    out.sort(key=lambda r: -r[1])
    return out


# ---------------------------------------------------------------- T1 / T2
def t1_t2(config):
    d = load("swing10.csv")
    s = d.state.values
    sec = d.sec.values
    attempts = []
    for i in range(1, len(s)):
        if s[i] == 1 and s[i - 1] == 0:
            t0 = sec[i]
            cap = None
            hold = 0.0
            for j in range(i + 1, min(i + 900, len(s))):
                if s[j] == 5:
                    cap = sec[j]
                    k = j
                    while k + 1 < len(s) and s[k + 1] == 5:
                        k += 1
                    hold = sec[k] - cap
                    break
                if s[j] == 0:
                    break
            attempts.append((t0, cap, None if cap is None else cap - t0, hold, cap is not None))
    good = [a for a in attempts if a[4] and a[3] >= 2.0]
    cap_t = [a[2] for a in attempts if a[4]]
    print("【T1 起摆成功率】")
    print("  起摆尝试次数          : %d" % len(attempts))
    print("  完成捕获（进 state5） : %d  (%.0f%%)" % (len(cap_t), 100.0 * len(cap_t) / len(attempts)))
    print("  捕获后站住 >=2 s      : %d  (%.0f%%)  <- 报告引用这个" %
          (len(good), 100.0 * len(good) / len(attempts)))
    print("【T2 起摆耗时】")
    if cap_t:
        print("  kick -> state5        : 中位 %.2f s，范围 %.2f~%.2f s" %
              (np.median(cap_t), min(cap_t), max(cap_t)))
    print()


# ---------------------------------------------------------------- T3 / T4
def t3_t4(config):
    d = load("hold10min.csv")
    rs = runs_of(d.state, 5)
    i0, n = rs[0]
    seg = d.iloc[i0:i0 + n]
    seg_t = seg.iloc[int(5 * FS):]                  # 去掉捕获后 5 s 瞬态
    ath = seg_t.th.abs()
    print("【T3 连续平衡】")
    print("  state5 最长连续段     : %d 帧 = %.1f s（共 %d 段）" % (n, n / FS, len(rs)))
    print("【T4 稳态精度】（去掉段首 5 s，%d 帧）" % len(seg_t))
    print("  |θ| p50 / p95 / rms   : %.2f° / %.2f° / %.2f°" %
          (np.percentile(ath, 50), np.percentile(ath, 95), np.sqrt((ath ** 2).mean())))
    print("  |θ| max（稳态段）     : %.2f°" % ath.max())
    print("  |θ| max（含捕获瞬态） : %.2f°" % seg.th.abs().max())
    print("  θ 范围                : %.1f° ~ %.1f°" % (seg_t.th.min(), seg_t.th.max()))
    print("  duty mean|.| / 峰值   : %.0f / %d（限幅 %d）" %
          (seg.duty.abs().mean(), seg.duty.abs().max(), 2000))
    print("  duty 饱和帧数         : %d" % int((seg.duty.abs() >= 2000).sum()))
    dp = int(seg.pos.iloc[-1] - seg.pos.iloc[0])
    pp = int(seg.pos.max() - seg.pos.min())
    print("  横臂净漂 / 峰峰       : %d counts (%.1f°) / %d counts (%.1f°)  用时 %.1f s" %
          (dp, dp * DEG_PER_COUNT, pp, pp * DEG_PER_COUNT, n / FS))
    print("  横臂漂移速率          : %.2f °/min" % (abs(dp * DEG_PER_COUNT) / (n / FS) * 60))

    # 稳态 duty 抖振特征（报告"已知特性"用）
    du = seg.duty.values.astype(float)
    du = du - du.mean()
    flips = int((np.diff(np.sign(du)) != 0).sum())
    print("  duty 抖振：符号翻转   : %.1f 次/秒；峰峰 %d counts" %
          (flips / (n / FS), seg.duty.max() - seg.duty.min()))
    # 主频（对 ±1000 抖振取样做 FFT）
    x = du[: int(60 * FS)]
    x = x * np.hanning(len(x))
    sp = np.abs(np.fft.rfft(x))
    fr = np.fft.rfftfreq(len(x), 1 / FS)
    band = (fr > 1) & (fr < 30)
    print("  duty 抖振主频（1~30 Hz）: %.1f Hz" % fr[band][np.argmax(sp[band])])
    print()


# ---------------------------------------------------------------- T5
def t5(config):
    d = load("disturb3.csv")
    s = d.state.values
    th = d.th.values
    n = len(th)

    # 1) 先找所有"超阈事件"：state5 内 |θ| >= 8°，到回落到 3° 以下为止
    raw = []
    i = 0
    while i < n:
        if s[i] == 5 and abs(th[i]) >= 8.0:
            j = i
            peak, ipk = abs(th[i]), i
            while j < n and s[j] == 5 and abs(th[j]) >= 3.0:
                if abs(th[j]) > peak:
                    peak, ipk = abs(th[j]), j
                j += 1
            fell = (j < n and s[j] != 5)
            raw.append((d.sec.iloc[ipk], peak, fell))
            i = j
        else:
            i += 1

    # 2) 合并：间隔 < 0.3 s 的事件算同一次推扰（手推一次会激起多个过零峰）
    merged = []
    for e in raw:
        if merged and (e[0] - merged[-1][0]) < 0.3:
            prev = merged[-1]
            merged[-1] = (prev[0] if prev[1] >= e[1] else e[0],
                          max(prev[1], e[1]),
                          prev[2] or e[2])
        else:
            merged.append(list(e) if False else e)
    merged = [tuple(m) for m in merged]

    ok = [m for m in merged if not m[2]]
    print("【T5 抗瞬时推扰】")
    print("  超阈事件（|θ|≥8°）    : %d 个；按 0.3 s 间隔合并为推扰 %d 次" %
          (len(raw), len(merged)))
    print("  未掉摆                : %d 次（%.0f%%）" %
          (len(ok), 100.0 * len(ok) / max(1, len(merged))))
    if ok:
        pk = [m[1] for m in ok]
        print("  恢复样本峰值角        : 中位 %.1f°，最大 %.1f°" % (np.median(pk), max(pk)))

    # 3) 恢复时间：峰值 -> |θ| 连续回到 2.5° 以内并保持 0.5 s
    q = int(0.5 * FS)
    rec_times = []
    for tpk, peak, fell in ok:
        ipk = int(tpk * FS)
        k = ipk
        while k + q < n and s[k] == 5:
            if np.all(np.abs(th[k:k + q]) <= 2.5):
                rec_times.append(d.sec.iloc[k] - tpk)
                break
            k += 1
    if rec_times:
        print("  峰值 -> |θ|≤2.5° 保持 0.5 s : 中位 %.2f s（最长 %.2f s，%d 个样本）" %
              (np.median(rec_times), max(rec_times), len(rec_times)))
    fail = [m for m in merged if m[2]]
    if fail:
        print("  掉摆                  : %d 次，峰值角 %s°" %
              (len(fail), ", ".join("%.1f" % m[1] for m in fail)))
    print()


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print("=" * 74)
    print("LQR_end 实测数据复算    数据目录:", DATA)
    print("=" * 74)
    t1_t2(None)
    t3_t4(None)
    t5(None)


if __name__ == "__main__":
    main()
