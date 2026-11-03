# -*- coding: utf-8 -*-
"""recompute_v30.py —— 定版 campaign（docs/data/v30_S*.csv）的复算脚本

用法：
    python tools/recompute_v30.py S4        # 单段
    python tools/recompute_v30.py all       # 全部

列定义（与 capture6.ps1 一致）：
    t    帧序号（÷100 = 秒）
    ch0  {sensor_lost, sw_state[3:0]} 打包 → state = (ch0>>4)&7
    duty 电机占空比 counts（±2000 限幅，2500 = 100%）
    pos  横臂编码器计数（824.5 counts/圈 → 0.4366°/count）
    theta 摆杆角 0.1°
    adc  MCP3201 原始值
    ch5  {sw_inv, e_dbg[14:0]}
"""
import csv
import os
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "docs", "data")

POS_DEG = 360.0 / 824.5          # counts -> 度
SAT = 2000


def load(name):
    path = os.path.join(DATA, "v30_%s.csv" % name)
    if not os.path.exists(path):
        path = os.path.join(DATA, name if name.endswith(".csv") else name + ".csv")
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["t"]), int(r["ch0"]), float(r["duty"]),
                             float(r["pos"]), float(r["theta"]), float(r["adc"])))
            except (KeyError, ValueError):
                continue
    return rows


def state(ch0):
    return (ch0 >> 4) & 7


def longest_true_run(idx):
    best = cur = 0
    start = best_start = 0
    for i, v in enumerate(idx):
        if v:
            if cur == 0:
                start = i
            cur += 1
            if cur > best:
                best, best_start = cur, start
        else:
            cur = 0
    return best, best_start


def pct(vals, q):
    s = sorted(vals)
    if not s:
        return float("nan")
    k = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return s[k]


def report(name):
    rows = load(name)
    if not rows:
        print("!! 没有数据：", name)
        return
    n = len(rows)
    dur = (rows[-1][0] - rows[0][0]) / 100.0
    stt = [state(r[1]) for r in rows]
    print("=" * 62)
    print("【%s】%d 帧 / %.2f s / %.1f Hz" % (name, n, dur, (n - 1) / dur if dur else 0))
    lost = sum(1 for r in rows if (r[1] >> 7) & 1)
    print("  sensor_lost 帧           : %d" % lost)
    print("  state 分布               : %s" %
          {k: stt.count(k) for k in sorted(set(stt))})

    run, s0 = longest_true_run([s == 5 for s in stt])
    if run:
        seg = rows[s0:s0 + run]
        th_signed = [r[4] / 10.0 for r in seg]
        th = [abs(x) for x in th_signed]
        dut = [r[2] for r in seg]
        pos = [r[3] for r in seg]
        tsec = run / 100.0
        mean_th = sum(th_signed) / len(th_signed)
        rms = (sum(x * x for x in th_signed) / len(th_signed)) ** 0.5
        # 主频：θ 去均值后的过零次数 / 2 / 秒
        zc = sum(1 for i in range(1, len(seg))
                 if (seg[i - 1][4] / 10.0 - mean_th) * (seg[i][4] / 10.0 - mean_th) < 0)
        f0 = zc / 2.0 / tsec if tsec else 0
        net = (pos[-1] - pos[0])
        print("-" * 62)
        print("  最长 state=5 段         : %d 帧 = %.1f s（起点 %.2f s）" %
              (run, tsec, s0 / 100.0))
        print("  |θ| p50 / p95 / rms     : %.2f° / %.2f° / %.2f°" %
              (pct(th, 0.50), pct(th, 0.95), rms))
        print("  |θ| max / θ 范围        : %.2f° / %.2f° ~ %.2f°" %
              (max(th), min(r[4] / 10.0 for r in seg), max(r[4] / 10.0 for r in seg)))
        print("  duty mean|.| / 峰值      : %.0f / %.0f（限幅 ±%d）" %
              (sum(abs(x) for x in dut) / len(dut), max(abs(x) for x in dut), SAT))
        print("  duty 饱和帧数            : %d" % sum(1 for x in dut if abs(x) >= SAT))
        print("  横臂净漂 / 峰峰          : %+d counts (%.1f°) / %d counts" %
              (net, net * POS_DEG, max(pos) - min(pos)))
        print("  横臂漂移速率             : %.2f °/min" %
              (abs(net * POS_DEG) / (tsec / 60.0)))
        print("  抖动主频（过零估）       : %.2f Hz" % f0)

    # 起摆段：state 0->1->5 的时间（仅单次捕获的段才有意义；S2 是多轮统计）
    single_capture = stt.count(5) == run if run else False
    if single_capture and 1 in stt:
        try:
            t1 = rows[stt.index(1)][0] / 100.0
            t5 = rows[s0][0] / 100.0
            print("-" * 62)
            print("  起 kick / 进平衡         : %.2f s / %.2f s（用时 %.2f s）" %
                  (t1, t5, t5 - t1))
        except ValueError:
            pass

    # 推扰段：|θ|>=4.5° 起判、2 s 内合并
    if name in ("S6",):
        ev, last = [], -999
        for i, r in enumerate(rows):
            if state(r[1]) != 5:
                continue
            if abs(r[4]) / 10.0 >= 4.5:
                t = r[0] / 100.0
                if t - last > 2.0:
                    ev.append([t, abs(r[4]) / 10.0])
                else:
                    ev[-1][1] = max(ev[-1][1], abs(r[4]) / 10.0)
                last = t
        print("-" * 62)
        print("  推扰事件（|θ|≥4.5°，2 s 合并）: %d 次" % len(ev))
        print("  各次峰值                 : %s" %
              " / ".join("%.1f°" % e[1] for e in ev))


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "S4"
    if arg.lower() == "all":
        for name in ["S0", "S0b2", "S1", "S2", "S3", "S4", "S6", "S7b", "S9", "S10"]:
            report(name)
    else:
        report(arg)


if __name__ == "__main__":
    main()
