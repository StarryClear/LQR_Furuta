# -*- coding: utf-8 -*-
"""plot_measured.py -- 把 docs/data/ 里的实测 CSV 画成报告用图。

不依赖 matplotlib（本机没有），用 Pillow 直接画。
输出到 docs/figures/*.png（1600x900，2 倍超采样后 Lanczos 缩小）。

用法：
    python tools/plot_measured.py

数据字段（VOFA JustFloat 6 通道，100 Hz 落盘）：
    t      采样序号（100 Hz -> 除以 100 得秒）
    theta  摆杆倾角，单位 0.1°
    duty   电机占空比，单位 counts（2500 = 100%）
    state  起摆状态机 sw_state：0 停 / 1 kick / 2 反踢 / 3 泵能 / 4 等待 / 5 平衡 / 6 掉摆
    pos    横臂编码器相对计数（824.5 counts/圈）
"""
import os
import sys

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "docs", "data")
OUT = os.path.join(ROOT, "docs", "figures")

S = 2                      # 超采样倍数
W, H = 1600, 900           # 最终尺寸

C_TH = (20, 90, 170)       # theta 曲线
C_DUTY = (200, 90, 20)     # duty 曲线
C_GRID = (225, 225, 225)
C_AXIS = (60, 60, 60)
C_MARK = (150, 150, 150)


def font(size, bold=False):
    cands = ["msyhbd.ttc", "msyh.ttc", "simhei.ttf", "simsun.ttc"] if bold else \
            ["msyh.ttc", "simhei.ttf", "simsun.ttc"]
    for c in cands:
        p = os.path.join("C:\\Windows\\Fonts", c)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size * S)
            except Exception:
                pass
    return ImageFont.load_default()


F_TITLE = font(30, True)
F_LABEL = font(20)
F_TICK = font(17)
F_SMALL = font(16)


class Panel:
    """一个坐标区：把数据坐标映射到像素"""

    def __init__(self, draw, box, xlim, ylim):
        self.d = draw
        self.x0, self.y0, self.x1, self.y1 = [v * S for v in box]
        self.xlim = xlim
        self.ylim = ylim

    def px(self, x):
        a, b = self.xlim
        return self.x0 + (x - a) / (b - a) * (self.x1 - self.x0)

    def py(self, y):
        a, b = self.ylim
        return self.y1 - (y - a) / (b - a) * (self.y1 - self.y0)

    def frame(self):
        self.d.rectangle([self.x0, self.y0, self.x1, self.y1], outline=C_AXIS, width=2 * S)

    def grid_y(self, ticks, fmt="%g"):
        for t in ticks:
            if not (self.ylim[0] <= t <= self.ylim[1]):
                continue
            y = self.py(t)
            self.d.line([self.x0, y, self.x1, y], fill=C_GRID, width=S)
            self.d.text((self.x0 - 10 * S, y), fmt % t, font=F_TICK,
                        fill=C_AXIS, anchor="rm")

    def grid_x(self, ticks, fmt="%g"):
        for t in ticks:
            if not (self.xlim[0] <= t <= self.xlim[1]):
                continue
            x = self.px(t)
            self.d.line([x, self.y0, x, self.y1], fill=C_GRID, width=S)
            self.d.text((x, self.y1 + 8 * S), fmt % t, font=F_TICK,
                        fill=C_AXIS, anchor="mt")

    def curve(self, xs, ys, color, width=2, wrap=None, clip=False):
        """wrap: 相邻两点跳变超过该值就断线（避免 ±180° 绕回画出假竖线）
        clip: 超出 ylim 的点直接丢弃并断线（避免画到画框外面去）"""
        xs = list(xs)
        ys = list(ys)
        seg = []

        def flush():
            if len(seg) > 1:
                self.d.line(seg, fill=color, width=width * S, joint="curve")

        for i in range(len(xs)):
            if clip and not (self.ylim[0] <= ys[i] <= self.ylim[1]):
                flush()
                seg = []
                continue
            if seg and wrap is not None and abs(ys[i] - ys[i - 1]) > wrap:
                flush()
                seg = []
            seg.append((self.px(xs[i]), self.py(ys[i])))
        flush()

    def ylabel(self, text, color=C_AXIS, size=20):
        # 竖排标签画在左侧（宽度按文字长度自适应）
        f = font(size)
        tw = int(f.getlength(text)) + 8 * S
        img_tmp = Image.new("RGBA", (tw, 32 * S), (255, 255, 255, 0))
        d2 = ImageDraw.Draw(img_tmp)
        d2.text((4 * S, 4 * S), text, font=f, fill=color)
        rot = img_tmp.rotate(90, expand=True)
        self.d._image.paste(rot, (int(self.x0 - 96 * S), int((self.y0 + self.y1) / 2 - rot.size[1] / 2)), rot)


def title(d, text, sub=None):
    d.text((60 * S, 26 * S), text, font=F_TITLE, fill=(25, 25, 25))
    if sub:
        d.text((60 * S, 62 * S), sub, font=F_SMALL, fill=(110, 110, 110))


def load(name):
    d = pd.read_csv(os.path.join(DATA, name))
    d["sec"] = d["t"] / 100.0
    d["th_deg"] = d["theta"] / 10.0
    return d


def fig_swingup():
    d = load("swing10.csv")
    lo, hi = 8.0, 32.0
    m = (d.sec >= lo) & (d.sec <= hi)
    dd = d[m]

    im = Image.new("RGB", (W * S, H * S), "white")
    dr = ImageDraw.Draw(im)
    im._image = im  # 供 ylabel 用
    title(dr, "图1  自动起摆过程（swing10.csv，第 2 次与第 3 次尝试）",
          "kick -> 泵能 -> 捕获进平衡；PWM 满量程 2500 counts；state 0 停 / 1 kick / 2 反踢 / 3 泵能 / 5 平衡 / 6 掉摆")

    p1 = Panel(dr, (185, 130, 1540, 520), (lo, hi), (-190, 190))
    p1.frame(); p1.grid_y([-180, -120, -60, 0, 60, 120, 180]); p1.grid_x(range(8, 33, 2), "%d")
    p1.curve(dd.sec, dd.th_deg, C_TH, 1, wrap=90, clip=True)
    p1.ylabel("摆杆倾角 θ (deg)", C_TH)

    p2 = Panel(dr, (185, 610, 1540, 850), (lo, hi), (-2100, 2100))
    p2.frame(); p2.grid_y([-2000, -1000, 0, 1000, 2000]); p2.grid_x(range(8, 33, 2), "%d")
    p2.curve(dd.sec, dd.duty, C_DUTY, clip=True)
    p2.ylabel("PWM 占空比 (counts)", C_DUTY, 18)

    # 状态切换竖线
    st = dd.state.values
    sec = dd.sec.values
    for i in range(1, len(st)):
        if st[i] != st[i - 1]:
            x = p1.px(sec[i])
            dr.line([x, p1.y0, x, p2.y1], fill=C_MARK, width=S)
            dr.text((x + 3 * S, p1.y0 + 4 * S), str(int(st[i])), font=F_SMALL, fill=(120, 120, 120))

    im.resize((W, H), Image.LANCZOS).save(os.path.join(OUT, "fig1_swingup.png"))
    print("fig1_swingup.png")


def fig_balance():
    d = load("hold10min.csv")
    s = d.state.values
    # 找最长 state==5 段
    best, cur, start = (0, 0), 0, 0
    for i, v in enumerate(s):
        if v == 5:
            if cur == 0:
                start = i
            cur += 1
            if cur > best[1]:
                best = (start, cur)
        else:
            cur = 0
    i0, n = best
    seg = d.iloc[i0:i0 + n].copy()
    seg["rel"] = (seg.sec - seg.sec.iloc[0])

    im = Image.new("RGB", (W * S, H * S), "white")
    dr = ImageDraw.Draw(im)
    im._image = im
    title(dr, "图2  连续平衡 514.8 s（hold10min.csv，唯一闭环段）",
          "|θ| p50 1.70° / p95 3.50° / max 4.6°（去前 5 s）；duty 峰值 1850，全程 0 帧饱和（限幅 2000）")

    p1 = Panel(dr, (185, 130, 1540, 500), (0, n / 100.0), (-10, 10))
    p1.frame(); p1.grid_y([-8, -6, -4, -2, 0, 2, 4, 6, 8]); p1.grid_x(range(0, 520, 50), "%d")
    p1.curve(seg.rel.values, seg.th_deg.values, C_TH, 1, wrap=90, clip=True)
    p1.ylabel("摆杆倾角 θ (deg)", C_TH)
    dr.text((p1.x0 + 12 * S, p1.y0 + 10 * S), "θ 带 = ±4°，全程未出带外 >5°",
            font=F_SMALL, fill=(120, 120, 120))
    for t in (-4, 4):
        dr.line([p1.x0, p1.py(t), p1.x1, p1.py(t)], fill=(235, 200, 200), width=S)

    p2 = Panel(dr, (185, 590, 1540, 850), (0, n / 100.0), (-2100, 2100))
    p2.frame(); p2.grid_y([-2000, -1000, 0, 1000, 2000]); p2.grid_x(range(0, 520, 50), "%d")
    p2.curve(seg.rel.values, seg.duty.values, C_DUTY, 1, clip=True)
    p2.ylabel("PWM 占空比 (counts)", C_DUTY, 18)

    im.resize((W, H), Image.LANCZOS).save(os.path.join(OUT, "fig2_balance.png"))
    print("fig2_balance.png  seg=%d frames (%.1f s)" % (n, n / 100.0))
    return seg


def fig_disturb():
    d = load("disturb3.csv")
    im = Image.new("RGB", (W * S, H * S), "white")
    dr = ImageDraw.Draw(im)
    im._image = im
    title(dr, "图3  抗瞬时推扰（disturb3.csv，120 s，手工推扰 35 次）",
          "33/35 次自行恢复，可恢复的最大峰值角 26.7°；2 次掉摆后进 S_FALL 并可重起")

    xmax = d.sec.max()
    p1 = Panel(dr, (185, 130, 1540, 560), (0, xmax), (-60, 60))
    p1.frame(); p1.grid_y([-60, -40, -20, 0, 20, 40, 60]); p1.grid_x(range(0, int(xmax) + 1, 10), "%d")
    p1.curve(d.sec.values, d.th_deg.values, C_TH, 1, wrap=90, clip=True)
    p1.ylabel("摆杆倾角 θ (deg)", C_TH)

    p2 = Panel(dr, (185, 640, 1540, 850), (0, xmax), (-2100, 2100))
    p2.frame(); p2.grid_y([-2000, -1000, 0, 1000, 2000]); p2.grid_x(range(0, int(xmax) + 1, 10), "%d")
    p2.curve(d.sec.values, d.duty.values, C_DUTY, 1, clip=True)
    p2.ylabel("PWM 占空比 (counts)", C_DUTY, 18)

    im.resize((W, H), Image.LANCZOS).save(os.path.join(OUT, "fig3_disturb.png"))
    print("fig3_disturb.png")


def fig_capture():
    """每次尝试的 kick->平衡 时间柱状图（T1/T2）"""
    d = load("swing10.csv")
    s = d.state.values
    sec = d.sec.values
    rows = []
    in_att = False
    for i in range(1, len(s)):
        if s[i] == 1 and s[i - 1] == 0:          # 一次起摆开始
            start = sec[i]
            ok = False
            for j in range(i + 1, min(i + 800, len(s))):
                if s[j] == 5:
                    rows.append((len(rows) + 1, sec[j] - start, True))
                    in_att = True
                    ok = True
                    break
                if s[j] == 0:
                    break
            if not ok:
                rows.append((len(rows) + 1, np.nan, False))
    times = [r[1] for r in rows if r[2]]
    ok_n = len(times)

    im = Image.new("RGB", (W * S, H * S), "white")
    dr = ImageDraw.Draw(im)
    im._image = im
    title(dr, "图4  起摆捕获时间（%d/%d 次成功，中位 %.2f s）" % (ok_n, len(rows), np.median(times)),
          "纵轴 = 从 state1(kick) 到 state5(平衡) 的时间；灰柱 = 该次未捕获")

    ymax = max(max(times), 2.0) * 1.15
    p = Panel(dr, (185, 140, 1540, 820), (0.3, len(rows) + 0.7), (0, ymax))
    p.frame(); p.grid_y([0, 0.5, 1.0, 1.5, 2.0][:int(ymax / 0.5) + 1])
    bw = (p.x1 - p.x0) / (len(rows) + 0.7) * 0.6
    for idx, tsec, ok in rows:
        x = p.px(idx)
        if ok:
            dr.rectangle([x - bw / 2, p.py(tsec), x + bw / 2, p.py(0)], fill=(90, 150, 210))
            dr.text((x, p.py(tsec) - 8 * S), "%.2f" % tsec, font=F_TICK, fill=C_AXIS, anchor="mb")
        else:
            dr.rectangle([x - bw / 2, p.py(0) - 14 * S, x + bw / 2, p.py(0)], fill=(190, 190, 190))
            dr.text((x, p.py(0) - 20 * S), "未捕获", font=F_SMALL, fill=(140, 140, 140), anchor="mb")
        dr.text((x, p.py(0) + 6 * S), str(idx), font=F_TICK, fill=C_AXIS, anchor="mt")
    p.ylabel("捕获时间 (s)")
    dr.line([p.x0, p.py(1.14), p.x1, p.py(1.14)], fill=(220, 140, 140), width=2 * S)
    dr.text((p.x0 + 8 * S, p.py(1.14) - 10 * S), "中位 1.14 s", font=F_SMALL,
            fill=(200, 100, 100), anchor="lb")

    im.resize((W, H), Image.LANCZOS).save(os.path.join(OUT, "fig4_capture_time.png"))
    print("fig4_capture_time.png  success=%d/%d" % (ok_n, len(rows)))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT, exist_ok=True)
    fig_swingup()
    fig_balance()
    fig_disturb()
    fig_capture()
    print("done ->", OUT)
