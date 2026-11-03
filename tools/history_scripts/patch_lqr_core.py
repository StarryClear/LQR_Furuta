# -*- coding: utf-8 -*-
"""patch_lqr_core.py -- 给 lqr_core.v 加 φ 外环 θ 目标钳位（LQR_2.1 版本 B）"""
import io, sys

P = r"D:\DeepSeek_Harness\对话文件\work\LQR_2.1_swing\src\lqr_core.v"
s = io.open(P, encoding="utf-8").read()

def rep(old, new, tag):
    global s
    n = s.count(old)
    if n != 1:
        print("!! %s: found %d" % (tag, n)); sys.exit(1)
    s = s.replace(old, new); print("ok %s" % tag)

rep("""    parameter signed [31:0] KDP  = 32'sd75         // φ̇ 阻尼：+50 与 +100 之间
)(""",
    """    parameter signed [31:0] KDP  = 32'sd75,        // φ̇ 阻尼：+50 与 +100 之间
    parameter signed [31:0] TH_TGT_LIM = 32'sd50   // φ 外环 θ 目标限幅（0.1°，50=5°）
)(""", "param")

rep("""    reg  signed [31:0] th_tgt_r;
    reg  signed [31:0] th_err_r;""",
    """    reg  signed [31:0] th_tgt_r;
    reg  signed [31:0] th_err_r;
    // ★ 2026-11-02（LQR_2.1）：原版 th_tgt = KPH*dphi/256 没有限幅。
    //   dphi 绕回 ±1800 时（起摆把横臂带过 180°，或平衡中横臂自己转过去），
    //   th_tgt 会瞬间跳到 ∓211（21.1°），再乘 KTH=4600 直接把 duty 打满 —
    //   这是"平衡中突然倒"的候选原因之一。这里钳到 ±TH_TGT_LIM（默认 5°）。
    //   平衡时 |dphi| 一般 < 5°，所以这刀平时根本不剪。
    wire signed [31:0] th_tgt_raw = ($signed(dphi_x2) - $signed(dphi_x32)) >>> 8;""", "wire")

rep("""            // KPH=-30
            th_tgt_r <= ($signed(dphi_x2) - $signed(dphi_x32)) >>> 8;""",
    """            // KPH=-30，再钳 ±TH_TGT_LIM
            th_tgt_r <= (th_tgt_raw >  TH_TGT_LIM) ?  TH_TGT_LIM :
                        (th_tgt_raw < -TH_TGT_LIM) ? -TH_TGT_LIM : th_tgt_raw;""", "clamp")

io.open(P, "w", encoding="utf-8", newline="\n").write(s)
print("written", len(s))
