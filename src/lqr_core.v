//======================================================================
//  lqr_core.v  --  1 kHz LQR 状态反馈
//
//  状态：[θ, θ̇, φ, φ̇]，单位都是 0.1° 或 0.1°/s
//  输出：duty（PWM counts，2500 = 100%）
//
//  u = −(Kθ·θ + Kdθ·θ̇ + Kφ·φ + Kdφ·φ̇)
//  定点：所有 K 乘 256（Q8），最后右移 8
//
//  增益来源（实测参数，Q=diag(100,1,10,1)、R=100）：
//    K = [−3.21, −0.31, −0.32, −0.18] N·m / (rad, rad/s, rad, rad/s)
//    1 个 0.1° = 1.7453e-3 rad；1 N·m = 11574 counts（0.216 N·m = 2500）
//    Kθ  = 3.21 × 1.7453e-3 × 11574 = 64.87  → Q8 16607
//    Kdθ = 0.31 × ...                =  6.26  → Q8  1603
//    Kφ  = 0.32 × ...                =  6.46  → Q8  1654
//    Kdφ = 0.18 × ...                =  3.64  → Q8   932
//
//  安全：
//    |θ| > 300（30°）→ 输出清零
//    sa_lost        → 输出清零
//    输出限幅 LIMIT
//======================================================================
`timescale 1ns/1ps

module lqr_core #(
    parameter integer TH_LIMIT = 300,     // |θ| > 30° 就停
    parameter signed [15:0] DZ_POS = 16'sd113,   // 正向死区（实测）
    parameter signed [15:0] DZ_NEG = 16'sd167,   // 反向死区（实测）
    parameter signed [31:0] KTH  = 32'sd4600,
    parameter signed [31:0] KDT  = 32'sd120,
    parameter signed [31:0] KPH  = -32'sd30,      // φ 外环：-10 方向已确认，阶梯加到 -30
    parameter signed [31:0] KDP  = 32'sd75,        // φ̇ 阻尼：+50 与 +100 之间
    parameter signed [31:0] TH_TGT_LIM = 32'sd50   // φ 外环 θ 目标限幅（0.1°，50=5°）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick1k,        // 1 kHz 使能
    input  wire               enable,        // 1 = 闭环
    input  wire               sa_lost,
    input  wire signed [15:0] limit,        // 输出限幅（counts）
    input  wire signed [15:0] theta10,       // 0.1°，±1800
    input  wire signed [15:0] phi10,         // 0.1°
    input  wire signed [15:0] phid10,        // 0.1°/s
    input  wire signed [15:0] phi_tgt,       // 5.0 跟随：横臂目标偏置（0.1°，旋钮给出）
    output wire               ready_o,       // 诊断：内部 init 完成
    output wire signed [15:0] dphi_out,      // 5.0：当前 φ−φ0（供跟随模式初始化目标）
    output reg  signed [15:0] duty,
    output reg  signed [15:0] thetad10,      // θ̇ 估计值（诊断用）
    output reg  signed [15:0] u_raw          // 限幅前的控制量（诊断用）
);

    // 30 ms 基线差分：每 5 ms 存一拍，用 6 拍前的值做差
    //   噪声：±0.13° 的位置噪声 -> 0.18°/30ms = 6°/s（1 ms 差分是 130°/s）
    reg  signed [15:0] h [0:4];
    // 积分项：消除纯 P 的稳态漂移
    localparam integer       KI    = 1;   // 10.0g: 2 -> 1（减弱积分，试压 0.1 Hz 慢漂；KI=0 历史上判废）
    localparam signed [31:0] I_LIM = 32'sd100000;
    reg  signed [31:0] integ;
    reg  signed [15:0] phi0;      // 启用瞬间的 φ，作为参考零点
    reg                en_d;
    reg  signed [15:0] theta_p;
    reg  signed [15:0] thd_lpf;      // 10.0i: θ̇ 低通状态
    reg  signed [15:0] thd_raw;      // 10.0i: θ̇ 限幅寄存
    reg         [2:0]  sub5;
    reg         [5:0]  init_cnt;
    reg                ready;
    integer            i;

    // 4 ms 基线差分：1/0.004 = 250，单位 0.1°/s
    wire signed [31:0] dth4 = ($signed(theta10) - $signed(h[4])) * 32'sd250;

    // ★ 2026-11-02 加：是否处在"可以正常控制"的范围内
    //   用它同时管两件事：出范围时输出清零、积分清零。
    assign ready_o = ready;

    wire in_range = !sa_lost &&
                    (theta10 <=  $signed(TH_LIMIT[15:0])) &&
                    (theta10 >= -$signed(TH_LIMIT[15:0])) &&
                    ready;

    // φ 相对启用时刻的偏差，绕回 ±1800
    // ★ 10.0c：φ 反馈一阶低通（tau≈8 ms），只滤量化噪声，不降环路增益
    reg  signed [15:0] phi_lpf;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phi_lpf <= 16'sd0;
        else        phi_lpf <= phi_lpf + (($signed(phi10) - $signed(phi_lpf)) >>> 3);
    end

    wire signed [16:0] dphi_w = $signed(phi_lpf) - $signed(phi0);
    wire signed [15:0] dphi_wr = (dphi_w >  17'sd1800) ? (dphi_w - 17'sd3600) :
                                 (dphi_w < -17'sd1800) ? (dphi_w + 17'sd3600) :
                                                         dphi_w[15:0];
    // ★ 5.0 跟随：把旋钮给的目标偏置从 φ 误差里减掉，横臂就会转去 phi_tgt
    wire signed [16:0] dphi_s = $signed(dphi_wr) - $signed(phi_tgt);
    assign dphi_out = dphi_wr;

    // ★ 9.0：φ 环死区 0.5°（5 个 0.1°）——消除编码器量化/静摩擦造成的 5 Hz 抖动
    localparam signed [15:0] PHI_DZ = 16'sd5;   // 10.0h: 0.5 度死区（配合 KI=1）
    wire signed [15:0] dphi_dz = (dphi < PHI_DZ) && (dphi > -PHI_DZ) ? 16'sd0 : dphi;
    // ★ 5.0：φ 误差限幅已撤销 —— 把它夹在 4° 会让位置环变成开关控制
    //   （偏差一超 4° 就是恒定的 ±10.5° 倾角指令），横臂被恒力矩推着自激转圈。
    //   真正的保护改成在 bal_top 里"目标不超出实际位置 5°"。
    wire signed [15:0] dphi   = (dphi_s >  17'sd1800) ? (dphi_s - 17'sd3600) :
                                (dphi_s < -17'sd1800) ? (dphi_s + 17'sd3600) :
                                                        dphi_s[15:0];

    // 外环：φ 偏差 -> θ 目标偏移（慢环，不直接出力矩）
    // 2026-10-30 时序：dphi 打拍后，用 (2-32)=-30 的移位实现，避免长加法链
    reg  signed [15:0] dphi_r;
    reg  signed [31:0] dphi_x32;
    reg  signed [31:0] dphi_x2;
    reg  signed [31:0] th_tgt_r;
    reg  signed [31:0] th_err_r;
    // ★ 2026-11-02（LQR_2.1）：原版 th_tgt = KPH*dphi/256 没有限幅。
    //   dphi 绕回 ±1800 时（起摆把横臂带过 180°，或平衡中横臂自己转过去），
    //   th_tgt 会瞬间跳到 ∓211（21.1°），再乘 KTH=4600 直接把 duty 打满 —
    //   这是"平衡中突然倒"的候选原因之一。这里钳到 ±TH_TGT_LIM（默认 5°）。
    //   平衡时 |dphi| 一般 < 5°，所以这刀平时根本不剪。
    wire signed [31:0] th_tgt_raw = ($signed(dphi_x2) - $signed(dphi_x32)) >>> 8;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dphi_r    <= 16'sd0;
            dphi_x32  <= 32'sd0;
            dphi_x2   <= 32'sd0;
            th_tgt_r  <= 32'sd0;
            th_err_r  <= 32'sd0;
        end
        else begin
            dphi_r   <= dphi_dz;
            dphi_x32 <= $signed(dphi_r) <<< 5;   // x32 -> KPH=-30
            dphi_x2  <= $signed(dphi_r) <<< 1;   // x2
            // KPH=-30，再钳 ±TH_TGT_LIM
            th_tgt_r <= (th_tgt_raw >  TH_TGT_LIM) ?  TH_TGT_LIM :
                        (th_tgt_raw < -TH_TGT_LIM) ? -TH_TGT_LIM : th_tgt_raw;
            th_err_r <= $signed(theta10) - th_tgt_r;
        end
    end

    // KDP=+75 的移位实现：+75 = 64 + 8 + 2 + 1
    reg  signed [31:0] kdp_x64;
    reg  signed [31:0] kdp_x8;
    reg  signed [31:0] kdp_x2;
    reg  signed [31:0] kdp_x1;
    reg  signed [31:0] kdp_term_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kdp_x64    <= 32'sd0;
            kdp_x8     <= 32'sd0;
            kdp_x2     <= 32'sd0;
            kdp_x1     <= 32'sd0;
            kdp_term_r <= 32'sd0;
        end
        else begin
            // ★ KDP = 130 = 128 + 2（实测甜点：75 荡得多、192 太钝容易倒）
            // ★ 10.0g: KDP 回到 130 = 128 + 2（保抗扰）
            kdp_x64 <= $signed(phid10) <<< 7;
            kdp_x8  <= $signed(phid10) <<< 1;
            kdp_x2  <= 32'sd0;
            kdp_x1  <= 32'sd0;
            kdp_term_r <= $signed(kdp_x64) + $signed(kdp_x8)
                        + $signed(kdp_x2) + $signed(kdp_x1);
        end
    end

    wire signed [31:0] prod = KTH * $signed(th_err_r)
                            + KI  * integ
                            + KDT * $signed(thetad10)
                            + kdp_term_r;
    // ★ 2026-10-30 修：这里原来还有一项 `+ KPH * phi10`，是串级改造时漏删的。
    //   它让同一个 KPH 既驱动外环（挪 θ 目标）又直接出力矩，且用的是上电原点的绝对 φ。
    //   之前两次 φ 实验（KPH=800 直接项、KPH=50 串级）因此全部作废，要重做。
    wire signed [31:0] u_p    = -(prod >>> 8);
    // 死区前馈：摩擦吃掉的那部分先补上，否则 |θ| 小于 ~0.9° 时电机根本不转
    //   补过头会抖动，所以只在输出非零时补，且限幅前加
    wire signed [31:0] u_c    = (u_p >  32'sd0) ? (u_p + 32'sd113) :
                                (u_p < -32'sd0) ? (u_p - 32'sd167) : 32'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            theta_p   <= 16'sd0;
            thetad10  <= 16'sd0;
            thd_lpf   <= 16'sd0;
            thd_raw   <= 16'sd0;
            duty      <= 16'sd0;
            u_raw     <= 16'sd0;
            init_cnt  <= 6'd0;
            ready     <= 1'b0;
            integ     <= 32'sd0;
            phi0      <= 16'sd0;
            en_d      <= 1'b0;
            sub5      <= 3'd0;
            for (i = 0; i < 5; i = i + 1) h[i] <= 16'sd0;
        end
        else if (tick1k) begin
            theta_p <= theta10;

            if (!enable) begin
                thetad10 <= 16'sd0;
                duty     <= 16'sd0;
                u_raw    <= 16'sd0;
                init_cnt <= 6'd0;
                ready    <= 1'b0;
                sub5     <= 3'd0;
                integ <= 32'sd0;
                en_d  <= 1'b0;
                for (i = 0; i < 5; i = i + 1) h[i] <= theta10;
            end
            else begin
                // 每 1 ms 移位一次，用 4 ms 基线做差分
                h[0] <= theta10; h[1] <= h[0]; h[2] <= h[1];
                h[3] <= h[2];    h[4] <= h[3];
                if (init_cnt != 6'd8) begin
                    init_cnt <= init_cnt + 6'd1;     // 8 ms 建立初值
                    thetad10 <= 16'sd0;
                    ready    <= 1'b0;
                end
                else begin
                    ready    <= 1'b1;
                    // 4 ms 基线差分（原来 30 ms，延迟太大）
                    //   延迟从 ~40 ms 降到 ~10 ms，摆杆时间常数 88 ms 才够裕度
                    // ★ 10.0i：θ̇ 两拍流水（限幅寄存 -> 低通），压 4 ms 差分的量化噪声
                    // ★ 1.1：8 ms -> 2 ms。8 ms 在 5 Hz 吃 14°、11.4 Hz 吃 33°，
                    //   会把 KDT 的相位超前吃光；KDT 同时 60 -> 120 补 5 Hz 阻尼。
                    thd_raw <= (dth4 > 32'sd20000) ? 16'sd20000 :
                               (dth4 < -32'sd20000) ? -16'sd20000 : dth4[15:0];
                    thd_lpf <= thd_lpf + ((thd_raw - thd_lpf) >>> 1);
                    thetad10 <= thd_lpf;
                end

                // 启用上升沿：把当前 φ 记作参考零点
                if (!en_d) phi0 <= phi10;
                en_d <= 1'b1;

                // 积分累加（限幅防饱和）
                // ★ 2026-11-02 修：原版不管 θ 在哪都在积分。摆杆下垂 θ=±1800 时
                //   每秒累加 1.8e6，50 ms 就把 integ 顶到 ±100000，KI 项变成
                //   恒定的 ±200000（= ±781 counts 力矩）。起摆/手扶把摆杆抬进
                //   30° 的那一瞬间会白挨一记满幅力矩 —— 这是"加了起摆就立不起来"
                //   的一个直接原因。现在只在 in_range 内积分，出范围直接清零。
                if (in_range) begin
                    if (integ > I_LIM) integ <= I_LIM;
                    else if (integ < -I_LIM) integ <= -I_LIM;
                    else integ <= integ + theta10;
                end
                else
                    integ <= 32'sd0;

                // 安全判据
                if (!in_range) begin
                    duty  <= 16'sd0;
                    u_raw <= 16'sd0;
                end
                else begin
                    u_raw <= (u_c >  32'sd30000) ? 16'sd30000 :
                             (u_c < -32'sd30000) ? -16'sd30000 : u_c[15:0];
                    if (u_c >  $signed(limit))
                        duty <= limit;
                    else if (u_c < -$signed(limit))
                        duty <= -limit;
                    else
                        duty <= u_c[15:0];
                end
            end
        end
    end

endmodule
