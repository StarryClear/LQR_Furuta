# -*- coding: utf-8 -*-
"""patch_bal_top.py -- 把 LQR_1.0 的 bal_top.v 改成 LQR_2.1（起摆 + 平衡）"""
import io, sys, os

P = r"D:\DeepSeek_Harness\对话文件\work\LQR_2.1_swing\src\bal_top.v"
s = io.open(P, encoding="utf-8").read()

def rep(old, new, tag):
    global s
    n = s.count(old)
    if n != 1:
        print("!! %s: found %d" % (tag, n))
        sys.exit(1)
    s = s.replace(old, new)
    print("ok %s" % tag)

HEAD_OLD = """//======================================================================
//  bal_top.v  --  LQR_2.0 版本 A：纯平衡固件（无起摆 / 无 θ 低通）
//
//  目的（2026-11-02）：
//    1.0 在加入起摆以后“立不起来”。这一版把起摆整段删掉，并把 θ 的
//    一阶低通去掉，回到最朴素的 LQR 平衡，用来确认 1.0 的平衡内核
//    本身还能不能站住。
//
//  和 LQR_1.0\\LQR_Furuta_Test\\src\\test_top.v 的差别（只有这 5 条）：
//    ① 删掉 swing_ctrl 实例、sw_* 信号、lqr_captured 锁存、excite 实例
//       -> LQR 由 `run`（D11 短按）直接使能：lqr_on = run
//    ② 删掉 θ 的一阶低通 theta_f，LQR 直接用 theta_lqr_r
//       （TH_LPF_EN = 0；改回 1 就恢复 1.0 原版的 τ≈16 ms 低通）
//    ③ 删掉一直没人用的法兰倾斜补偿链路（ph_mod / slot / tilt_tab，
//       纯死代码，ch5 仍然输出 position_raw）
//    ④ lqr_core 里积分项加了抗饱和：|θ|>TH_LIMIT 时清零，不再在摆杆
//       下垂时把积分积到满幅（见 lqr_core.v 注释）
//    ⑤ 其余（θ/φ/φ̇ 链路、死区、限幅、OLED、遥测、按键）一字未改
//
//  安全：
//    |θ|>30°           -> lqr_core 内部清零输出
//    sensor_lost       -> 清零输出
//    ADC>1800 连续5ms  -> fault 锁存，电机 EN 拉低（换模式/重新启动清）
//
//  遥测（100 Hz VOFA JustFloat 6 通道）：
//    ch0 mode / ch1 duty / ch2 pos / ch3 θ / ch4 adc / ch5 sa
//======================================================================"""

HEAD_NEW = """//======================================================================
//  bal_top.v  --  LQR_2.1 版本 B：起摆 + LQR 平衡
//
//  基线 = LQR_1.0（版本 A，实测能站住的纯平衡固件）。平衡内核原样保留：
//    θ/φ/φ̇ 链路、KTH4600/KDT60/KDP75/KI2/KPH-30、死区 113/167、限幅 2000。
//  相对版本 A 只改四件事：
//    ① φ 的 `% 3600` 换成 udiv32 顺序除法（1 ms 一次、32 拍），
//       摘掉版本 A 里 slack 只有 +0.001 ns 的 mult_384 -> phi_mod_r 路径；
//    ② 接回 swing_ctrl（6.0 / 666.4 的能量泵起摆，参数用 666.4 实测组），
//       mA->duty 拆成 3 级流水（1.0 含起摆那一版就是被这条链拖到 -2.97 ns）；
//    ③ 交接用状态机握手：lqr_on = run && sw_state==5。
//       不再用 1.0 那个“|θ|<15° 就永久锁存”的 lqr_captured（起摆还没
//       真正把摆送到位就把电机抢走，结果必然倒）；掉出 25° 自动重摆；
//    ④ lqr_core 的 φ 外环 θ 目标加 ±5° 钳位，避免 dphi 绕回 ±1800 时
//       出现 ±21° 的阶跃（README 里“平衡中突然倒”的另一候选）。
//
//  起摆参数（666.4 实测能起摆的那一组）：
//    invert_pd=1  L_MM=200  KICK 1300mA/250ms  floor 700->1500mA
//    ANG_CAP=200(20°)  W_CAP=100°/s  E_STOP=-5898  E_RESUME=-10486
//  上板若发现“越摆越低 / 一启动就往一边甩”，把 SW_INVERT 改 0 再编译。
//
//  安全：
//    |θ|>30°(300)      -> lqr_core 清零输出；起摆状态机 2 帧后重摆
//    sensor_lost       -> 起摆停、LQR 清零
//    ADC>1800 连续5ms  -> fault 锁存，电机 EN 拉低（换模式/重新启动清）
//
//  遥测（100 Hz VOFA JustFloat 6 通道）：
//    ch0 = mode + 16*sw_state（bit3..0 = mode，bit6..4 = 起摆状态 0..6）
//    ch1 duty / ch2 pos / ch3 θ / ch4 adc / ch5 sa
//======================================================================"""
rep(HEAD_OLD, HEAD_NEW, "header")

LP_OLD = """    // 0 = θ 直接进 LQR（本版）；1 = 恢复 1.0 原版 θ 一阶低通（τ≈16 ms）
    localparam integer TH_LPF_EN = 0;"""
LP_NEW = """    // 0 = θ 直接进 LQR（本版）；1 = 恢复 1.0 原版 θ 一阶低通（τ≈16 ms）
    localparam integer TH_LPF_EN = 0;
    // 1 = 起摆使能；0 = 退化成版本 A 的纯平衡（平衡逻辑全保留，便于 A/B）
    localparam integer SWING_EN   = 1;
    // 起摆泵能方向：666.4 实测为 1；若越摆越低就改 0
    localparam integer SW_INVERT  = 1;"""
rep(LP_OLD, LP_NEW, "localparam")

PHI_OLD = """    //------------------------------------------------------------------
    // φ：编码器 counts → 0.1°，绕回 ±1800
    //------------------------------------------------------------------
    reg  signed [31:0] phi_raw_r;
    reg  signed [31:0] phi_mod_r;
    reg  signed [15:0] phi10_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            phi_raw_r <= 32'sd0;
            phi_mod_r <= 32'sd0;
            phi10_r   <= 16'sd0;
        end
        else begin
            phi_raw_r <= (pos_r * 32'sd4471) >>> 10;   // 3600/824.5
            phi_mod_r <= phi_raw_r % 32'sd3600;
            phi10_r   <= (phi_mod_r > 32'sd1800) ? (phi_mod_r - 32'sd3600)
                       : (phi_mod_r < -32'sd1800) ? (phi_mod_r + 32'sd3600)
                       : phi_mod_r[15:0];
        end
    end
    wire signed [15:0] phi10 = phi10_r;"""

PHI_NEW = """    //------------------------------------------------------------------
    // φ：编码器 counts → 0.1°，绕回 ±1800
    //   版本 B：`% 3600` 换成 udiv32 顺序除法（1 ms 一次、32 拍），
    //   把版本 A 里 mult_384_s2/DOUT[26] -> phi_mod_r 那条 19.964 ns
    //   （slack 0.001 ns）的关键路径整条摘掉。
    //------------------------------------------------------------------
    reg  signed [31:0] phi_raw_r;
    reg  signed [15:0] phi10_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            phi_raw_r <= 32'sd0;
        else
            phi_raw_r <= (pos_r * 32'sd4471) >>> 10;   // 3600/824.5
    end

    wire signed [31:0] phi_abs_w = phi_raw_r[31] ? -phi_raw_r : phi_raw_r;
    wire        [31:0] phi_rem;
    wire               phi_div_done, phi_div_busy;
    reg                phi_div_sign;
    wire               phi_div_start = tick1k && !phi_div_busy;

    udiv32 u_phi_div (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .start    (phi_div_start),
        .dividend (phi_abs_w),
        .divisor  (32'd3600),
        .quot     (),
        .rem      (phi_rem),
        .busy     (phi_div_busy),
        .done     (phi_div_done)
    );

    wire signed [31:0] phi_mod_w = phi_div_sign ? -$signed(phi_rem) : $signed(phi_rem);
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            phi10_r      <= 16'sd0;
            phi_div_sign <= 1'b0;
        end
        else begin
            if (phi_div_start)
                phi_div_sign <= phi_raw_r[31];
            if (phi_div_done)
                phi10_r <= (phi_mod_w >  32'sd1800) ? (phi_mod_w - 32'sd3600)
                         : (phi_mod_w < -32'sd1800) ? (phi_mod_w + 32'sd3600)
                         : phi_mod_w[15:0];
        end
    end
    wire signed [15:0] phi10 = phi10_r;"""
rep(PHI_OLD, PHI_NEW, "phi-div")

LQR_OLD = """    wire lqr_on = run;

    lqr_core u_lqr (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .tick1k   (tick1k),
        .enable   (lqr_on),
        .sa_lost  (sensor_lost),
        .limit    (lqr_limit),
        .theta10  (theta_lqr_in),
        .phi10    (phi_lqr_r),
        .phid10   (phid_lqr_r),
        .duty     (lqr_duty),
        .thetad10 (lqr_thetad),
        .u_raw    (lqr_uraw)
    );

    wire               en_out;
    wire               dir_out;
    wire [15:0]        duty_mag;

    wire signed [15:0] duty_f = run ? lqr_duty : 16'sd0;
    wire               en_f   = run && (lqr_duty != 16'sd0);"""

LQR_NEW = """    //------------------------------------------------------------------
    // 自动起摆：swing_ctrl（能量泵 + kick，6.0 / 666.4 那一版）
    //   frame_tick = angle_valid（856 Hz 角度帧）
    //   lqr_on = run && sw_state==BAL：只有起摆状态机说“进平衡”才把电机交给 LQR
    //------------------------------------------------------------------
    wire               sw_active, sw_drive, sw_bal_entry;
    wire signed [15:0] sw_i_ref;
    wire        [2:0]  sw_state;

    // 掉摆：进平衡后 |θ|>25° 连续 2 帧 -> 起摆状态机重新起摆
    //   （lqr_core 的硬门限是 30°，25° 先动手，给重摆留余量）
    reg  [1:0] fall_cnt;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            fall_cnt <= 2'd0;
        else if (!(run && (sw_state == 3'd5) && !sensor_lost))
            fall_cnt <= 2'd0;
        else if (angle_valid && ((theta10 > 16'sd250) || (theta10 < -16'sd250)))
            fall_cnt <= (fall_cnt == 2'd3) ? 2'd3 : fall_cnt + 2'd1;
        else
            fall_cnt <= 2'd0;
    end
    wire force_fall = (fall_cnt >= 2'd2) || sensor_lost;

    swing_ctrl #(
        .FRAME_HZ          (856),
        .L_MM              (200),
        .KICK_MS           (250),
        .I_KICK            (16'sd1300),
        .K_MA_PER_DPS      (16'sd6),
        .I_FLOOR_START     (16'sd700),
        .I_FLOOR_STEP      (16'sd100),
        .I_FLOOR_MAX       (16'sd1500),
        .I_MAX             (16'sd1500),
        .E_STOP_Q          (-16'sd5898),
        .E_RESUME_Q        (-16'sd10486),
        .W_CAP             (16'sd100),
        .ANG_CAP           (16'sd200),
        .ANG_FALL          (16'sd600),
        .REST_ANG          (16'sd1600),
        .REST_SPD          (16'sd50),
        .ARM_REST_SPD      (16'sd1500),
        .REST_MS           (400),
        .SETTLE_TIMEOUT_MS (6000),
        .WAIT_MS           (150),
        .STALL_MS          (300),
        .ESCALATE_MS       (2500),
        .TIMEOUT_MS        (30000),
        .SW_SIGN           (1),
        .AUTO_RESWING      (1)
    ) u_swing (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .frame_tick (angle_valid),
        .run_en     ((SWING_EN != 0) ? (run && !sensor_lost) : 1'b0),
        .angle      (theta10),
        .arm_speed  (phid10),
        .invert_pd  (SW_INVERT != 0),
        .force_fall (force_fall),
        .sw_active  (sw_active),
        .sw_drive   (sw_drive),
        .i_swing    (sw_i_ref),
        .sw_state   (sw_state),
        .bal_entry  (sw_bal_entry)
    );

    //------------------------------------------------------------------
    // 起摆电流 mA -> duty：3 级流水
    //   1.0 含起摆那一版的 -2.97 ns 就出在这一段：u_swing/state ->
    //   i_swing mux -> ×1.5 -> 限幅 -> duty mux -> motor_drv 挤在一个周期里。
    //   现在拆成 打拍 / 乘 / 限幅 三级，选择器只比两个寄存器。
    //------------------------------------------------------------------
    reg signed [15:0] sw_i_r;
    reg signed [31:0] sw_duty_mul;
    reg signed [15:0] sw_duty_r;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sw_i_r      <= 16'sd0;
            sw_duty_mul <= 32'sd0;
            sw_duty_r   <= 16'sd0;
        end
        else begin
            sw_i_r      <= sw_i_ref;
            // 6.0 的 1200 mA ≈ 本工程 1800 counts，取 1.5 倍保守换算
            sw_duty_mul <= ($signed(sw_i_r) * 32'sd3) >>> 1;
            sw_duty_r   <= (sw_duty_mul >  32'sd2000) ?  16'sd2000 :
                           (sw_duty_mul < -32'sd2000) ? -16'sd2000 : sw_duty_mul[15:0];
        end
    end

    //------------------------------------------------------------------
    // LQR 闭环（平衡内核与版本 A 完全一致，只加 TH_TGT_LIM 钳位）
    //------------------------------------------------------------------
    wire signed [15:0] lqr_duty, lqr_thetad, lqr_uraw;
    wire signed [15:0] lqr_limit = 16'sd2000;   // 80% duty

    wire lqr_on = run && (sw_state == 3'd5) && !sensor_lost;

    lqr_core #(
        .TH_TGT_LIM (32'sd50)   // φ 外环 θ 目标 ±5°，防 dphi 绕回产生 ±21° 阶跃
    ) u_lqr (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .tick1k   (tick1k),
        .enable   (lqr_on),
        .sa_lost  (sensor_lost),
        .limit    (lqr_limit),
        .theta10  (theta_lqr_in),
        .phi10    (phi_lqr_r),
        .phid10   (phid_lqr_r),
        .duty     (lqr_duty),
        .thetad10 (lqr_thetad),
        .u_raw    (lqr_uraw)
    );

    wire               en_out;
    wire               dir_out;
    wire [15:0]        duty_mag;

    // 两级都是寄存器输出，这里只做一次 2 选 1
    wire signed [15:0] duty_f = (SWING_EN != 0)
                              ? (run ? (lqr_on ? lqr_duty : sw_duty_r) : 16'sd0)
                              : (run ? lqr_duty : 16'sd0);
    wire               en_f   = run && (duty_f != 16'sd0);"""
rep(LQR_OLD, LQR_NEW, "lqr+swing")

# 遥测 ch0 带上 sw_state
TEL_OLD = """        .ch0      ({12'd0, mode}),"""
TEL_NEW = """        .ch0      ({9'd0, sw_state, mode}),"""
rep(TEL_OLD, TEL_NEW, "telemetry")

io.open(P, "w", encoding="utf-8", newline="\n").write(s)
print("written", len(s))
