//======================================================================
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
//======================================================================
`timescale 1ns/1ps

module bal_top (
    input  wire sys_clk,
    input  wire sys_rst_n,
    input  wire pwm_data,
    input  wire e1,
    input  wire e2,
    input  wire adc_dout,
    input  wire key_start,
    input  wire knob_btn,
    input  wire knob_a,
    input  wire knob_b,
    input  wire uart_rx,
    output wire uart_tx,
    output wire mot_in1,
    output wire mot_in2,
    output wire adc_cs_n,
    output wire adc_sclk,
    output wire led3,
    output wire oled_scl,
    inout  wire oled_sda,
    output wire [7:0] seg        // G1 板载共阳 1 位数码管（低有效）
);

    // 0 = θ 直接进 LQR（本版）；1 = 恢复 1.0 原版 θ 一阶低通（τ≈16 ms）
    localparam integer TH_LPF_EN = 0;   // 10.0l: 关掉 theta 低通（加滞后会导致立不起来）
    // 1 = 起摆使能；0 = 退化成版本 A 的纯平衡（平衡逻辑全保留，便于 A/B）
    localparam integer SWING_EN   = 1;
    // 起摆泵能方向：666.4 实测为 1；若越摆越低就改 0
    localparam integer SW_INVERT  = 1;

    wire rst_n;
    reg signed [15:0] theta_f;

    clk_reset u_rst (
        .clk      (sys_clk),
        .rst_n_in (sys_rst_n),
        .rst_n_out(rst_n)
    );

    //------------------------------------------------------------------
    // 10 ms / 1 kHz 时基
    //------------------------------------------------------------------
    reg [18:0] t10_cnt;
    reg        tick10;
    reg [15:0] cnt1k;
    reg        tick1k;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt1k  <= 16'd0;
            tick1k <= 1'b0;
        end
        else if (cnt1k >= 16'd49999) begin
            cnt1k  <= 16'd0;
            tick1k <= 1'b1;
        end
        else begin
            cnt1k  <= cnt1k + 16'd1;
            tick1k <= 1'b0;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            t10_cnt <= 19'd0;
            tick10  <= 1'b0;
        end
        else if (t10_cnt >= 19'd499_999) begin
            t10_cnt <= 19'd0;
            tick10  <= 1'b1;
        end
        else begin
            t10_cnt <= t10_cnt + 19'd1;
            tick10  <= 1'b0;
        end
    end

    //------------------------------------------------------------------
    // 编码器
    //------------------------------------------------------------------
    wire [31:0] enc_pos;
    wire        enc_dir;
    wire [15:0] enc_err;

    quad_decoder u_quad (
        .clk  (sys_clk),
        .rst_n(rst_n),
        .a    (e1),
        .b    (e2),
        .pos  (enc_pos),
        .dir  (enc_dir),
        .err  (enc_err)
    );

    //------------------------------------------------------------------
    // 按键 / 模式
    //   D11 短按 -> 切换 run（LQR 闭环）
    //   D11 长按 -> 编码器清零（pos_base 锁存）
    //   串口 '0'..'5' 设 mode（只上遥测，不影响控制）
    //------------------------------------------------------------------
    wire [3:0] mode;
    wire       clr;
    wire       trig;
    wire       run;

    wire [7:0] rx_byte;
    wire       rx_valid;
    wire [3:0] ser_mode;
    wire       ser_set;
    wire       ser_clr;

    uart_rx #(
        .CLK_FREQ(50_000_000),
        .BAUD    (115200)
    ) u_rx (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .rx        (uart_rx),
        .data_out  (rx_byte),
        .data_valid(rx_valid)
    );

    cmd_rx u_cmd (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .byte_in   (rx_byte),
        .byte_valid(rx_valid),
        .ser_mode  (ser_mode),
        .ser_set   (ser_set),
        .ser_clr   (ser_clr)
    );

    mode_ctrl u_mode (
        .clk     (sys_clk),
        .rst_n   (rst_n),
        .key_in  (key_start),
        .ser_mode(ser_mode),
        .ser_set (ser_set),
        .ser_clr (ser_clr),
        .mode    (mode),
        .clr     (clr),
        .run     (run),
        .trig    (trig)
    );

    // 长按清零：锁存基准，显示相对计数
    reg [31:0] pos_base;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            pos_base <= 32'd0;
        else if (clr)
            pos_base <= enc_pos;
    end

    wire signed [31:0] pos_r = $signed(enc_pos) - $signed(pos_base);

    //------------------------------------------------------------------
    // 电流采样，20 kHz（与 PWM 同周期）
    //------------------------------------------------------------------
    reg [15:0] adc_cnt;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            adc_cnt <= 16'd0;
        else if (adc_cnt >= 16'd2499)
            adc_cnt <= 16'd0;
        else
            adc_cnt <= adc_cnt + 16'd1;
    end

    wire        adc_busy;
    wire        adc_valid;
    wire [11:0] adc_data;
    wire        adc_null;

    mcp3201_spi u_adc (
        .clk     (sys_clk),
        .rst_n   (rst_n),
        .start   (adc_cnt == 16'd0 && !adc_busy),
        .dout    (adc_dout),
        .cs_n    (adc_cs_n),
        .sclk    (adc_sclk),
        .data    (adc_data),
        .null_bit(adc_null),
        .valid   (adc_valid),
        .busy    (adc_busy)
    );

    //------------------------------------------------------------------
    // SA100 角度（PWM 捕获，经滑环）
    //------------------------------------------------------------------
    wire pwm_sync;
    sync_ff #(.WIDTH(1)) u_sync_pwm (
        .clk(sys_clk), .rst_n(rst_n), .din(pwm_data), .dout(pwm_sync)
    );

    wire [31:0] high_ticks, period_ticks;
    wire        pwm_valid, sensor_lost;

    pwm_capture u_pwm (
        .clk(sys_clk), .rst_n(rst_n), .pwm_in(pwm_sync),
        .high_ticks(high_ticks), .period_ticks(period_ticks),
        .valid(pwm_valid), .lost(sensor_lost)
    );

    wire [11:0] position_raw;
    wire signed [15:0] angle_x10;
    wire        angle_valid, angle_busy;

    wire [7:0] rej_cnt;

    angle_calc u_ang (
        .clk(sys_clk), .rst_n(rst_n), .start(pwm_valid),
        .high_ticks(high_ticks), .period_ticks(period_ticks),
        .position_raw(position_raw), .angle_x10(angle_x10),
        .valid(angle_valid), .busy(angle_busy),
        .rej_cnt(rej_cnt)
    );

    //------------------------------------------------------------------
    // 电流保护：ADC > 1800 (≈1.47A) 连续 100 次（5ms）就断
    //------------------------------------------------------------------
    reg [7:0]  over_cnt;
    reg        fault;
    reg [3:0]  mode_d2;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            over_cnt <= 8'd0;
            fault    <= 1'b0;
            mode_d2  <= 3'd0;
        end
        else begin
            mode_d2 <= mode;

            if (mode != mode_d2)
                fault <= 1'b0;                   // 换模式清故障

            if (adc_valid) begin
                if (adc_data > 12'd1800) begin
                    if (over_cnt >= 8'd99) begin
                        over_cnt <= 8'd99;
                        fault    <= 1'b1;
                    end
                    else
                        over_cnt <= over_cnt + 8'd1;
                end
                else
                    over_cnt <= 8'd0;
            end
        end
    end

    //------------------------------------------------------------------
    // θ：SA100 原始码减去零点 1330，绕回到 ±1800（0.1°）
    //------------------------------------------------------------------
    reg [11:0]        th_d_r;
    reg signed [15:0] th_w_r;
    reg signed [31:0] th_prod_r;
    reg signed [15:0] theta10_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            th_d_r    <= 12'd0;
            th_w_r    <= 16'sd0;
            th_prod_r <= 32'sd0;
            theta10_r <= 16'sd0;
        end
        else begin
            th_d_r <= position_raw - 12'd1330;
            th_w_r <= (th_d_r >= 12'd2048)
                      ? ($signed({4'b0, th_d_r}) - 16'sd4096)
                      :  $signed({4'b0, th_d_r});
            th_prod_r <= th_w_r * 16'sd225;
            theta10_r <= th_prod_r >>> 8;
        end
    end
    wire signed [15:0] theta10 = theta10_r;

    //------------------------------------------------------------------
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
    wire signed [15:0] phi10 = phi10_r;

    // φ̇：speed 是 counts/s → 0.1°/s
    wire signed [15:0] spd;
    wire signed [31:0] phid32 = (spd * 32'sd4471) >>> 10;
    wire signed [15:0] phid10 = (phid32 > 32'sd30000) ? 16'sd30000 :
                                (phid32 < -32'sd30000) ? -16'sd30000 : phid32[15:0];

    //------------------------------------------------------------------
    // LQR 闭环
    //   版本 A：run = 1 就一直使能，没有起摆、没有捕获锁存
    //------------------------------------------------------------------
    wire signed [15:0] lqr_duty, lqr_thetad, lqr_uraw;
    wire signed [15:0] lqr_limit = 16'sd2000;   // 80% duty

    // ★ 3.0i：LQR 输入角度也过 3 点中值（单帧野值会让 4ms 差分的 θ̇ 算错 → 掉摆）
    //   theta10 由 position_raw 经 4 级流水得到，所以用 angle_valid 延迟 4 拍做采样节拍。
    reg  signed [15:0] qm1, qm2;
    reg  [3:0]         av_dly;
    wire signed [15:0] th_med = (theta10 > qm1)
        ? ((qm1 > qm2) ? qm1 : ((theta10 > qm2) ? qm2 : theta10))
        : ((qm1 < qm2) ? qm1 : ((theta10 > qm2) ? theta10 : qm2));
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            qm1    <= 16'sd0;
            qm2    <= 16'sd0;
            av_dly <= 4'd0;
        end
        else begin
            av_dly <= {av_dly[2:0], angle_valid};
            if (av_dly[3]) begin
                qm1 <= theta10;
                qm2 <= qm1;
            end
        end
    end

    //------------------------------------------------------------------
    // 10.0r：野值爆发保持已删除
    //   二分定位：4.0（无此逻辑）第一次起摆能接住；5.0 hold 版（有此逻辑）立不起来。
    //   机理：起摆时横臂高速转 → SA100 连丢帧 → 保持被触发 → 冻结状态跨过捕获
    //   时刻，摆杆路过 θ=0 时被锁在 0 附近 → LQR 以为已平衡 → duty≈0 → 倒。
    //------------------------------------------------------------------
    wire [2:0] sw_state;

    // 输入先打一拍，切断 position_raw -> 乘法链
    reg signed [15:0] theta_lqr_r;
    reg signed [15:0] phi_lqr_r;
    reg signed [15:0] phid_lqr_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            theta_lqr_r <= 16'sd0;
            phi_lqr_r   <= 16'sd0;
            phid_lqr_r  <= 16'sd0;
        end
        else begin
            theta_lqr_r <= th_med;          // 10.0r: 去掉 ang_hold，回到 4.0 行为
            phi_lqr_r   <= phi10;
            phid_lqr_r  <= phid10;
        end
    end

    // ① ② θ 低通可选：TH_LPF_EN=0 时直接把 theta_lqr_r 送进 LQR（不经 theta_f）
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            theta_f <= 16'sd0;
        else if (tick1k)
            theta_f <= theta_f + (($signed(theta_lqr_r) - $signed(theta_f)) >>> 3);   // 10.0k: 16ms -> 8ms，减小滞后
    end
    wire signed [15:0] theta_lqr_in = TH_LPF_EN ? theta_f : theta_lqr_r;

    //------------------------------------------------------------------
    // 自动起摆：swing_ctrl（能量泵 + kick，6.0 / 666.4 那一版）
    //   frame_tick = angle_valid（856 Hz 角度帧）
    //   lqr_on = run && sw_state==BAL：只有起摆状态机说“进平衡”才把电机交给 LQR
    //------------------------------------------------------------------
    wire               sw_active, sw_drive, sw_bal_entry;
    wire signed [15:0] sw_i_ref;

    // 掉摆：进平衡后 |θ|>25° 连续 2 帧 -> 起摆状态机重新起摆
    //   （lqr_core 的硬门限是 30°，25° 先动手，给重摆留余量）
    reg  [1:0] fall_cnt;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            fall_cnt <= 2'd0;
        else if (!(run && (sw_state == 3'd5) && !sensor_lost))
            fall_cnt <= 2'd0;
        else if (angle_valid && ((theta10 > 16'sd290) || (theta10 < -16'sd290)))
            fall_cnt <= (fall_cnt == 2'd3) ? 2'd3 : fall_cnt + 2'd1;
        else
            fall_cnt <= 2'd0;
    end
    wire force_fall = (fall_cnt >= 2'd2);

    // 泵能方向在线翻转：串口发 'i' 取反（不用重新烧片做 A/B）
    reg sw_inv_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            sw_inv_r <= (SW_INVERT != 0);
        else if (rx_valid && ((rx_byte == 8'h69) || (rx_byte == 8'h49)))
            sw_inv_r <= ~sw_inv_r;
    end
    wire signed [14:0] sw_e_dbg;

    // ★ 10.0k：上电后先攒够 800 个有效角度帧（≈0.93 s）再允许起摆
    //   SA100 上电头几百毫秒数据是旧的，这时按 D11 起摆必失败（实测）
    reg [15:0] ang_ok_cnt;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            ang_ok_cnt <= 16'd0;
        else if (angle_valid && !sensor_lost && (ang_ok_cnt != 16'hFFFF))
            ang_ok_cnt <= ang_ok_cnt + 16'd1;
    end
    wire ang_ready = 1'b1;   // 10.0o: 去掉上电门限（用户怀疑它导致第一次接不住）

    swing_ctrl #(
        .FRAME_HZ          (856),
        .L_MM              (76),
        .KICK_MS           (250),
        .I_KICK            (16'sd1300),
        .K_MA_PER_DPS      (16'sd6),
        .I_FLOOR_START     (16'sd700),
        .I_FLOOR_STEP      (16'sd100),
        .I_FLOOR_MAX       (16'sd1500),
        .I_MAX             (16'sd1500),
        .E_STOP_Q          (16'sd32767),
        .PUMP_PROP         (1),
        .E_TGT_Q           (-16'sd655),
        .KP_PUMP           (16'sd120),
        .I_PROP_MIN        (16'sd400),
        .E_STOP_DIS        (1),
        .E_RESUME_Q        (-16'sd6554),
        .W_CAP             (16'sd150),
        .ANG_CAP           (16'sd250),
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
        .frame_tick (angle_valid && !sensor_lost),
        .run_en     ((SWING_EN != 0) ? (run && ang_ready) : 1'b0),
        .angle      (theta10),
        .arm_speed  (phid10),
        .invert_pd  (sw_inv_r),
        .force_fall (force_fall),
        .sw_active  (sw_active),
        .sw_drive   (sw_drive),
        .i_swing    (sw_i_ref),
        .sw_state   (sw_state),
        .bal_entry  (sw_bal_entry),
        .e_dbg      (sw_e_dbg)
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
            sw_i_r      <= sensor_lost ? 16'sd0 : sw_i_ref;
            // 6.0 的 1200 mA ≈ 本工程 1800 counts，取 1.5 倍保守换算
            sw_duty_mul <= ($signed(sw_i_r) * 32'sd3) >>> 1;
            sw_duty_r   <= (sw_duty_mul >  32'sd2000) ?  16'sd2000 :
                           (sw_duty_mul < -32'sd2000) ? -16'sd2000 : sw_duty_mul[15:0];
        end
    end

    //------------------------------------------------------------------
    // LQR 闭环（平衡内核与版本 A 完全一致，只加 TH_TGT_LIM 钳位）
    //------------------------------------------------------------------
    // ★ 10.0m：sensor_lost 丢帧容忍（50 ms）
    //   实测：起摆捕获瞬间 SA100 偶发丢帧 → lqr_on 立刻断开、duty 清零
    //   → 摆杆 0.24 s 内从 +3° 滑到 -69° 掉摆（"到顶了却不接住"）。
    //   短暂丢帧(<50 ms)继续用上一个角度撑住，超过 50 ms 才真断。
    reg [7:0] lost_ms;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            lost_ms <= 8'd0;
        else if (tick1k) begin
            if (sensor_lost) begin
                if (lost_ms != 8'd255) lost_ms <= lost_ms + 8'd1;
            end
            else
                lost_ms <= 8'd0;
        end
    end
    wire lost_hard = sensor_lost && (lost_ms >= 8'd50);

    wire lqr_on = run && (sw_state == 3'd5) && !lost_hard;
    wire lqr_ready;

    // 5.0 跟随模式（线网先声明，逻辑在文件末尾）
    wire               follow_en;
    wire signed [15:0] phi_tgt_w;
    wire signed [15:0] dphi_now;
    wire        [1:0]  traj_sel_w;      // 轨迹页档位（0=跟随手转，1/2/3=正弦）
    wire signed [15:0] sin_y;           // 正弦输出（0.1°）
    wire               sin_active;      // 正弦幅度非零

    lqr_core #(
        .TH_TGT_LIM (32'sd105)  // φ 外环 θ 目标限：|dphi|<90° 不剪，只在绕回时封顶 10.5°
    ) u_lqr (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .tick1k   (tick1k),
        .enable   (lqr_on),
        .sa_lost  (lost_hard),          // 10.0m: 短丢帧不再清零输出
        .ready_o  (lqr_ready),
        .limit    (lqr_limit),
        .theta10  (theta_lqr_in),
        .phi10    (phi_lqr_r),
        .phid10   (phid_lqr_r),
        .phi_tgt  (phi_tgt_w),
        .dphi_out (dphi_now),
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
    wire               en_f   = run && (duty_f != 16'sd0);

    assign en_out   = en_f & ~fault;
    assign dir_out  = duty_f[15];
    assign duty_mag = duty_f[15] ? (~duty_f + 16'd1) : duty_f;

    motor_drv u_motor (
        .clk  (sys_clk),
        .rst_n(rst_n),
        .en   (en_out),
        .dir  (dir_out),
        .duty (duty_mag),
        .pwm  (mot_in1),
        .ph   (mot_in2)
    );

    //------------------------------------------------------------------
    // 速度
    //------------------------------------------------------------------
    speed_est u_spd (
        .clk  (sys_clk),
        .rst_n(rst_n),
        .pos  (enc_pos),
        .tick (tick10),
        .speed(spd)
    );

    //------------------------------------------------------------------
    // 遥测
    //------------------------------------------------------------------
    wire [7:0] tx_byte;
    wire       tx_ok;
    wire       uart_busy;
    wire       csv_busy;

    wire [11:0] sa_out = position_raw;

    vofa6 u_vofa (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .start    (tick10),
        .ch0      ({8'd0, sensor_lost, sw_state, mode}),
        .ch1      (duty_f),
        .ch2      (pos_r[15:0]),
        .ch3      (theta10),       // 诊断 θ（0.1°）
        .ch4      ({4'd0, adc_data}),     // 电流采样（MCP3201）
        .ch5      ({sw_inv_r, sw_e_dbg}),          // 起摆能量诊断
        .uart_busy(uart_busy),
        .tx_data  (tx_byte),
        .tx_valid (tx_ok),
        .busy     (csv_busy)
    );

    uart_tx #(
        .CLK_FREQ(50_000_000),
        .BAUD    (115200)
    ) u_uart (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .data_in   (tx_byte),
        .data_valid(tx_ok),
        .busy      (uart_busy),
        .tx        (uart_tx)
    );

    //------------------------------------------------------------------
    // LED：fault 快闪 / run 常亮 / 停机灭
    //------------------------------------------------------------------
    reg [22:0] blink_cnt;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            blink_cnt <= 23'd0;
        else
            blink_cnt <= blink_cnt + 23'd1;
    end

    assign led3 = fault          ? blink_cnt[21] :
                  run ? 1'b0 :
                         ~en_out;

    //------------------------------------------------------------------
    // OLED 实时仪表盘：只显示，不参与控制
    //------------------------------------------------------------------
    wire knob_step, knob_dir, knob_btn_short, knob_btn_long;
    knob_encoder #(.CLK_FREQ(50_000_000), .LONG_MS(800)) u_knob (
        .clk(sys_clk), .rst_n(rst_n), .a(knob_a), .b(knob_b), .btn(knob_btn),
        .step(knob_step), .dir(knob_dir), .btn_short(knob_btn_short), .btn_long(knob_btn_long)
    );

    wire menu_oled_scl, menu_oled_sda, ui_oled_scl, ui_oled_sda;
    wire speed_oled_scl, speed_oled_sda, param_mode, traj_mode_ui, speed_req, speed_active;
    oled_menu u_menu (
        .clk(sys_clk), .rst_n(rst_n),
        .knob_step(speed_active ? 1'b0 : (follow_en ? 1'b0 : knob_step)), .knob_dir(speed_active ? 1'b0 : knob_dir),
        .knob_btn_short(speed_active ? 1'b0 : knob_btn_short), .knob_btn_long(speed_active ? 1'b0 : knob_btn_long),
        .param_mode(param_mode), .traj_mode(traj_mode_ui), .traj_sel(traj_sel_w), .speed_req(speed_req),
        .oled_scl(menu_oled_scl), .oled_sda(menu_oled_sda)
    );

    wire [1:0] speed_sel_w;          // ★ 速度页选择（0=最慢 … 3=最快）
    oled_speed u_speed (
        .clk(sys_clk), .rst_n(rst_n), .enter(speed_req),
        .knob_step(knob_step), .knob_dir(knob_dir),
        .knob_btn_short(knob_btn_short), .knob_btn_long(knob_btn_long),
        .active(speed_active), .speed_sel(speed_sel_w),
        .oled_scl(speed_oled_scl), .oled_sda(speed_oled_sda)
    );

    oled_ui #(
        .CLK_FREQ (50_000_000),
        .ADC_ZERO (0),
        .SCL_HZ   (200_000),
        .X_OFF    (8'd0)
    ) u_oled (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .mode        (run ? 2'd3 : 2'd0),
        .theta_x10   (theta10),
        .phi_cnt     (pos_r[31:0]),
        .enc_err     (16'd0),
        .sa_lost     (sensor_lost),
        .angle_valid (angle_valid),
        .duty        (duty_f),
        .adc_raw     (adc_data),
        .oled_scl    (ui_oled_scl),
        .oled_sda    (ui_oled_sda)
    );

    assign oled_scl = speed_active ? speed_oled_scl : (param_mode ? ui_oled_scl : menu_oled_scl);
    assign oled_sda = speed_active ? speed_oled_sda : (param_mode ? ui_oled_sda : menu_oled_sda);

    //------------------------------------------------------------------
    // 5.0 跟随（v3）：从"当前位置"起步，退出后原地保持
    //   · 进入跟随的瞬间：目标 = 当前横臂角（不动），之后每格 18° 从这里累加
    //   · 退出跟随：目标停在最后的值 → 横臂原地静止，不回原位
    //   · 起摆期间（lqr_on=0）：目标清零，捕获后从捕获点开始
    //   · 方向：实测反了，knob_dir 取反
    //   · 限速 100°/s + |θ|>8° 冻结目标（保护摆杆）
    //------------------------------------------------------------------
    localparam signed [15:0] FOLLOW_STEP = 16'sd180;   // 每格 18°
    localparam signed [15:0] FOLLOW_TH_G = 16'sd80;    // |θ|>8° 时冻结目标

    // ★ 8.0：跟随限速由"速度页"选择（speed_sel）
    //   0 = 50°/s   1 = 100°/s   2 = 150°/s   3 = 200°/s
    reg signed [15:0] follow_slew;
    always @(*) begin
        case (speed_sel_w)
            2'd0:    follow_slew = 16'sd1;    // 10°/s  (一格 18° 走 1.8 s)
            2'd1:    follow_slew = 16'sd2;    // 20°/s  (0.9 s)
            2'd2:    follow_slew = 16'sd5;    // 50°/s  (0.36 s)
            default: follow_slew = 16'sd10;   // 100°/s (0.18 s)
        endcase
    end

    assign follow_en = traj_mode_ui;                   // 轨迹页 + 已选中

    function signed [16:0] wrap1800;
        input signed [16:0] v;
        begin
            wrap1800 = (v >  17'sd1800) ? (v - 17'sd3600) :
                       (v < -17'sd1800) ? (v + 17'sd3600) : v;
        end
    endfunction

    reg  signed [15:0] phi_cmd;   // 旋钮累积目标（相对 φ0）
    reg  signed [15:0] phi_ref;   // 限速后的实际目标（送 LQR）
    reg                follow_d;  // 上一拍 follow_en，用于取上升沿

    wire signed [16:0] cmd_nx  = $signed(phi_cmd) +
                                 (knob_dir ? -$signed(FOLLOW_STEP) : $signed(FOLLOW_STEP));
    wire signed [16:0] err_cmd = $signed(phi_cmd) - $signed(phi_ref);
    wire signed [16:0] err_w   = wrap1800(err_cmd);
    // ★ 9.0-final：跟随限速 = 速度页标签值 30/60/90/120 °/s
    //   1 count = 0.1°，以 3 ms 为基准：每 3 ms 推进 N count
    //     30°/s -> 1 count/3ms (33°/s)   60°/s -> 2 (67°/s)
    //     90°/s -> 3 (100°/s)           120°/s -> 4 (133°/s)
    //   注：低于 ~30°/s 时电机静摩擦（死区 113/167）会让横臂几乎不动，故下限取 30
    reg [1:0] ms3;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)      ms3 <= 2'd0;
        else if (tick1k) ms3 <= (ms3 == 2'd2) ? 2'd0 : ms3 + 2'd1;
    end
    wire slew_go = (ms3 == 2'd2);          // 每 3 ms 一次

    reg [3:0] step_n;
    always @(*) begin
        case (speed_sel_w)
            2'd0:    step_n = 4'd1;        // ~33 °/s
            2'd1:    step_n = 4'd2;        // ~67 °/s
            2'd2:    step_n = 4'd3;        // ~100 °/s
            default: step_n = 4'd4;        // ~133 °/s
        endcase
    end

    wire [16:0] err_abs  = err_w[16] ? ((~err_w) + 17'sd1) : {1'b0, err_w[15:0]};
    // 软着陆：离目标 <30（3°）时按 err/2 逼近，避免"到点急刹"激起过冲
    wire [16:0] slew_abs = (err_abs < 17'sd30) ? (err_abs >>> 1) : {13'd0, step_n};
    wire [16:0] step_abs = (slew_abs > err_abs) ? err_abs : slew_abs;
    wire signed [16:0] step_w = err_w[16] ? -$signed({1'b0, step_abs}) : $signed({1'b0, step_abs});
    wire signed [16:0] ref_nx  = wrap1800($signed(phi_ref) + step_w);
    wire               follow_ok = (theta10 <=  FOLLOW_TH_G) && (theta10 >= -FOLLOW_TH_G);

    // ★ 11.0：正弦基准——进入正弦时记下"当前横臂位置"，正弦叠加在该基准上
    //   原来 phi_tgt = sin_y（绝对零），切进正弦时目标会跳到 0 → 横臂先跑回某处再摆
    reg  signed [15:0] sin_base;
    reg                sin_mode_d;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sin_base  <= 16'sd0;
            sin_mode_d <= 1'b0;
        end
        else begin
            sin_mode_d <= sin_mode;
            // 非正弦期间一直贴着实际位置；进入正弦瞬间自然就是"切换那一刻的位置"
            if (!sin_mode)
                sin_base <= dphi_now;
        end
    end
    wire signed [16:0] sin_sum = wrap1800($signed(sin_base) + $signed(sin_y));

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            phi_cmd  <= 16'sd0;
            phi_ref  <= 16'sd0;
            follow_d <= 1'b0;
        end
        else begin
            follow_d <= follow_en;
            if (!lqr_on) begin
                // 起摆 / 掉摆：目标清零，捕获后从捕获点开始
                phi_cmd <= 16'sd0;
                phi_ref <= 16'sd0;
            end
            else if (follow_en && !follow_d) begin
                // 刚选中跟随：从当前位置起步（不移动）
                phi_cmd <= dphi_now;
                phi_ref <= dphi_now;
            end
            else if (sin_mode) begin
                // 正弦模式期间跟随目标贴着实际位置，退出正弦后无缝交还
                phi_cmd <= dphi_now;
                phi_ref <= dphi_now;
            end
            else if (follow_en) begin
                if (knob_step)
                    phi_cmd <= wrap1800(cmd_nx)[15:0];
                if (tick1k && follow_ok && slew_go)
                    phi_ref <= ref_nx[15:0];
            end
            // 退出跟随：phi_cmd / phi_ref 保持不变 → 横臂停在原地
        end
    end

    //------------------------------------------------------------------
    // 轨迹模式：横臂正弦摆动（档 1/2/3），档 0 = 跟随手转
    //   相位步进 step = 65536*f/1000；幅度单位 0.1°
    //   y = A*sin(2*pi*phase/65536)，幅度带斜坡，起振/退出都不跳
    //------------------------------------------------------------------
    reg  [15:0]        sin_step;
    reg  signed [15:0] sin_amp;

    // ★ 8.0：正弦幅度由档位决定（幅值不变）
    always @(*) begin
        case (traj_sel_w)
            2'd1:    sin_amp = 16'sd200;   // 20°
            2'd2:    sin_amp = 16'sd350;   // 35°
            2'd3:    sin_amp = 16'sd500;   // 50°
            default: sin_amp = 16'sd0;
        endcase
    end

    // ★ 8.0：正弦频率由"速度页"决定（4 档）
    always @(*) begin
        case (speed_sel_w)
            2'd0:    sin_step = 16'd10;    // 0.15 Hz
            2'd1:    sin_step = 16'd16;    // 0.25 Hz
            2'd2:    sin_step = 16'd23;    // 0.35 Hz
            default: sin_step = 16'd33;    // 0.50 Hz
        endcase
    end

    // ★ 正弦只在闭环使能时运行：起摆 / 掉摆期间幅度自动降到 0，
    //   否则捕获瞬间 phi_tgt = sin_y（最大 ±50°）会把摆杆直接甩下来。
    wire sin_mode = follow_en && (traj_sel_w != 2'd0) && lqr_on;

    sin_gen u_sin (
        .clk(sys_clk), .rst_n(rst_n), .tick1k(tick1k),
        .en(sin_mode), .step(sin_step), .amp(sin_amp),
        .y(sin_y), .active(sin_active)
    );

    // 正弦幅度归零那一刻之前都用正弦输出，之后交还跟随目标（不会跳）
    // ★ 10.0j：打一拍再送 LQR（原来正弦模块组合直连，是时序违例路径）
    wire signed [15:0] phi_tgt_mux = sin_active ? sin_sum[15:0] : phi_ref;   // 11.0: 正弦叠加在切换点基准上
    reg  signed [15:0] phi_tgt_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) phi_tgt_r <= 16'sd0;
        else        phi_tgt_r <= phi_tgt_mux;
    end
    assign phi_tgt_w = phi_tgt_r;

    //------------------------------------------------------------------
    // 数码管：共阳，低有效（移植自 LQR_Furuta_666.4 / formal_top.v）
    //   0 = 停机（!run）    1 = 起摆中     2 = 平衡中     8 = 故障（ADC 过流）
    //   段码：0xC0=0  0xF9=1  0xA4=2  0xB0=3  0x00=8+小数点（全亮）
    //------------------------------------------------------------------
    reg [3:0] seg_code;
    always @(*) begin
        if (fault)
            seg_code = 4'd8;
        else if (!run)
            seg_code = 4'd0;
        else if (traj_mode_ui && (sw_state == 3'd5))
            seg_code = 4'd3;   // 轨迹页面 + 平衡中（与 666.4 一致）
        else if (sw_state == 3'd5)
            seg_code = 4'd2;
        else
            seg_code = 4'd1;
    end

    reg [7:0] seg_r;
    always @(*) begin
        case (seg_code)
            4'd0: seg_r = 8'hC0;   // 0
            4'd1: seg_r = 8'hF9;   // 1
            4'd2: seg_r = 8'hA4;   // 2
            4'd3: seg_r = 8'hB0;   // 3
            4'd8: seg_r = 8'h00;   // 8 + 小数点
            default: seg_r = 8'hFF; // 灭
        endcase
    end
    assign seg = seg_r;

endmodule
