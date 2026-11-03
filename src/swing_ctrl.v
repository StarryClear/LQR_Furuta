//======================================================================
//  swing_ctrl.v  --  自动起摆控制器 v3（2026-10-23，给 5.0 版）
//
//  方案：连续能量泵（不再用"折返点+固定推 100ms"那套）
//      E  = 0.5·(L/g)·θ̇² + cosθ − 1        （竖直静止 E=0；下垂 E=−2）
//      u  = −sign(G)·max(K·|θ̇|, I_FLOOR)·sign(θ̇·cosθ)
//      E > E_STOP  -> 滑行；|θ|<45° 且 |θ̇|<W_CAP -> 交回 PD
//
//  ★ 和 v1.2（1.2 包里那一版）的差别 —— 针对"有反应但力度不够、反复摆上不去"：
//    1) 泵能电流增益 K 从 3 提到 6 mA/(°/s)，并把"最小泵能电流"从 550 提到
//       700mA 起；每 2.5s 没进捕获 / 每次卡住 re-kick 再 +100mA，直到 1500mA。
//       原来 |θ̇|×3、下限 550，摆起来以后电流只有 200~500mA，跟摩擦打平就卡住。
//    2) 能量系数按【真实摆长 L_MM】算（默认 200mm）。原来固定按 L=200mm，
//       如果摆杆实际更短，能量会被高估 -> 提前停泵 -> 反复摆上不去。
//       换 L 只改 Furuta.v 例化处的 L_MM（枢轴到摆锤中心，单位 mm）。
//    3) 起摆内部用【帧差分的快速角速度】τ≈2.3ms（原来的 speed_filt τ≈19ms），
//       换向点不再滞后 19ms 才反方向，注能效率更高（不改你原来的 speed_filt）。
//    4) kick 900/200ms -> 1300mA/250ms；卡住 300ms 自动换方向再 kick。
//    5) 所有计时严格按 frame_tick（856Hz）计数 —— v1.2 的 50MHz 计数 bug 不会再有。
//    6) v5.2：掉摆后等静止再起摆；再起摆的力度/流程与首次起摆完全一致
//       v5.3：静止判据加入"横臂也停"，并把内部速度估计清零，起摆初态与首次一致
//       （升档只在泵能态进行，kick 前 i_level 复位到 I_FLOOR_START）。
//
//  ★ 不新增顶层端口、不改引脚约束 ★
//======================================================================
`timescale 1ns/1ps

module swing_ctrl #(
    parameter integer        FRAME_HZ      = 856,        // 角度帧率（pwm_sample）
    parameter integer        L_MM          = 200,        // 摆长 mm（枢轴->摆锤中心）
    parameter integer        KICK_MS       = 250,        // kick 每段时长
    parameter signed [15:0]  I_KICK        = 16'sd1300,  // kick 电流 mA
    parameter signed [15:0]  K_MA_PER_DPS  = 16'sd6,     // 泵能增益 mA/(°/s)
    parameter signed [15:0]  I_FLOOR_START = 16'sd700,   // 最小泵能电流（起始）
    parameter signed [15:0]  I_FLOOR_STEP  = 16'sd100,   // 每次升级 +mA
    parameter signed [15:0]  I_FLOOR_MAX   = 16'sd1500,  // 最小泵能电流上限
    parameter signed [15:0]  I_MAX         = 16'sd1500,  // 泵能电流上限
    parameter signed [15:0]  E_STOP_Q      = -16'sd5898, // Q15 −0.18：停止泵能
    parameter signed [15:0]  E_RESUME_Q    = -16'sd10486,// Q15 −0.32：重新泵能
    parameter signed [15:0]  W_CAP         = 16'sd180,   // 捕获允许角速度 °/s
    parameter signed [15:0]  ANG_CAP       = 16'sd450,   // ±45.0° 捕获区
    parameter signed [15:0]  ANG_FALL      = 16'sd600,   // ±60.0° 掉摆
    parameter signed [15:0]  REST_ANG      = 16'sd1600,  // 静止判据：|θ|>160°（离下垂 ±20°）
    parameter signed [15:0]  REST_SPD      = 16'sd50,    // 静止判据：|θ̇|<50°/s
    parameter signed [15:0]  ARM_REST_SPD  = 16'sd150,   // 横臂也要停：|臂速|<150°/s
    parameter integer        REST_MS       = 400,        // 静止要持续多久才起摆
    parameter integer        SETTLE_TIMEOUT_MS = 6000,   // 等静止超时 -> 也走 kick（兜底）
    parameter integer        WAIT_MS       = 150,        // 滑行态最短驻留（防 3/4 快速抖动）
    parameter integer        STALL_MS      = 300,        // 卡住多久换方向 re-kick
    parameter integer        ESCALATE_MS   = 2500,       // 泵能多久没捕获就加电流
    parameter integer        TIMEOUT_MS    = 30000,      // 总超时
    // ★ LQR_2.1e：比例泵能。恒定大力矩会在顶上继续推，摆高速穿顶抓不住；
    //   改成 力矩 ∝ (E_TGT - E)，越接近目标越小，顶上自然收力。
    //   KP_PUMP: 每个 Q15 能量缺口对应的 mA/1024；23 => 缺口 2.0(挂底) 时 1472 mA
    parameter integer        PUMP_PROP    = 0,          // 1 = 比例泵能
    parameter signed [15:0]  E_TGT_Q      = -16'sd655,  // 目标能量 Q15（-0.02 ≈ 竖直 11.5°）
    parameter signed [15:0]  KP_PUMP      = 16'sd23,    // 比例增益
    // 比例项的最小出力：低于死区就等于没出力，摆会卡在半路（实测卡在 34~46°）
    parameter signed [15:0]  I_PROP_MIN   = 16'sd400,
    parameter integer        E_STOP_DIS   = 0,          // 1 = 取消 E_STOP 停泵判据
    parameter integer        SW_SIGN       = 1,          // 方向总开关 +1/-1
    parameter integer        AUTO_RESWING  = 1
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_tick,  // 856Hz 角度帧单拍
    input  wire               run_en,      // KEY0 运行态
    input  wire signed [15:0] angle,       // 摆杆角度 0.1°，竖直=0
    input  wire signed [15:0] arm_speed,   // 横臂角速度 1°/s（静止判据用）
    input  wire               invert_pd,   // V1=1，V0=0
    input  wire               force_fall,  // 1 = 强制退出平衡，重新起摆
    output wire               sw_active,   // 1 = 起摆中（PD/位置环/90°门限旁路）
    output wire               sw_drive,    // 1 = 正在出电流（kick/泵能）
    output wire signed [15:0] i_swing,     // 起摆电流指令 mA
    output wire [2:0]         sw_state,    // 0停 1K+ 2K- 3泵 4滑行 5平衡
    output reg                bal_entry,   // 进入平衡单拍
    output wire signed [14:0] e_dbg        // 能量估计 e_f[16:2]（Q15/4），只做遥测
);

    //------------------------------------------------------------------
    // 1. cos LUT（64 项，Q15，索引 = |θ|/32）
    //------------------------------------------------------------------
    function signed [15:0] cos_q15;
        input [5:0] idx;
        begin
            case (idx)
                6'd 0: cos_q15 = 16'sd32767;
                6'd 1: cos_q15 = 16'sd32717;
                6'd 2: cos_q15 = 16'sd32564;
                6'd 3: cos_q15 = 16'sd32309;
                6'd 4: cos_q15 = 16'sd31954;
                6'd 5: cos_q15 = 16'sd31499;
                6'd 6: cos_q15 = 16'sd30945;
                6'd 7: cos_q15 = 16'sd30296;
                6'd 8: cos_q15 = 16'sd29551;
                6'd 9: cos_q15 = 16'sd28715;
                6'd10: cos_q15 = 16'sd27789;
                6'd11: cos_q15 = 16'sd26776;
                6'd12: cos_q15 = 16'sd25680;
                6'd13: cos_q15 = 16'sd24504;
                6'd14: cos_q15 = 16'sd23251;
                6'd15: cos_q15 = 16'sd21926;
                6'd16: cos_q15 = 16'sd20533;
                6'd17: cos_q15 = 16'sd19075;
                6'd18: cos_q15 = 16'sd17558;
                6'd19: cos_q15 = 16'sd15986;
                6'd20: cos_q15 = 16'sd14365;
                6'd21: cos_q15 = 16'sd12698;
                6'd22: cos_q15 = 16'sd10992;
                6'd23: cos_q15 = 16'sd9252;
                6'd24: cos_q15 = 16'sd7483;
                6'd25: cos_q15 = 16'sd5690;
                6'd26: cos_q15 = 16'sd3880;
                6'd27: cos_q15 = 16'sd2058;
                6'd28: cos_q15 = 16'sd229;
                6'd29: cos_q15 = -16'sd1601;
                6'd30: cos_q15 = -16'sd3425;
                6'd31: cos_q15 = -16'sd5239;
                6'd32: cos_q15 = -16'sd7036;
                6'd33: cos_q15 = -16'sd8812;
                6'd34: cos_q15 = -16'sd10560;
                6'd35: cos_q15 = -16'sd12275;
                6'd36: cos_q15 = -16'sd13952;
                6'd37: cos_q15 = -16'sd15585;
                6'd38: cos_q15 = -16'sd17170;
                6'd39: cos_q15 = -16'sd18701;
                6'd40: cos_q15 = -16'sd20174;
                6'd41: cos_q15 = -16'sd21584;
                6'd42: cos_q15 = -16'sd22927;
                6'd43: cos_q15 = -16'sd24198;
                6'd44: cos_q15 = -16'sd25393;
                6'd45: cos_q15 = -16'sd26510;
                6'd46: cos_q15 = -16'sd27544;
                6'd47: cos_q15 = -16'sd28492;
                6'd48: cos_q15 = -16'sd29351;
                6'd49: cos_q15 = -16'sd30118;
                6'd50: cos_q15 = -16'sd30792;
                6'd51: cos_q15 = -16'sd31369;
                6'd52: cos_q15 = -16'sd31849;
                6'd53: cos_q15 = -16'sd32230;
                6'd54: cos_q15 = -16'sd32510;
                6'd55: cos_q15 = -16'sd32688;
                6'd56: cos_q15 = -16'sd32765;
                6'd57: cos_q15 = -16'sd32768;
                6'd58: cos_q15 = -16'sd32768;
                6'd59: cos_q15 = -16'sd32768;
                6'd60: cos_q15 = -16'sd32768;
                6'd61: cos_q15 = -16'sd32768;
                6'd62: cos_q15 = -16'sd32768;
                6'd63: cos_q15 = -16'sd32768;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // 2. 状态 / 常量
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE=3'd0, S_KICK1=3'd1, S_KICK2=3'd2,
                     S_PUMP=3'd3, S_WAIT=3'd4, S_BAL=3'd5, S_FALL=3'd6;

    localparam integer KICK_FRAMES     = (FRAME_HZ * KICK_MS)     / 1000;
    localparam integer REST_FRAMES          = (FRAME_HZ * REST_MS)           / 1000;
    localparam integer SETTLE_TIMEOUT_FRAMES= (FRAME_HZ * SETTLE_TIMEOUT_MS) / 1000;
    localparam integer WAIT_FRAMES     = (FRAME_HZ * WAIT_MS)     / 1000;
    localparam integer STALL_FRAMES    = (FRAME_HZ * STALL_MS)    / 1000;
    localparam integer ESCALATE_FRAMES = (FRAME_HZ * ESCALATE_MS) / 1000;
    localparam integer TIMEOUT_FRAMES  = (FRAME_HZ * TIMEOUT_MS)  / 1000;

    // E 动能项系数（Q8）：0.5085·L_m × 256 ≈ 0.1302·L_mm
    localparam signed [15:0] E_K_Q8 = (L_MM * 133 + 512) >>> 10;

    //------------------------------------------------------------------
    // 3. 快速角速度（帧差分 + 一阶低通 τ≈2.3ms，只用在本模块内）
    //------------------------------------------------------------------
    // 两级流水（2026-10-23）：原来 d_ang×86 + 低通 + 寄存 一条组合链，
    // 实测把 Fmax 压到 50.07MHz（slack 0.03ns）。拆成两拍后余量恢复。
    reg  signed [15:0] ang_r;     // 第 0 级：角度输入打拍（切断 angle_x10 的大扇出）
    reg  signed [15:0] ang_prev;
    reg  signed [15:0] d_r;       // 第 1 级：帧差分
    reg  signed [23:0] raw_r;     // 第 2 级：×86 后的原始角速度
    reg  signed [23:0] spd_f;
    reg  signed [15:0] spd_q;
    reg                spd_clear;   // 再起摆前清一次速度估计（和上电首次一致）

    // ★ 3 点中值：单帧野值被顶掉，且不会像"丢帧"那样连锁（丢帧后下一帧差分变跨帧）
    reg  signed [15:0] am1, am2;
    wire signed [15:0] a_med = (angle > am1)
        ? ((am1 > am2) ? am1 : ((angle > am2) ? am2 : angle))
        : ((am1 < am2) ? am1 : ((angle > am2) ? angle : am2));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            am1 <= 16'sd0;
            am2 <= 16'sd0;
        end
        else if (frame_tick) begin
            am1 <= angle;
            am2 <= am1;
        end
    end

    wire signed [15:0] ang_now = ang_r;
    wire signed [15:0] d_ang   = ang_now - ang_prev;
    wire signed [15:0] d_ang_w = (d_ang >  16'sd1800) ? (d_ang - 16'sd3600) :
                                 (d_ang < -16'sd1800) ? (d_ang + 16'sd3600) : d_ang;
    wire signed [23:0] raw_c   = d_r * 16'sd86;        // 0.1°/帧 -> °/s
    wire signed [23:0] spd_nxt = spd_f + ((raw_r - spd_f) >>> 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ang_r    <= 16'sd0;
            ang_prev <= 16'sd0;
            d_r      <= 16'sd0;
            raw_r    <= 24'sd0;
            spd_f    <= 24'sd0;
            spd_q    <= 16'sd0;
        end
        else if (frame_tick) begin
            ang_r    <= a_med;
            ang_prev <= ang_now;
            if (spd_clear) begin
                d_r   <= 16'sd0;
                raw_r <= 24'sd0;
                spd_f <= 24'sd0;
                spd_q <= 16'sd0;
            end
            else begin
                d_r   <= d_ang_w;
                raw_r <= raw_c;
                spd_f <= spd_nxt;
                spd_q <= (spd_nxt >  24'sd3000) ?  16'sd3000 :
                         (spd_nxt < -24'sd3000) ? -16'sd3000 : spd_nxt[15:0];
            end
        end
    end

    //------------------------------------------------------------------
    // 4. 角度绝对值 / 能量估计
    //------------------------------------------------------------------
    wire        [15:0] ang_u   = ang_now[15] ? (~ang_now + 16'd1) : ang_now;
    wire        [5:0]  cos_idx = (ang_u > 16'd1792) ? 6'd56 : ang_u[10:5];
    wire signed [15:0] cos_q   = cos_q15(cos_idx);

    // 2026-10-23 v5.3：能量/余弦打拍（纯时序）——原来 cos LUT + 乘法 + 比较
    // 串在 angle_last 后面把 Fmax 压到 50.4MHz；打两拍后余量恢复。
    wire signed [31:0] spd_sq  = spd_q * spd_q;

    reg  signed [15:0] cos_q_r;
    reg  signed [31:0] e_kin_r;
    reg  signed [31:0] e_q_r;
    // ★ LQR_2.1c：能量一阶低通（tau≈9 帧≈10 ms）。
    //   e_q 的动能项来自 1 帧差分速度，噪声 ±86°/s，在 300°/s 时会让 E 抖 ±0.06，
    //   而 E_STOP 本身就是 0.06 量级 —— 不滤就会在阈值附近抖成“泵一下停一下”。
    reg  signed [31:0] e_f;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cos_q_r <= 16'sd0;
            e_kin_r <= 32'sd0;
            e_q_r   <= 32'sd0;
            e_f     <= -32'sd65536;
        end
        else begin
            cos_q_r <= cos_q;
            e_kin_r <= (spd_sq * E_K_Q8) >>> 8;
            e_q_r   <= e_kin_r + cos_q_r - 32'sd32768;
            if (spd_clear)
                e_f <= -32'sd65536;                      // 重新起摆：当作“没能量”
            else if (frame_tick)
                e_f <= e_f + ((e_q_r - e_f) >>> 3);
        end
    end

    wire signed [31:0] e_q = e_q_r;

    //------------------------------------------------------------------
    // 5. 泵能电流（恒流 + 自动升级）
    //------------------------------------------------------------------
    wire        w_neg   = spd_q[15];
    wire        cos_neg = (ang_u > 16'd900);
    wire        g_neg   = invert_pd ^ (SW_SIGN < 0);
    wire        u_neg   = ~(g_neg ^ (w_neg ^ cos_neg));

    reg  signed [15:0] i_level;      // 当前最小泵能电流（卡住/超时自动升）
    reg  signed [15:0] i_pump_r;
    wire signed [15:0] spd_abs  = w_neg ? -spd_q : spd_q;
    wire signed [31:0] mag_raw  = spd_abs * K_MA_PER_DPS;
    wire signed [15:0] mag_lim  = (mag_raw > I_MAX) ? I_MAX : mag_raw[15:0];
    wire signed [15:0] mag_flr  = (spd_abs > 16'sd20)
                                  ? ((mag_lim < i_level) ? i_level : mag_lim)
                                  : mag_lim;
    //  ★ 分区间给电流（治"竖直附近机械抖"）：
    //    水平以下(|θ|>90°)：max(K·|θ̇|, i_level)，保证能把臂推动；
    //    水平以上(|θ|<90°)：只用 K·|θ̇|（不用 floor）。
    //    原因：竖直附近 θ̇ 本来就小，方向判断会被噪声/机械回弹反复翻转；
    //    如果这时还按 700~1500mA 的 floor 硬切，摆臂就会高频抖动（VOFA 上看到的锯齿）。
    wire               pump_below = (ang_u > 16'd900);
    // 比例泵能：e_def = E_TGT - e_f，正数才出力
    wire signed [31:0] e_def    = $signed(E_TGT_Q) - $signed(e_f);
    wire signed [31:0] p_mul    = (e_def > 32'sd0) ? ((e_def * KP_PUMP) >>> 10) : 32'sd0;
    wire signed [15:0] mag_prop = (e_def <= 32'sd0)        ? 16'sd0 :
                                 (p_mul > $signed(I_MAX)) ? I_MAX :
                                 (p_mul < $signed(I_PROP_MIN)) ? I_PROP_MIN : p_mul[15:0];
    wire signed [15:0] mag_use  = (PUMP_PROP != 0) ? mag_prop
                                                  : (pump_below ? mag_flr : mag_lim);
    wire signed [15:0] i_pump_c   = u_neg ? -mag_use : mag_use;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) i_pump_r <= 16'sd0;
        else        i_pump_r <= i_pump_c;
    end

    //------------------------------------------------------------------
    // 6. 状态机
    //------------------------------------------------------------------
    reg  [2:0]  state;
    reg  [31:0] ms_cnt;
    reg  [31:0] sw_ms;
    reg  [31:0] stall_cnt;
    reg  [31:0] esc_cnt;
    reg  [31:0] rest_cnt;
    reg         kick_dir;
    reg         run_d;
    reg  [1:0]  fall_cnt;      // v5.6：连续超阈值帧数（掉摆判据去抖）

    assign sw_state  = state;
    assign e_dbg     = e_f[16:2];
    assign sw_active = run_en && ((state==S_KICK1)||(state==S_KICK2)||
                                  (state==S_PUMP)||(state==S_WAIT)||(state==S_FALL));
    assign sw_drive  = run_en && ((state==S_KICK1)||(state==S_KICK2)||(state==S_PUMP));

    wire signed [15:0] kick_pos = kick_dir ? -I_KICK :  I_KICK;
    wire signed [15:0] kick_neg = kick_dir ?  I_KICK : -I_KICK;

    assign i_swing = (!run_en)          ? 16'sd0 :
                     (state==S_KICK1)   ? kick_pos :
                     (state==S_KICK2)   ? kick_neg :
                     (state==S_PUMP)    ? i_pump_r : 16'sd0;

    // 捕获判据（用快速速度）
    wire cap_raw = (ang_now < ANG_CAP) && (ang_now > -ANG_CAP) &&
                   (spd_q   < W_CAP)   && (spd_q   > -W_CAP);

    // 连续 2 帧满足才允许捕获，避免单帧高速穿越误切
    reg [1:0] cap_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cap_cnt <= 2'd0;
        else if (frame_tick)
            cap_cnt <= cap_raw ? ((cap_cnt == 2'd2) ? 2'd2 : cap_cnt + 2'd1)
                               : 2'd0;
    end
    wire cap_ok = cap_raw && (cap_cnt >= 2'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            ms_cnt    <= 32'd0;
            sw_ms     <= 32'd0;
            stall_cnt <= 32'd0;
            esc_cnt   <= 32'd0;
            rest_cnt  <= 32'd0;
            kick_dir  <= 1'b0;
            run_d     <= 1'b0;
            fall_cnt  <= 2'd0;
            i_level   <= I_FLOOR_START;
            bal_entry <= 1'b0;
        end
        else begin
            run_d     <= run_en;
            bal_entry <= 1'b0;
            spd_clear <= 1'b0;

            //  掉摆去抖计数器：只要不在 S_BAL 就清零（每次进平衡都从 0 起算）
            if (state != S_BAL) fall_cnt <= 2'd0;

            // 总计时 / 升级计时（都按 frame_tick）
            if (!sw_active) begin
                sw_ms   <= 32'd0;
                esc_cnt <= 32'd0;
            end
            else if (frame_tick) begin
                if (sw_ms < TIMEOUT_FRAMES) sw_ms <= sw_ms + 32'd1;
                //  ★ 2026-10-23：升档只在【泵能态】做。原来在 kick/滑行/等静止阶段
                //    也在升，导致"掉摆后重新起摆"一开始的力度就比初次起摆大。
                if (state == S_PUMP) begin
                    if (esc_cnt < ESCALATE_FRAMES) begin
                        esc_cnt <= esc_cnt + 32'd1;
                    end
                    else begin
                        esc_cnt <= 32'd0;
                        if (i_level < (I_FLOOR_MAX - I_FLOOR_STEP)) i_level <= i_level + I_FLOOR_STEP;
                        else i_level <= I_FLOOR_MAX;
                    end
                end
                else begin
                    esc_cnt <= 32'd0;
                end
            end

            case (state)

            S_IDLE: begin
                ms_cnt   <= 32'd0;
                i_level  <= I_FLOOR_START;
                kick_dir <= 1'b0;
                if (run_en && !run_d) begin
                    // LQR_2.1b：手扶启动时只看角度。原来还要求 |θ̇|<W_CAP，
                    // 手抖一下没满足就先去 kick，等于把用户扶好的摆杆踢飞。
                    if (cap_ok || ((ang_now < ANG_CAP) && (ang_now > -ANG_CAP))) begin
                        state     <= S_BAL;
                        bal_entry <= 1'b1;
                    end
                    else begin
                        state <= S_KICK1;
                    end
                end
            end

            S_KICK1: begin
                if (!run_en) state <= S_IDLE;
                else if (frame_tick) begin
                    if (ms_cnt >= (KICK_FRAMES - 1)) begin
                        ms_cnt <= 32'd0;
                        state  <= S_KICK2;
                    end
                    else ms_cnt <= ms_cnt + 32'd1;
                end
            end

            S_KICK2: begin
                if (!run_en) state <= S_IDLE;
                else if (frame_tick) begin
                    if (ms_cnt >= (KICK_FRAMES - 1)) begin
                        ms_cnt    <= 32'd0;
                        stall_cnt <= 32'd0;
                        state     <= S_PUMP;
                    end
                    else ms_cnt <= ms_cnt + 32'd1;
                end
            end

            S_PUMP: begin
                if (!run_en) state <= S_IDLE;
                else if (sw_ms >= TIMEOUT_FRAMES) state <= S_IDLE;
                else if (cap_ok) begin
                    state     <= S_BAL;
                    bal_entry <= 1'b1;
                end
                else if ((E_STOP_DIS == 0) && (e_f > E_STOP_Q)) begin
                    ms_cnt <= 32'd0;              // 进入滑行：清驻留计时
                    state  <= S_WAIT;
                end
                else if (frame_tick) begin
                    // 卡在静摩擦里：换方向 re-kick，并把电流升一档
                    if (spd_q < 16'sd25 && spd_q > -16'sd25) begin
                        if (stall_cnt >= STALL_FRAMES) begin
                            stall_cnt <= 32'd0;
                            ms_cnt    <= 32'd0;
                            kick_dir  <= ~kick_dir;
                            if (i_level < (I_FLOOR_MAX - I_FLOOR_STEP)) i_level <= i_level + I_FLOOR_STEP;
                            else i_level <= I_FLOOR_MAX;
                            state <= S_KICK1;
                        end
                        else stall_cnt <= stall_cnt + 32'd1;
                    end
                    else stall_cnt <= 32'd0;
                end
            end

            S_WAIT: begin
                if (!run_en) state <= S_IDLE;
                else if (sw_ms >= TIMEOUT_FRAMES) state <= S_IDLE;
                else if (cap_ok) begin
                    state     <= S_BAL;
                    bal_entry <= 1'b1;
                end
                else if (frame_tick) begin
                    if (ms_cnt < WAIT_FRAMES) ms_cnt <= ms_cnt + 32'd1;
                    //  掉回水平以下(|θ|>120°) 才重新泵能：
                    //  只用能量阈值会在竖直附近被速度噪声反复触发，出现 3/4 高频切换。
                    else if ((e_f < E_RESUME_Q) || (ang_u > 16'd1200)) state <= S_PUMP;
                end
            end

            S_BAL: begin
                if (!run_en) state <= S_IDLE;
                else if (force_fall) begin
                    i_level   <= I_FLOOR_START;
                    stall_cnt <= 32'd0;
                    esc_cnt   <= 32'd0;
                    rest_cnt  <= 32'd0;
                    ms_cnt    <= 32'd0;
                    state     <= S_FALL;
                end
                //  ★ v5.6：掉摆判据加 3 帧确认（2.3ms）。
                //    旧写法单帧 |θ|>60° 就立刻进 S_FALL、把 i_ref 清 0，
                //    一次角度野值就能让电机瞬间断电 —— 实测"电流突然为 0"就是它。
                //    真掉摆时多等 2 帧完全没影响（60° 走到 78° 还要十几 ms）。
                else if (frame_tick && (ang_now > ANG_FALL || ang_now < -ANG_FALL)) begin
                    if (fall_cnt >= 2'd2) begin
                        if (AUTO_RESWING != 0) begin
                            // 掉摆后不要立刻用最大电流硬泵（那样只会来回甩、反复抖），
                            // 先进 S_FALL 等摆杆在下垂位静止，再按冷启动流程重新起摆。
                            i_level   <= I_FLOOR_START;
                            stall_cnt <= 32'd0;
                            esc_cnt   <= 32'd0;
                            rest_cnt  <= 32'd0;
                            ms_cnt    <= 32'd0;
                            state     <= S_FALL;
                        end
                        else state <= S_IDLE;
                    end
                    else fall_cnt <= fall_cnt + 2'd1;
                end
            end

            // 掉摆恢复（2026-10-23 第三版）：
            //   不再“滑 250ms 就直接泵能”，而是【等摆杆静止在下垂位】再重新起摆，
            //   避免在动态下注能导致抖动/能量抵消。
            //   判据：|θ|>150° 且 |θ̇|<60°/s 连续 400ms；
            //   若 6s 还没停，兜底退回“直接泵能”的老策略。
            S_FALL: begin
                if (!run_en) begin
                    state <= S_IDLE;
                end
                else if (frame_tick) begin
                    ms_cnt <= ms_cnt + 32'd1;

                    if (ang_u > REST_ANG && spd_q < REST_SPD && spd_q > -REST_SPD &&
                        arm_speed < ARM_REST_SPD && arm_speed > -ARM_REST_SPD)
                        rest_cnt <= rest_cnt + 32'd1;
                    else
                        rest_cnt <= 32'd0;

                    if (rest_cnt >= (REST_FRAMES - 1)) begin
                        // 已静止：完全按“冷启动”流程（与 IDLE 首次起摆一致）
                        ms_cnt    <= 32'd0;
                        rest_cnt  <= 32'd0;
                        stall_cnt <= 32'd0;
                        esc_cnt   <= 32'd0;
                        i_level   <= I_FLOOR_START;   // 力度回到 700mA 起
                        kick_dir  <= 1'b0;            // 和上电首次起摆完全一致（+kick 先）
                        spd_clear <= 1'b1;            // 速度估计清零，泵能从零状态开始
                        state     <= S_KICK1;
                    end
                    else if (ms_cnt >= (SETTLE_TIMEOUT_FRAMES - 1)) begin
                        //  ★ 2026-10-23 v5.4：等太久没静止，也【不能跳过 kick】。
                        //    旧版这里直接进 PUMP，等于再起摆没有"反转甩臂"那一下，
                        //    摆杆根本甩不起来 —— 这正是"再起摆和第一次不一样"的原因。
                        //    现在兜底同样走 KICK1->KICK2->PUMP，参数与首次一致。
                        ms_cnt    <= 32'd0;
                        rest_cnt  <= 32'd0;
                        stall_cnt <= 32'd0;
                        esc_cnt   <= 32'd0;
                        i_level   <= I_FLOOR_START;
                        kick_dir  <= 1'b0;
                        spd_clear <= 1'b1;
                        state     <= S_KICK1;
                    end
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
