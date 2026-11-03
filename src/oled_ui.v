//======================================================================
//  oled_ui.v  --  OLED 仪表盘：定标 -> 文本 -> 显存 -> I2C
//
//  屏幕 16 列 x 8 行（8x8 点阵），每行 = 4 字符标签 + 7 字符数值 + 5 字符单位
//
//  行内容（当前底层工程）：
//    0  MODE   运行秒数        STOP/FWD /REV /RUN
//    1  TH     摆杆角 (0.1°)   SA100
//    2  THD    摆杆角速度      °/s（10ms 差分）
//    3  PH     臂角 (0.1°)     编码器 -> 圈内角度
//    4  PHD    臂角速度        °/s（10ms 差分）
//    5  U      控制量          %（PWM counts -> 0.1%）
//    6  I      电流            mA（ADC counts，零点/增益待标定）
//    7  SA1 EN0 ERR 000        传感器健康 + 编码器错误计数
//
//  刷新：I2C 每发完一帧给 frame_done，本模块立刻重算 8 行 -> 渲染 -> 交回总线。
//        200kHz 下一帧约 50ms（20fps），重算+渲染约 90us，不占控制时间。
//======================================================================
`timescale 1ns/1ps

module oled_ui #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer ADC_ZERO = 0,          // 电流零点 counts（标定后改这里）
    parameter integer SCL_HZ   = 200_000,    // 上拉弱就降到 100_000
    parameter [7:0]   X_OFF    = 8'd0        // SSD1306=0，SH1106=2
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [1:0]         mode,          // 0=STOP 1=FWD 2=REV 3=RUN
    input  wire signed [15:0] theta_x10,     // 摆杆角，0.1°
    input  wire [31:0]        phi_cnt,       // 编码器计数
    input  wire [15:0]        enc_err,       // 编码器非法跳变计数
    input  wire               sa_lost,       // SA100 掉线
    input  wire               angle_valid,   // SA100 新帧
    input  wire [15:0]        duty,          // PWM counts (0..2500)
    input  wire [11:0]        adc_raw,       // MCP3201 原始码
    output wire               oled_scl,
    inout  wire               oled_sda
);

    reg ui_busy;

    // 上电 1ms 后先自己刷一屏，避免 I2C 第一帧发出未初始化的显存
    reg [16:0] boot_cnt;
    reg        boot_fmt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            boot_cnt <= 17'd0;
            boot_fmt <= 1'b0;
        end
        else if (boot_cnt >= 17'd50_000) begin
            boot_fmt <= 1'b0;
        end
        else begin
            boot_cnt <= boot_cnt + 17'd1;
            boot_fmt <= (boot_cnt == 17'd49_999);
        end
    end

    //==================================================================
    // 1. 10ms 时基 / 导数 / 运行时间
    //==================================================================
    reg [18:0] t10_cnt;
    reg        t10;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t10_cnt <= 19'd0;
            t10     <= 1'b0;
        end
        else if (t10_cnt >= 19'd499_999) begin
            t10_cnt <= 19'd0;
            t10     <= 1'b1;
        end
        else begin
            t10_cnt <= t10_cnt + 19'd1;
            t10     <= 1'b0;
        end
    end

    //---- 摆杆角速度：每个 SA100 帧累加增量，10ms 归一化 ----
    reg signed [15:0] th_prev;
    reg signed [23:0] th_acc;

    wire signed [16:0] dth_raw = $signed(theta_x10) - $signed(th_prev);
    reg  signed [16:0] dth;
    always @(*) begin
        if (dth_raw > 17'sd1800)
            dth = dth_raw - 17'sd3600;
        else if (dth_raw < -17'sd1800)
            dth = dth_raw + 17'sd3600;
        else
            dth = dth_raw;
    end

    //---- 臂角速度：10ms 差分 ----
    reg [31:0] phi_prev;
    wire signed [31:0] dphi = $signed(phi_cnt) - $signed(phi_prev);

    reg signed [15:0] thd_x10;
    reg signed [15:0] phd_x10;

    wire signed [31:0] thd_wide = th_acc * 32'sd100;
    wire signed [47:0] phd_wide = dphi * 48'sd436;

    function signed [15:0] clamp9k;
        input signed [47:0] v;
        begin
            if (v > 48'sd9999)
                clamp9k = 16'sd9999;
            else if (v < -48'sd9999)
                clamp9k = -16'sd9999;
            else
                clamp9k = v[15:0];
        end
    endfunction

    //---- 运行时间：0.1s ----
    reg [31:0] uptime;
    reg [3:0]  t10_div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            th_prev  <= 16'sd0;
            th_acc   <= 24'sd0;
            phi_prev <= 32'd0;
            thd_x10  <= 16'sd0;
            phd_x10  <= 16'sd0;
            uptime   <= 32'd0;
            t10_div  <= 4'd0;
        end
        else begin
            if (angle_valid) begin
                th_prev <= theta_x10;
                th_acc  <= th_acc + {{7{dth[16]}}, dth};
            end

            if (t10) begin
                thd_x10  <= clamp9k(thd_wide);
                phd_x10  <= clamp9k(phd_wide);
                th_acc   <= 24'sd0;
                phi_prev <= phi_cnt;

                if (t10_div == 4'd9) begin
                    t10_div <= 4'd0;
                    if (uptime != 32'd9999)
                        uptime <= uptime + 32'd1;
                end
                else
                    t10_div <= t10_div + 4'd1;
            end
        end
    end

    //==================================================================
    // 2. 文本缓冲 128 字符（16 列 x 8 行）
    //==================================================================
    reg [7:0] text [0:127];

    wire [6:0] txt_addr;      // 由渲染器驱动
    reg  [7:0] txt_data;

    always @(posedge clk)
        txt_data <= text[txt_addr];

    //==================================================================
    // 3. 除法器
    //==================================================================
    reg         div_start;
    reg  [31:0] div_dividend;
    reg  [31:0] div_divisor;
    wire [31:0] div_quot;
    wire [31:0] div_rem;
    wire        div_busy;
    wire        div_done;

    udiv32 u_div (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (div_start),
        .dividend (div_dividend),
        .divisor  (div_divisor),
        .quot     (div_quot),
        .rem      (div_rem),
        .busy     (div_busy),
        .done     (div_done)
    );

    //==================================================================
    // 4. 渲染器 / 显存 / 字模
    //==================================================================
    reg         rend_start;
    wire        rend_busy;
    wire [10:0] font_addr;
    wire [7:0]  font_data;
    wire        fb_we;
    wire        fb_set;

    wire [9:0]  fb_waddr;
    wire [7:0]  fb_wdata;
    wire [9:0]  fb_raddr;
    wire [7:0]  fb_rdata;

    oled_render12 u_render (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (rend_start),
        .busy    (rend_busy),
        .taddr   (txt_addr),
        .tdata   (txt_data),
        .faddr   (font_addr),
        .fdata   (font_data),
        .fb_we   (fb_we),
        .fb_set  (fb_set),

        .fb_addr (fb_waddr),
        .fb_data (fb_wdata)
    );

    font_rom12 u_font (
        .clk  (clk),
        .addr (font_addr),
        .dout (font_data)
    );

    oled_fb u_fb (
        .clk   (clk),
        .we    (fb_we),
        .wset  (fb_set),
        .wxor  (1'b0),
        .waddr (fb_waddr),
        .wdata (fb_wdata),
        .raddr (fb_raddr),
        .rdata (fb_rdata)
    );

    //==================================================================
    // 5. I2C 显示控制器
    //==================================================================
    wire frame_done;

    sh1106_i2c #(
        .CLK_FREQ (CLK_FREQ),
        .SCL_HZ   (SCL_HZ),
        .DEV_ADDR (7'h3C),
        .X_OFF    (X_OFF)
    ) u_i2c (
        .clk         (clk),
        .rst_n       (rst_n),
        .scl         (oled_scl),
        .sda         (oled_sda),
        .fb_addr     (fb_raddr),
        .fb_data     (fb_rdata),
        .frame_done  (frame_done),
        .render_busy (ui_busy)
    );

    //==================================================================
    // 6. 刷新状态机：算值 -> 组行 -> 送 16 字节 -> 渲染
    //==================================================================
    localparam [4:0] S_IDLE   = 5'd0,
                     S_DPHI   = 5'd1,
                     S_DPHIW  = 5'd2,
                     S_DU     = 5'd3,
                     S_DUW    = 5'd4,
                     S_ROW    = 5'd5,
                     S_VINIT  = 5'd6,
                     S_VDIV   = 5'd7,
                     S_VDIVW  = 5'd8,
                     S_VPUT   = 5'd9,
                     S_VZERO  = 5'd10,
                     S_VSIGN  = 5'd11,
                     S_HEALTH = 5'd12,
                     S_EMIT   = 5'd13,
                     S_EMIT2  = 5'd14,
                     S_RENDW  = 5'd15,
                     S_BLANK  = 5'd16;

    reg [4:0] st;

    reg [2:0]  r;
    reg [3:0]  bc;

    reg signed [15:0] fv;
    reg               fdec;
    reg               fdec2;
    reg signed [15:0] fmag;
    reg               fsign;
    reg [3:0]         bp;
    reg [3:0]         bn;
    reg [7:0]         vb [0:6];

    reg [15:0] phi_deg;
    reg [15:0] u_x10;

    reg [127:0] row_sh;
    reg [127:0] hrow_q;
    reg         is_health;

    reg        seen_r;
    reg [15:0] rend_wdog;

    //---- 电流：mA = (raw - ZERO) * 13/16 ≈ 0.8125 mA/count ----
    wire signed [16:0] cur_raw = $signed({1'b0, adc_raw}) - ADC_ZERO;
    wire signed [31:0] cur_ma_w = (cur_raw * 32'sd13) >>> 4;
    wire signed [16:0] cur_ma  = cur_ma_w[16:0];

    //---- 臂角：0..3599 映射到 ±1800 ----
    wire signed [16:0] phi_s = (phi_deg > 16'd1800) ?
                               ($signed({1'b0, phi_deg}) - 17'sd3600) :
                                $signed({1'b0, phi_deg});

    //---- 十六进制字符 ----
    function [7:0] hexc;
        input [3:0] v;
        begin
            hexc = (v < 4'd10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});
        end
    endfunction

    //---- 行标签 / 单位 ----
    reg [31:0] mode_lab;
    always @(*) begin
        case (mode)
            2'd0:    mode_lab = "STOP";
            2'd1:    mode_lab = "FWD ";
            2'd2:    mode_lab = "REV ";
            default: mode_lab = "RUN ";
        endcase
    end

    reg [31:0] lab_r;
    always @(*) begin
        case (r)
            3'd0:    lab_r = "TIME";
            3'd1:    lab_r = "I   ";
            3'd2:    lab_r = "TH  ";
            3'd3:    lab_r = "PH  ";
            3'd4:    lab_r = "PHD ";
            default: lab_r = "    ";
        endcase
    end

    reg [39:0] unit_r;
    always @(*) begin
        case (r)
            3'd0:    unit_r = "s    ";
            3'd1:    unit_r = "mA   ";
            3'd2:    unit_r = "deg  ";
            3'd3:    unit_r = "deg  ";
            3'd4:    unit_r = "d/s  ";
            default: unit_r = "     ";
        endcase
    end

    //---- 第 7 行：SA / EN / ERR ----
    wire [7:0] c_sa = sa_lost ? 8'h30 : 8'h31;
    wire [7:0] c_en = (enc_err != 16'd0) ? 8'h31 : 8'h30;

    wire [127:0] health_row = {
        8'h53, 8'h41, c_sa, 8'h20,
        8'h45, 8'h4E, c_en, 8'h20,
        8'h45, 8'h52, 8'h52, 8'h20,
        hexc(enc_err[11:8]), hexc(enc_err[7:4]), hexc(enc_err[3:0]), 8'h20
    };

    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            ui_busy      <= 1'b0;
            r            <= 3'd0;
            bc           <= 4'd0;
            fv           <= 16'sd0;
            fdec         <= 1'b1;
            fdec2        <= 1'b0;
            fmag         <= 16'sd0;
            fsign        <= 1'b0;
            bp           <= 4'd6;
            bn           <= 4'd0;
            phi_deg      <= 16'd0;
            u_x10        <= 16'd0;
            row_sh       <= 128'd0;
            hrow_q       <= 128'd0;
            is_health    <= 1'b0;
            rend_start   <= 1'b0;
            seen_r       <= 1'b0;
            rend_wdog    <= 16'd0;
            div_start    <= 1'b0;
            div_dividend <= 32'd0;
            div_divisor  <= 32'd10;
            for (k = 0; k < 7; k = k + 1)
                vb[k] <= 8'h20;
        end
        else begin
            div_start  <= 1'b0;
            rend_start <= 1'b0;

            case (st)
                //----------------------------------------------
                S_IDLE: begin
                    if (frame_done || boot_fmt) begin
                        ui_busy <= 1'b1;
                        st      <= S_DPHI;
                    end
                end

                //----------------------------------------------
                S_DPHI: begin
                    div_dividend <= phi_cnt;
                    div_divisor  <= 32'd825;
                    div_start    <= 1'b1;
                    st           <= S_DPHIW;
                end

                S_DPHIW: begin
                    if (div_done) begin
                        // 3600/825 = 4.3636 ≈ 4 + 93/256
                        phi_deg <= (({22'd0, div_rem[9:0]} * 32'd4) +
                                    (({22'd0, div_rem[9:0]} * 32'd93) >> 8));
                        st <= S_DU;
                    end
                end

                //----------------------------------------------
                S_DU: begin
                    div_dividend <= {14'd0, duty, 1'b0};   // duty*2
                    div_divisor  <= 32'd5;
                    div_start    <= 1'b1;
                    st           <= S_DUW;
                end

                S_DUW: begin
                    if (div_done) begin
                        u_x10 <= div_quot[15:0];
                        r     <= 3'd0;
                        st    <= S_ROW;
                    end
                end

                //----------------------------------------------
                S_ROW: begin
                    if (r >= 3'd5) begin
                        st <= S_BLANK;
                    end
                    else begin
                        case (r)
                            3'd0:    fv <= uptime[15:0];
                            3'd1:    fv <= cur_ma[15:0];
                            3'd2:    fv <= theta_x10;
                            3'd3:    fv <= phi_deg[15:0];
                            default: fv <= phd_x10;
                        endcase
                        fdec2 <= 1'b0;
                        fdec  <= (r == 3'd0) || (r == 3'd2) || (r == 3'd3) || (r == 3'd4);
                        st    <= S_VINIT;
                    end
                end

                //----------------------------------------------
                S_VINIT: begin
                    fsign <= fv[15];
                    fmag  <= fv[15] ? (16'sd0 - fv) : fv;
                    bp    <= 4'd6;
                    bn    <= 4'd0;
                    for (k = 0; k < 7; k = k + 1)
                        vb[k] <= 8'h20;
                    st    <= S_VDIV;
                end

                S_VDIV: begin
                    div_dividend <= {16'd0, fmag};
                    div_divisor  <= 32'd10;
                    div_start    <= 1'b1;
                    st           <= S_VDIVW;
                end

                S_VDIVW: begin
                    if (div_done)
                        st <= S_VPUT;
                end

                // 从右往左写一位数字；dec=1 时第一位是小数位，后面补 '.'
                S_VPUT: begin
                    if (bp <= 4'd9)
                        vb[bp[2:0]] <= 8'h30 + {4'd0, div_rem[3:0]};

                    bn   <= bn + 4'd1;
                    fmag <= div_quot[15:0];

                    if (fdec2 && bn == 4'd1) begin
                        if (bp >= 4'd2)
                            vb[bp[2:0] - 3'd1] <= 8'h2E;   // '.'
                        bp <= (bp >= 4'd2) ? (bp - 4'd2) : 4'd0;
                        st <= (div_quot[15:0] == 16'd0) ? S_VZERO : S_VDIV;
                    end
                    else if (fdec && bn == 4'd0) begin
                        if (bp >= 4'd2)
                            vb[bp[2:0] - 3'd1] <= 8'h2E;   // '.'
                        bp <= (bp >= 4'd2) ? (bp - 4'd2) : 4'd0;
                        // 至少还要一位整数位
                        st <= (div_quot[15:0] == 16'd0) ? S_VZERO : S_VDIV;
                    end
                    else begin
                        bp <= (bp == 4'd0) ? 4'd0 : (bp - 4'd1);
                        st <= (div_quot[15:0] == 16'd0) ? S_VSIGN : S_VDIV;
                    end
                end

                // 0.x 的整数位补 '0'
                S_VZERO: begin
                    if (bp <= 4'd9)
                        vb[bp[2:0]] <= 8'h30;
                    bp <= (bp == 4'd0) ? 4'd0 : (bp - 4'd1);
                    st <= S_VSIGN;
                end

                S_VSIGN: begin
                    if (fsign && bp <= 4'd6)
                        vb[bp[2:0]] <= 8'h2D;    // '-'
                    is_health <= 1'b0;
                    st        <= S_EMIT;
                end

                //----------------------------------------------
                S_HEALTH: begin
                    hrow_q    <= health_row;
                    is_health <= 1'b1;
                    st        <= S_EMIT;
                end

                //----------------------------------------------
                S_EMIT: begin
                    if (is_health)
                        row_sh <= hrow_q;
                    else
                        row_sh <= {lab_r, vb[0], vb[1], vb[2], vb[3], vb[4], vb[5], vb[6], unit_r};
                    is_health <= 1'b0;
                    bc        <= 4'd0;
                    st        <= S_EMIT2;
                end

                S_EMIT2: begin
                    text[{r, bc}] <= row_sh[127:120];
                    row_sh        <= {row_sh[119:0], 8'h00};
                    bc            <= bc + 4'd1;

                    if (bc == 4'd15) begin
                        if (r == 3'd7) begin
                            rend_start <= 1'b1;
                            seen_r     <= 1'b0;
                            rend_wdog  <= 16'd0;
                            st         <= S_RENDW;
                        end
                        else begin
                            r  <= r + 3'd1;
                            st <= S_ROW;
                        end
                    end
                end

                //----------------------------------------------
                S_BLANK: begin
                    for (k = 0; k < 7; k = k + 1)
                        vb[k] <= 8'h20;
                    is_health <= 1'b0;
                    st <= S_EMIT;
                end

                S_RENDW: begin
                    if (rend_busy)
                        seen_r <= 1'b1;

                    if (seen_r && !rend_busy) begin
                        ui_busy <= 1'b0;
                        st      <= S_IDLE;
                    end
                    else if (rend_wdog[15]) begin
                        ui_busy <= 1'b0;      // 兜底：约 0.65ms
                        st      <= S_IDLE;
                    end
                    else
                        rend_wdog <= rend_wdog + 16'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
