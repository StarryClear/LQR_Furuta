//======================================================================
//  angle_calc.v  --  SA100 PWM -> position / angle（带野值门限）
//
//  WHEELTEC SA100 manual:
//      Position = ton * 4098 / (ton + toff) - 1
//      Position = 0 .. 4095  (12-bit)
//      angle_x10 = Position * 3600 / 4096
//
//  3600/4096 = 225/256，所以角度换算是乘 225 再右移 8。
//
//  ★ 2026-10-30 新增野值门限：
//    中值滤波挡住"单帧"幽灵脉冲，但 EMI 偶尔会连续 2~4 帧。
//    这里对 position 做逐帧跳变判据：一帧变化超过 300 counts（26°）
//    就判为野值，保持上一个位置和角度；连续 100 帧都被拒才强制重同步
//    （防止上电时初值不匹配被永久锁死）。
//    真实最大角速度 448°/s，一帧（1.17ms）最多 0.52°，300 counts 余量足够。
//======================================================================
`timescale 1ns/1ps

module angle_calc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] high_ticks,
    input  wire [31:0] period_ticks,
    output reg  [11:0] position_raw,      // 0 .. 4095
    output reg  signed [15:0] angle_x10,  // -1800 .. +1800
    output reg         valid,
    output reg         busy,
    output reg  [7:0]  rej_cnt            // 被拒次数（诊断）
);

    // ★ 3.0h：门限 300(26°/帧=22000°/s) 太松，摆杆物理极限只有 450°/s(≈5 counts/帧)；
    //   而且 12 位整圈野值会被环绕逻辑算成 ±196 counts 的"合法"跳变直接放行。
    //   收到 40 counts(4°/帧≈3400°/s，仍有 7 倍余量) 就能在源头拦掉整圈野值。
    localparam signed [12:0] JUMP_MAX = 13'sd40;   // 4°/帧
    localparam [7:0]         REJ_MAX  = 8'd100;     // 强制重同步

    reg [31:0] dividend;
    reg [31:0] divisor;
    reg [31:0] remainder;
    reg [31:0] quotient;
    reg [5:0]  count;
    reg [11:0] pos_prev;
    reg [7:0]  rej;

    wire [31:0] numerator = high_ticks * 32'd4098;
    wire [31:0] pos_tmp   = (quotient > 32'd0) ? (quotient - 32'd1) : 32'd0;
    wire [31:0] pos_clamp = (pos_tmp > 32'd4095) ? 32'd4095 : pos_tmp;
    wire [11:0] pnew      = pos_clamp[11:0];

    // 环形差（0/4095 是相邻的）
    wire signed [12:0] d0 = $signed({1'b0, pnew}) - $signed({1'b0, pos_prev});
    wire signed [12:0] d1 = (d0 > 13'sd2048) ? (d0 - 13'sd4096) :
                            (d0 < -13'sd2048) ? (d0 + 13'sd4096) : d0;
    wire               step_ok = (d1 <= JUMP_MAX) && (d1 >= -JUMP_MAX);

    wire [31:0] ang_tmp = (pnew * 32'd225) >> 8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dividend     <= 32'd0;
            divisor      <= 32'd0;
            remainder    <= 32'd0;
            quotient     <= 32'd0;
            count        <= 6'd0;
            position_raw <= 12'd0;
            angle_x10    <= 16'sd0;
            valid        <= 1'b0;
            busy         <= 1'b0;
            pos_prev     <= 12'd0;
            rej          <= 8'd0;
            rej_cnt      <= 8'd0;
        end
        else begin
            valid <= 1'b0;

            if (start && !busy) begin
                if (period_ticks == 32'd0) begin
                    valid <= 1'b1;              // 保持原值，不更新
                end
                else begin
                    dividend  <= numerator;
                    divisor   <= period_ticks;
                    remainder <= 32'd0;
                    quotient  <= 32'd0;
                    count     <= 6'd32;
                    busy      <= 1'b1;
                end
            end
            else if (busy) begin
                if (count == 6'd0) begin
                    // 门限判决：正常接受 / 野值保持 / 超时强制接受
                    if (step_ok || (rej >= REJ_MAX)) begin
                        position_raw <= pnew;
                        pos_prev     <= pnew;
                        angle_x10    <= (ang_tmp > 32'd1800)
                                        ? ($signed(ang_tmp[15:0]) - 16'sd3600)
                                        :  $signed(ang_tmp[15:0]);
                        rej          <= 8'd0;
                    end
                    else begin
                        if (rej != 8'd255) rej <= rej + 8'd1;
                        rej_cnt <= rej_cnt + 8'd1;
                    end
                    valid <= 1'b1;
                    busy  <= 1'b0;
                end
                else begin
                    remainder <= {remainder[30:0], dividend[31]};
                    dividend  <= {dividend[30:0], 1'b0};

                    if ({remainder[30:0], dividend[31]} >= divisor) begin
                        remainder <= {remainder[30:0], dividend[31]} - divisor;
                        quotient  <= {quotient[30:0], 1'b1};
                    end
                    else begin
                        quotient <= {quotient[30:0], 1'b0};
                    end

                    count <= count - 6'd1;
                end
            end
        end
    end

endmodule
