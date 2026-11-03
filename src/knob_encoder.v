//======================================================================
//  knob_encoder.v  --  EC11 旋钮：A/B 正交 + 按键
//
//  输入默认上拉，按键低有效。
//  每格（A 相下降沿）输出一个 step 脉冲；dir=1 表示顺时针。
//  5 格 = 90°（机械角）；菜单/跟随的换算在外部做。
//
//  按键：
//    - 短按：释放时且按住 < LONG_MS，输出 btn_short 单拍
//    - 长按：按住达到 LONG_MS，输出 btn_long 单拍
//======================================================================
`timescale 1ns/1ps

module knob_encoder #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer LONG_MS  = 800
)(
    input  wire clk,
    input  wire rst_n,
    input  wire a,
    input  wire b,
    input  wire btn,
    output reg  step,
    output reg  dir,
    output reg  btn_short,
    output reg  btn_long
);

    // ---- 同步 ----
    reg a_s1, a_s2, b_s1, b_s2, btn_s1, btn_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_s1 <= 1'b1; a_s2 <= 1'b1;
            b_s1 <= 1'b1; b_s2 <= 1'b1;
            btn_s1 <= 1'b1; btn_s2 <= 1'b1;
        end
        else begin
            a_s1 <= a; a_s2 <= a_s1;
            b_s1 <= b; b_s2 <= b_s1;
            btn_s1 <= btn; btn_s2 <= btn_s1;
        end
    end

    // ---- 1 ms 采样 ----
    reg [15:0] ms_cnt;
    reg        ms_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt  <= 16'd0;
            ms_tick <= 1'b0;
        end
        else if (ms_cnt >= CLK_FREQ/1000 - 1) begin
            ms_cnt  <= 16'd0;
            ms_tick <= 1'b1;
        end
        else begin
            ms_cnt  <= ms_cnt + 16'd1;
            ms_tick <= 1'b0;
        end
    end

    // ---- A 相下降沿：每格一个 step ----
    reg a_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step <= 1'b0;
            dir  <= 1'b1;
            a_d  <= 1'b1;
        end
        else begin
            step <= 1'b0;
            if (ms_tick) begin
                a_d <= a_s2;
                if (a_d && !a_s2) begin
                    step <= 1'b1;
                    dir  <= ~b_s2;      // 方向；如相反，交换 A/B 或取反
                end
            end
        end
    end

    // ---- 按键消抖 + 短/长按 ----
    localparam integer LONG_CNT = LONG_MS;   // 1 ms 单位
    reg [15:0] stable_cnt;
    reg        btn_stable;
    reg [15:0] hold_cnt;
    reg        long_sent;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_cnt <= 16'd0;
            btn_stable <= 1'b1;
            hold_cnt   <= 16'd0;
            long_sent  <= 1'b0;
            btn_short  <= 1'b0;
            btn_long   <= 1'b0;
        end
        else begin
            btn_short <= 1'b0;
            btn_long  <= 1'b0;

            if (ms_tick) begin
                if (btn_s2 != btn_stable) begin
                    if (stable_cnt >= 16'd10) begin
                        btn_stable <= btn_s2;
                        stable_cnt <= 16'd0;
                        if (!btn_s2) begin
                            hold_cnt  <= 16'd0;
                            long_sent <= 1'b0;
                        end
                        else if (!long_sent) begin
                            btn_short <= 1'b1;   // 释放且未到长按
                        end
                    end
                    else
                        stable_cnt <= stable_cnt + 16'd1;
                end
                else
                    stable_cnt <= 16'd0;

                if (!btn_stable) begin
                    if (hold_cnt < LONG_CNT)
                        hold_cnt <= hold_cnt + 16'd1;
                    else if (!long_sent) begin
                        long_sent <= 1'b1;
                        btn_long  <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
