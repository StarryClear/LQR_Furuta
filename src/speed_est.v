//======================================================================
//  speed_est.v  --  编码器测速（10 ms 差分 + 一阶低通）
//
//  2026-11-03 时序修复：
//    原来 32 位减法进位链 + ×100 挤在同一拍里，实测 24 ns，
//    这是 LQR_7.0 Fmax 49.9（slack −0.037）的元凶。
//    拆两拍：tick 时算差分并寄存 -> 下一拍再 ×100 / 限幅 / 低通。
//======================================================================
`timescale 1ns/1ps

module speed_est (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [31:0]        pos,
    input  wire               tick,        // 10ms 脉冲
    output reg  signed [15:0] speed
);

    reg  [31:0]        prev;
    reg  signed [32:0] dpos_r;      // 一拍寄存的差分
    reg                tick_d;      // tick 延时一拍
    reg  signed [15:0] raw_r;

    wire signed [47:0] raw48 = dpos_r * 48'sd100;

    always @(*) begin
        if (raw48 > 48'sd32767)
            raw_r = 16'sd32767;
        else if (raw48 < -48'sd32767)
            raw_r = -16'sd32767;
        else
            raw_r = raw48[15:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev   <= 32'd0;
            dpos_r <= 33'sd0;
            tick_d <= 1'b0;
            speed  <= 16'sd0;
        end
        else begin
            tick_d <= tick;

            if (tick) begin
                dpos_r <= $signed({1'b0, pos}) - $signed({1'b0, prev});
                prev   <= pos;
            end

            if (tick_d)
                speed <= speed + ((raw_r - speed) >>> 2);
        end
    end

endmodule
