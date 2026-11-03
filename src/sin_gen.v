//======================================================================
//  sin_gen.v  --  正弦轨迹发生器（1 kHz 相位累加 + 256 点表线性插值）
//
//  y = amp * sin(2*pi*phase/65536)
//   · phase 每 ms 加 step，step = 65536*f/1000
//   · 幅度做斜坡（每 ms 2 = 0.2°/ms），上电/退出都不跳变
//   · 幅度为 0 时相位暂停，输出恒 0
//======================================================================
`timescale 1ns/1ps

module sin_gen (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                tick1k,
    input  wire                en,          // 1 = 正弦模式
    input  wire [15:0]         step,        // 每 ms 相位步进
    input  wire signed [15:0]  amp,         // 目标幅度（0.1°）
    output wire signed [15:0]  y,           // 输出（0.1°）
    output wire                active       // 幅度非零
);

    reg  [15:0]        phase;
    reg  signed [15:0] amp_now;

    // 幅度斜坡
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            amp_now <= 16'sd0;
        else if (tick1k) begin
            if (en) begin
                if (amp_now < amp)
                    amp_now <= (amp - amp_now > 16'sd2) ? amp_now + 16'sd2 : amp;
                else if (amp_now > amp)
                    amp_now <= (amp_now - amp > 16'sd2) ? amp_now - 16'sd2 : amp;
            end
            else if (amp_now > 16'sd0)
                amp_now <= (amp_now > 16'sd2) ? amp_now - 16'sd2 : 16'sd0;
        end
    end

    // 相位累加（幅度为 0 时暂停，保证从 0 相位起振）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase <= 16'd0;
        else if (tick1k && (amp_now != 16'sd0))
            phase <= phase + step;
    end

    wire [7:0]  idx  = phase[15:8];
    wire [1:0]  frac = phase[7:6];
    wire signed [15:0] s0, s1;

    sin_lut u_lut0 (.idx(idx),           .val(s0));
    sin_lut u_lut1 (.idx(idx + 8'd1),    .val(s1));

    // 插值（Q15 → Q15）
    reg  signed [31:0] yi_r;
    wire signed [31:0] dlt = $signed(s1) - $signed(s0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            yi_r <= 32'sd0;
        else
            yi_r <= $signed(s0) + ((dlt * $signed({30'd0, frac})) >>> 2);
    end

    // Q15 × 幅度
    wire signed [31:0] yq = (yi_r * $signed(amp_now)) >>> 15;

    assign y      = (yq >  32'sd2000) ?  16'sd2000 :
                    (yq < -32'sd2000) ? -16'sd2000 : yq[15:0];
    assign active = (amp_now != 16'sd0);

endmodule
