//======================================================================
//  quad_decoder.v  --  A/B 四倍频，带 5us 数字滤波
//
//  电刷噪声会在编码器线上产生微秒级毛刺，直接把 4 倍频计数打爆
//  （实测 20% 占空比时 10ms 跳 855 counts，相当于 100 转/秒）。
//  这里加一级"状态稳定计数"：原始 A/B 状态必须连续稳定 FILT 个时钟
//  （默认 256 = 5.12us @50MHz）才承认跳变，之后的格雷码判决与原来一致。
//
//  真实最高计数率：825 counts/rev x 4 rev/s = 3300/s，边沿间隔 ~75us，
//  远离 5us，不会误伤。rej 记录被滤波丢掉的毛刺次数（诊断用）。
//======================================================================
`timescale 1ns/1ps

module quad_decoder #(
    parameter integer FILT = 256          // 稳定窗口，单位：时钟
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        a,
    input  wire        b,
    output reg  [31:0] pos,
    output reg         dir,
    output reg  [15:0] err,
    output reg  [15:0] rej
);

    reg [1:0] a_sync, b_sync;
    reg [1:0] raw_d;
    reg [8:0] st_cnt;
    reg [1:0] filt;
    reg [1:0] prev;

    wire [1:0] raw = {a_sync[1], b_sync[1]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
            raw_d  <= 2'b00;
            st_cnt <= 9'd0;
            filt   <= 2'b00;
            prev   <= 2'b00;
            pos    <= 32'd0;
            dir    <= 1'b0;
            err    <= 16'd0;
            rej    <= 16'd0;
        end
        else begin
            a_sync <= {a_sync[0], a};
            b_sync <= {b_sync[0], b};
            raw_d  <= raw;

            // 稳定窗口：raw 变化就重新计时，稳定够久才更新 filt
            if (raw != raw_d) begin
                st_cnt <= 9'd0;
                if (rej != 16'hFFFF)
                    rej <= rej + 16'd1;
            end
            else if (st_cnt != FILT[8:0])
                st_cnt <= st_cnt + 9'd1;
            else
                filt <= raw;

            prev <= filt;

            case ({prev, filt})
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: begin
                    pos <= pos + 32'd1;
                    dir <= 1'b1;
                end
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: begin
                    pos <= pos - 32'd1;
                    dir <= 1'b0;
                end
                4'b00_00, 4'b01_01, 4'b11_11, 4'b10_10: begin
                    // no change
                end
                default: begin
                    if (err != 16'hFFFF)
                        err <= err + 16'd1;
                end
            endcase
        end
    end

endmodule
