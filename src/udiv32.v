//======================================================================
//  udiv32.v  --  32 位无符号除法器（恢复余数法，32 拍）
//
//  start 后 busy 拉高，32 拍后 done 给出 1 拍脉冲，
//  quot / rem 有效。divisor = 0 时 quot = 0xFFFFFFFF、rem = 0。
//
//  显示模块用它做两件事：
//    · 编码器计数 -> 圈内余数（/825）
//    · 占空比 counts -> 0.1%（*2/5）
//======================================================================
`timescale 1ns/1ps

module udiv32 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] dividend,
    input  wire [31:0] divisor,
    output reg  [31:0] quot,
    output reg  [31:0] rem,
    output reg         busy,
    output reg         done
);

    reg [31:0] dvd;
    reg [31:0] dvs;
    reg [32:0] acc;
    reg [5:0]  cnt;

    wire [33:0] acc_sh = {acc[32:0], dvd[31]};
    wire        ge     = (acc_sh >= {2'b00, dvs});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dvd  <= 32'd0;
            dvs  <= 32'd0;
            acc  <= 33'd0;
            cnt  <= 6'd0;
            quot <= 32'd0;
            rem  <= 32'd0;
            busy <= 1'b0;
            done <= 1'b0;
        end
        else begin
            done <= 1'b0;

            if (start && !busy) begin
                if (divisor == 32'd0) begin
                    quot <= 32'hFFFF_FFFF;
                    rem  <= 32'd0;
                    done <= 1'b1;
                end
                else begin
                    dvd  <= dividend;
                    dvs  <= divisor;
                    acc  <= 33'd0;
                    cnt  <= 6'd32;
                    quot <= 32'd0;
                    busy <= 1'b1;
                end
            end
            else if (busy) begin
                if (cnt == 6'd0) begin
                    rem  <= acc[31:0];
                    busy <= 1'b0;
                    done <= 1'b1;
                end
                else begin
                    acc  <= ge ? (acc_sh[32:0] - {1'b0, dvs}) : acc_sh[32:0];
                    dvd  <= {dvd[30:0], 1'b0};
                    quot <= {quot[30:0], ge};
                    cnt  <= cnt - 6'd1;
                end
            end
        end
    end

endmodule
