//======================================================================
//  cmd_rx.v  --  单字节串口命令解析
//
//  '0'..'5' : 直接设置模式（ser_set + ser_mode）
//  'c' / 'C': 编码器基准清零（ser_clr）
//  其它字节（含 \r \n）忽略
//
//  用单字节命令，避免行尾差异；PC 端每次只发 1 个字符。
//======================================================================
`timescale 1ns/1ps

module cmd_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] byte_in,
    input  wire       byte_valid,
    output reg  [3:0] ser_mode,
    output reg        ser_set,
    output reg        ser_clr
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ser_mode <= 4'd0;
            ser_set  <= 1'b0;
            ser_clr  <= 1'b0;
        end
        else begin
            ser_set <= 1'b0;
            ser_clr <= 1'b0;

            if (byte_valid) begin
                if (byte_in >= 8'h30 && byte_in <= 8'h39) begin
                    ser_mode <= byte_in[3:0];      // '0'..'9'
                    ser_set  <= 1'b1;
                end
                else if (byte_in == 8'h41) begin
                    ser_mode <= 4'd10;             // 'A' = LQR 闭环
                    ser_set  <= 1'b1;
                end
                else if (byte_in == 8'h42) begin
                    ser_mode <= 4'd11;             // 'B' = 闭环但输出限幅 400
                    ser_set  <= 1'b1;
                end
                else if (byte_in == 8'h43) begin
                    ser_mode <= 4'd12;             // 'C' = 符号测试 +
                    ser_set  <= 1'b1;
                end
                else if (byte_in == 8'h44) begin
                    ser_mode <= 4'd13;             // 'D' = 符号测试 -
                    ser_set  <= 1'b1;
                end
                else if (byte_in == 8'h63) begin
                    ser_clr <= 1'b1;               // 'c' 小写 = 清零
                end
            end
        end
    end

endmodule
