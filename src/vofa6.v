//======================================================================
//  vofa6.v  --  VOFA+ JustFloat，6 通道
//
//  帧格式： float32 ch0..ch5 | 00 00 80 7F   （小端，共 28 字节）
//  16 位有符号整数 -> IEEE754 单精度，纯组合逻辑，1 拍完成。
//
//  通道：mode, duty, pos[15:0], speed, adc, sa
//  100Hz 下 2.8 kB/s，115200 占用 24%
//======================================================================
`timescale 1ns/1ps

module vofa6 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [15:0] ch0,
    input  wire signed [15:0] ch1,
    input  wire signed [15:0] ch2,
    input  wire signed [15:0] ch3,
    input  wire signed [15:0] ch4,
    input  wire signed [15:0] ch5,
    input  wire               uart_busy,
    output reg  [7:0]         tx_data,
    output reg                tx_valid,
    output reg                busy
);

    function [31:0] i16_to_f32;
        input signed [15:0] x;
        reg        sign;
        reg [15:0] mag;
        reg [3:0]  p;
        reg [7:0]  exp;
        reg [31:0] m32;
        begin
            sign = x[15];
            mag  = sign ? (~x + 16'd1) : x[15:0];
            if      (mag[15]) p = 4'd15;
            else if (mag[14]) p = 4'd14;
            else if (mag[13]) p = 4'd13;
            else if (mag[12]) p = 4'd12;
            else if (mag[11]) p = 4'd11;
            else if (mag[10]) p = 4'd10;
            else if (mag[9])  p = 4'd9;
            else if (mag[8])  p = 4'd8;
            else if (mag[7])  p = 4'd7;
            else if (mag[6])  p = 4'd6;
            else if (mag[5])  p = 4'd5;
            else if (mag[4])  p = 4'd4;
            else if (mag[3])  p = 4'd3;
            else if (mag[2])  p = 4'd2;
            else if (mag[1])  p = 4'd1;
            else              p = 4'd0;

            if (mag == 16'd0) begin
                i16_to_f32 = 32'h0000_0000;
            end
            else begin
                exp = 8'd127 + {4'd0, p};
                m32 = {16'd0, mag} << (23 - p);
                i16_to_f32 = {sign, exp, m32[22:0]};
            end
        end
    endfunction

    localparam [0:0] S_IDLE = 1'b0, S_SEND = 1'b1;
    localparam [5:0] LEN    = 6'd28;

    reg        st;
    reg [31:0] f0, f1, f2, f3, f4, f5;
    reg [5:0]  idx;

    wire [5:0]  csel  = idx[4:2];
    wire [31:0] cur32 = (csel == 6'd0) ? f0 :
                        (csel == 6'd1) ? f1 :
                        (csel == 6'd2) ? f2 :
                        (csel == 6'd3) ? f3 :
                        (csel == 6'd4) ? f4 :
                        (csel == 6'd5) ? f5 : 32'h7F80_0000;

    wire [31:0] cur_shifted = cur32 >> {idx[1:0], 3'b000};
    wire [7:0]  cur_byte    = cur_shifted[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            busy     <= 1'b0;
            tx_valid <= 1'b0;
            tx_data  <= 8'd0;
            idx      <= 6'd0;
            f0 <= 32'd0; f1 <= 32'd0; f2 <= 32'd0;
            f3 <= 32'd0; f4 <= 32'd0; f5 <= 32'd0;
        end
        else begin
            case (st)
                S_IDLE: begin
                    tx_valid <= 1'b0;
                    if (start) begin
                        f0 <= i16_to_f32(ch0);
                        f1 <= i16_to_f32(ch1);
                        f2 <= i16_to_f32(ch2);
                        f3 <= i16_to_f32(ch3);
                        f4 <= i16_to_f32(ch4);
                        f5 <= i16_to_f32(ch5);
                        idx  <= 6'd0;
                        busy <= 1'b1;
                        st   <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (!tx_valid) begin
                        if (idx == LEN) begin
                            busy <= 1'b0;
                            st   <= S_IDLE;
                        end
                        else if (!uart_busy) begin
                            tx_data  <= cur_byte;
                            tx_valid <= 1'b1;
                        end
                    end
                    else if (uart_busy) begin
                        tx_valid <= 1'b0;
                        idx      <= idx + 6'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
