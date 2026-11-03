//======================================================================
//  uart_rx.v  --  8N1 串口接收模块（波特率可参数化，中点采样）
//
//  收到一个完整字节后 data_valid 输出一个时钟周期的高脉冲，
//  同时 data_out 给出该字节。
//======================================================================
`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟频率(Hz)
    parameter integer BAUD     = 115200        // 波特率
)(
    input  wire       clk,          // 系统时钟
    input  wire       rst_n,        // 异步复位，低电平有效
    input  wire       rx,           // 串口接收引脚
    output reg  [7:0] data_out,     // 收到的字节
    output reg        data_valid    // 接收完成脉冲
);

    localparam [15:0] BAUD_CNT = (CLK_FREQ + BAUD / 2) / BAUD;  // 一位的时钟数
    localparam [15:0] HALF_CNT = BAUD_CNT / 2;                  // 半位（用于起始位确认）

    reg [2:0]  rx_sync;     // 输入同步打拍，消除亚稳态
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;     // 0:起始位确认  1~8:数据位  9:停止位
    reg [7:0]  data_buf;
    reg        rx_busy;

    wire rx_d = rx_sync[2]; // 同步后的接收电平

    // 输入同步
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 3'b111;
        else        rx_sync <= {rx_sync[1:0], rx};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_busy    <= 1'b0;
            baud_cnt   <= 16'd0;
            bit_idx    <= 4'd0;
            data_buf   <= 8'd0;
            data_out   <= 8'd0;
            data_valid <= 1'b0;
        end
        else begin
            data_valid <= 1'b0;
            if (!rx_busy) begin
                if (!rx_d) begin                // 捕获起始位下降沿
                    rx_busy  <= 1'b1;
                    baud_cnt <= 16'd0;
                    bit_idx  <= 4'd0;
                end
            end
            else begin
                // 起始位只在半个位宽后确认一次，之后每 1 位采样一次
                if (baud_cnt == ((bit_idx == 4'd0) ? HALF_CNT : BAUD_CNT) - 16'd1) begin
                    baud_cnt <= 16'd0;
                    if (bit_idx == 4'd0) begin
                        if (rx_d) rx_busy <= 1'b0;      // 是毛刺，放弃本帧
                        else      bit_idx <= 4'd1;
                    end
                    else if (bit_idx <= 4'd8) begin     // 采 8 个数据位（LSB 先到）
                        data_buf[bit_idx - 4'd1] <= rx_d;
                        bit_idx <= bit_idx + 4'd1;
                    end
                    else begin                          // bit_idx == 9，停止位
                        data_out   <= data_buf;
                        data_valid <= 1'b1;
                        rx_busy    <= 1'b0;
                    end
                end
                else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end
        end
    end

endmodule
