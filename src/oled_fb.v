//======================================================================
//  oled_fb.v  --  128x64 单色显存（页格式）
//
//  地址 = {page[2:0], col[6:0]}，共 1024 字节；
//  一个字节 = 该列 8 个像素，bit0 在上。
//
//  一个写口（渲染器），一个读口（I2C 发送）。读数据晚 1 拍。
//  综合成 1 块 BSRAM（1K x 8 简单双口）。
//======================================================================
`timescale 1ns/1ps

module oled_fb (
    input  wire       clk,
    input  wire       we,
    input  wire       wset,
    input  wire       wxor,
    input  wire [9:0] waddr,
    input  wire [7:0] wdata,
    input  wire [9:0] raddr,
    output reg  [7:0] rdata
);

    reg [7:0] mem [0:1023];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wxor ? (mem[waddr] ^ wdata) :
                          wset ? (mem[waddr] | wdata) : wdata;
        rdata <= mem[raddr];
    end

endmodule
