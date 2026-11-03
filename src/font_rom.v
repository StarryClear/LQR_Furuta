//======================================================================
//  font_rom.v  --  5x7 ASCII 点阵，OLED 页格式（列优先）
//
//  字符集：ASCII 0x00..0x7F（128 个），地址 = {char[6:0], col[2:0]}
//  一个字节 = **同一列的 8 个竖直像素**，bit0 在最上面（SSD1306 页格式）。
//  字形占 x=1..5、y=0..6；小写字母借用大写字形。
//
//  数据由 scripts/gen_font.py 生成（font8x8.mem，1024 字节）。
//  综合成 1 块 BSRAM；输出寄存一拍（1 周期读延迟）。
//======================================================================
`timescale 1ns/1ps

module font_rom (
    input  wire        clk,
    input  wire [9:0]  addr,   // {char[6:0], col[2:0]}
    output reg  [7:0]  dout
);

    reg [7:0] rom [0:1023];

    initial $readmemh("src/font8x8.mem", rom);

    always @(posedge clk)
        dout <= rom[addr];

endmodule
