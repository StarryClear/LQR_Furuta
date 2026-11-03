//======================================================================
//  menu_bg_rom.v  --  主菜单静态背景：中间选中框 + 下方空心三角形
//======================================================================
`timescale 1ns/1ps

module menu_bg_rom (
    input  wire       clk,
    input  wire [9:0] addr,
    output reg  [7:0] dout
);

    reg [7:0] rom [0:1023];

    initial $readmemh("src/menu_bg.mem", rom);

    always @(posedge clk)
        dout <= rom[addr];

endmodule
