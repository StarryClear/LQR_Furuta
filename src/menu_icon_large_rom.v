// 48x48 icons: 3 icons x 6 pages x 48 cols
`timescale 1ns/1ps
module menu_icon_large_rom (
    input  wire        clk,
    input  wire [10:0] addr,  // {icon[1:0], page[2:0], col[5:0]}
    output reg  [7:0]  dout
);
    reg [7:0] rom [0:2047];
    initial $readmemh("src/menu_icon_large.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule
