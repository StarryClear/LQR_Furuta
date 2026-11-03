// no-frame menu background (triangle only)
`timescale 1ns/1ps
module menu_bg_noframe_rom (
    input  wire        clk,
    input  wire [9:0]  addr,
    output reg  [7:0]  dout
);
    reg [7:0] rom [0:1023];
    initial $readmemh("src/menu_bg_noframe.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule