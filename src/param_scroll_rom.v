// 9 frames of 128x64: vertical scroll from page A to page B
`timescale 1ns/1ps
module param_scroll_rom (
    input  wire        clk,
    input  wire [13:0] addr,   // {frame[3:0], fb_addr[9:0]}
    output reg  [7:0]  dout
);
    reg [7:0] rom [0:16383];
    initial $readmemh("src/param_scroll.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule
