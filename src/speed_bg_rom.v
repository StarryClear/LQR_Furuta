`timescale 1ns/1ps
module speed_bg_rom (
    input  wire       clk,
    input  wire [9:0] addr,
    output reg  [7:0] dout
);
    reg [7:0] rom [0:1023];
    initial $readmemh("src/speed_bg_v2.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule