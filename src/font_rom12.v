// 8x12 ASCII font: 96 chars x 12 rows, row-major
`timescale 1ns/1ps
module font_rom12 (
    input  wire        clk,
    input  wire [10:0] addr,
    output reg  [7:0]  dout
);
    reg [7:0] rom [0:2047];
    initial $readmemh("src/font8x12.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule