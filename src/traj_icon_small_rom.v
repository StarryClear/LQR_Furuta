// 24x24 trajectory icons: 4 icons x 3 pages x 24 cols
`timescale 1ns/1ps
module traj_icon_small_rom (
    input  wire       clk,
    input  wire [8:0] addr,   // {icon[1:0], page[1:0], col[4:0]}
    output reg  [7:0] dout
);
    reg [7:0] rom [0:511];
    initial $readmemh("src/traj_icon_small.mem", rom);
    always @(posedge clk) dout <= rom[addr];
endmodule