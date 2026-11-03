//======================================================================
//  sync_ff.v  --  two-flop synchronizer for asynchronous inputs
//======================================================================
`timescale 1ns/1ps

module sync_ff #(
    parameter integer WIDTH = 1
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] din,
    output wire [WIDTH-1:0] dout
);

    reg [WIDTH-1:0] sync0;
    reg [WIDTH-1:0] sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0 <= {WIDTH{1'b0}};
            sync1 <= {WIDTH{1'b0}};
        end
        else begin
            sync0 <= din;
            sync1 <= sync0;
        end
    end

    assign dout = sync1;

endmodule
