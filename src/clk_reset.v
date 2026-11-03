//======================================================================
//  clk_reset.v  --  power-on / external reset synchronizer
//  50 MHz input. rst_n_in is active-low asynchronous.
//  Output stays low for about 2^19 clocks (~10.5 ms) after release.
//======================================================================
`timescale 1ns/1ps

module clk_reset (
    input  wire clk,
    input  wire rst_n_in,
    output wire rst_n_out
);

    reg [2:0]  rst_sync;
    reg [19:0] por_cnt;

    always @(posedge clk or negedge rst_n_in) begin
        if (!rst_n_in) begin
            rst_sync <= 3'b000;
            por_cnt  <= 20'd0;
        end
        else begin
            rst_sync <= {rst_sync[1:0], 1'b1};
            if (!por_cnt[19])
                por_cnt <= por_cnt + 20'd1;
        end
    end

    assign rst_n_out = rst_sync[2] & por_cnt[19];

endmodule
