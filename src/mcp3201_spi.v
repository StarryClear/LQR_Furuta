//======================================================================
//  mcp3201_spi.v  --  MCP3201 12-bit SPI ADC readout
//
//  CS# low -> sample + conversion start.
//  SCLK = 1 MHz. 14 clocks are issued; DOUT is sampled on SCLK rising.
//  The frame is aligned by finding the first 0 (null bit).
//
//  Timing: 1 us CS setup + 14 us clocks ~= 15 us.
//  At 20 kHz PWM (50 us) this fits one conversion per PWM period.
//======================================================================
`timescale 1ns/1ps

module mcp3201_spi #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SCLK_HZ  = 1_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        dout,
    output reg         cs_n,
    output reg         sclk,
    output reg  [11:0] data,
    output reg         null_bit,
    output reg         valid,
    output reg         busy
);

    localparam integer HALF = CLK_FREQ / (SCLK_HZ * 2); // 25 at 50 MHz / 1 MHz
    localparam integer TSU  = CLK_FREQ / 1_000_000;     // 50 at 50 MHz

    localparam [1:0] S_IDLE  = 2'd0,
                     S_SETUP = 2'd1,
                     S_CLK   = 2'd2,
                     S_HOLD  = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [3:0]  nbits;
    reg [13:0] sr;

    wire [13:0] sr_next = {sr[12:0], dout};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            cs_n     <= 1'b1;
            sclk     <= 1'b0;
            cnt      <= 16'd0;
            nbits    <= 4'd0;
            sr       <= 14'd0;
            data     <= 12'd0;
            null_bit <= 1'b0;
            valid    <= 1'b0;
            busy     <= 1'b0;
        end
        else begin
            valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        cs_n  <= 1'b0;
                        cnt   <= 16'd0;
                        state <= S_SETUP;
                    end
                end

                S_SETUP: begin
                    if (cnt == TSU - 1) begin
                        cnt   <= 16'd0;
                        nbits <= 4'd0;
                        state <= S_CLK;
                    end
                    else
                        cnt <= cnt + 16'd1;
                end

                S_CLK: begin
                    if (cnt == HALF - 1) begin
                        cnt  <= 16'd0;
                        sclk <= ~sclk;
                        if (!sclk) begin
                            sr    <= sr_next;
                            nbits <= nbits + 4'd1;
                            if (nbits == 4'd13)
                                state <= S_HOLD;
                        end
                    end
                    else
                        cnt <= cnt + 16'd1;
                end

                S_HOLD: begin
                    sclk <= 1'b0;
                    cs_n <= 1'b1;

                    if (!sr_next[13]) begin
                        data     <= sr_next[12:1];
                        null_bit <= 1'b0;
                    end
                    else if (!sr_next[12]) begin
                        data     <= sr_next[11:0];
                        null_bit <= 1'b0;
                    end
                    else begin
                        data     <= 12'd0;
                        null_bit <= 1'b1;
                    end

                    valid <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
