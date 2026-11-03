//======================================================================
//  oled_render12.v -- 8x12 text renderer, 16x5 rows, direct framebuffer
//  Clear framebuffer, then OR glyph pixels and divider via fb_set.
//======================================================================
`timescale 1ns/1ps

module oled_render12 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         busy,
    output reg  [6:0]  taddr,
    input  wire [7:0]  tdata,
    output reg  [10:0] faddr,
    input  wire [7:0]  fdata,
    output reg         fb_we,
    output reg         fb_set,
    output reg  [9:0]  fb_addr,
    output reg  [7:0]  fb_data
);

    localparam [3:0] S_IDLE  = 4'd0,
                     S_CLR   = 4'd1,
                     S_ROW   = 4'd2,
                     S_TWAIT = 4'd3,
                     S_GY    = 4'd4,
                     S_FWAIT = 4'd5,
                     S_GX    = 4'd6,
                     S_LINE  = 4'd7,
                     S_EDGE  = 4'd8;

    reg [3:0] st;
    reg [2:0] row;
    reg [3:0] ch;
    reg [3:0] gy;
    reg [2:0] gx;
    reg [6:0] chdat;
    reg [7:0] frow;
    reg [6:0] line_col;
    reg [9:0] addr;
    reg [2:0] edge_page;
    reg [6:0] edge_col;
    reg       edge_phase;

    wire [7:0] y = {1'b0, row} * 8'd12 + (row == 3'd0 ? 8'd0 : 8'd2) + {4'd0, gy};
    wire [6:0] col = {ch, 3'b000} + {4'd0, gx};
    wire [9:0] pix_addr = {y[5:3], col};
    wire [7:0] mask = 8'd1 << y[2:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            busy     <= 1'b0;
            taddr    <= 7'd0;
            faddr    <= 11'd0;
            fb_we    <= 1'b0;
            fb_set   <= 1'b0;
            fb_addr  <= 10'd0;
            fb_data  <= 8'd0;
            row      <= 3'd0;
            ch       <= 4'd0;
            gy       <= 4'd0;
            gx       <= 3'd0;
            chdat    <= 7'd0;
            frow     <= 8'd0;
            line_col <= 7'd0;
            addr     <= 10'd0;
            edge_page <= 3'd0;
            edge_col  <= 7'd0;
            edge_phase<= 1'b0;
        end
        else begin
            fb_we  <= 1'b0;
            fb_set <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        addr <= 10'd0;
                        st   <= S_CLR;
                    end
                end

                S_CLR: begin
                    fb_we   <= 1'b1;
                    fb_set  <= 1'b0;
                    fb_addr <= addr;
                    fb_data <= 8'd0;
                    if (addr == 10'd1023) begin
                        addr <= 10'd0;
                        row  <= 3'd0;
                        st   <= S_ROW;
                    end
                    else addr <= addr + 10'd1;
                end

                S_ROW: begin
                    ch    <= 4'd0;
                    gy    <= 4'd0;
                    gx    <= 3'd0;
                    taddr <= {row, 4'd0};
                    st    <= S_TWAIT;
                end

                S_TWAIT: begin
                    chdat <= tdata[6:0];
                    gy    <= 4'd0;
                    st    <= S_GY;
                end

                S_GY: begin
                    faddr <= (chdat - 7'd32) * 11'd12 + {7'd0, gy};
                    st    <= S_FWAIT;
                end

                S_FWAIT: begin
                    frow <= fdata;
                    gx   <= 3'd0;
                    st   <= S_GX;
                end

                S_GX: begin
                    if (frow[3'd7 - gx]) begin
                        fb_we   <= 1'b1;
                        fb_set  <= 1'b1;
                        fb_addr <= pix_addr;
                        fb_data <= mask;
                    end

                    if (gx == 3'd7) begin
                        if (gy == 4'd11) begin
                            if (ch == 4'd15) begin
                                if (row == 3'd4) begin
                                    edge_page  <= 3'd0;
                                    edge_col   <= 7'd0;
                                    edge_phase <= 1'b0;
                                    st         <= S_EDGE;
                                end
                                else begin
                                    row   <= row + 3'd1;
                                    ch    <= 4'd0;
                                    gy    <= 4'd0;
                                    gx    <= 3'd0;
                                    taddr <= {row + 3'd1, 4'd0};
                                    st    <= S_TWAIT;
                                end
                            end
                            else begin
                                ch    <= ch + 4'd1;
                                gy    <= 4'd0;
                                gx    <= 3'd0;
                                taddr <= {row, ch + 4'd1};
                                st    <= S_TWAIT;
                            end
                        end
                        else begin
                            gy <= gy + 4'd1;
                            gx <= 3'd0;
                            st <= S_GY;
                        end
                    end
                    else begin
                        gx <= gx + 3'd1;
                        st <= S_GX;
                    end
                end

                S_EDGE: begin
                    fb_we  <= 1'b1;
                    fb_set <= 1'b0;
                    if (!edge_phase) begin
                        fb_addr <= {edge_page, edge_col};
                        fb_data <= 8'd0;
                        if (edge_col == 7'd7) begin
                            edge_col <= 7'd0;
                            if (edge_page == 3'd7) begin
                                edge_page  <= 3'd0;
                                edge_phase <= 1'b1;
                            end
                            else edge_page <= edge_page + 3'd1;
                        end
                        else edge_col <= edge_col + 7'd1;
                    end
                    else begin
                        fb_addr <= {edge_page, 7'd120 + edge_col};
                        fb_data <= 8'd0;
                        if (edge_col == 7'd7) begin
                            edge_col <= 7'd0;
                            if (edge_page == 3'd7) begin
                                line_col <= 7'd0;
                                st       <= S_LINE;
                            end
                            else edge_page <= edge_page + 3'd1;
                        end
                        else edge_col <= edge_col + 7'd1;
                    end
                end

                S_LINE: begin
                    fb_we   <= 1'b1;
                    fb_set  <= 1'b1;
                    fb_addr <= {3'd1, line_col};
                    fb_data <= 8'h20;
                    if (line_col == 7'd127) begin
                        busy <= 1'b0;
                        st   <= S_IDLE;
                    end
                    else line_col <= line_col + 7'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule