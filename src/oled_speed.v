//======================================================================
//  oled_speed.v -- speed preset list, reverse-video sliding cursor
//  Static 128x64 background + XOR highlight band.
//======================================================================
`timescale 1ns/1ps

module oled_speed (
    input  wire clk,
    input  wire rst_n,
    input  wire enter,
    input  wire knob_step,
    input  wire knob_dir,
    input  wire knob_btn_short,
    input  wire knob_btn_long,
    output reg  active,
    output wire [1:0] speed_sel,
    output wire oled_scl,
    inout  wire oled_sda
);

    localparam integer CLK_FREQ = 50_000_000;
    localparam integer SCL_HZ   = 400_000;

    localparam [2:0] S_IDLE = 3'd0,
                     S_WAIT = 3'd1,
                     S_PRE  = 3'd2,
                     S_COPY = 3'd3,
                     S_XOR  = 3'd4,
                     S_DONE = 3'd5;

    reg [2:0]  st;
    reg [1:0]  sel;
    assign speed_sel = sel;
    reg [6:0]  hl_y;
    reg [6:0]  target_y;
    reg [9:0]  addr;
    reg [3:0]  ycnt;
    reg [6:0]  xcnt;
    reg        dirty;
    reg        ui_busy;

    reg        fb_we;
    reg        fb_wset;
    reg        fb_wxor;
    reg [9:0]  fb_waddr;
    reg [7:0]  fb_wdata;
    wire [9:0] fb_raddr;
    wire [7:0] fb_rdata;
    wire       frame_done;

    reg [9:0]  bg_addr;
    wire [7:0] bg_data;

    speed_bg_rom u_bg (
        .clk  (clk),
        .addr (bg_addr),
        .dout (bg_data)
    );

    oled_fb u_fb (
        .clk   (clk),
        .we    (fb_we),
        .wset  (fb_wset),
        .wxor  (fb_wxor),
        .waddr (fb_waddr),
        .wdata (fb_wdata),
        .raddr (fb_raddr),
        .rdata (fb_rdata)
    );

    sh1106_i2c #(
        .CLK_FREQ (CLK_FREQ),
        .SCL_HZ   (SCL_HZ),
        .DEV_ADDR (7'h3C),
        .X_OFF    (8'd0)
    ) u_i2c (
        .clk         (clk),
        .rst_n       (rst_n),
        .scl         (oled_scl),
        .sda         (oled_sda),
        .fb_addr     (fb_raddr),
        .fb_data     (fb_rdata),
        .frame_done  (frame_done),
        .render_busy (ui_busy)
    );

    wire [7:0] hy = hl_y + {4'd0, ycnt};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            active    <= 1'b0;
            sel       <= 2'd1;
            hl_y      <= 7'd26;
            target_y  <= 7'd26;
            addr      <= 10'd0;
            ycnt      <= 4'd0;
            xcnt      <= 7'd0;
            dirty     <= 1'b0;
            ui_busy   <= 1'b0;
            fb_we     <= 1'b0;
            fb_wset   <= 1'b0;
            fb_wxor   <= 1'b0;
            fb_waddr  <= 10'd0;
            fb_wdata  <= 8'd0;
            bg_addr   <= 10'd0;
        end
        else begin
            fb_we <= 1'b0;

            if (!active && enter) begin
                active   <= 1'b1;
                // ★ 8.0：不再每次进入重置为 1 档，保留上次的选择
                // ★ 8.0：高亮条按当前选择定位（原来固定 26 = 0 档，和 sel=1 不符）
                hl_y     <= 7'd14 + {5'd0, sel} * 7'd12;
                target_y <= 7'd14 + {5'd0, sel} * 7'd12;
                dirty    <= 1'b1;
                st       <= S_WAIT;
            end
            else if (active) begin
                if (knob_step) begin
                    if (knob_dir && (sel < 2'd3)) begin
                        sel      <= sel + 2'd1;
                        target_y <= 7'd14 + ({5'd0, sel} + 6'd1) * 7'd12;
                        dirty    <= 1'b1;
                    end
                    else if (!knob_dir && (sel > 2'd0)) begin
                        sel      <= sel - 2'd1;
                        target_y <= 7'd14 + ({5'd0, sel} - 6'd1) * 7'd12;
                        dirty    <= 1'b1;
                    end
                end

                if (knob_btn_short || knob_btn_long) begin
                    active  <= 1'b0;
                    ui_busy <= 1'b0;
                    st      <= S_IDLE;
                end
                else begin
                    case (st)
                        S_IDLE: st <= S_WAIT;

                        S_WAIT: begin
                            if (dirty && frame_done && !ui_busy) begin
                                ui_busy <= 1'b1;
                                bg_addr <= 10'd0;
                                addr    <= 10'd0;
                                st      <= S_PRE;
                            end
                        end

                        S_PRE: begin
                            bg_addr <= 10'd0;
                            addr    <= 10'd0;
                            st      <= S_COPY;
                        end

                        S_COPY: begin
                            fb_we    <= 1'b1;
                            fb_wset  <= 1'b0;
                            fb_wxor  <= 1'b0;
                            fb_waddr <= addr;
                            fb_wdata <= bg_data;
                            if (addr == 10'd1023) begin
                                ycnt <= 4'd0;
                                xcnt <= 7'd0;
                                st   <= S_XOR;
                            end
                            else begin
                                addr    <= addr + 10'd1;
                                bg_addr <= addr + 10'd1;
                            end
                        end

                        S_XOR: begin
                            fb_we    <= 1'b1;
                            fb_wset  <= 1'b0;
                            fb_wxor  <= 1'b1;
                            fb_waddr <= {hy[5:3], xcnt};
                            fb_wdata <= 8'd1 << hy[2:0];
                            if (xcnt == 7'd127) begin
                                xcnt <= 7'd0;
                                if (ycnt == 4'd11)
                                    st <= S_DONE;
                                else
                                    ycnt <= ycnt + 4'd1;
                            end
                            else xcnt <= xcnt + 7'd1;
                        end

                        S_DONE: begin
                            ui_busy <= 1'b0;
                            if (hl_y != target_y) begin
                                if (hl_y < target_y)
                                    hl_y <= hl_y + 7'd2;
                                else
                                    hl_y <= hl_y - 7'd2;
                                dirty <= 1'b1;
                            end
                            else dirty <= 1'b0;
                            st <= S_WAIT;
                        end

                        default: st <= S_WAIT;
                    endcase
                end
            end
            else begin
                ui_busy <= 1'b0;
                st      <= S_IDLE;
            end
        end
    end

endmodule