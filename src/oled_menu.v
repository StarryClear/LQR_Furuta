//======================================================================
//  oled_menu.v  --  icon-only 3-page main menu, no intermediate animation
//
//  Order: SPEED (0), PARAM (1), TRACE (2).
//  Default selection at power-up: PARAM (1).
//  Selected icon: 48x48 at center. Side icons: 24x24 lower.
//  Knob step changes selection immediately; no sliding/scale animation.
//  No wrap: stops at both ends.
//  Center frame rounded/thick; hollow triangle below.
//
//  SSD1306 128x64, I2C 0x3C, SCL=N8, SDA=R8.
//======================================================================
`timescale 1ns/1ps

module oled_menu #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SCL_HZ   = 400_000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire knob_step,
    input  wire knob_dir,
    input  wire knob_btn_short,
    input  wire knob_btn_long,
    output wire param_mode,
    output wire traj_mode,
    output wire [1:0] traj_sel,
    output reg  speed_req,
    output wire oled_scl,
    inout  wire oled_sda
);

    localparam [3:0] S_IDLE       = 4'd0,
                     S_BG         = 4'd1,
                     S_BG_W       = 4'd2,
                     S_SMALL_ADDR = 4'd3,
                     S_SMALL_WR   = 4'd4,
                     S_LARGE_ADDR = 4'd5,
                     S_LARGE_WR   = 4'd6,
                     S_COPY       = 4'd7,
                     S_COPY_W     = 4'd8,
                     S_WAIT       = 4'd9;

    reg [3:0]  st;
    reg [9:0]  cnt;
    reg [1:0]  icon;
    reg [2:0]  page;
    reg [5:0]  col;
    reg        render_busy;
    reg        render_req;

    reg [1:0]  sel;
    reg [1:0]  r_sel;
    reg        traj_page;
    reg        r_traj_page;
    reg        frame_on;
    reg        r_frame_on;
    reg        scr_page;
    reg        r_scr_page;
    reg [3:0]  param_frame;
    reg [3:0]  r_param_frame;
    reg [1:0]  target;
    reg        moving;
    reg        anim_dir;
    reg signed [7:0] anim;
    reg signed [7:0] r_anim;

    reg [7:0] frm [0:1023];

    wire [7:0] menu_bg_data;
    wire [7:0] noframe_bg_data;
    wire [7:0] param_data;
    wire [13:0] param_addr = {r_param_frame, cnt};
    wire [7:0] bg_data = r_scr_page ? param_data :
                       ((r_traj_page && !r_frame_on) ? noframe_bg_data : menu_bg_data);
    wire [7:0] menu_small_data;
    wire [7:0] traj_small_data;
    wire [7:0] small_data = r_traj_page ? traj_small_data : menu_small_data;
    wire [7:0] menu_large_data;
    wire [7:0] traj_large_data;
    wire [7:0] large_data = r_traj_page ? traj_large_data : menu_large_data;
    reg  [9:0] bg_addr;
    wire [8:0]  small_addr = icon * 9'd72  + page[1:0] * 9'd24 + col[4:0];
    wire [10:0] large_addr = r_sel * 11'd288 + page[2:0] * 11'd48 + col[5:0];
    wire [9:0] fb_raddr;
    wire [7:0] fb_rdata;
    wire       fb_we;
    reg  [9:0] fb_waddr;
    reg  [7:0] fb_wdata;
    wire       frame_done;

    menu_bg_rom u_bg (
        .clk  (clk),
        .addr (bg_addr),
        .dout (menu_bg_data)
    );

    param_scroll_rom u_param (
        .clk  (clk),
        .addr (param_addr),
        .dout (param_data)
    );

    menu_bg_noframe_rom u_noframe (
        .clk  (clk),
        .addr (bg_addr),
        .dout (noframe_bg_data)
    );

    menu_icon_small_rom u_small (
        .clk  (clk),
        .addr (small_addr),
        .dout (menu_small_data)
    );

    menu_icon_large_rom u_large (
        .clk  (clk),
        .addr (large_addr),
        .dout (menu_large_data)
    );

    traj_icon_small_rom u_tsmall (
        .clk  (clk),
        .addr (small_addr),
        .dout (traj_small_data)
    );

    traj_icon_large_rom u_tlarge (
        .clk  (clk),
        .addr (large_addr),
        .dout (traj_large_data)
    );

    oled_fb u_fb (
        .clk   (clk),
        .we    (fb_we),
        .wset  (1'b0),
        .wxor  (1'b0),
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
        .render_busy (render_busy)
    );

    assign fb_we = (st == S_COPY_W);
    assign param_mode = scr_page;
    assign traj_mode  = traj_page && frame_on;
    assign traj_sel   = sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            speed_req <= 1'b0;
        else begin
            speed_req <= 1'b0;
            if (knob_btn_short && !scr_page && !traj_page && !moving && (sel == 2'd0))
                speed_req <= 1'b1;
        end
    end

    // 0=center, 1=right, 2=left, 3=hidden (snapshot r_sel)
    reg [1:0] rel_c;
    always @(*) begin
        if (icon == r_sel)
            rel_c = 2'd0;
        else if ((r_sel != 2'd3) && (icon == r_sel + 2'd1))
            rel_c = 2'd1;
        else if ((r_sel != 2'd0) && (icon + 2'd1 == r_sel))
            rel_c = 2'd2;
        else
            rel_c = 2'd3;
    end

    // side icon center x: left 20, right 108
    reg signed [8:0] small_cx;
    always @(*) begin
        case (rel_c)
            2'd1: small_cx = 9'sd108 + r_anim;
            2'd2: small_cx = 9'sd20 + r_anim;
            default: small_cx = 9'sd0;
        endcase
    end

    wire signed [8:0] small_sum = small_cx - 9'sd12 + $signed({3'b0, col});
    wire [2:0] small_page = 3'd3 + {1'b0, page[1:0]};
    wire signed [8:0] large_sum = 9'sd40 + r_anim + $signed({3'b0, col});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            cnt         <= 10'd0;
            icon        <= 2'd0;
            page        <= 3'd0;
            col         <= 6'd0;
            render_busy <= 1'b1;
            render_req  <= 1'b1;
            // 2026-10-31：上电默认选中 PARAM（icon 1），不再默认停在 SPEED
            sel         <= 2'd1;
            r_sel       <= 2'd1;
            target      <= 2'd1;
            moving      <= 1'b0;
            anim_dir    <= 1'b1;
            anim        <= 8'sd0;
            r_anim      <= 8'sd0;
            traj_page   <= 1'b0;
            frame_on    <= 1'b0;
            scr_page    <= 1'b0;
            r_traj_page <= 1'b0;
            r_frame_on  <= 1'b0;
            r_scr_page  <= 1'b0;
            param_frame <= 4'd0;
            r_param_frame <= 4'd0;
            bg_addr     <= 10'd0;
            fb_waddr    <= 10'd0;
            fb_wdata    <= 8'd0;
        end
        else begin
            // enter PARAM page: short press on PARAM icon; long press = back
            if (knob_btn_short && !scr_page && !traj_page && !moving && (sel == 2'd1)) begin
                scr_page    <= 1'b1;
                param_frame <= 4'd0;
                render_req  <= 1'b1;
            end
            // enter trajectory page: short press on SPEED icon
            if (knob_btn_short && !scr_page && !traj_page && !moving && (sel == 2'd2)) begin
                traj_page  <= 1'b1;
                sel        <= 2'd0;
                target     <= 2'd0;
                anim       <= 8'sd0;
                moving     <= 1'b0;
                render_req <= 1'b1;
            end

            // short in trajectory browse: lock frame (active)
            if (knob_btn_short && traj_page && !frame_on && !moving) begin
                frame_on   <= 1'b1;
                render_req <= 1'b1;
            end

            // ★ 选中后再短按一次 = 取消（原来要长按）；长按仍用于退出页面
            if (knob_btn_short && traj_page && frame_on && !moving) begin
                frame_on   <= 1'b0;
                render_req <= 1'b1;
            end

            // long while browsing: back to main menu
            if (knob_btn_long && traj_page && !frame_on) begin
                traj_page  <= 1'b0;
                sel        <= 2'd2;
                target     <= 2'd0;
                anim       <= 8'sd0;
                moving     <= 1'b0;
                render_req <= 1'b1;
            end

            if (knob_btn_long && scr_page) begin
                scr_page   <= 1'b0;
                render_req <= 1'b1;
            end

            // in parameter page: right = scroll down, left = scroll up
            if (knob_step && scr_page) begin
                if (knob_dir && (param_frame < 4'd8)) begin
                    param_frame <= param_frame + 4'd1;
                    render_req  <= 1'b1;
                end
                else if (!knob_dir && (param_frame != 4'd0)) begin
                    param_frame <= param_frame - 4'd1;
                    render_req  <= 1'b1;
                end
            end

            // slide only, 2 px/frame, no intermediate size
            // ★ 11.0：轨迹页只有两项（跟随 / 正弦），梯形那项已删除
            if (knob_step && !moving && !scr_page && (!traj_page || !frame_on)) begin
                if (knob_dir && (sel != (traj_page ? 2'd1 : 2'd2))) begin

                    target     <= sel + 2'd1;
                    anim_dir   <= 1'b1;
                    anim       <= 8'sd0;
                    moving     <= 1'b1;
                    render_req <= 1'b1;
                end
                else if (!knob_dir && (sel != 2'd0)) begin
                    target     <= sel - 2'd1;
                    anim_dir   <= 1'b0;
                    anim       <= 8'sd0;
                    moving     <= 1'b1;
                    render_req <= 1'b1;
                end
            end

            if (frame_done && moving) begin
                if (anim_dir) begin
                    if (anim <= -8'sd45) begin
                        anim   <= 8'sd0;
                        sel    <= target;
                        moving <= 1'b0;
                    end
                    else
                        anim <= anim - 8'sd3;
                end
                else begin
                    if (anim >= 8'sd45) begin
                        anim   <= 8'sd0;
                        sel    <= target;
                        moving <= 1'b0;
                    end
                    else
                        anim <= anim + 8'sd3;
                end
                render_req <= 1'b1;
            end

            case (st)
                S_IDLE: begin
                    if (render_req) begin
                        render_busy <= 1'b1;
                        r_sel       <= sel;
                        r_anim      <= anim;
                        r_traj_page <= traj_page;
                        r_frame_on  <= frame_on;
                        r_scr_page  <= scr_page;
                        r_param_frame <= param_frame;
                        st          <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (frame_done) begin
                        cnt     <= 10'd0;
                        bg_addr <= 10'd0;
                        st      <= S_BG;
                    end
                end

                S_BG: begin
                    bg_addr <= cnt;
                    st      <= S_BG_W;
                end

                S_BG_W: begin
                    frm[cnt] <= bg_data;
                    if (cnt == 10'd1023) begin
                        cnt  <= 10'd0;
                        icon <= 2'd0;
                        page <= 3'd0;
                        col  <= 6'd0;
                        if (r_scr_page)
                            st <= S_COPY;
                        else
                            st <= S_SMALL_ADDR;
                    end
                    else begin
                        cnt <= cnt + 10'd1;
                        st  <= S_BG;
                    end
                end

                S_SMALL_ADDR: begin
                    st <= S_SMALL_WR;
                end

                S_SMALL_WR: begin
                    if ((rel_c == 2'd1 || rel_c == 2'd2) &&
                        (small_sum >= 0) && (small_sum < 128)) begin
                        frm[{small_page, small_sum[6:0]}] <=
                            frm[{small_page, small_sum[6:0]}] | small_data;
                    end

                    if (col == 6'd23) begin
                        col <= 6'd0;
                        if (page == 3'd2) begin
                            page <= 3'd0;
                            if (icon == 2'd2) begin
                                cnt <= 10'd0;
                                st  <= S_LARGE_ADDR;
                            end
                            else begin
                                icon <= icon + 2'd1;
                                st   <= S_SMALL_ADDR;
                            end
                        end
                        else begin
                            page <= page + 3'd1;
                            st   <= S_SMALL_ADDR;
                        end
                    end
                    else begin
                        col <= col + 6'd1;
                        st  <= S_SMALL_ADDR;
                    end
                end

                S_LARGE_ADDR: begin
                    st <= S_LARGE_WR;
                end

                S_LARGE_WR: begin
                    if ((large_sum >= 0) && (large_sum < 128)) begin
                        frm[{3'd1 + page, large_sum[6:0]}] <=
                            frm[{3'd1 + page, large_sum[6:0]}] | large_data;
                    end

                    if (col == 6'd47) begin
                        col <= 6'd0;
                        if (page == 3'd5) begin
                            cnt <= 10'd0;
                            st  <= S_COPY;
                        end
                        else begin
                            page <= page + 3'd1;
                            st   <= S_LARGE_ADDR;
                        end
                    end
                    else begin
                        col <= col + 6'd1;
                        st  <= S_LARGE_ADDR;
                    end
                end

                S_COPY: begin
                    fb_waddr <= cnt;
                    fb_wdata <= frm[cnt];
                    st       <= S_COPY_W;
                end

                S_COPY_W: begin
                    if (cnt == 10'd1023) begin
                        render_req  <= 1'b0;
                        render_busy <= 1'b0;
                        st          <= S_IDLE;
                    end
                    else begin
                        cnt <= cnt + 10'd1;
                        st  <= S_COPY;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
