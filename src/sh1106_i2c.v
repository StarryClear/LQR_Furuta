//======================================================================
//  sh1106_i2c.v  --  0.96"/1.3" 128x64 OLED（I2C）显示控制器
//
//  源自 1.0 工程的 sh1106_i2c.v（已验证上板），改动：
//    · 帧数据不再来自 emoji_rom，而是从显存 oled_fb 顺序读 1024 字节
//    · 增加 frame_done（一帧结束脉冲）与 render_busy 握手：
//      每帧结束后停在总线空闲，等渲染器把新画面写完再发下一帧，
//      避免"边写显存边发送"造成撕裂
//    · 初始化/页命令/地址探测/200 帧重初始化全部保留
//
//  ★ 0.96 寸 128x64 基本都是 SSD1306；1.3 寸常见 SH1106。
//    两者命令集几乎一样，区别只有两点：
//      · SSD1306：发 0x20 0x02（页寻址），列偏移 X_OFF = 0
//      · SH1106 ：不发 0x20（它没这条），显存 132 列，X_OFF = 2
//    本模块默认按 SSD1306 配（X_OFF=0）。换 1.3 寸 SH1106 时把 X_OFF 改 2。
//
//  · 纯写、无读；SDA 开漏（模块上有上拉），SCL 推挽
//  · 上电等 80ms 再初始化（OLED 内部复位要求）
//  · 一帧 8 页：命令事务（页/列地址 3 字节）+ 数据事务（0x40 + 128 字节）
//  · 200kHz 时一帧约 50ms（20fps）；模块上拉弱的话把 SCL_HZ 降到 100k
//======================================================================
`timescale 1ns/1ps

module sh1106_i2c #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SCL_HZ   = 200_000,
    parameter [6:0]   DEV_ADDR = 7'h3C,
    parameter [7:0]   X_OFF    = 8'd0       // SSD1306=0；SH1106=2（132 列偏移）
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         scl,
    inout  wire        sda,
    output wire [9:0]  fb_addr,             // 显存读地址 {page, col}
    input  wire [7:0]  fb_data,             // 1 拍延迟
    output reg         frame_done,          // 一帧发完，1 拍脉冲
    input  wire        render_busy          // 渲染中：暂停下一帧
);

    localparam integer QDIV = (CLK_FREQ / (SCL_HZ * 4) < 1) ? 1 : (CLK_FREQ / (SCL_HZ * 4));

    //------------------------------------------------------------------
    // 0. SDA 开漏
    //------------------------------------------------------------------
    reg  sda_oe;      // 1 = 驱动，0 = 释放（ACK 那一拍释放）
    reg  sda_drv;
    assign sda = sda_oe ? sda_drv : 1'bz;
    wire sda_i = sda;

    //------------------------------------------------------------------
    // 1. 上电延时 80ms
    //------------------------------------------------------------------
    reg [22:0] pw_cnt;
    reg        started;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pw_cnt  <= 23'd0;
            started <= 1'b0;
        end
        else if (!started) begin
            if (pw_cnt == 23'd4_000_000) started <= 1'b1;
            else                         pw_cnt  <= pw_cnt + 23'd1;
        end
    end

    //------------------------------------------------------------------
    // 2. 位引擎：一次发 1 个字节 / START / STOP
    //------------------------------------------------------------------
    localparam [2:0] B_IDLE  = 3'd0,
                     B_START = 3'd1,
                     B_BIT   = 3'd2,
                     B_ACK   = 3'd3,
                     B_STOP  = 3'd4,
                     B_HOLD  = 3'd5;

    reg [2:0]  bst;
    reg [1:0]  bph;          // 位内四相
    reg [7:0]  qcnt;
    reg [7:0]  sh;
    reg [3:0]  bitn;
    reg        done;         // 1 拍脉冲
    reg        busy;
    reg        ack_r;

    reg        req;
    reg [1:0]  req_kind;     // 0=字节 1=START 2=STOP
    wire [7:0] byte_src;     // 组合：本拍要发的字节

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bst <= B_IDLE; bph <= 2'd0; qcnt <= 8'd0;
            scl <= 1'b1; sda_oe <= 1'b0; sda_drv <= 1'b1;
            sh <= 8'hFF; bitn <= 4'd0;
            done <= 1'b0; busy <= 1'b0; ack_r <= 1'b0;
        end
        else begin
            done <= 1'b0;

            if (qcnt >= QDIV - 1) begin
                qcnt <= 8'd0;
                bph  <= bph + 2'd1;

                case (bst)
                    B_IDLE: begin
                        scl    <= 1'b1;
                        sda_oe <= 1'b1;
                        sda_drv <= 1'b1;
                        if (req) begin
                            sh    <= byte_src;
                            bitn  <= 4'd0;
                            busy  <= 1'b1;
                            bph   <= 2'd0;
                            bst   <= (req_kind == 2'd1) ? B_START :
                                     (req_kind == 2'd2) ? B_STOP  : B_BIT;
                        end
                    end

                    B_START: begin
                        case (bph)
                            2'd0: begin scl <= 1'b1; sda_oe <= 1'b1; sda_drv <= 1'b1; end
                            2'd1: scl <= 1'b1;
                            2'd2: sda_drv <= 1'b0;      // SCL 高时拉低 SDA
                            2'd3: begin
                                scl  <= 1'b0;
                                bst  <= B_HOLD;
                                busy <= 1'b0;
                                done <= 1'b1;
                            end
                        endcase
                    end

                    B_BIT: begin
                        case (bph)
                            2'd0: scl <= 1'b0;
                            2'd1: begin
                                sda_oe  <= 1'b1;
                                sda_drv <= sh[7];
                            end
                            2'd2: scl <= 1'b1;                  // 上升沿，从机采样
                            2'd3: begin
                                scl <= 1'b0;
                                sh  <= {sh[6:0], 1'b0};
                                bitn <= bitn + 4'd1;
                                if (bitn == 4'd7) begin
                                    bst <= B_ACK;
                                    bph <= 2'd0;
                                end
                            end
                        endcase
                    end

                    B_ACK: begin
                        case (bph)
                            2'd0: begin scl <= 1'b0; sda_oe <= 1'b0; end  // 释放 SDA 给从机 ACK
                            2'd1: scl <= 1'b1;
                            2'd2: ack_r <= sda_i;
                            2'd3: begin
                                scl  <= 1'b0;
                                bst  <= B_HOLD;
                                busy <= 1'b0;
                                done <= 1'b1;
                            end
                        endcase
                    end

                    B_STOP: begin
                        case (bph)
                            2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_drv <= 1'b0; end
                            2'd1: scl <= 1'b1;
                            2'd2: sda_drv <= 1'b1;                     // SCL 高时拉高 SDA = STOP
                            2'd3: begin
                                scl  <= 1'b1;
                                bst  <= B_IDLE;
                                busy <= 1'b0;
                                done <= 1'b1;
                            end
                        endcase
                    end

                    B_HOLD: begin
                        scl <= 1'b0;
                        if (req) begin
                            sh   <= byte_src;
                            bitn <= 4'd0;
                            bph  <= 2'd0;
                            bst  <= (req_kind == 2'd2) ? B_STOP : B_BIT;
                        end
                    end

                    default: bst <= B_IDLE;
                endcase
            end
            else
                qcnt <= qcnt + 8'd1;
        end
    end

    //------------------------------------------------------------------
    // 3. 事务序列
    //------------------------------------------------------------------
    localparam [3:0] X_P_START = 4'd0,
                     X_P_ADDR  = 4'd1,
                     X_P_STOP  = 4'd2,
                     X_START   = 4'd3,
                     X_ADDR    = 4'd4,
                     X_CTRL    = 4'd5,
                     X_DATA    = 4'd6,
                     X_STOP    = 4'd7,
                     X_NEXT    = 4'd8,
                     X_WAIT    = 4'd9;

    localparam [1:0] TX_INIT = 2'd0, TX_PCMD = 2'd1, TX_DATA = 2'd2;

    reg [3:0]  xs;
    reg [1:0]  ttype;
    reg [7:0]  txn_n;        // 本事务字节数
    reg [7:0]  txn_i;
    reg [2:0]  page;
    reg [7:0]  frame_cnt;

    //---- 地址自动探测：先试 0x3C，NACK 就换 0x3D ----
    reg        p_try;
    reg [6:0]  probe_addr;
    reg [6:0]  dev_addr_r;

    //---- 渲染握手 ----
    reg        seen_busy;
    reg [19:0] wdog;

    //---- 初始化命令表（27 条，与逻辑派官方例程逐个一致）----
    function [7:0] init_byte(input [4:0] i);
        case (i)
            5'd0:  init_byte = 8'hAE;   // 关显示
            5'd1:  init_byte = 8'h00;   // 列地址低
            5'd2:  init_byte = 8'h10;   // 列地址高
            5'd3:  init_byte = 8'h40;   // 起始行
            5'd4:  init_byte = 8'hB0;   // 页地址
            5'd5:  init_byte = 8'h81;   // 对比度
            5'd6:  init_byte = 8'hFF;
            5'd7:  init_byte = 8'hA1;   // 左右方向
            5'd8:  init_byte = 8'hA6;   // 正常显示
            5'd9:  init_byte = 8'hA8;   // 复用率
            5'd10: init_byte = 8'h3F;
            5'd11: init_byte = 8'hC8;   // 上下方向
            5'd12: init_byte = 8'hD3;   // 显示偏移
            5'd13: init_byte = 8'h00;
            5'd14: init_byte = 8'hD5;   // 时钟分频
            5'd15: init_byte = 8'h80;
            5'd16: init_byte = 8'hD8;   // COM 配置
            5'd17: init_byte = 8'h05;
            5'd18: init_byte = 8'hD9;   // 预充电
            5'd19: init_byte = 8'hF1;
            5'd20: init_byte = 8'hDA;   // COM 引脚
            5'd21: init_byte = 8'h12;
            5'd22: init_byte = 8'hDB;   // VCOMH
            5'd23: init_byte = 8'h30;
            5'd24: init_byte = 8'h8D;   // 充电泵开
            5'd25: init_byte = 8'h14;
            5'd26: init_byte = 8'hAF;   // 开显示
            default: init_byte = 8'h00;
        endcase
    endfunction

    //---- 页命令字节（页地址 + 列地址）----
    function [7:0] pcmd_byte(input [7:0] i, input [2:0] pg);
        case (i)
            8'd0: pcmd_byte = 8'hB0 | {5'd0, pg};
            8'd1: pcmd_byte = X_OFF;
            8'd2: pcmd_byte = 8'h10 | {4'd0, X_OFF[7:4]};
            default: pcmd_byte = 8'h00;
        endcase
    endfunction

    wire [7:0] ctrl_byte = (ttype == TX_DATA) ? 8'h40 : 8'h00;

    wire [6:0] cur_addr  = (xs == X_P_ADDR) ? probe_addr : dev_addr_r;
    wire [7:0] addr_byte = {cur_addr, 1'b0};

    assign byte_src =
        ((xs == X_ADDR) || (xs == X_P_ADDR)) ? addr_byte :
        (xs == X_CTRL) ? ctrl_byte :
        (ttype == TX_INIT) ? init_byte(txn_i[4:0]) :
        (ttype == TX_PCMD) ? pcmd_byte(txn_i, page) :
        fb_data;

    // 显存地址：数据阶段按 {页, 列} 顺序读
    assign fb_addr = {page, txn_i[6:0]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xs          <= X_P_START;
            req         <= 1'b0;
            req_kind    <= 2'd0;
            ttype       <= TX_INIT;
            txn_n       <= 8'd27;
            txn_i       <= 8'd0;
            page        <= 3'd0;
            frame_cnt   <= 8'd0;
            p_try       <= 1'b0;
            probe_addr  <= 7'h3C;
            dev_addr_r  <= 7'h3C;
            frame_done  <= 1'b0;
            seen_busy   <= 1'b0;
            wdog        <= 20'd0;
        end
        else begin
            frame_done <= 1'b0;

            if (done) begin
                req <= 1'b0;
                case (xs)
                    X_P_START: xs <= X_P_ADDR;
                    X_P_ADDR:  xs <= X_P_STOP;
                    X_P_STOP: begin
                        if (ack_r && !p_try) begin
                            p_try      <= 1'b1;
                            probe_addr <= 7'h3D;
                            xs         <= X_P_START;
                        end
                        else begin
                            dev_addr_r <= probe_addr;
                            xs         <= X_START;
                        end
                    end
                    X_START: xs <= X_ADDR;
                    X_ADDR:  xs <= X_CTRL;
                    X_CTRL:  xs <= (txn_n == 8'd0) ? X_STOP : X_DATA;
                    X_DATA: begin
                        if (txn_i + 8'd1 >= txn_n) xs <= X_STOP;
                        txn_i <= txn_i + 8'd1;
                    end
                    X_STOP:  xs <= X_NEXT;
                    default: ;
                endcase
            end
            else if (!req && started) begin
                case (xs)
                    X_P_START: begin req_kind <= 2'd1; req <= 1'b1; end
                    X_P_ADDR:  begin req_kind <= 2'd0; req <= 1'b1; end
                    X_P_STOP:  begin req_kind <= 2'd2; req <= 1'b1; end
                    X_START:   begin req_kind <= 2'd1; req <= 1'b1; end
                    X_ADDR:    begin req_kind <= 2'd0; req <= 1'b1; end
                    X_CTRL:    begin req_kind <= 2'd0; req <= 1'b1; end
                    X_DATA:    begin req_kind <= 2'd0; req <= 1'b1; end
                    X_STOP:    begin req_kind <= 2'd2; req <= 1'b1; end

                    X_NEXT: begin
                        txn_i <= 8'd0;
                        case (ttype)
                            TX_INIT: begin
                                ttype <= TX_PCMD;
                                txn_n <= 8'd3;
                                page  <= 3'd0;
                                xs    <= X_START;
                            end
                            TX_PCMD: begin
                                ttype <= TX_DATA;
                                txn_n <= 8'd128;
                                xs    <= X_START;
                            end
                            default: begin                 // 一页数据发完
                                if (page != 3'd7) begin
                                    // 还有下一页
                                    ttype <= TX_PCMD;
                                    txn_n <= 8'd3;
                                    page  <= page + 3'd1;
                                    xs    <= X_START;
                                end
                                else begin
                                    // 一帧结束：交给渲染器，然后停一拍再发下一帧
                                    frame_cnt  <= frame_cnt + 8'd1;
                                    frame_done <= 1'b1;

                                    if (frame_cnt >= 8'd199) begin
                                        // 200 帧重初始化：模块上电晚/被半截命令搞乱时自恢复
                                        frame_cnt <= 8'd0;
                                        ttype     <= TX_INIT;
                                        txn_n     <= 8'd27;
                                    end
                                    else begin
                                        ttype <= TX_PCMD;
                                        txn_n <= 8'd3;
                                    end
                                    page      <= 3'd0;
                                    seen_busy <= 1'b0;
                                    wdog      <= 20'd0;
                                    xs        <= X_WAIT;
                                end
                            end
                        endcase
                    end

                    X_WAIT: begin
                        // 等渲染器把新画面写完；从没见到 busy 就等 20us 放行，
                        // 兜底 10ms 超时，避免总线卡死
                        if (render_busy)
                            seen_busy <= 1'b1;

                        if ((seen_busy && !render_busy) || (!seen_busy && wdog[9]) || wdog[19])
                            xs <= X_START;
                        else
                            wdog <= wdog + 20'd1;
                    end

                    default: xs <= X_START;
                endcase
            end
        end
    end

endmodule
