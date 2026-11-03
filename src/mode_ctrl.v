//======================================================================
//  mode_ctrl.v  --  LQR 启停控制（R14 按键）
//
//  R14（低有效）：
//    短按      -> 切换 run（LQR 闭环开关）
//    长按 1 s  -> 编码器基准清零
//  串口 '0'..'9' 仍然可以设 mode（其它测试用）
//
//  ★ 2026-10-30 改：原来短按是"模式+1"，现在改成 LQR 启停开关，
//    上板调试时由人按 R14 决定什么时候闭环比串口方便。
//======================================================================
`timescale 1ns/1ps

module mode_ctrl #(
    parameter integer CLK_FREQ = 50_000_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       key_in,      // 低有效
    input  wire [3:0] ser_mode,
    input  wire       ser_set,
    input  wire       ser_clr,
    output reg  [3:0] mode,
    output reg        clr,
    output reg        run,         // 1 = LQR 闭环
    output reg        trig          // 兼容旧逻辑，未用
);

    localparam integer DB   = CLK_FREQ / 100;   // 10 ms 消抖
    localparam integer LONG = CLK_FREQ;         // 1 s 长按

    reg [1:0]  k_sync;
    reg        k_st;
    reg        k_st_d;
    reg [19:0] db_cnt;
    reg [26:0] press_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            k_sync    <= 2'b11;
            k_st      <= 1'b1;
            k_st_d    <= 1'b1;
            db_cnt    <= 20'd0;
            press_cnt <= 27'd0;
            mode      <= 4'd0;
            clr       <= 1'b0;
            run       <= 1'b0;
            trig      <= 1'b0;
        end
        else begin
            clr    <= 1'b0;
            trig   <= 1'b0;
            k_sync <= {k_sync[0], key_in};

            // 消抖
            if (k_sync[1] != k_st) begin
                if (db_cnt == DB - 1) begin
                    k_st   <= k_sync[1];
                    db_cnt <= 20'd0;
                end
                else
                    db_cnt <= db_cnt + 20'd1;
            end
            else
                db_cnt <= 20'd0;

            k_st_d <= k_st;

            // 按住计时（下降沿开始）
            if (k_st_d && !k_st)
                press_cnt <= 27'd0;
            else if (!k_st && press_cnt < LONG)
                press_cnt <= press_cnt + 27'd1;

            // 串口设模式 / 清零
            if (ser_set)
                mode <= ser_mode;
            else if (ser_clr)
                clr <= 1'b1;
            // 松手：判定短按还是长按
            else if (!k_st_d && k_st) begin
                if (press_cnt >= LONG)
                    clr <= 1'b1;            // 长按：编码器清零
                else
                    run <= ~run;            // 短按：切换 LQR 启停
            end
        end
    end

endmodule
