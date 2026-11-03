//======================================================================
//  pwm_capture.v  --  SA100 PWM 高电平/周期捕获（带 3 点中值滤波）
//
//  pwm_in 必须已经同步到 clk（用 sync_ff）。
//
//  ★ 为什么需要中值滤波（2026-10-30 实测 + 6.0 工程记录）：
//    电机一加电，PWM 的 EMI 会串到 SA100 信号线上，产生"幽灵脉冲"，
//    把某一帧的高电平从 ~19000 拍打断成 ~500 拍。按占空比公式算，
//    这一帧会解出 position = 0（或固定错误角度）。
//    角度本身影响不大，但角速度是差分，一帧跳 120° 会被算成上万 °/s，
//    经 D 项放大直接把输出顶到满量程。
//    3 点中值滤波把单帧野值丢掉，真实快变只延迟 1 帧（1.17ms）。
//
//  TIMEOUT 5ms：超过就置 lost（SA100 周期约 1.18ms）
//======================================================================
`timescale 1ns/1ps

module pwm_capture #(
    parameter integer TIMEOUT = 250_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pwm_in,
    output reg  [31:0] high_ticks,
    output reg  [31:0] period_ticks,
    output reg         valid,
    output reg         lost
);

    reg        pwm_d;
    wire       rise = pwm_in & ~pwm_d;
    wire       fall = ~pwm_in & pwm_d;

    reg [31:0] timer;
    reg [31:0] high_cnt;
    reg [31:0] h1, h2;        // 上两帧的高电平
    reg [1:0]  fill;          // 已采到几帧（启动时旁路中值）
    reg        high_seen;

    // 2026-10-30 时序：3 点中值拆成两级流水，切断 h1/h2/median 长组合链
    reg [31:0] m_a, m_b, m_c;
    reg [31:0] m_lo, m_hi;
    reg [1:0]  m_stage;
    wire [31:0] m_med = (m_c < m_lo) ? m_lo :
                        (m_c > m_hi) ? m_hi : m_c;

    function [31:0] med3;
        input [31:0] a, b, c;
        reg   [31:0] lo, hi;
        begin
            lo   = (a < b) ? a : b;
            hi   = (a < b) ? b : a;
            med3 = (c < lo) ? lo : (c > hi) ? hi : c;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_d        <= 1'b0;
            timer        <= 32'd0;
            high_cnt     <= 32'd0;
            h1           <= 32'd0;
            h2           <= 32'd0;
            fill         <= 2'd0;
            high_ticks   <= 32'd0;
            period_ticks <= 32'd0;
            valid        <= 1'b0;
            lost         <= 1'b1;
            high_seen    <= 1'b0;
            m_a          <= 32'd0;
            m_b          <= 32'd0;
            m_c          <= 32'd0;
            m_lo         <= 32'd0;
            m_hi         <= 32'd0;
            m_stage      <= 2'd0;
        end
        else begin
            pwm_d <= pwm_in;
            valid <= 1'b0;

            if (rise) begin
                period_ticks <= timer;
                if (high_seen) begin
                    if (fill >= 2'd2) begin
                        m_a     <= high_cnt;
                        m_b     <= h1;
                        m_c     <= h2;
                        m_stage <= 2'd1;
                    end
                    else begin
                        high_ticks <= high_cnt;
                        valid      <= 1'b1;
                    end
                end
                timer     <= 32'd1;
                high_seen <= 1'b0;
                lost      <= 1'b0;
            end
            else if (fall) begin
                high_cnt  <= timer;
                h1        <= high_cnt;
                h2        <= h1;
                if (fill != 2'd3) fill <= fill + 2'd1;
                high_seen <= 1'b1;
                if (timer < 32'd10_000_000)
                    timer <= timer + 32'd1;
            end
            else begin
                if (timer < TIMEOUT)
                    timer <= timer + 32'd1;
                else
                    lost <= 1'b1;
            end

            // 两级中值流水：先算 lo/hi，再取中值；valid 延后两拍
            if (m_stage == 2'd1) begin
                m_lo    <= (m_a < m_b) ? m_a : m_b;
                m_hi    <= (m_a < m_b) ? m_b : m_a;
                m_stage <= 2'd2;
            end
            else if (m_stage == 2'd2) begin
                high_ticks <= m_med;
                valid      <= 1'b1;
                m_stage    <= 2'd0;
            end
        end
    end

endmodule
