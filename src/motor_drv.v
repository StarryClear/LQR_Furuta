//======================================================================
//  motor_drv.v  --  DRV8874 PH/EN mode, 20 kHz PWM
//
//  pwm -> EN/IN1, ph -> PH/IN2.
//  en=0 or duty=0 -> EN low -> coast.
//======================================================================
`timescale 1ns/1ps

module motor_drv #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer PWM_HZ   = 20_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        dir,
    input  wire [15:0] duty,
    output reg         pwm,
    output reg         ph
);

    localparam integer PERIOD = CLK_FREQ / PWM_HZ; // 2500

    reg [15:0] cnt;
    reg [15:0] duty_lat;

    wire [15:0] duty_use = (duty > PERIOD) ? PERIOD[15:0] : duty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 16'd0;
            duty_lat <= 16'd0;
            pwm      <= 1'b0;
            ph       <= 1'b0;
        end
        else begin
            if (cnt >= PERIOD - 1) begin
                cnt      <= 16'd0;
                duty_lat <= duty_use;
            end
            else
                cnt <= cnt + 16'd1;

            pwm <= en && (cnt < duty_lat);
            ph  <= dir;
        end
    end

endmodule
