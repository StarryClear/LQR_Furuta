// LQR_2.0 timing constraints（与 LQR_Furuta_Test 相同）
create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]

set_false_path -from [get_ports {sys_rst_n}]
set_false_path -from [get_ports {pwm_data}]
set_false_path -from [get_ports {e1}]
set_false_path -from [get_ports {e2}]
set_false_path -from [get_ports {knob_a}]
set_false_path -from [get_ports {knob_b}]
set_false_path -from [get_ports {knob_btn}]
set_false_path -from [get_ports {key_start}]
set_false_path -from [get_ports {adc_dout}]
set_false_path -from [get_ports {uart_rx}]
