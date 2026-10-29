#!/bin/sh
# Behavioural check of fpga/adc/hdl/adc_capture.v with Icarus Verilog (apt install iverilog).
# xpm_stub.v stands in for the Xilinx XPM CDC macros. Prints PASS or FAIL.
cd "$(dirname "$0")"
iverilog -g2012 -o tb.vvp tb_adc_capture.v xpm_stub.v ../hdl/adc_capture.v && vvp -n tb.vvp | grep -E "PASS|FAIL|ERROR|constant"
rm -f tb.vvp
