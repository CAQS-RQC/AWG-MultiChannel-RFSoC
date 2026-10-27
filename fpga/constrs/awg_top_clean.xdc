# awg_top_clean.xdc -- constraints for the full AWG block-design wrapper (plain XDC commands only).
# Use in IMPLEMENTATION only: the clocks below exist once the RFDC IP constraints are loaded.
#   set_property USED_IN_SYNTHESIS false [get_files awg_top_clean.xdc]

# external trigger pin (Pmod0 pin 1)
set_property IOSTANDARD LVCMOS18 [get_ports control_trigger_0]
set_property PACKAGE_PIN AG17    [get_ports control_trigger_0]

# pmod_out_0 test output, Pmod0 pin 0 (board file part0_pins.xml: pmod0_0 = AF16). Leave it unconnected.
set_property IOSTANDARD LVCMOS18 [get_ports pmod_out_0]
set_property PACKAGE_PIN AF16    [get_ports pmod_out_0]

# PS clock and DAC clocks are asynchronous
set_clock_groups -asynchronous -group [get_clocks clk_pl_0] -group [get_clocks RFDAC0_CLK]
set_clock_groups -asynchronous -group [get_clocks clk_pl_0] -group [get_clocks RFDAC2_CLK]
