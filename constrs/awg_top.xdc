# awg_top.xdc -- constraints for the FULL design (block-design wrapper as top).
# Taken from constrs/constrs.xdc, keeping only what applies at wrapper level.
# NOTE: XDC files do not support Tcl 'if' (Vivado ignores those lines with a CRITICAL WARNING),
# so everything here is plain commands. The clocks below exist after the RFDC IP constraints are
# loaded, so this file should be used in IMPLEMENTATION only:
#   set_property USED_IN_SYNTHESIS false [get_files awg_top.xdc]

# external trigger pin
set_property IOSTANDARD LVCMOS18 [get_ports control_trigger_0]
set_property PACKAGE_PIN AG17    [get_ports control_trigger_0]

# PS clock vs DAC clocks are asynchronous (same two lines as the original constrs.xdc)
set_clock_groups -asynchronous -group [get_clocks clk_pl_0] -group [get_clocks RFDAC0_CLK]
set_clock_groups -asynchronous -group [get_clocks clk_pl_0] -group [get_clocks RFDAC2_CLK]

# The ADC capture domain (RFADC0_CLK, 307.2 MHz) already meets timing with its normal
# clock-converter constraints, so it needs no extra exceptions.
