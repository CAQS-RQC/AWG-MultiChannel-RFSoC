# adc_capture.xdc  -- add to constrs_1 after running tcl/add_adc_capture.tcl
# The ADC capture clock domain (RFDC clk_adc, 614.4 MHz) and the PS clock (pl_clk0) only meet in the async
# FIFO inside axis_clock_converter_adc (and the AXI-Lite crossing), so they are declared asynchronous.
# Guarded: if the RFDC-generated clock has a different name, nothing happens. Check the clock names in
# Report Clock Networks / Timing Summary after synthesis and adjust.
if {[llength [get_clocks -quiet RFADC0_CLK]] > 0} {
    set_clock_groups -asynchronous -group [get_clocks clk_pl_0] -group [get_clocks RFADC0_CLK]
    if {[llength [get_clocks -quiet RFDAC0_CLK]] > 0} {
        set_clock_groups -asynchronous -group [get_clocks RFADC0_CLK] -group [get_clocks RFDAC0_CLK]
    }
    if {[llength [get_clocks -quiet RFDAC2_CLK]] > 0} {
        set_clock_groups -asynchronous -group [get_clocks RFADC0_CLK] -group [get_clocks RFDAC2_CLK]
    }
}
