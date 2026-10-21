# add_adc_capture.tcl  (v2)
#
# Adds ONE ADC channel + a DMA capture path to the open AWG block design.
# It does not touch the DAC / AWG logic, and it leaves every existing address as it is.
#
#   ADC slice -> packet_generator_adc -> axis_clock_converter_adc -> axi_dma_adc (S2MM) -> PS DDR (via axi_smc)
#
# ADC mode = the configuration used by the RFSoC4x2 base overlay and the Strathclyde SAM design
# (4.9152 GSps, I/Q output with a 0 Hz mixer, decimation 2 -> 2.4576 GSps per stream, 8 samples per
# beat, 307.2 MHz stream clock). Only the I stream is captured; the Q stream is left unconnected.
#
# How to use (Vivado 2024.1):
#   1. Back up the vivado_awg folder first.
#   2. Open a CLEAN block design (designCompiled2024) so it is the active BD.
#   3. Tcl console:
#        source C:/Users/Shogun/Documents/GitCloned/AWG-MultiChannel-RFSoC/tcl/add_adc_capture.tcl
#   4. If it stops with an ERROR before "STEP 3", nothing was built: close the BD WITHOUT saving.
#      Send me the console output (or the file tcl/add_adc_capture.log written next to this script).

set ADC_SLICE "00"        ;# 00 -> vin0_01, 02 -> vin0_23, 20 -> vin2_01, 22 -> vin2_23

set script_dir [file dirname [file normalize [info script]]]
set repo_dir   [file normalize [file join $script_dir ..]]
set ip_dir     [file join $repo_dir ip]
set logf [open [file join $script_dir add_adc_capture.log] w]
proc say {msg} { global logf; puts $msg; puts $logf $msg; flush $logf }

set tile  [string index $ADC_SLICE 0]
set blk   [string index $ADC_SLICE 1]
set vin   [expr {$blk < 2 ? "vin${tile}_01" : "vin${tile}_23"}]
set rfdc_name usp_rf_data_converter_0

# ------------------------------------------------------------ STEP 0: sanity checks (no changes)
current_bd_design
set rfdc [get_bd_cells -quiet $rfdc_name]
if {[llength $rfdc] == 0} { close $logf; error "Cell $rfdc_name not found: open the AWG block design first." }
if {[llength [get_bd_cells -quiet axi_dma_adc]] != 0} { close $logf; error "axi_dma_adc already exists: patch already applied (or BD not clean)." }
foreach c {axi_smc ps8_0_axi_periph zynq_ultra_ps_e_0 rst_ps8_0_99M} {
    if {[llength [get_bd_cells -quiet $c]] == 0} { close $logf; error "Expected cell $c is missing in this block design." }
}
say "INFO: patching BD '[current_bd_design]' with ADC slice $ADC_SLICE ($vin), RFDC tile $tile"
if {[current_bd_design] ne "designCompiled2024"} { say "WARNING: active BD is not designCompiled2024 - make sure this is the one you want to build." }

set repos [get_property ip_repo_paths [current_project]]
if {[lsearch -exact $repos $ip_dir] < 0} { set_property ip_repo_paths [concat $repos [list $ip_dir]] [current_project] }
update_ip_catalog -rebuild
if {[llength [get_ipdefs -quiet xilinx.com:ip:packet_generator:1.0]] == 0} { close $logf; error "packet_generator IP not found in $ip_dir" }

# ------------------------------------------------------------ STEP 1: RFDC ADC configuration
# set_property -dict is all-or-nothing: if any value is rejected the previous config is restored.
array set want [list \
    CONFIG.ADC_Slice${ADC_SLICE}_Enable     true \
    CONFIG.ADC${tile}_Sampling_Rate         4.9152 \
    CONFIG.ADC${tile}_PLL_Enable            true \
    CONFIG.ADC${tile}_Refclk_Freq           491.520 \
    CONFIG.ADC${tile}_Outclk_Freq           307.200 \
    CONFIG.ADC_Data_Type${ADC_SLICE}        1 \
    CONFIG.ADC_Data_Width${ADC_SLICE}       8 \
    CONFIG.ADC_Decimation_Mode${ADC_SLICE}  2 \
    CONFIG.ADC_Mixer_Type${ADC_SLICE}       2 \
    CONFIG.ADC_Mixer_Mode${ADC_SLICE}       0 ]
set cfg {}
foreach k [array names want] { lappend cfg $k $want($k) }
if {[catch {set_property -dict $cfg $rfdc} err]} {
    say "ERROR: RFDC rejected the ADC settings: $err"
    foreach k [lsort [array names want]] {
        catch { say [format "  %-34s wanted %-8s valid: %s" $k $want($k) [list_property_value $k $rfdc]] }
    }
    close $logf
    error "STOPPED at step 1. Nothing else was changed. Close the BD without saving and send me the console text."
}
say "INFO: RFDC ADC settings as read back:"
set bad 0
foreach k [lsort [array names want]] {
    set got [get_property $k $rfdc]
    if {[string is double -strict $got] && [string is double -strict $want($k)]} {
        set ok [expr {abs($got - $want($k)) < 1e-9}]
    } else { set ok [expr {[string tolower $got] eq [string tolower $want($k)]}] }
    say [format "  %-34s %-10s %s" $k $got [expr {$ok ? "ok" : "<-- MISMATCH (wanted $want($k))"}]]
    if {!$ok} { incr bad }
}
set mpin [get_bd_intf_pins -quiet $rfdc_name/m${ADC_SLICE}_axis]
if {[llength $mpin] == 0} { incr bad; say "  stream m${ADC_SLICE}_axis: MISSING" } else {
    set nb [get_property CONFIG.TDATA_NUM_BYTES $mpin]
    say "  m${ADC_SLICE}_axis TDATA_NUM_BYTES = $nb (need 16 = 8 samples x 16 bit)"
    if {$nb != 16} { incr bad }
}
foreach p [list clk_adc${tile} m${tile}_axis_aclk m${tile}_axis_aresetn] {
    if {[llength [get_bd_pins -quiet $rfdc_name/$p]] == 0} { incr bad; say "  pin $p: MISSING" }
}
foreach p [list adc${tile}_clk $vin] {
    if {[llength [get_bd_intf_pins -quiet $rfdc_name/$p]] == 0} { incr bad; say "  interface pin $p: MISSING" }
}
if {$bad} { close $logf; error "STOPPED at step 1: ADC readback does not match. Close the BD without saving and send me the console text." }
say "INFO: ADC stream clock clk_adc${tile} = [expr {[get_property CONFIG.FREQ_HZ [get_bd_pins $rfdc_name/clk_adc${tile}]]/1e6}] MHz"

# ------------------------------------------------------------ STEP 2: external ports
say "STEP 2: external ports"
set adc_clk_port [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 adc${tile}_clk_1]
set_property CONFIG.FREQ_HZ 491520000 $adc_clk_port
connect_bd_intf_net $adc_clk_port [get_bd_intf_pins $rfdc_name/adc${tile}_clk]
set vin_port [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_analog_io_rtl:1.0 ${vin}_1]
connect_bd_intf_net $vin_port [get_bd_intf_pins $rfdc_name/$vin]

# ------------------------------------------------------------ STEP 3: capture chain
say "STEP 3: capture chain"
set pgen [create_bd_cell -type ip -vlnv xilinx.com:ip:packet_generator:1.0 packet_generator_adc]
set_property CONFIG.C_S_AXIS_DATA_WIDTH 128 $pgen
set cc [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_clock_converter:1.1 axis_clock_converter_adc]
set_property CONFIG.SYNCHRONIZATION_STAGES 5 $cc
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_adc]
set_property -dict [list \
    CONFIG.c_addr_width 64 \
    CONFIG.c_include_mm2s 0 \
    CONFIG.c_include_sg 0 \
    CONFIG.c_s2mm_burst_size 256 \
    CONFIG.c_sg_length_width 26] $dma
set rst_adc [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_adc]

connect_bd_intf_net [get_bd_intf_pins $rfdc_name/m${ADC_SLICE}_axis] [get_bd_intf_pins $pgen/S_AXIS_RE]
connect_bd_intf_net [get_bd_intf_pins $pgen/M_AXIS_RE]               [get_bd_intf_pins $cc/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins $cc/M_AXIS]                    [get_bd_intf_pins $dma/S_AXIS_S2MM]

set_property CONFIG.NUM_SI 3 [get_bd_cells axi_smc]
connect_bd_intf_net [get_bd_intf_pins $dma/M_AXI_S2MM] [get_bd_intf_pins axi_smc/S02_AXI]

set_property CONFIG.NUM_MI 10 [get_bd_cells ps8_0_axi_periph]
connect_bd_intf_net [get_bd_intf_pins ps8_0_axi_periph/M08_AXI] [get_bd_intf_pins $dma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins ps8_0_axi_periph/M09_AXI] [get_bd_intf_pins $pgen/S_AXI]

# ------------------------------------------------------------ STEP 4: clocks and resets
say "STEP 4: clocks and resets"
set ps_clk   [get_bd_pins zynq_ultra_ps_e_0/pl_clk0]
set ps_rstn  [get_bd_pins rst_ps8_0_99M/peripheral_aresetn]
set adc_clk  [get_bd_pins $rfdc_name/clk_adc${tile}]
connect_bd_net $ps_clk  [get_bd_pins $dma/s_axi_lite_aclk] [get_bd_pins $dma/m_axi_s2mm_aclk] [get_bd_pins $cc/m_axis_aclk]
connect_bd_net $ps_rstn [get_bd_pins $dma/axi_resetn] [get_bd_pins $cc/m_axis_aresetn]
connect_bd_net $adc_clk [get_bd_pins $rfdc_name/m${tile}_axis_aclk] [get_bd_pins $pgen/aclk] \
                        [get_bd_pins $cc/s_axis_aclk] [get_bd_pins $rst_adc/slowest_sync_clk] \
                        [get_bd_pins ps8_0_axi_periph/M09_ACLK]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins $rst_adc/ext_reset_in]
connect_bd_net [get_bd_pins $rst_adc/peripheral_aresetn] [get_bd_pins $rfdc_name/m${tile}_axis_aresetn] \
                        [get_bd_pins $pgen/aresetn] [get_bd_pins $cc/s_axis_aresetn] \
                        [get_bd_pins ps8_0_axi_periph/M09_ARESETN]

# ------------------------------------------------------------ STEP 5: addresses, validate, save
say "STEP 5: addresses / validate / save"
assign_bd_address   ;# only assigns segments that have no address yet
set rc [catch {validate_bd_design} verr]
save_bd_design
say ""
say "=============================================================="
if {$rc} { say "VALIDATION FAILED: $verr" } else { say "Block design validated." }
say "Expected CRITICAL WARNING: packet_generator_adc/s_axis_im_tvalid tied to 0 (we only use the I stream)."
say "Any other CRITICAL WARNING about TDATA_NUM_BYTES means the ADC config is wrong - tell me."
say "Next: Address Editor check (existing addresses unchanged), regenerate output products,"
say "      create the HDL wrapper, set it as top, add constrs/adc_capture.xdc, run synthesis."
say "=============================================================="
close $logf

# Notes
#  * Slice 00 is most likely the board's ADC_A; confirm on the bench with a tone.
#  * Q stream (m01_axis) is intentionally unconnected; the RFDC does not need a ready signal.
#  * On the board, set the ADC mixer to 0 Hz (adc_scope.setup_adc does this) so I = the input waveform.
