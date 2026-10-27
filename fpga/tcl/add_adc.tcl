# add_adc.tcl  (v3)  -  triggered ADC capture for the AWG block design
#
# Replaces the v2 DMA capture (packet_generator / clock converter / FIFO / axi_dma) by:
#
#   RFDC m00_axis (ADC_A, 2.4576 GSps, 8 samples/beat @ 307.2 MHz)
#        |
#   adc      (adc/hdl/adc_core.v : trigger engine + AXI-Lite registers)
#        | BRAM port A, ADC clock (never back-pressures the ADC, never loses a sample)
#   blk_mem_adc  (true dual-port BRAM, 16384 x 128 bit = 131072 samples = 53 us)
#        | BRAM port B, PS clock
#   axi_bram_ctrl_adc  <- ps8_0_axi_periph/M08   (PS reads the samples)
#   adc/s_axi      <- ps8_0_axi_periph/M09   (PS reads/writes the registers)
#
# Trigger sources: immediate, ADC level rising/falling (sample accurate), AWG ch0/ch2 sequence
# start (new seq_tog0/seq_tog2 outputs of awg_BRAM_dual), control_trigger edges, manual.
#
# Normally you do NOT run this by hand: fpga/awg_multichannel_rfsoc.tcl sources it (with DO_BUILD 0).
# Manual use (Vivado 2024.1, AWG block design open):
#   source <repo>/fpga/tcl/add_adc.tcl
# It can be run again: it removes what v2/v3 added before building.
# Log: fpga/tcl/add_adc.log

if {![info exists DO_BUILD]} { set DO_BUILD 1 }   ;# 1 = after a clean validation, regenerate and launch synth+impl+bitstream (the master script sets 0)
set DEPTH_LOG2 14         ;# must match DEPTH_LOG2 / MEM_SIZE in adc_core.v (16 << 14 = 256 KiB)
set MEM_BYTES  [expr {16 << $DEPTH_LOG2}]

set script_dir [file dirname [file normalize [info script]]]
set repo_dir   [file normalize [file join $script_dir ..]]
set core_file  [file join $repo_dir adc hdl adc_core.v]
set logf [open [file join $script_dir add_adc.log] w]
proc say {msg} { global logf; puts $msg; puts $logf $msg; flush $logf }
proc die {msg} { global logf; say "ERROR: $msg"; close $logf; error $msg }

proc pin {p} {
    set x [get_bd_pins -quiet $p]
    if {[llength $x] == 0} { die "pin $p not found" }
    return $x
}
proc unhook {p} {
    set n [get_bd_nets -quiet -of_objects [pin $p]]
    if {[llength $n]} { disconnect_bd_net $n [pin $p] }
}
# connect pin $src to all pins in $args (re-using the net already on $src)
proc hook {src args} {
    set sp [pin $src]
    foreach d $args { unhook $d }
    set n [get_bd_nets -quiet -of_objects $sp]
    if {[llength $n] == 0} {
        set dl {}
        foreach d $args { lappend dl [pin $d] }
        connect_bd_net $sp {*}$dl
    } else {
        foreach d $args { connect_bd_net -net $n [pin $d] }
    }
}
proc ihook {a b} {
    foreach p [list $a $b] {
        set ip [get_bd_intf_pins -quiet $p]
        if {[llength $ip] == 0} { die "interface pin $p not found" }
        set n [get_bd_intf_nets -quiet -of_objects $ip]
        if {[llength $n]} { delete_bd_objs $n }
    }
    connect_bd_intf_net [get_bd_intf_pins $a] [get_bd_intf_pins $b]
}

set rfdc_name usp_rf_data_converter_0
set S 00 ; set t 0          ;# ADC slice 00 = tile 224 block 0 = ADC_A (vin0_01)

# ------------------------------------------------------------ STEP 0: checks
current_bd_design
set bd [current_bd_design]
say "INFO: block design '$bd'"
foreach c [list $rfdc_name zynq_ultra_ps_e_0 ps8_0_axi_periph rst_ps8_0_99M axi_smc awg_BRAM_dual_0 control_or_gate_0] {
    if {[llength [get_bd_cells -quiet $c]] == 0} { die "cell $c missing - open the AWG block design first" }
}
if {![file exists $core_file]} { die "missing $core_file" }

# ------------------------------------------------------------ STEP 1: RFDC ADC config (same as v2)
set rfdc [get_bd_cells $rfdc_name]
array set want [list \
    CONFIG.ADC_Slice${S}_Enable     true \
    CONFIG.ADC${t}_Sampling_Rate    4.9152 \
    CONFIG.ADC${t}_PLL_Enable       true \
    CONFIG.ADC${t}_Refclk_Freq      491.520 \
    CONFIG.ADC${t}_Outclk_Freq      307.200 \
    CONFIG.ADC_Data_Type${S}        1 \
    CONFIG.ADC_Data_Width${S}       8 \
    CONFIG.ADC_Decimation_Mode${S}  2 \
    CONFIG.ADC_Mixer_Type${S}       2 \
    CONFIG.ADC_Mixer_Mode${S}       0 ]
proc cfg_ok {rfdc wantv} {
    array set want $wantv
    set bad {}
    foreach k [lsort [array names want]] {
        set got [get_property $k $rfdc]
        if {[string is double -strict $got] && [string is double -strict $want($k)]} {
            if {abs($got - $want($k)) > 1e-9} { lappend bad "$k=$got (want $want($k))" }
        } elseif {[string tolower $got] ne [string tolower $want($k)]} { lappend bad "$k=$got (want $want($k))" }
    }
    return $bad
}
set bad [cfg_ok $rfdc [array get want]]
if {[llength $bad]} {
    say "INFO: ADC not configured yet, applying: $bad"
    set cfg {}
    foreach k [array names want] { lappend cfg $k $want($k) }
    if {[catch {set_property -dict $cfg $rfdc} err]} { die "RFDC rejected ADC settings: $err" }
    set bad [cfg_ok $rfdc [array get want]]
    if {[llength $bad]} { die "ADC readback mismatch: $bad" }
}
set nb [get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins $rfdc_name/m${S}_axis]]
if {$nb != 16} { die "m${S}_axis is $nb bytes, expected 16" }
say "INFO: ADC_A config ok (4.9152 GSps, I/Q dec 2 -> 2.4576 GSps, 8 samples/beat, clk_adc0 [expr {[get_property CONFIG.FREQ_HZ [pin $rfdc_name/clk_adc0]]/1e6}] MHz)"

# ------------------------------------------------------------ STEP 2: remove v2 / previous v3 cells
say "STEP 2: cleanup"
# (old pre-rename cell names are listed too, so a re-run cleans up older builds)
foreach c {packet_generator_adc axis_clock_converter_adc axis_fifo_adc axi_dma_adc proc_sys_reset_adc \
           adc_scope blk_mem_scope axi_bram_ctrl_scope adc blk_mem_adc axi_bram_ctrl_adc} {
    set x [get_bd_cells -quiet $c]
    if {[llength $x]} { say "  delete $c"; delete_bd_objs $x }
}
# axi_smc back to its original 2 slaves (S02 was the v2 DMA)
set smc [get_bd_cells axi_smc]
if {[get_property CONFIG.NUM_SI $smc] > 2} {
    # whatever is still on S02 can only be a leftover of the v2 DMA path: show it, then remove it
    set s2 [get_bd_intf_pins -quiet axi_smc/S02_AXI]
    if {[llength $s2]} {
        set n2 [get_bd_intf_nets -quiet -of_objects $s2]
        if {[llength $n2]} {
            foreach n $n2 {
                set others {}
                foreach q [get_bd_intf_pins -quiet -of_objects $n] { lappend others $q }
                say "  S02_AXI leftover net $n connects: $others -> deleting"
            }
            delete_bd_objs $n2
        }
    }
    # clock / reset pins of the SI-2 side: disconnect so shrinking NUM_SI cannot fail
    foreach q [get_bd_pins -quiet axi_smc/aclk2] {
        set nn [get_bd_nets -quiet -of_objects $q]
        if {[llength $nn]} { disconnect_bd_net $nn $q }
    }
    set_property CONFIG.NUM_SI 2 $smc
    say "  axi_smc NUM_SI -> 2"
}
set ic [get_bd_cells ps8_0_axi_periph]
if {[get_property CONFIG.NUM_MI $ic] < 10} { set_property CONFIG.NUM_MI 10 $ic }

# ------------------------------------------------------------ STEP 3: RTL sources
say "STEP 3: sources"
if {[llength [get_files -quiet $core_file]] == 0} {
    add_files -norecurse -fileset sources_1 $core_file
}
update_compile_order -fileset sources_1
if {[llength [get_bd_pins -quiet awg_BRAM_dual_0/seq_tog0]] == 0} {
    say "  refreshing awg_BRAM_dual_0 module reference (new seq_tog0/seq_tog2 outputs)"
    set mref "${bd}_awg_BRAM_dual_0_0"
    if {[catch {update_module_reference $mref} err]} { say "  update_module_reference: $err" }
    current_bd_design $bd
    if {[llength [get_bd_pins -quiet awg_BRAM_dual_0/seq_tog0]] == 0} {
        die "awg_BRAM_dual_0 has no seq_tog0 pin: is src/awg_BRAM_dual.v the new version? In the BD, right-click awg_BRAM_dual_0 > Refresh Module, then run this script again."
    }
}

# ------------------------------------------------------------ STEP 4: external ADC ports
say "STEP 4: ADC ports"
if {[llength [get_bd_intf_ports -quiet adc${t}_clk_1]] == 0} {
    set p [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 adc${t}_clk_1]
    set_property CONFIG.FREQ_HZ 491520000 $p
}
if {[llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $rfdc_name/adc${t}_clk]]] == 0} {
    connect_bd_intf_net [get_bd_intf_ports adc${t}_clk_1] [get_bd_intf_pins $rfdc_name/adc${t}_clk]
}
if {[llength [get_bd_intf_ports -quiet vin0_01_1]] == 0} {
    create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_analog_io_rtl:1.0 vin0_01_1
}
if {[llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $rfdc_name/vin0_01]]] == 0} {
    connect_bd_intf_net [get_bd_intf_ports vin0_01_1] [get_bd_intf_pins $rfdc_name/vin0_01]
}

# ------------------------------------------------------------ STEP 5: cells
say "STEP 5: ADC capture cells"
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_adc
set adc_cell [create_bd_cell -type module -reference adc_core adc]
set_property CONFIG.DEPTH_LOG2 $DEPTH_LOG2 $adc_cell
set bctl [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_adc]
set_property -dict [list CONFIG.DATA_WIDTH 128 CONFIG.SINGLE_PORT_BRAM 1 CONFIG.ECC_TYPE 0] $bctl
set bmg [create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 blk_mem_adc]
if {[catch {set_property -dict [list CONFIG.use_bram_block BRAM_Controller CONFIG.Memory_Type True_Dual_Port_RAM] $bmg} err]} {
    say "  note: blk_mem_gen dict: $err"
    set_property CONFIG.Memory_Type True_Dual_Port_RAM $bmg
}

# ------------------------------------------------------------ STEP 6: connections
say "STEP 6: connections"
ihook $rfdc_name/m${S}_axis             adc/s_axis
ihook adc/BRAM_PORTA              blk_mem_adc/BRAM_PORTA
ihook axi_bram_ctrl_adc/BRAM_PORTA    blk_mem_adc/BRAM_PORTB
ihook ps8_0_axi_periph/M08_AXI          axi_bram_ctrl_adc/S_AXI
ihook ps8_0_axi_periph/M09_AXI          adc/s_axi

hook zynq_ultra_ps_e_0/pl_clk0  ps8_0_axi_periph/M08_ACLK ps8_0_axi_periph/M09_ACLK \
                                axi_bram_ctrl_adc/s_axi_aclk adc/s_axi_aclk
hook rst_ps8_0_99M/peripheral_aresetn  ps8_0_axi_periph/M08_ARESETN ps8_0_axi_periph/M09_ARESETN \
                                axi_bram_ctrl_adc/s_axi_aresetn adc/s_axi_aresetn
hook $rfdc_name/clk_adc${t}     $rfdc_name/m${t}_axis_aclk adc/adc_clk proc_sys_reset_adc/slowest_sync_clk
hook zynq_ultra_ps_e_0/pl_resetn0  proc_sys_reset_adc/ext_reset_in
hook proc_sys_reset_adc/peripheral_aresetn  $rfdc_name/m${t}_axis_aresetn adc/adc_aresetn
hook awg_BRAM_dual_0/seq_tog0   adc/awg_seq0_tog
hook awg_BRAM_dual_0/seq_tog2   adc/awg_seq2_tog
hook control_or_gate_0/control_trigger  adc/ctrl_trig

# ------------------------------------------------------------ STEP 7: addresses, validate, save
say "STEP 7: addresses / validate / save"
set space [get_bd_addr_spaces zynq_ultra_ps_e_0/Data]
set mseg  [get_bd_addr_segs axi_bram_ctrl_adc/S_AXI/Mem0]
if {[catch {assign_bd_address -target_address_space $space -range $MEM_BYTES $mseg} err]} {
    say "  note: assign with range: $err"
    assign_bd_address -target_address_space $space $mseg
    foreach s [get_bd_addr_segs -quiet -of_objects $space] {
        if {[string match "*axi_bram_ctrl_adc*" $s]} { set_property range $MEM_BYTES $s }
    }
}
assign_bd_address
foreach s [get_bd_addr_segs -quiet -of_objects $space] {
    if {[string match "*adc*" $s]} {
        say [format "  %-50s offset %s range %s" $s [get_property offset $s] [get_property range $s]]
    }
}
set rc [catch {validate_bd_design} verr]
save_bd_design
if {$rc} {
    die "VALIDATION FAILED: $verr (scroll up in the Tcl console to the first ERROR line)"
}
say "Block design validated and saved."

# post-route phys_opt: fixes the small hold violation seen in the DAC0 domain
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

if {$DO_BUILD} {
    say "STEP 8: generate + synth + impl + bitstream (runs in the background, watch the Design Runs tab)"
    generate_target all [get_files ${bd}.bd]
    reset_run synth_1
    launch_runs impl_1 -to_step write_bitstream -jobs 4
}
if {$DO_BUILD} {
    say "Done. When the run finishes: check WNS >= 0 and WHS >= 0 in the routed timing summary,"
    say "then copy ${bd}_wrapper.bit + ${bd}.hwh to the board under one common name."
} else {
    say "ADC capture added and block design validated (DO_BUILD 0: no build started)."
}
close $logf
