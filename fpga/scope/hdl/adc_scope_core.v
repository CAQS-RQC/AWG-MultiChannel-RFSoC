`timescale 1ns / 1ps
// ============================================================================
// adc_scope_core.v  -  triggered capture engine for one RFDC ADC stream
//
//  * Input : RFDC ADC AXI-Stream, 8 x int16 samples per beat (128 bit), sample 0 in bits [15:0].
//            With the AWG overlay config (4.9152 GSps, I/Q, decimation 2, mixer 0 Hz) this is
//            2.4576 GSps on the ADC fabric clock (307.2 MHz). The stream is never back-pressured.
//  * Memory: writes every beat straight into a true-dual-port BRAM (port A, ADC clock).
//            The PS reads it through an AXI BRAM controller on port B (PS clock).
//            -> No FIFO, no DMA, no rate matching: the ADC side can never lose a sample.
//  * Ring buffer with pre-trigger: after ARM, PRE beats are filled, then the engine waits
//    for the trigger, writes POST more beats (trigger beat included) and stops (DONE).
//    The valid window is beats [TRIG_ADDR-PRE, TRIG_ADDR+POST-1] (modulo DEPTH).
//  * Trigger sources (MODE register):
//      0 immediate        4 AWG ch2 sequence start (seq toggle from the AWG, DAC tile 2)
//      1 level rising     5 control_trigger rising edge
//      2 level falling    6 control_trigger falling edge
//      3 AWG ch0 sequence start (seq toggle from the AWG, DAC tile 0)
//      7 manual only (FORCE bit)
//    Level triggers are sample accurate: TRIG_SUB gives the sample (0..7) inside the
//    trigger beat. FORCE triggers at once (used by software as an "auto" timeout).
//  * All clock-domain crossings use XPM CDC macros (auto-constrained by Vivado).
//
// Register map (AXI4-Lite, byte offsets)
//   0x00 CTRL    W  bit0 ARM, bit1 FORCE, bit2 ABORT (write 1, self-clearing)
//   0x04 MODE    RW [2:0] trigger source
//   0x08 LEVEL   RW [15:0] signed threshold (int16 ADC code)
//   0x0C PRE     RW beats before the trigger beat (0 .. DEPTH-1)
//   0x10 POST    RW beats from the trigger beat on (1 .. DEPTH-PRE)
//   0x14 STATUS  R  bit0 DONE, bit1 BUSY (arm not yet seen by ADC side), [6:4] state
//   0x18 TADDR   R  beat address of the trigger beat
//   0x1C TSUB    R  sample index of the trigger inside the beat (level modes), else 0
//   0x20 TTIME   R  ADC clock cycles from ARM to trigger
//   0x24 DEPTH   R  buffer depth in beats
//   0x28 ID      R  0xADC5_0003
//   0x2C ALIVE   R  free-running count of valid ADC beats (shows the ADC is streaming)
// ============================================================================
module adc_scope_core #(
    parameter DEPTH_LOG2 = 14            // 16384 beats = 131072 samples = 53.3 us at 2.4576 GSps
                                         // keep BRAM_PORTA MEM_SIZE below = 16 * 2**DEPTH_LOG2
)(
    // ---------------- AXI4-Lite (PS clock) ----------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
    input  wire         s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire         s_axi_aresetn,
    input  wire [6:0]   s_axi_awaddr,
    input  wire [2:0]   s_axi_awprot,
    input  wire         s_axi_awvalid,
    output reg          s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output reg          s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output reg          s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [6:0]   s_axi_araddr,
    input  wire [2:0]   s_axi_arprot,
    input  wire         s_axi_arvalid,
    output reg          s_axi_arready,
    output reg  [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output reg          s_axi_rvalid,
    input  wire         s_axi_rready,

    // ---------------- ADC stream (ADC fabric clock) ----------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 adc_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis, ASSOCIATED_RESET adc_aresetn" *)
    input  wire         adc_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 adc_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire         adc_aresetn,
    input  wire [127:0] s_axis_tdata,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,

    // ---------------- capture memory, port A of a blk_mem_gen ----------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL, MEM_ECC NONE, MEM_WIDTH 128, MEM_SIZE 262144, READ_WRITE_MODE WRITE_ONLY" *)
    output wire         bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    output wire         bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    output wire         bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    output wire [15:0]  bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    output wire [31:0]  bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    output wire [127:0] bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    input  wire [127:0] bram_dout,

    // ---------------- external trigger sources (asynchronous) ----------------
    input  wire         awg_seq0_tog,   // toggles at each AWG ch0 sequence start (DAC0 clock)
    input  wire         awg_seq2_tog,   // toggles at each AWG ch2 sequence start (DAC2 clock)
    input  wire         ctrl_trig       // control_trigger (pin OR axi_control)
);

    localparam DEPTH = 1 << DEPTH_LOG2;
    localparam PW    = DEPTH_LOG2 + 1;            // width of PRE / POST / counters
    localparam [31:0] CORE_ID = 32'hADC5_0003;

    localparam [2:0] S_IDLE = 3'd0, S_FILL = 3'd1, S_ARMED = 3'd2, S_POST = 3'd3, S_DONE = 3'd4;

    // =====================================================================
    // PS clock domain: AXI4-Lite registers
    // =====================================================================
    reg  [2:0]  r_mode  = 3'd1;
    reg  [15:0] r_level = 16'd3277;               // ~0.1 of full scale
    reg  [PW-1:0] r_pre  = DEPTH / 4;
    reg  [PW-1:0] r_post = DEPTH / 2;
    reg         arm_tog_ps   = 1'b0;
    reg         force_tog_ps = 1'b0;
    reg         abort_tog_ps = 1'b0;

    // status coming back from the ADC domain (synchronised below)
    wire        fin_tog_ps, ack_tog_ps;
    wire [2:0]  state_ps;
    wire [DEPTH_LOG2-1:0] taddr_ps;
    wire [2:0]  tsub_ps;
    wire [31:0] ttime_ps;
    wire [31:0] alive_ps;

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    wire wr_go = s_axi_awvalid & s_axi_wvalid & ~s_axi_awready & ~s_axi_bvalid;
    wire rd_go = s_axi_arvalid & ~s_axi_arready & ~s_axi_rvalid;
    wire busy_ps = arm_tog_ps ^ ack_tog_ps;          // ARM not yet taken by the ADC side
    wire done_ps = ~(arm_tog_ps ^ fin_tog_ps) & ~busy_ps;   // capture for the CURRENT arm finished

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            // ---- write: AW and W accepted together, response after the handshake
            s_axi_awready <= wr_go;
            s_axi_wready  <= wr_go;
            if (s_axi_awready) begin
                s_axi_bvalid <= 1'b1;
                case (s_axi_awaddr[6:2])
                    5'h00: begin
                        if (s_axi_wdata[0]) arm_tog_ps   <= ~arm_tog_ps;
                        if (s_axi_wdata[1]) force_tog_ps <= ~force_tog_ps;
                        if (s_axi_wdata[2]) abort_tog_ps <= ~abort_tog_ps;
                    end
                    5'h01: r_mode  <= s_axi_wdata[2:0];
                    5'h02: r_level <= s_axi_wdata[15:0];
                    5'h03: r_pre   <= s_axi_wdata[PW-1:0];
                    5'h04: r_post  <= s_axi_wdata[PW-1:0];
                    default: ;
                endcase
            end else if (s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
            // ---- read
            s_axi_arready <= rd_go;
            if (s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                case (s_axi_araddr[6:2])
                    5'h01: s_axi_rdata <= {29'd0, r_mode};
                    5'h02: s_axi_rdata <= {16'd0, r_level};
                    5'h03: s_axi_rdata <= {{(32-PW){1'b0}}, r_pre};
                    5'h04: s_axi_rdata <= {{(32-PW){1'b0}}, r_post};
                    5'h05: s_axi_rdata <= {25'd0, state_ps, 2'b00, busy_ps, done_ps};
                    5'h06: s_axi_rdata <= {{(32-DEPTH_LOG2){1'b0}}, taddr_ps};
                    5'h07: s_axi_rdata <= {29'd0, tsub_ps};
                    5'h08: s_axi_rdata <= ttime_ps;
                    5'h09: s_axi_rdata <= DEPTH;
                    5'h0A: s_axi_rdata <= CORE_ID;
                    5'h0B: s_axi_rdata <= alive_ps;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // =====================================================================
    // CDC  PS -> ADC
    // =====================================================================
    // configuration: quasi-static (software writes it before ARM; ARM goes through more sync stages)
    localparam CFGW = 3 + 16 + PW + PW;
    wire [CFGW-1:0] cfg_adc;
    xpm_cdc_array_single #(.DEST_SYNC_FF(2), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0),
                           .SRC_INPUT_REG(1), .WIDTH(CFGW))
        u_cdc_cfg (.src_clk(s_axi_aclk), .src_in({r_mode, r_level, r_pre, r_post}),
                   .dest_clk(adc_clk), .dest_out(cfg_adc));
    wire [2:0]    mode_a  = cfg_adc[CFGW-1 -: 3];
    wire [15:0]   level_a = cfg_adc[2*PW +: 16];
    wire [PW-1:0] pre_a   = cfg_adc[PW +: PW];
    wire [PW-1:0] post_a  = cfg_adc[0 +: PW];

    wire arm_s, force_s, abort_s;
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_arm   (.src_clk(s_axi_aclk), .src_in(arm_tog_ps),   .dest_clk(adc_clk), .dest_out(arm_s));
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_force (.src_clk(s_axi_aclk), .src_in(force_tog_ps), .dest_clk(adc_clk), .dest_out(force_s));
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_abort (.src_clk(s_axi_aclk), .src_in(abort_tog_ps), .dest_clk(adc_clk), .dest_out(abort_s));

    // external triggers (no source register: they come from other clock domains / a pin)
    wire seq0_s, seq2_s, ctrl_s;
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_seq0 (.src_clk(1'b0), .src_in(awg_seq0_tog), .dest_clk(adc_clk), .dest_out(seq0_s));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_seq2 (.src_clk(1'b0), .src_in(awg_seq2_tog), .dest_clk(adc_clk), .dest_out(seq2_s));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_ctrl (.src_clk(1'b0), .src_in(ctrl_trig),    .dest_clk(adc_clk), .dest_out(ctrl_s));

    // =====================================================================
    // ADC clock domain
    // =====================================================================
    assign s_axis_tready = 1'b1;     // never back-pressure the RFDC

    reg arm_d = 1'b0, force_d = 1'b0, abort_d = 1'b0;
    reg seq0_d = 1'b0, seq2_d = 1'b0, ctrl_d = 1'b0;
    always @(posedge adc_clk) begin
        arm_d <= arm_s;   force_d <= force_s;   abort_d <= abort_s;
        seq0_d <= seq0_s; seq2_d <= seq2_s;     ctrl_d <= ctrl_s;
    end
    wire arm_ev   = arm_s   ^ arm_d;
    wire force_ev = force_s ^ force_d;
    wire abort_ev = abort_s ^ abort_d;
    wire seq0_ev  = seq0_s  ^ seq0_d;
    wire seq2_ev  = seq2_s  ^ seq2_d;
    wire ctrl_r   = ctrl_s & ~ctrl_d;
    wire ctrl_f   = ~ctrl_s & ctrl_d;

    // ---- data / level-trigger pipeline -----------------------------------
    // st0: register the RFDC output          (d0)
    // st1: compare each sample with LEVEL    (d1, ge1)
    // st2: find crossings                    (d2, rise2, fall2)
    // st3: trigger decision + BRAM write of d2
    reg [127:0] d0, d1, d2;
    reg         v0 = 1'b0, v1 = 1'b0, v2 = 1'b0;
    reg [7:0]   ge1, rise2, fall2;
    reg         ge_last = 1'b0;      // ge of the last sample of the previous valid beat
    reg [15:0]  level_r = 16'd0;

    integer i;
    always @(posedge adc_clk) begin
        level_r <= level_a;
        d0 <= s_axis_tdata;  v0 <= s_axis_tvalid;
        d1 <= d0;            v1 <= v0;
        for (i = 0; i < 8; i = i + 1)
            ge1[i] <= ($signed(d0[16*i +: 16]) >= $signed(level_r));
        d2 <= d1;            v2 <= v1;
        if (v1) begin
            rise2[0] <=  ge1[0] & ~ge_last;
            fall2[0] <= ~ge1[0] &  ge_last;
            for (i = 1; i < 8; i = i + 1) begin
                rise2[i] <=  ge1[i] & ~ge1[i-1];
                fall2[i] <= ~ge1[i] &  ge1[i-1];
            end
            ge_last <= ge1[7];
        end else begin
            rise2 <= 8'd0;
            fall2 <= 8'd0;
        end
    end

    function [2:0] first_one;      // index of the lowest set bit (earliest sample)
        input [7:0] x;
        begin
            first_one = x[0] ? 3'd0 : x[1] ? 3'd1 : x[2] ? 3'd2 : x[3] ? 3'd3 :
                        x[4] ? 3'd4 : x[5] ? 3'd5 : x[6] ? 3'd6 : 3'd7;
        end
    endfunction

    // ---- capture state machine -------------------------------------------
    reg [2:0]            state = S_IDLE;
    reg [DEPTH_LOG2-1:0] wptr  = {DEPTH_LOG2{1'b0}};
    reg [PW-1:0]         cnt   = {PW{1'b0}};
    reg [PW-1:0]         pre_l = {PW{1'b0}}, post_l = {PW{1'b0}};
    reg [2:0]            mode_l = 3'd0;
    reg                  force_pend = 1'b0;
    reg                  done  = 1'b0;
    reg                  ack_tog = 1'b0;
    reg                  fin_tog = 1'b0;     // set to the ARM toggle value when that capture is done
    reg [3:0]            ack_dly = 4'd0;     // ack leaves 4 cycles after fin, so the PS never sees a stale fin
    reg [DEPTH_LOG2-1:0] taddr = {DEPTH_LOG2{1'b0}};
    reg [2:0]            tsub  = 3'd0;
    reg [31:0]           tcnt  = 32'd0, ttime = 32'd0;
    reg [31:0]           alive = 32'd0;

    reg       hit;
    reg [2:0] hit_sub;
    always @(*) begin
        hit = 1'b0; hit_sub = 3'd0;
        case (mode_l)
            3'd0: hit = 1'b1;
            3'd1: begin hit = |rise2; hit_sub = first_one(rise2); end
            3'd2: begin hit = |fall2; hit_sub = first_one(fall2); end
            3'd3: hit = seq0_ev;
            3'd4: hit = seq2_ev;
            3'd5: hit = ctrl_r;
            3'd6: hit = ctrl_f;
            default: hit = 1'b0;
        endcase
    end

    wire writing = v2 & ((state == S_FILL) | (state == S_ARMED) | (state == S_POST));

    always @(posedge adc_clk) begin
        if (v0) alive <= alive + 32'd1;
        ack_dly <= {ack_dly[2:0], ack_tog};
        if (!adc_aresetn) begin
            state <= S_IDLE;
            done  <= 1'b0;
            force_pend <= 1'b0;
        end else if (abort_ev) begin
            state <= S_IDLE;
            done  <= 1'b0;
            force_pend <= 1'b0;
        end else if (arm_ev) begin
            pre_l   <= pre_a;
            post_l  <= (post_a == {PW{1'b0}}) ? {{(PW-1){1'b0}}, 1'b1} : post_a;
            mode_l  <= mode_a;
            cnt     <= {PW{1'b0}};
            tcnt    <= 32'd0;
            done    <= 1'b0;
            force_pend <= 1'b0;
            ack_tog <= arm_s;
            fin_tog <= ~arm_s;     // 'not finished' for this arm
            state   <= (pre_a == {PW{1'b0}}) ? S_ARMED : S_FILL;
        end else begin
            if (force_ev) force_pend <= 1'b1;
            if ((state == S_FILL) | (state == S_ARMED) | (state == S_POST))
                tcnt <= tcnt + 32'd1;
            if (writing) begin
                wptr <= wptr + 1'b1;
                case (state)
                    S_FILL: begin
                        cnt <= cnt + 1'b1;
                        if (cnt == pre_l - 1'b1) state <= S_ARMED;
                    end
                    S_ARMED: begin
                        if (hit | force_pend | force_ev) begin
                            taddr      <= wptr;
                            tsub       <= hit ? hit_sub : 3'd0;
                            ttime      <= tcnt;
                            force_pend <= 1'b0;
                            if (post_l == {{(PW-1){1'b0}}, 1'b1}) begin
                                state <= S_DONE;
                                done  <= 1'b1;
                                fin_tog <= ack_tog;
                            end else begin
                                state <= S_POST;
                                cnt   <= {{(PW-1){1'b0}}, 1'b1};
                            end
                        end
                    end
                    S_POST: begin
                        cnt <= cnt + 1'b1;
                        if (cnt == post_l - 1'b1) begin
                            state <= S_DONE;
                            done  <= 1'b1;
                            fin_tog <= ack_tog;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    assign bram_clk  = adc_clk;
    assign bram_rst  = 1'b0;
    assign bram_en   = writing;
    assign bram_we   = {16{writing}};
    assign bram_addr = {{(28-DEPTH_LOG2){1'b0}}, wptr, 4'b0000};
    assign bram_din  = d2;

    // =====================================================================
    // CDC  ADC -> PS
    // =====================================================================
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_fin  (.src_clk(adc_clk), .src_in(fin_tog), .dest_clk(s_axi_aclk), .dest_out(fin_tog_ps));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_ack  (.src_clk(adc_clk), .src_in(ack_dly[3]), .dest_clk(s_axi_aclk), .dest_out(ack_tog_ps));

    // trigger results are frozen before DONE rises, so a plain array sync is safe for them
    localparam STW = 3 + DEPTH_LOG2 + 3 + 32;
    wire [STW-1:0] st_ps;
    xpm_cdc_array_single #(.DEST_SYNC_FF(2), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0),
                           .SRC_INPUT_REG(1), .WIDTH(STW))
        u_cdc_stat (.src_clk(adc_clk), .src_in({state, taddr, tsub, ttime}),
                    .dest_clk(s_axi_aclk), .dest_out(st_ps));
    assign state_ps = st_ps[STW-1 -: 3];
    assign taddr_ps = st_ps[35 +: DEPTH_LOG2];
    assign tsub_ps  = st_ps[32 +: 3];
    assign ttime_ps = st_ps[31:0];

    // free-running counter: gray-code crossing, always coherent
    xpm_cdc_gray #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .REG_OUTPUT(1), .SIM_ASSERT_CHK(0),
                   .SIM_LOSSLESS_GRAY_CHK(0), .WIDTH(32))
        u_cdc_alive (.src_clk(adc_clk), .src_in_bin(alive), .dest_clk(s_axi_aclk), .dest_out_bin(alive_ps));

endmodule
