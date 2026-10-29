`timescale 1ns/1ps
module xpm_cdc_single #(parameter DEST_SYNC_FF=4, INIT_SYNC_FF=0, SIM_ASSERT_CHK=0, SRC_INPUT_REG=1)
 (input src_clk, input src_in, input dest_clk, output dest_out);
  reg s = 0; always @(posedge src_clk) s <= src_in;
  wire si = SRC_INPUT_REG ? s : src_in;
  reg [DEST_SYNC_FF-1:0] r = 0; always @(posedge dest_clk) r <= {r[DEST_SYNC_FF-2:0], si};
  assign dest_out = r[DEST_SYNC_FF-1];
endmodule
module xpm_cdc_array_single #(parameter DEST_SYNC_FF=2, INIT_SYNC_FF=0, SIM_ASSERT_CHK=0, SRC_INPUT_REG=1, WIDTH=2)
 (input src_clk, input [WIDTH-1:0] src_in, input dest_clk, output [WIDTH-1:0] dest_out);
  reg [WIDTH-1:0] s = 0; always @(posedge src_clk) s <= src_in;
  reg [WIDTH-1:0] a = 0, b = 0; always @(posedge dest_clk) begin a <= SRC_INPUT_REG ? s : src_in; b <= a; end
  assign dest_out = b;
endmodule
module xpm_cdc_gray #(parameter DEST_SYNC_FF=3, INIT_SYNC_FF=0, REG_OUTPUT=1, SIM_ASSERT_CHK=0, SIM_LOSSLESS_GRAY_CHK=0, WIDTH=32)
 (input src_clk, input [WIDTH-1:0] src_in_bin, input dest_clk, output [WIDTH-1:0] dest_out_bin);
  reg [WIDTH-1:0] s = 0; always @(posedge src_clk) s <= src_in_bin;
  reg [WIDTH-1:0] a = 0, b = 0; always @(posedge dest_clk) begin a <= s; b <= a; end
  assign dest_out_bin = b;
endmodule
