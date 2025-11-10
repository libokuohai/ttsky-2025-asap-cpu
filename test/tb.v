`default_nettype none
`timescale 1ns / 1ps

module tb ();

`ifndef COCOTB_SIM
  // Only dump waves when NOT running under cocotb (avoids FST warnings)
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
  end
`endif

  // Drive lines cocotb will set (avoid X right at t=0)
  reg        clk   = 1'b0;
  reg        rst_n = 1'b0;
  reg        ena   = 1'b0;
  reg  [7:0] ui_in = 8'h00;
  reg  [7:0] uio_in= 8'h00;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_libokuohai_asap_cpu_v1 user_project (
`ifdef GL_TEST
      .VPWR   (VPWR),
      .VGND   (VGND),
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

endmodule
