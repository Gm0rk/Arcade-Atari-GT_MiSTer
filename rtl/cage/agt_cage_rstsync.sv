// agt_cage_rstsync.sv -- reset synchroniser into clk_dsp: asserts at once,
// releases on a clock edge.
//
// Every CAGE reset crossing uses this module so all have the same shape and
// the same register name for the SDC:
//   set_false_path -to [get_registers {*cagerst_sync*}]
// Keep the name `cagerst_sync`.
`default_nettype none

module agt_cage_rstsync (
	input  wire  clk,
	input  wire  arst_n,        // any domain; low = reset
	output wire  rst_n          // clk's domain
);
	logic [1:0] cagerst_sync;
	always_ff @(posedge clk or negedge arst_n) begin
		if (!arst_n) cagerst_sync <= 2'b00;
		else         cagerst_sync <= {cagerst_sync[0], 1'b1};
	end
	assign rst_n = cagerst_sync[1];
endmodule

`default_nettype wire
