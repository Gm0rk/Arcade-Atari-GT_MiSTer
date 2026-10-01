// agt_cage_gov.sv -- real-time governor: keeps the DSP's model time at the
// chip's rate.
//
// Everything audible (timers, DMA, the serial port) runs in the DSP model's
// cycles, so pitch and tempo follow the rate the core earns them. Faster than
// the chip's 16.9344 MHz, the sound plays fast and sharp.
//
// A credit in 1/DEN of a model cycle gains NUM every clk_dsp cycle and loses
// DEN per model cycle the core reports (mcyc_stb/mcyc_add). NUM/DEN is the
// chip's model cycles per clk_dsp cycle: 16.9344 / (630/17) = 1428/3125.
// While the credit is negative the core is ahead of real time and `hold`
// stalls it at its next S_EXEC.
//
// The credit is capped at CAP model cycles: what a slow stretch may bank for
// a fast one to spend. The sound program mixes a burst (slower than the chip)
// and then polls for DMA (faster); a cap longer than one burst (49,152 model
// cycles) lets the poll make up the mixing's deficit, so only time beyond the
// chip's rate is held. Over any T clk_dsp cycles the core earns at most
// T*NUM/DEN + CAP + 48 model cycles (48: the step that crosses zero plus the
// reporting pipeline).
//
// mcyc_add * DEN is built from shifts and adds, so no DSP block is used.
`default_nettype none

module agt_cage_gov #(
	parameter int NUM = 1428,           // 16.9344 MHz * 17 / 630 MHz = 1428/3125
	parameter int DEN = 3125,
	parameter int CAP = 65536           // model cycles a slow stretch may bank
) (
	input  wire         clk,
	input  wire         rst_n,
	input  wire         en,             // 0: never hold
	input  wire         mcyc_stb,       // core finished a step...
	input  wire  [4:0]  mcyc_add,       // ...earning this many model cycles
	output wire         hold,           // core is ahead of real time
	output logic [31:0] n_hold          // cycles held (for benches)
);

	// Credit range: about -48*DEN to CAP*DEN.
	localparam int W = $clog2((CAP + 64) * DEN) + 2;
	localparam logic signed [W-1:0] CAPV = CAP * DEN;
	// Floor below anything an obeyed hold reaches, so a core that ignores `hold`
	// cannot wrap the credit around.
	localparam logic signed [W-1:0] FLOORV = -(64 * DEN);

	logic signed [W-1:0] credit;

	// mcyc_add * DEN; shift-and-add form when DEN is 3125
	wire [W-1:0] m = {{(W-5){1'b0}}, mcyc_add};
	wire [W-1:0] mxd;
	generate
		if (DEN == 3125) begin : g_3125
			assign mxd = (m << 11) + (m << 10) + (m << 5) + (m << 4) + (m << 2) + m;
		end else begin : g_any
			assign mxd = m * DEN;
		end
	endgenerate
	wire signed [W-1:0] spend = mcyc_stb ? $signed(mxd) : '0;
	wire signed [W-1:0] nxt   = credit + NUM - spend;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			credit <= CAPV;
			n_hold <= 32'd0;
		end else begin
			if (!en)               credit <= CAPV;      // disabled: full credit, never hold
			else if (nxt > CAPV)   credit <= CAPV;
			else if (nxt < FLOORV) credit <= FLOORV;
			else                   credit <= nxt;
			if (hold) n_hold <= n_hold + 32'd1;
		end
	end

	assign hold = en && credit[W-1];    // negative: ahead of real time

endmodule

`default_nettype wire
