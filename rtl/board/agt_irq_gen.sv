// agt_irq_gen.sv -- Atari GT interrupt sources (from MAME's atarigt.cpp)
//
//   * VIDEO int (IRQ4): set at the rising edge of vblank. Acked by the game's
//     write to 0xE0C000 (in agt_main_memmap; this module only makes the set
//     pulse).
//   * SCANLINE int (IRQ6): a flat 250 Hz periodic timer, not a scanline
//     compare. Acked at 0xE0A000.
//   * vblank_level: the live vblank line, read through sport2 bit 7; the frame
//     loop spins on it before kicking mo_command.
//
// Sources are in the pixel clock domain; the memmap consumes them in the CPU
// domain. Pulses cross as toggles (edge -> toggle -> 2FF -> XOR), the level
// through a 2FF, so no pulse is lost or doubled at any clock ratio.
//
// SCANLINE_DIV: 7,159,090 Hz / 250 Hz = 28,636.36, so the integer divider
// runs 0.0013% fast; the game counts interrupts, not their phase.
`default_nettype none

module agt_irq_gen #(
	parameter int SCANLINE_DIV = 28636      // pixel clocks per 250 Hz tick
) (
	// pixel clock domain
	input  wire  pix_clk,
	input  wire  pix_rst_n,
	input  wire  vblank_in,                 // level from agt_video_timing

	// CPU clock domain
	input  wire  cpu_clk,
	input  wire  cpu_rst_n,
	output logic video_int_set,             // 1-cycle pulse, vblank rise
	output logic scanline_int_set,          // 1-cycle pulse, 250 Hz
	output logic vblank_level               // synced live vblank
);

	// pixel domain: edges -> toggles
	logic        vb_d;
	logic        vid_tgl, scan_tgl;
	logic [15:0] div_cnt;

	always_ff @(posedge pix_clk or negedge pix_rst_n) begin
		if (!pix_rst_n) begin
			vb_d    <= 1'b0;
			vid_tgl <= 1'b0;
			scan_tgl<= 1'b0;
			div_cnt <= 16'd0;
		end else begin
			vb_d <= vblank_in;
			if (vblank_in && !vb_d)
				vid_tgl <= ~vid_tgl;        // one toggle per vblank rise
			if (div_cnt == SCANLINE_DIV[15:0] - 16'd1) begin
				div_cnt  <= 16'd0;
				scan_tgl <= ~scan_tgl;      // one toggle per 250 Hz tick
			end else
				div_cnt <= div_cnt + 16'd1;
		end
	end

	// CPU domain: 2FF sync, toggles -> pulses
	logic [2:0] vid_s, scan_s;
	logic [1:0] vb_s;

	always_ff @(posedge cpu_clk or negedge cpu_rst_n) begin
		if (!cpu_rst_n) begin
			vid_s  <= 3'd0;
			scan_s <= 3'd0;
			vb_s   <= 2'd0;
			video_int_set    <= 1'b0;
			scanline_int_set <= 1'b0;
			vblank_level     <= 1'b0;
		end else begin
			vid_s  <= {vid_s[1:0],  vid_tgl};
			scan_s <= {scan_s[1:0], scan_tgl};
			vb_s   <= {vb_s[0], vblank_in};
			video_int_set    <= vid_s[2] ^ vid_s[1];
			scanline_int_set <= scan_s[2] ^ scan_s[1];
			vblank_level     <= vb_s[1];
		end
	end

endmodule
`default_nettype wire
