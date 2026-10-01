// agt_video_timing.sv -- Atari GT H/V timing generator.
//
// Counters and blanking from MAME's atarigt.cpp screen config:
//   m_screen->set_raw(14.318181_MHz_XTAL/2, 456, 0, 336, 262, 0, 240);
// i.e. ~7.159 MHz pixel clock, htotal 456 (336 visible), vtotal 262 (240
// visible). MAME notes these come from published specs, not the board (a
// pair of GALs sets the real timing). `clk` is the pixel clock.
//
// Sync position and width are not given by MAME; the porch/sync values
// below are placeholders based on typical arcade timing.

module agt_video_timing #(
	parameter int H_TOTAL   = 456,
	parameter int H_VISIBLE = 336,
	parameter int H_SYNC_START = 336 + 12,      // placeholder front porch 12px
	parameter int H_SYNC_END   = 336 + 12 + 32, // placeholder sync width 32px
	parameter int V_TOTAL   = 262,
	parameter int V_VISIBLE = 240,
	parameter int V_SYNC_START = 240 + 4,    // placeholder front porch 4 lines
	parameter int V_SYNC_END   = 240 + 4 + 4 // placeholder sync width 4 lines
) (
	input  logic clk,      // pixel clock (~7.159090 MHz)
	input  logic rst_n,

	output logic [8:0] hcount,   // 0 .. H_TOTAL-1
	output logic [8:0] vcount,   // 0 .. V_TOTAL-1

	output logic hblank,
	output logic vblank,
	output logic hsync,         // placeholder timing
	output logic vsync,         // placeholder timing

	output logic pixel_active,  // !hblank && !vblank
	output logic frame_start,   // 1-cycle pulse at hcount==0 && vcount==0
	output logic line_start     // 1-cycle pulse at hcount==0
);

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			hcount <= 9'd0;
			vcount <= 9'd0;
		end else begin
			if (hcount == H_TOTAL - 1) begin
				hcount <= 9'd0;
				if (vcount == V_TOTAL - 1)
					vcount <= 9'd0;
				else
					vcount <= vcount + 9'd1;
			end else begin
				hcount <= hcount + 9'd1;
			end
		end
	end

	assign hblank = (hcount >= H_VISIBLE);
	assign vblank = (vcount >= V_VISIBLE);
	assign hsync  = (hcount >= H_SYNC_START) && (hcount < H_SYNC_END);
	assign vsync  = (vcount >= V_SYNC_START) && (vcount < V_SYNC_END);
	assign pixel_active = !hblank && !vblank;
	assign frame_start = (hcount == 9'd0) && (vcount == 9'd0);
	assign line_start  = (hcount == 9'd0);

endmodule
