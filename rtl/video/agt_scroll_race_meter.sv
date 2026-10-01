// agt_scroll_race_meter.sv -- PFLW: is line 0's scroll entry written after
// the core has read it?
//
// agt_video renders line 0 for the coming frame during vblank, reading line
// 0's scroll entry at fetch_line (agt_video LINE0_FETCH_LINE). Primal Rage
// rewrites that entry every vblank on floor stages (0x245C8 the Y word,
// 0x2465C the X word). Line 0's scroll is the sky's, so a write after the
// fetch draws the backdrop one frame stale.
//
// Counts the game's vblank writes to line 0's entry, split at the fetch:
//   late_q  = writes at or after fetch_line (too late for this frame)
//   early_q = writes before fetch_line (in time)
// Latched once per frame on entering fetch_line.
//
// snoop_addr is the long index in the 64 KB shared window (0xD70000).
// Line 0's entry is byte 0xD76060 (alpha word 0x30/0x31), long 0x1818: X
// word in the high half, Y in the low; the game's entry-address helper at
// 0x244FE returns 0xFFD76060 for line 0.

module agt_scroll_race_meter (
	input  wire        clk,
	input  wire        rst_n,
	input  wire        snoop_we,
	input  wire [13:0] snoop_addr,
	input  wire        vblank,
	input  wire [8:0]  vcount,
	input  wire [8:0]  fetch_line,          // 240 (vblank start) or 259
	output logic [15:0] late_q,
	output logic [15:0] early_q
);
	wire is_line0 = snoop_we && (snoop_addr == 14'h1818);
	wire at_fetch = (vcount == fetch_line);

	logic [15:0] late, early;
	logic        fetch_d;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			late <= 16'd0; early <= 16'd0; fetch_d <= 1'b0;
			late_q <= 16'd0; early_q <= 16'd0;
		end else begin
			fetch_d <= at_fetch;
			// Latch on entering the fetch line: `early` holds this vblank's writes so
			// far; `late` the previous vblank's writes at or after its fetch line.
			if (at_fetch && !fetch_d) begin
				late_q <= late; early_q <= early;
				late <= 16'd0;  early <= 16'd0;
				// a write on the fetch line's first cycle is late
				if (is_line0 && vblank) late <= 16'd1;
			end else if (is_line0 && vblank) begin
				if (vcount >= fetch_line) begin
					if (late != 16'hFFFF) late <= late + 16'd1;
				end else begin
					if (early != 16'hFFFF) early <= early + 16'd1;
				end
			end
		end
	end
endmodule
