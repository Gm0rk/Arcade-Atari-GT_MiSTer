// agt_scanline_scroll.sv -- Atari GT per-scanline playfield scroll/bank regs
//
// Implements MAME's scanline_update (atarigt_v.cpp:142-196). Each scanline,
// two control words are read from fixed slots in the alpha tilemap's own RAM
// (the CPU parks them in tile positions that hold no visible characters). If
// a word's valid bit (15) is set, it updates the playfield registers:
//
//   X-word (offset):   xscroll   = (word >> 5) & 0x3ff;
//                      color_bank = word & 0x1f
//   Y-word (offset+1): yscroll   = ((word >> 6) - scanline) & 0x1ff;
//                      tile_bank = word & 0xf
//   offset = ((scanline & ~7) << 3) + 48 + 2*(scanline & 7)
//
// offset >= 0x800 means the line has no control slot (MAME's range guard,
// which covers the non-visible lines); it is skipped.
//
// MAME calls the function once per 8 lines and loops over i=0..7; here it
// runs once per real scanline with one word-pair each, which gives the same
// sequence of reads and updates. MAME's update_partial()/mark_all_dirty()
// side effects are emulator redraw bookkeeping and are not modelled.

module agt_scanline_scroll (
	input  logic       clk,
	input  logic       rst_n,

	input  logic        start,        // 1-cycle pulse: process this line
	input  logic [8:0]  scanline,     // 0-261

	output logic [10:0] alpharam_addr,
	output logic        alpharam_rd,
	input  logic [15:0] alpharam_data,
	input  logic        alpharam_data_valid,

	output logic [9:0] xscroll,
	output logic [8:0] yscroll,
	output logic [4:0] color_bank,
	output logic [3:0] tile_bank,
	// Raw control words as read from alpha RAM, for the last scanline that had
	// a valid Y-word (debug overlay).
	output logic [15:0] dbg_xword,
	output logic [15:0] dbg_yword,

	output logic busy,
	output logic done
);

	typedef enum logic [2:0] { ST_IDLE, ST_CHECK, ST_REQ_X, ST_WAIT_X, ST_REQ_Y, ST_WAIT_Y, ST_UPDATE } state_t;
	state_t state;

	logic [8:0]  scanline_r;
	logic [11:0] offset;        // 12 bits to detect >= 0x800
	logic [15:0] xword_r;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0; alpharam_rd <= 1'b0;
			xscroll <= 10'd0; yscroll <= 9'd0; color_bank <= 5'd0; tile_bank <= 4'd0;
			dbg_xword <= 16'd0; dbg_yword <= 16'd0;
		end else begin
			done <= 1'b0;
			alpharam_rd <= 1'b0;

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy <= 1'b1;
						scanline_r <= scanline;
						// offset = ((scanline & ~7) << 3) + 48 + 2*(scanline & 7)
						offset <= ({6'd0, scanline[8:3], 3'b000} << 3) + 12'd48 + {8'd0, scanline[2:0], 1'b0};
						state <= ST_CHECK;
					end
				end

				ST_CHECK: begin
					if (offset >= 12'h800) begin
						// no control slot on this line
						busy <= 1'b0;
						done <= 1'b1;
						state <= ST_IDLE;
					end else begin
						state <= ST_REQ_X;
					end
				end

				ST_REQ_X: begin
					alpharam_addr <= offset[10:0];
					alpharam_rd   <= 1'b1;
					state         <= ST_WAIT_X;
				end
				ST_WAIT_X: begin
					if (alpharam_data_valid) begin
						xword_r <= alpharam_data;
						state   <= ST_REQ_Y;
					end
				end

				ST_REQ_Y: begin
					alpharam_addr <= offset[10:0] + 11'd1;
					alpharam_rd   <= 1'b1;
					state         <= ST_WAIT_Y;
				end
				ST_WAIT_Y: begin
					if (alpharam_data_valid) begin
						state <= ST_UPDATE;

						// X-word (latched in xword_r)
						if (xword_r[15]) begin
							xscroll    <= xword_r[14:5];
							color_bank <= xword_r[4:0];
						end
						// Y-word (alpharam_data this cycle)
						if (alpharam_data[15]) begin
							yscroll   <= (alpharam_data[14:6] - scanline_r) & 9'h1ff;
							tile_bank <= alpharam_data[3:0];
							dbg_xword <= xword_r;
							dbg_yword <= alpharam_data;
						end
					end
				end

				ST_UPDATE: begin
					busy  <= 1'b0;
					done  <= 1'b1;
					state <= ST_IDLE;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
