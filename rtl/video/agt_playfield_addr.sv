// agt_playfield_addr.sv -- Atari GT playfield tile fetch.
//
// Maps a tile position (col 0-127, row 0-63) to its playfield RAM address,
// reads the word and splits it into (tile_code, color, hflip) for
// agt_tile_decode.sv. From MAME's atarigt_v.cpp:
//
// playfield_scan:
//   index = (((col & 0x40) ^ 0x40) << 6) | ((row & 0x3f) << 6) | (col & 0x3f)
// Not row-major: col bit 6 is inverted and becomes the top index bit, so
// cols 0-63 are in the upper half of the 8192-word RAM (0x1000-0x1FFF) and
// cols 64-127 in the lower half.
//
// get_playfield_tile_info:
//   tile_code = (tile_bank << 12) | (data & 0xfff)
//   color     = (data >> 12) & 7
//   hflip     = data bit 15
// The bank is 4 bits (scanline_update's `word & 15`; the "2 bits" in
// MAME's header comment is wrong), giving 16-bit codes that cover all 65536
// tiles in the ROM.
//
// tile_bank comes from agt_scanline_scroll; this module holds no scroll or
// bank state.

module agt_playfield_addr (
	input  logic       clk,
	input  logic       rst_n,

	input  logic        start,        // 1-cycle pulse
	input  logic [6:0]  col,
	input  logic [5:0]  row,
	input  logic [3:0]  tile_bank,    // from agt_scanline_scroll

	// Playfield RAM port (synchronous, 8192 words)
	output logic [12:0] pfram_addr,
	output logic        pfram_rd,
	input  logic [15:0] pfram_data,
	input  logic        pfram_data_valid,

	// Decoded tile, for agt_tile_decode
	output logic        tile_valid,   // 1-cycle pulse
	output logic [15:0] tile_code,
	output logic [2:0]  tile_color,
	output logic        tile_hflip,

	output logic busy,
	output logic done
);

	typedef enum logic [1:0] { ST_IDLE, ST_REQ, ST_WAIT, ST_DECODE } state_t;
	state_t state;

	// playfield_scan, with explicit 13-bit widths.
	wire [12:0] scan_index = ({12'd0, (col[6] ^ 1'b1)} << 12) |
							  ({7'd0, row}              << 6) |
							  ({7'd0, col[5:0]});

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0;
			done <= 1'b0;
			tile_valid <= 1'b0;
			pfram_rd <= 1'b0;
		end else begin
			done <= 1'b0;
			tile_valid <= 1'b0;
			pfram_rd <= 1'b0;

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy  <= 1'b1;
						state <= ST_REQ;
					end
				end
				ST_REQ: begin
					pfram_addr <= scan_index;
					pfram_rd   <= 1'b1;
					state      <= ST_WAIT;
				end
				ST_WAIT: begin
					if (pfram_data_valid) begin
						tile_code  <= {tile_bank, pfram_data[11:0]};
						tile_color <= pfram_data[14:12];
						tile_hflip <= pfram_data[15];
						tile_valid <= 1'b1;
						busy  <= 1'b0;
						done  <= 1'b1;
						state <= ST_IDLE;
					end
				end
				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
