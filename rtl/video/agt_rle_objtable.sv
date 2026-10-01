// agt_rle_objtable.sv -- Atari GT RLE motion-object header lookup.
//
// Given a 16-bit motion-object code (from an object-list entry in shared
// RAM), fetches the object's 4-word header from the "rle" gfx ROM and
// decodes it into what the row/blit engine needs: xoffs, yoffs, table_sel
// (which RLE table in agt_rle_decode.sv applies) and data_offset (word
// address of the object's compressed pixel stream in the same ROM).
//
// Translates the header-parsing half of MAME's prescan_rle()
// (atarirle.cpp:350-374). The width/height pre-scan that follows it in MAME
// has no hardware equivalent: width and height emerge from walking rows in
// the scaled blit (agt_rle_blit.sv).
//
// Header (4 words at ROM word address code*4):
//   word0        : xoffs, signed 16-bit
//   word1        : yoffs, signed 16-bit
//   word2[15:11] : unused by this hardware revision
//   word2[10:8]  : table_sel (0-7) -- selects one of 5 RLE decode tables
//   word2[7:0]   : high byte of a 24-bit data offset
//   word3        : low 16 bits of the 24-bit data offset
//   data_offset = {word2[7:0], word3}   (word-indexed, not byte-indexed)
//
// As in MAME, an object is invalid if data_offset < code*4 (points into the
// header table) or data_offset >= ROM_NUM_WORDS (past the populated ROM).
// Callers must skip rendering when valid == 0, as MAME's draw_rle() does
// when info.data == nullptr.
//
// ROM port: word address + read strobe out; the ROM side returns one
// 16-bit word with a one-cycle rom_data_valid pulse. Latency is not assumed
// fixed (tb_prage_rle_objtable.sv models 2 cycles).
//
// One lookup is 4 sequential word reads. `start` is a single-cycle pulse,
// ignored while busy. `done` pulses for one cycle with all outputs valid.

module agt_rle_objtable #(
	parameter int ROM_ADDR_WIDTH               = 24,        // word address; fits the 24-bit offset field
	// ROM_NUM_WORDS is a count (exclusive bound), not an address: Primal
	// Rage's 2^24 words need one more bit than ROM_ADDR_WIDTH, and data_offset
	// is zero-extended by that bit for the compare. Truncating it to 24 bits
	// makes `valid` always false. Words populated in the game's "rle" region:
	//   Primal Rage: 0x2000000 bytes / 2 = 0x1000000 words
	//   T-Mek:       0x1000000 bytes / 2 = 0x800000  words
	parameter logic [ROM_ADDR_WIDTH:0] ROM_NUM_WORDS = 25'h1000000
) (
	input  logic clk,
	input  logic rst_n,

	// request
	input  logic        start,   // 1-cycle pulse: begin lookup for `code`
	input  logic [15:0] code,    // from the object list entry

	// generic synchronous ROM port (word-addressed)
	output logic [ROM_ADDR_WIDTH-1:0] rom_addr,
	output logic                      rom_rd,
	input  logic [15:0]               rom_data,
	input  logic                      rom_data_valid,

	// decoded header, valid for the cycle `done` is high
	output logic                      done,
	output logic signed [15:0]        xoffs,
	output logic signed [15:0]        yoffs,
	output logic [2:0]                table_sel,
	output logic [ROM_ADDR_WIDTH-1:0] data_offset,
	output logic                      valid
);

	typedef enum logic [1:0] {
		ST_IDLE,
		ST_REQ,    // drive rom_addr/rom_rd for header word
		ST_WAIT,   // wait for rom_data_valid
		ST_DONE    // one-cycle pulse of the decoded outputs
	} state_t;

	state_t state;
	logic [1:0] word_idx;                       // header word being fetched (0-3)
	logic [ROM_ADDR_WIDTH-1:0] base_addr;       // code * 4, latched at `start`
	logic [15:0] hdr [0:3];                     // raw header words

	// Explicit zero-extend (not a width cast) so simulators and synthesis
	// agree; 4 words per header entry.
	wire [ROM_ADDR_WIDTH-1:0] code_zext = {{(ROM_ADDR_WIDTH-16){1'b0}}, code};

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state     <= ST_IDLE;
			rom_rd    <= 1'b0;
			done      <= 1'b0;
			word_idx  <= 2'd0;
			base_addr <= '0;
		end else begin
			done   <= 1'b0;
			rom_rd <= 1'b0;

			unique case (state)

				ST_IDLE: begin
					if (start) begin
						base_addr <= code_zext << 2;
						word_idx  <= 2'd0;
						state     <= ST_REQ;
					end
				end

				ST_REQ: begin
					rom_addr <= base_addr + ROM_ADDR_WIDTH'(word_idx);
					rom_rd   <= 1'b1;
					state    <= ST_WAIT;
				end

				// latency is not assumed fixed
				ST_WAIT: begin
					if (rom_data_valid) begin
						hdr[word_idx] <= rom_data;
						if (word_idx == 2'd3) begin
							state <= ST_DONE;
						end else begin
							word_idx <= word_idx + 2'd1;
							state    <= ST_REQ;
						end
					end
				end

				// all 4 header words latched: decode and pulse `done`
				ST_DONE: begin
					// $signed(), not signed'(): Icarus 11 rejects the
					// apostrophe cast. Equivalent here, since hdr and both
					// destinations are 16 bits wide.
					xoffs       <= $signed(hdr[0]);
					yoffs       <= $signed(hdr[1]);
					table_sel   <= hdr[2][10:8];
					data_offset <= {hdr[2][7:0], hdr[3]};
					// valid = data_offset in [base_addr, ROM_NUM_WORDS);
					// data_offset is zero-extended to ROM_NUM_WORDS' width.
					valid       <= ({hdr[2][7:0], hdr[3]} >= base_addr) &&
								   ({1'b0, hdr[2][7:0], hdr[3]} < ROM_NUM_WORDS);
					done        <= 1'b1;
					state       <= ST_IDLE;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
