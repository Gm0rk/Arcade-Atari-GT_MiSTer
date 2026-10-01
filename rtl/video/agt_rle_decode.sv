// agt_rle_decode.sv -- Atari GT RLE motion-object byte decoder.
//
// Decodes one compressed byte from the "rle" ROM into (run_count,
// pixel_value), matching MAME's build_rle_tables() (atarirle.cpp). Purely
// combinational. The row sequencer decodes each 16-bit RLE word low byte
// first, then high byte, as MAME's draw_rle_zoom does.
//
// Each object selects its table with a 3-bit table_sel from header word 2
// bits [10:8] (see agt_rle_objtable.sv). 5bpp and 6bpp each have a "plain"
// table and an "escape" table, where a byte with low nibble 0 is a long run
// of transparent pixels instead. 4bpp has one table.
//   table_sel 0    -> 4bpp
//   table_sel 1    -> 5bpp escape
//   table_sel 2, 3 -> 5bpp plain
//   table_sel 4, 6 -> 6bpp escape
//   table_sel 5, 7 -> 6bpp plain
//
//   4bpp        : count = byte[7:4]+1 (1-16), value = byte[3:0] (0-15)
//   5bpp escape : byte[3:0]==0: count = byte[7:4]+1 (1-16), value = 0
//                 else        : count = byte[7:5]+1 (1-8),  value = byte[4:0]
//   5bpp plain  : count = byte[7:5]+1 (1-8),  value = byte[4:0] (0-31)
//   6bpp escape : byte[3:0]==0: count = byte[7:4]+1 (1-16), value = 0
//                 else        : count = byte[7:6]+1 (1-4),  value = byte[5:0]
//   6bpp plain  : count = byte[7:6]+1 (1-4),  value = byte[5:0] (0-63)
//
// pixel_value 0 is transparent; the blitter (agt_rle_blit.sv) skips it.
// Outputs are unregistered; the caller registers them if needed.

module agt_rle_decode (
	input  logic [7:0] byte_in,     // byte from the "rle" ROM
	input  logic [2:0] table_sel,   // object header word2[10:8]
	output logic [4:0] run_count,   // run length, 1-16
	output logic [5:0] pixel_value  // raw pixel index, 0-63 (pre-palette)
);

	// Long transparent-run escape code (used by table_sel 1, 4 and 6 only).
	wire is_escape_code = (byte_in[3:0] == 4'h0);

	always_comb begin
		unique case (table_sel)

			// 4bpp
			3'd0: begin
				run_count   = {1'b0, byte_in[7:4]} + 5'd1;
				pixel_value = {2'b00, byte_in[3:0]};
			end

			// 5bpp escape
			3'd1: begin
				if (is_escape_code) begin
					run_count   = {1'b0, byte_in[7:4]} + 5'd1;
					pixel_value = 6'd0;
				end else begin
					run_count   = {2'b00, byte_in[7:5]} + 5'd1;
					pixel_value = {1'b0, byte_in[4:0]};
				end
			end

			// 5bpp plain (2 and 3 alias the same table)
			3'd2, 3'd3: begin
				run_count   = {2'b00, byte_in[7:5]} + 5'd1;
				pixel_value = {1'b0, byte_in[4:0]};
			end

			// 6bpp escape
			3'd4, 3'd6: begin
				if (is_escape_code) begin
					run_count   = {1'b0, byte_in[7:4]} + 5'd1;
					pixel_value = 6'd0;
				end else begin
					run_count   = {3'b000, byte_in[7:6]} + 5'd1;
					pixel_value = byte_in[5:0];
				end
			end

			// 6bpp plain
			3'd5, 3'd7: begin
				run_count   = {3'b000, byte_in[7:6]} + 5'd1;
				pixel_value = byte_in[5:0];
			end

			// Unreachable: all 8 values are covered above.
			default: begin
				run_count   = 5'd1;
				pixel_value = 6'd0;
			end
		endcase
	end

endmodule
