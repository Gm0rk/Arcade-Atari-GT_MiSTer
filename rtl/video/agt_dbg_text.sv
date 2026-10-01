// agt_dbg_text.sv -- hex text overlay for hardware bring-up
//
// Puts debug values on screen as readable hex, so they can be read straight
// off a photograph instead of being decoded from the block overlay.
//
// Layout: 13 lines x 13 chars in 8x8 cells, "LLLL VVVVVVVV":
//   cols 0-3  4-character label (constant per line)
//   col  4    space
//   cols 5-12 the 32-bit value, most significant nibble first
// 104 x 104 native pixels, placed below the block rows.
//
// Row 0 is the build stamp, "BLD YYMMDDNNN", a constant input rather than a
// dbg_words slot. Its label's fourth character is already a space, so its
// nine digits fill cols 4-12. Rows 1..12 are dbg_words 0..11.
//
// The font is a localparam, not a $readmemh file: a missing init file is only
// a Critical Warning and zero-fills, so the text would silently vanish.
// Tables are packed flat because iverilog -g2012 rejects assignment patterns
// on unpacked localparam arrays, and this file must elaborate in both
// toolchains.
`default_nettype none

module agt_dbg_text #(
	// Guards against a stale copy of this file: a wider dbg_words on an older,
	// narrower port would silently keep only the low slots, but overriding a
	// parameter the old file lacks fails to elaborate.
	parameter int NSLOTS = 12,
	// Same guard for the build stamp (an older 32-bit port would drop the
	// year's tens digit). Nine is the only value the 13-column layout supports.
	parameter int BUILD_DIGITS = 9,
	parameter int X0 = 8,
	parameter int Y0 = 80
) (
	input  wire  [8:0]   hcount,
	input  wire  [8:0]   vcount,
	input  wire  [NSLOTS*32-1:0] dbg_words,   // slot N at [N*32 +: 32]
	// BCD {YYMMDD, 3-digit build tag}; constant.
	input  wire  [BUILD_DIGITS*4-1:0] build_id,
	output logic         text_on
);

	localparam logic [40*64-1:0] FONT_FLAT = {
		64'h7088081020002000, // 39 query
		64'h0000000000606000, // 38 dot
		64'h000000F800000000, // 37 dash
		64'h0000000000000000, // 36 space
		64'hF80810204080F800, // 35 Z
		64'h8888502020202000, // 34 Y
		64'h8888502050888800, // 33 X
		64'h888888A8A8D88800, // 32 W
		64'h8888888888502000, // 31 V
		64'h8888888888887000, // 30 U
		64'hF820202020202000, // 29 T
		64'h788080700808F000, // 28 S
		64'hF08888F0A0908800, // 27 R
		64'h70888888A8906800, // 26 Q
		64'hF08888F080808000, // 25 P
		64'h7088888888887000, // 24 O
		64'h88C8A89888888800, // 23 N
		64'h88D8A8A888888800, // 22 M
		64'h808080808080F800, // 21 L
		64'h8890A0C0A0908800, // 20 K
		64'h3810101010906000, // 19 J
		64'h7020202020207000, // 18 I
		64'h888888F888888800, // 17 H
		64'h708880B888887800, // 16 G
		64'hF88080F080808000, // 15 F
		64'hF88080F08080F800, // 14 E
		64'hE09088888890E000, // 13 D
		64'h7088808080887000, // 12 C
		64'hF08888F08888F000, // 11 B
		64'h708888F888888800, // 10 A
		64'h7088887808106000, // 9 9
		64'h7088887088887000, // 8 8
		64'hF808102040404000, // 7 7
		64'h304080F088887000, // 6 6
		64'hF880F00808887000, // 5 5
		64'h10305090F8101000, // 4 4
		64'hF810201008887000, // 3 3
		64'h708808102040F800, // 2 2
		64'h2060202020207000, // 1 1
		64'h708898A8C8887000  // 0 0
	};

	// Labels must be changed together with whatever drives dbg_words: a label
	// that no longer matches its value is worse than none, because it will be
	// believed. Entries are FONT_FLAT indices, packed MSB-first, so the first
	// one is slot 11's col 3. tools/gen_dbg_labels.py emits this block and
	// checks it against the dbg_words annotations in Arcade-Atari-GT.sv.
	localparam logic [12*4*8-1:0] LABEL_FLAT = {
		8'd14,
		8'd28,
		8'd21,
		8'd27,
		8'd29,
		8'd21,
		8'd10,
		8'd17,
		8'd23,
		8'd19,
		8'd11,
		8'd24,
		8'd32,
		8'd12,
		8'd10,
		8'd13,
		8'd29,
		8'd10,
		8'd29,
		8'd28,
		8'd21,
		8'd27,
		8'd31,
		8'd24,
		8'd21,
		8'd29,
		8'd28,
		8'd13,
		8'd29,
		8'd21,
		8'd17,
		8'd12,
		8'd18,
		8'd30,
		8'd25,
		8'd12,
		8'd23,
		8'd27,
		8'd31,
		8'd24,
		8'd27,
		8'd30,
		8'd13,
		8'd27,
		8'd36,
		8'd36,
		8'd12,
		8'd25
	};

	// Row 0's label, packed like LABEL_FLAT (col 3 first): "BLD ".
	localparam logic [4*8-1:0] BUILD_LABEL = {8'd36, 8'd13, 8'd21, 8'd11};

	wire [8:0] rx = hcount - X0[8:0];
	wire [8:0] ry = vcount - Y0[8:0];
	// 13 rows of 8 px = 104. At Y0 = 80 the box ends at line 184 of the 240
	// visible.
	localparam int BOX_H = (NSLOTS + 1) * 8;   // build stamp + NSLOTS rows
	wire in_box = (hcount >= X0[8:0]) && (rx < 9'd104) &&
				  (vcount >= Y0[8:0]) && (ry < BOX_H[8:0]);

	wire [3:0] cx   = rx[6:3];      // character column 0..12
	wire [3:0] rowi = ry[6:3];      // screen row 0..12; 0 is the build stamp
	wire       is_build = (rowi == 4'd0);
	// Rows 1..12 map to dbg_words 0..11. Held at 0 on the build row, where it
	// is unused: an unqualified rowi-1 would wrap and read a slot.
	wire [3:0] cy   = is_build ? 4'd0 : (rowi - 4'd1);
	wire [2:0] px = rx[2:0];
	wire [2:0] py = ry[2:0];

	// 36 bits so the build row's ninth digit fits. Col 12 is nibble 0 on every
	// row, so `12 - cx` gives nibbles 7..0 on cols 5..12 for a trace and 8..0
	// on cols 4..12 for the build stamp. A trace's col 4 is the space, decided
	// first below.
	wire [35:0] val     = is_build ? build_id : {4'd0, dbg_words[{cy, 5'd0} +: 32]};
	wire [3:0]  nib_sel = 4'd12 - cx;                // col 12 = nibble 0
	wire [3:0]  nib     = val[{nib_sel, 2'd0} +: 4];

	logic [5:0] code;
	always_comb begin
		if (cx <= 4'd3)
			code = is_build ? BUILD_LABEL[{cx[1:0], 3'd0} +: 6]
							: LABEL_FLAT[{cy, cx[1:0], 3'd0} +: 6];
		else if (cx == 4'd4 && !is_build)
			code = 6'd36;                      // space (the build row's is col 3)
		else
			code = {2'd0, nib};                // '0'-'9','A'-'F'
	end

	wire [63:0] glyph = FONT_FLAT[{code, 6'd0} +: 64];
	wire [7:0]  row   = glyph[{(3'd7 - py), 3'd0} +: 8];

	always_comb text_on = in_box && row[3'd7 - px];

endmodule

`default_nettype wire
