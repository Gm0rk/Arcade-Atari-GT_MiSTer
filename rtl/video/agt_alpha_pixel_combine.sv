// agt_alpha_pixel_combine.sv -- builds the alpha pen value.
//
// Combines a 4-bit alpha tile pixel (agt_alpha_tile_decode) with the tile's
// 4-bit colour (agt_alpha_addr) into the an_pixel that
// agt_colormix_primrage expects. MAME's pen rule is
//   an_pixel = color_codes_start + tile_color*granularity + raw_pixel
// with color_codes_start = 0 and granularity 16 (atarigt.cpp's
// GFXDECODE_ENTRY("chars", 0, gfx_8x8x4_packed_msb, 0x000, 16)). The pixel
// fills exactly 4 bits, so the add never carries and is a concatenation.

module agt_alpha_pixel_combine (
	input  logic [3:0]  raw_pixel,
	input  logic [3:0]  tile_color,
	output logic [15:0] an_pixel
);

	assign an_pixel = {8'd0, tile_color, raw_pixel};

endmodule
