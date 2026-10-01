// agt_playfield_pixel_combine.sv -- builds the playfield pen value.
//
// Combines a 6-bit tile pixel (agt_tile_decode), the tile's 3-bit colour
// (agt_playfield_addr) and the 5-bit playfield colour bank
// (agt_scanline_scroll) into screen_update's pf[x], using MAME's tilemap
// pen rule:
//   pf_pixel = (color_bank << 8) + (tile_color << 5) + raw_pixel
// This is an add, not an OR: blend_gfx keeps pflayout's granularity of 32
// while the blended pixel has 64 values, so the pixel carries into the
// colour field.

module agt_playfield_pixel_combine (
	input  logic [5:0] raw_pixel,
	input  logic [2:0] tile_color,
	input  logic [4:0] color_bank,
	output logic [15:0] pf_pixel
);

	assign pf_pixel = ({11'd0, color_bank} << 8) + ({13'd0, tile_color} << 5) + {10'd0, raw_pixel};

endmodule
