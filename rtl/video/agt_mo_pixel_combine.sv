// agt_mo_pixel_combine.sv -- builds the MO/TMO VRAM pixel value.
//
// Adds a decoded RLE pixel (0-63, agt_rle_blit's pix_value) to the object's
// 16-bit colour/priority field (agt_rle_objlist's color), as MAME's
// atarirle.cpp does:
//   palettebase  = (m_palettebase + color) & ~((1 << bpp) - 1)
//   stored_value = raw_pixel_value + palettebase
// m_palettebase is 0 for Primal Rage (atarigt.cpp's modesc). Transparent
// pixels (value 0) are never written; agt_rle_blit's pix_valid already
// excludes them.

module agt_mo_pixel_combine (
	input  logic [5:0]  raw_pixel_value,
	input  logic [15:0] color,
	// Object colour depth, 4/5/6, from m_rle_bpp[table_sel].
	input  logic [2:0]  bpp,
	output logic [15:0] mo_pixel
);

	// The pen base is aligned to the object's depth so colour and pixel bits
	// cannot overlap (e.g. 6bpp pixel [5:0] vs colour at [11:4]); otherwise the
	// add carries into the colour field. MAME: "Fixes the yellow blood in
	// Primal Rage".
	wire [15:0] depth_mask = ~((16'd1 << bpp) - 16'd1);
	assign mo_pixel = ((({10'd0, raw_pixel_value}) + (color & depth_mask)))
					  & 16'hffff;

endmodule
