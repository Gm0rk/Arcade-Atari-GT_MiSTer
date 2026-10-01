// agt_rom_download.sv -- MRA/HPS_IO ROM download stream decoder
//
// hps_io streams the .mra-assembled ROM image one byte per ioctl_wr, at a
// linearly increasing ioctl_addr, while ioctl_download is high. This module
// splits that stream into per-region writes {strobe, region-relative address,
// data}; it stores nothing, so downstream storage need not know the layout.
//
// The layout must match gen_mra.py's LAYOUT table exactly (that script checks
// each region against atarigt.cpp's ROM_LOAD directives):
//
//   0x0000000  maincpu   0x0200000  68020 program (32-bit BE interleaved)
//   0x0200000  tiles     0x0300000  playfield gfx
//   0x0500000  chars     0x0020000  alpha gfx
//   0x0520000  proms     0x0000600  growth-renderer microcode
//   0x0540000  rle       0x2000000  motion-object gfx (16-bit BE interleaved)
//   0x2540000  cage:boot 0x0080000  CAGE boot EPROM, raw file
//   0x25C0000  cage      0x0400000  CAGE sound data, the region's 0x400000
//                                   window = DSP words 0xD00000..
//
// The pad between proms and rle only aligns rle; bytes in it are dropped.
//
// Only ioctl_index 0 (the main ROM payload) is decoded; other indices (e.g.
// NVRAM saves) are ignored rather than misrouted into ROM regions.

module agt_rom_download #(
	// Region sizes are parameters because T-MEK's layout differs from Primal
	// Rage's. The defaults are Primal Rage's; a T-MEK build overrides them to
	// match its own .mra.
	parameter int SZ_MAINCPU = 'h0200000,
	parameter int SZ_TILES   = 'h0300000,
	parameter int SZ_CHARS   = 'h0020000,
	parameter int SZ_PROMS   = 'h0000600,
	parameter int SZ_RLE     = 'h2000000,
	// rle is aligned, not packed against proms; the pad decodes to no region.
	parameter int BASE_RLE_P = 'h0540000,
	// CAGE boot EPROM, appended after rle. Regions are split by fixed address
	// ranges, so appending is free but inserting earlier would shift every
	// later region: new regions must go at the end.
	//
	// 0x80000 is the raw file size. MAME's `cage:boot` is a 2 MB
	// ROM_REGION32_LE loaded at stride 4, but the C3x boot loader reads an
	// 8-bit source, so a DSP word is four consecutive file bytes. We stream
	// the file, not the region.
	parameter int SZ_CAGEBOOT = 'h0080000,
	// CAGE sound data, appended after cage:boot.
	//
	// MAME's `cage` is a 16 MB ROM_REGION32_LE whose loads start at region
	// byte 0x400000 = DSP word 0xD00000 (cage_map's 0xC00000-0xFFFFFF bank).
	// gen_mra.py streams the window from 0x400000 to the end of the loads, so
	// the decoder's address is (word - 0xD00000)*4.
	//
	// 0x400000 is Primal Rage's window. T-MEK's is 0x600000 with a lane shape
	// gen_mra.py cannot express as one <interleave>, so a T-MEK .mra carries
	// no sound data and this would be 0 for it.
	parameter int SZ_CAGE = 'h0400000
) (
	input  logic        clk,
	input  logic        rst_n,

	// hps_io ioctl download interface
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic [7:0]  ioctl_dout,
	input  logic [7:0]  ioctl_index,

	// Decoded per-region write stream. Exactly one *_wr is high at a time;
	// *_addr is region-relative.
	output logic        maincpu_wr,
	output logic [20:0] maincpu_addr,   // 0x200000
	output logic        tiles_wr,
	output logic [21:0] tiles_addr,     // 0x300000
	output logic        chars_wr,
	output logic [16:0] chars_addr,     // 0x020000
	output logic        proms_wr,
	output logic [10:0] proms_addr,     // 0x000600
	output logic        rle_wr,
	output logic [25:0] rle_addr,       // 0x2000000, 27-bit map
	// CAGE boot EPROM. agt_cage_boot parses this stream as it arrives and
	// stores nothing; the address is exported for the bench.
	output logic        cageboot_wr,
	output logic [18:0] cageboot_addr,  // 0x080000
	// CAGE sound data. Byte 0 is DSP word 0xD00000; 23 bits covers Primal
	// Rage's 4 MB and T-MEK's 6 MB.
	output logic        cage_wr,
	output logic [22:0] cage_addr,      // 0x400000 (Primal Rage)
	output logic [7:0]  rom_data,

	// status
	output logic        rom_loading,    // high for the duration of the download
	output logic        rom_loaded,     // latches high once a download completes

	// Game selection: the .mra carries a one-byte payload on rom index 1
	// (MiSTer's convention for multi-game cores). T-MEK and Primal Rage
	// differ in the colour-mix stage and the protection device.
	//
	// 0 = T-MEK, 1 = Primal Rage. Defaults to 1 so an .mra without the byte
	// runs Primal Rage rather than the unimplemented T-MEK paths.
	output logic [7:0]  game_id,

	// Control-panel variant (rom index 1, byte 1). The Primal Rage revisions
	// share every ROM region but not their control panel (atarigt.cpp's
	// primrage vs primrageo input ports): the P1_P2 bits carrying
	// START1/START2 on the parent carry BUTTON1 on the older sets. game_id
	// cannot tell revisions apart.
	//
	//   0 = dedicated start button      (primrage, v2.3 Jan 1995)
	//   1 = button 1 doubles as start   (primrageo v2.3 Dec 1994,
	//                                    primrage20 v2.0)
	//
	// Defaults to 0 (the parent layout) for an .mra without this byte.
	output logic [7:0]  panel_id
);

	localparam logic [26:0] SIZE_MAINCPU = SZ_MAINCPU[26:0];
	localparam logic [26:0] SIZE_TILES   = SZ_TILES[26:0];
	localparam logic [26:0] SIZE_CHARS   = SZ_CHARS[26:0];
	localparam logic [26:0] SIZE_PROMS   = SZ_PROMS[26:0];
	localparam logic [26:0] SIZE_RLE     = SZ_RLE[26:0];
	localparam logic [26:0] SIZE_CAGEBOOT = SZ_CAGEBOOT[26:0];
	localparam logic [26:0] SIZE_CAGE     = SZ_CAGE[26:0];

	localparam logic [26:0] BASE_MAINCPU = 26'h0000000;
	localparam logic [26:0] BASE_TILES   = BASE_MAINCPU + SIZE_MAINCPU;
	localparam logic [26:0] BASE_CHARS   = BASE_TILES   + SIZE_TILES;
	localparam logic [26:0] BASE_PROMS   = BASE_CHARS   + SIZE_CHARS;
	localparam logic [26:0] BASE_RLE     = BASE_RLE_P[26:0];
	localparam logic [26:0] BASE_CAGEBOOT = BASE_RLE + SIZE_RLE;
	localparam logic [26:0] BASE_CAGE     = BASE_CAGEBOOT + SIZE_CAGEBOOT;

	wire index0 = (ioctl_index == 8'd0);
	wire index1 = (ioctl_index == 8'd1);
	// Strictly index 0: accepting any other index would let e.g. DIP data
	// (index 254) be written into the ROM regions.
	wire wr     = ioctl_download && ioctl_wr && index0;

	wire in_maincpu = (ioctl_addr >= BASE_MAINCPU) && (ioctl_addr < BASE_MAINCPU + SIZE_MAINCPU);
	wire in_tiles   = (ioctl_addr >= BASE_TILES)   && (ioctl_addr < BASE_TILES   + SIZE_TILES);
	wire in_chars   = (ioctl_addr >= BASE_CHARS)   && (ioctl_addr < BASE_CHARS   + SIZE_CHARS);
	wire in_proms   = (ioctl_addr >= BASE_PROMS)   && (ioctl_addr < BASE_PROMS   + SIZE_PROMS);
	wire in_rle     = (ioctl_addr >= BASE_RLE)     && (ioctl_addr < BASE_RLE     + SIZE_RLE);
	wire in_cageboot = (ioctl_addr >= BASE_CAGEBOOT) &&
					   (ioctl_addr <  BASE_CAGEBOOT + SIZE_CAGEBOOT);
	wire in_cage     = (ioctl_addr >= BASE_CAGE) &&
					   (ioctl_addr <  BASE_CAGE + SIZE_CAGE);

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			maincpu_wr <= 1'b0;
			tiles_wr   <= 1'b0;
			chars_wr   <= 1'b0;
			proms_wr   <= 1'b0;
			rle_wr     <= 1'b0;
			cageboot_wr <= 1'b0;
			cage_wr     <= 1'b0;
		end else begin
			maincpu_wr <= wr && in_maincpu;
			tiles_wr   <= wr && in_tiles;
			chars_wr   <= wr && in_chars;
			proms_wr   <= wr && in_proms;
			rle_wr     <= wr && in_rle;
			cageboot_wr <= wr && in_cageboot;
			cage_wr     <= wr && in_cage;

			// Region offsets are deliberately narrower than ioctl_addr's 27
			// bits; the explicit casts say so.
			rom_data     <= ioctl_dout;
			// maincpu: linear copy. The .mra's <interleave output="32"> makes
			// MiSTer's loader weave the four program ROMs, so ioctl already
			// delivers the assembled 32-bit big-endian image. Do not
			// deinterleave again here.
			maincpu_addr <= 21'((ioctl_addr - BASE_MAINCPU));
			tiles_addr   <= 22'((ioctl_addr - BASE_TILES));
			chars_addr   <= 17'((ioctl_addr - BASE_CHARS));
			proms_addr   <= 11'((ioctl_addr - BASE_PROMS));
			rle_addr     <= 26'((ioctl_addr - BASE_RLE));
			cageboot_addr <= 19'((ioctl_addr - BASE_CAGEBOOT));
			cage_addr     <= 23'((ioctl_addr - BASE_CAGE));
		end
	end

	// download status
	logic download_d;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rom_loading <= 1'b0;
			rom_loaded  <= 1'b0;
			download_d  <= 1'b0;
			game_id     <= 8'd1;   // default: Primal Rage (see port comment)
			panel_id    <= 8'd0;   // default: dedicated start (parent set)
		end else begin
			download_d  <= ioctl_download;
			// The index-1 payload is two bytes, so each latch is qualified by
			// address.
			if (ioctl_download && ioctl_wr && index1) begin
				if (ioctl_addr == 26'd0) game_id  <= ioctl_dout;
				if (ioctl_addr == 27'd1) panel_id <= ioctl_dout;
			end
			rom_loading <= ioctl_download && index0;
			// Any new download invalidates the previous image; completion
			// re-validates. Not gated on index0: the index arrives with the
			// strobes, not at the rise.
			if (ioctl_download && !download_d) rom_loaded <= 1'b0;
			// falling edge of a download => image is in memory
			if (download_d && !ioctl_download) rom_loaded <= 1'b1;
		end
	end

endmodule
