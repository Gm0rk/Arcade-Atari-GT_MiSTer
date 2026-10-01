// agt_demo_memories.sv -- BRAM-backed video memories: tilemaps, object list,
// colorram.
//
// Mirrors of the game's playfield, alpha and object-list RAM, filled from the
// board core's shared-RAM write snoop; an object-list snapshot for the sprite
// renderer; and an agt_colorram instance. On hardware every array starts at
// zero and holds only what the game writes.
//
// Simulation only (`AGT_SIM_DEMO_SCENE`): a demo scene -- the playfield
// tilemap, a palette replay and a windowed slice of the tile ROM -- from hex
// files generated from the game ROMs (gen_demo_files.py). They are game data,
// so they never reach a build.
//
// Tile ROM windowing: for tile codes 0-511 the decode layouts only touch
// three 8KB windows of the tiles region, 1MB apart (the bitplanes are split
// across the region), folded into one contiguous array:
//   0x000000-0x001FFF -> 0x0000-0x1FFF
//   0x100000-0x101FFF -> 0x2000-0x3FFF
//   0x200000-0x201FFF -> 0x4000-0x5FFF
// Accesses outside the windows return 0 (blank pixels, not garbage).
//
// Colorram init (simulation only): a replay FSM streams (addr,data) pairs
// from a table into agt_colorram's write port after reset, and only then
// releases video_rst_n, so the palette is set up before rendering starts.
// Without the demo scene the FSM releases video_rst_n straight away.

module agt_demo_memories #(
	// 1 = build the object-list snapshot (MLAB, no M10K). 0 = the renderer
	// reads the live mirror; obj_use_live and obj_snap_start are ignored.
	parameter bit OBJ_SNAPSHOT = 1'b1
) (
	input  logic clk_sys,
	input  logic rst_n,           // system reset (active low)

	output logic video_rst_n,     // released only after colorram init completes

	// playfield RAM port (matches agt_video's)
	input  logic [12:0] pfram_addr,
	input  logic        pfram_rd,
	output logic [15:0] pfram_data,
	output logic        pfram_data_valid,

	// tile ROM port
	input  logic [21:0] tile_rom_addr,
	input  logic        tile_rom_rd,
	// burst3: the decoder's USE_BURST3 path asks for all three planes in one
	// request. The SDRAM side answers with a 3-read burst; here the planes
	// are three BRAM banks read together.
	input  logic        tile_rom_rd3,
	output logic [95:0] tile_rom_data96,
	output logic [7:0]  tile_rom_data,
	output logic [31:0] tile_rom_data32,
	output logic        tile_rom_data_valid,

	// alpha RAM port
	input  logic [10:0] alpharam_addr,
	input  logic        alpharam_rd,
	output logic [15:0] alpharam_data,
	output logic        alpharam_data_valid,

	// colorram lookup ports (pass-through from an internal instance)
	input  logic [13:0] cram_addr,
	output logic [15:0] cram_data,
	output logic [15:0] color_latch,
	input  logic [14:0] pen_addr_r,
	input  logic [14:0] pen_addr_g,
	input  logic [14:0] pen_addr_b,
	output logic [23:0] pen_data_r,
	output logic [23:0] pen_data_g,
	output logic [23:0] pen_data_b,

	// CPU colorram write-through (clk_sys), decoded by agt_colorram into
	// cram/color_latch/pens as MAME's colorram_w does. The init FSM owns the
	// write port from reset until INIT_DONE, then hands it over for good; the
	// CPU is held in reset through the ROM download (far longer than the
	// 1154-cycle init), so the two writers never overlap.
	//
	// pal_fixture_en: 1 replays the demo palette at boot (512 entries into cram
	// words 0x000-0x0FFF, none into the MO palette); 0 skips it, so only the
	// game's own writes set the colours. No effect without AGT_SIM_DEMO_SCENE.
	input  logic        pal_fixture_en,
	input  logic        cpu_cr_wr,
	input  logic [18:0] cpu_cr_addr,
	input  logic [15:0] cpu_cr_data,

	// Shared-RAM write snoop (from the board core): every CPU write beat into
	// shared RAM. Mirrored here: the playfield window (shared words
	// 0x0800-0x17FF = 0xD72000-0xD75FFF), the alpha window (0x1800-0x1BFF =
	// 0xD76000-0xD76FFF) and the object list. The demo content seeds the
	// arrays at power-up and the game overwrites it.
	input  logic        snoop_we,
	input  logic [13:0] snoop_addr,
	// object-list read port for the sprite renderer: 2048 x 16-bit
	input  logic [10:0] obj_rd_addr,
	input  logic        obj_rd,
	output logic [15:0] obj_rd_data,
	output logic        obj_rd_valid,
	// Object-list snapshot. MAME's sort_and_render reads all 256 entries at
	// the MOGO edge; ours reads them across most of a frame while the game
	// may rewrite the list. A pulse on obj_snap_start copies the live mirror
	// into a second set of arrays in 1,024 cycles and the renderer reads only
	// the copy; obj_snap_busy holds the render until the copy is done.
	input  logic        obj_snap_start,
	output logic        obj_snap_busy,
	// 1 = the renderer reads the live mirror and no copy runs; 0 = the
	// snapshot (MAME's semantics). The OSD's `Obj List` switch, P1O[22].
	input  logic        obj_use_live,
	// Snoop-bus census of the object hflip bit, counted before the mirror:
	//   dbg_ob_hf_set -- writes to entry word 0 (snoop_addr[1:0] == 0), lane 3
	//                    enabled, with snoop_wd[31] (hflip) high
	//   dbg_ob_wr     -- writes to entry word 0 with lane 3 enabled
	output logic [15:0] dbg_ob_hf_set,
	output logic [15:0] dbg_ob_wr,
	// The same two events as 1-cycle pulses, so the top level can count them
	// per frame (the counters above saturate at FFFF).
	output logic        dbg_ob_w0_pulse,   // qualified write to entry word 0
	output logic        dbg_ob_hf_pulse,   // ...and it carried bit 31
	input  logic [3:0]  snoop_be,
	input  logic [31:0] snoop_wd
);

	// Playfield RAM (8192 x 16). Split by entry parity: one snoop beat carries
	// two tilemap entries (high half -> even, low half -> odd) and Cyclone V
	// BRAM has one write port. Split by byte lane: a partial write to a
	// 16-bit array defeats BRAM inference, so every write is full width for
	// its own 8-bit array. Each array is then 1W + 1R (simple dual port).
	// Pinned to M10K: under fitter pressure Quartus otherwise demotes arrays
	// to registers and the design no longer fits.
	(* ramstyle = "M10K" *) logic [7:0] pfmem_even_hi [0:4095];
	(* ramstyle = "M10K" *) logic [7:0] pfmem_even_lo [0:4095];
	(* ramstyle = "M10K" *) logic [7:0] pfmem_odd_hi  [0:4095];
	(* ramstyle = "M10K" *) logic [7:0] pfmem_odd_lo  [0:4095];
	// Zero at power-up; the game's snoop writes fill it.
	integer pfi;
	initial for (pfi = 0; pfi < 4096; pfi = pfi + 1) begin
		pfmem_even_hi[pfi] = 8'd0; pfmem_even_lo[pfi] = 8'd0;
		pfmem_odd_hi[pfi]  = 8'd0; pfmem_odd_lo[pfi]  = 8'd0;
	end

	// Simulation: the demo scene (tb_video_frame's reference) over the
	// zero-fill. $readmemh cannot split a 16-bit file across two 8-bit
	// arrays, so the scene is loaded via a scratch array and copied.
`ifdef AGT_SIM_DEMO_SCENE
	logic [15:0] pf_init_even [0:4095];
	logic [15:0] pf_init_odd  [0:4095];
	integer pfs;
	initial begin
		$readmemh("mem/demo_primrage_pfram_even.hex", pf_init_even);
		$readmemh("mem/demo_primrage_pfram_odd.hex",  pf_init_odd);
		for (pfs = 0; pfs < 4096; pfs = pfs + 1) begin
			pfmem_even_hi[pfs] = pf_init_even[pfs][15:8];
			pfmem_even_lo[pfs] = pf_init_even[pfs][7:0];
			pfmem_odd_hi[pfs]  = pf_init_odd[pfs][15:8];
			pfmem_odd_lo[pfs]  = pf_init_odd[pfs][7:0];
		end
	end
`endif
	// Snoop windows. One 32-bit beat is two 16-bit tilemap entries
	// (big-endian: high half first), written per byte lane.
	wire        sn_pf    = (snoop_addr >= 14'h0800) && (snoop_addr < 14'h1800);
	wire        sn_al    = (snoop_addr >= 14'h1800) && (snoop_addr < 14'h1C00);
	// Object list: 0xd78000-0xd78fff, MAME's share("rle"), the 256 sprite
	// slots the RLE renderer scans.
	//   0xd78000 -> (0xd78000-0xd70000)/4 = word 0x2000
	wire        sn_ob    = (snoop_addr >= 14'h2000) && (snoop_addr < 14'h2400);

	// entry N is the high half of word N>>1 when N is even, low half when odd
	//
	// Object-list snapshot. MAME's sort_and_render walks all 256 entries at
	// one instant (the MOGO edge); ours walks the list across a pass of up to
	// a frame, during which the game may clear and rebuild it. So the live
	// mirror (objm_*, M10K, written by the snoop) is copied into objs_* on
	// obj_snap_start (the DRAW MOGO) in 1,024 cycles, and the renderer reads
	// objs_*. obj_snap_busy holds the render start until the copy is done;
	// the erase sweep already holds it for up to 22 lines, so the copy
	// normally costs nothing.
	//
	// The mirror has one read port: its address register `om_ra` loads the
	// copy's address while a copy runs and the renderer's otherwise (a mux
	// after the register would break M10K inference). Each array is read in
	// exactly one statement, into `om_q`: Quartus builds a read port per
	// statement, so a second reader would duplicate the whole mirror. The
	// copy writes from om_q and obj_rd_data is a mux of it, valid in the
	// obj_rd_valid cycle (t+2).
	//
	// objs_* are MLAB: 4 x 1,024 x 8 = 32,768 bits, 0 M10K. MLAB content is
	// not initialised on Cyclone V, so a copy runs once after reset instead.
	wire        snap_sel = OBJ_SNAPSHOT && !obj_use_live;
	logic       cp_run, cp_boot;           // issuing mirror reads / first cycle after reset
	logic [9:0] cp_addr;                   // next mirror long to read
	logic       cp_v1, cp_v2;              // om_ra holds a copy address / om_q its data
	logic [9:0] cp_a1, cp_a2;
	logic [31:0] om_q;                         // mirror read register (the only reader)
	logic        sel_q;                        // odd/even half the renderer asked for
	logic [15:0] snap_q;                       // the snapshot word, registered with it
	logic [9:0] om_ra;                         // mirror read address register
	wire [15:0] snap_word_even, snap_word_odd; // objs_* at obaddr_p1 (MLAB, async read)

	logic       obrd_p1;
	logic [10:0] obaddr_p1;
	always_ff @(posedge clk_sys) begin
		obrd_p1   <= obj_rd;
		obaddr_p1 <= obj_rd_addr;
		obj_rd_valid <= obrd_p1;
		om_ra     <= cp_run ? cp_addr : obj_rd_addr[10:1];
		om_q      <= {objm_even_hi[om_ra], objm_even_lo[om_ra],
					  objm_odd_hi[om_ra],  objm_odd_lo[om_ra]};
		if (obrd_p1) begin
			sel_q  <= obaddr_p1[0];
			snap_q <= obaddr_p1[0] ? snap_word_odd : snap_word_even;
		end
	end
	assign obj_rd_data = snap_sel ? snap_q : (sel_q ? om_q[15:0] : om_q[31:16]);

	// the copy engine: read long k at om_ra, capture it in om_q one edge
	// later, write it into objs_* the edge after that
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			cp_run <= 1'b0; cp_boot <= 1'b1; cp_addr <= 10'd0;
			cp_v1 <= 1'b0; cp_v2 <= 1'b0; cp_a1 <= 10'd0; cp_a2 <= 10'd0;
		end else begin
			cp_boot <= 1'b0;
			if (!cp_run && !cp_v1 && !cp_v2 && snap_sel && (obj_snap_start || cp_boot)) begin
				cp_run <= 1'b1; cp_addr <= 10'd0;
			end else if (cp_run) begin
				cp_addr <= cp_addr + 10'd1;
				if (cp_addr == 10'd1023) cp_run <= 1'b0;
			end
			cp_v1 <= cp_run; cp_a1 <= cp_addr;   // om_ra <= cp_addr on this edge
			cp_v2 <= cp_v1;  cp_a2 <= cp_a1;     // om_q <= objm[cp_a1] on this edge
		end
	end
	// high until the edge that performs the last write (address 1023)
	assign obj_snap_busy = cp_run | cp_v1 | cp_v2;

	generate if (OBJ_SNAPSHOT) begin : g_snap
		(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] objs_even_hi [0:1023];
		(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] objs_even_lo [0:1023];
		(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] objs_odd_hi  [0:1023];
		(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] objs_odd_lo  [0:1023];
		always_ff @(posedge clk_sys) begin
			if (cp_v2) begin
				objs_even_hi[cp_a2] <= om_q[31:24];
				objs_even_lo[cp_a2] <= om_q[23:16];
				objs_odd_hi[cp_a2]  <= om_q[15:8];
				objs_odd_lo[cp_a2]  <= om_q[7:0];
			end
		end
		assign snap_word_even = {objs_even_hi[obaddr_p1[10:1]], objs_even_lo[obaddr_p1[10:1]]};
		assign snap_word_odd  = {objs_odd_hi[obaddr_p1[10:1]],  objs_odd_lo[obaddr_p1[10:1]]};
	end else begin : g_nosnap
		assign snap_word_even = 16'd0;
		assign snap_word_odd  = 16'd0;
	end endgenerate
	logic pfrd_p1;
	logic [12:0] pfaddr_p1;
	logic        pf_sel2;
	logic [15:0] pf_rd_even, pf_rd_odd;
	// read/select pipeline, 2-cycle latency: the output is a mux of the two
	// array-output registers
	always_ff @(posedge clk_sys) begin
		pfrd_p1 <= pfram_rd;
		pfaddr_p1 <= pfram_addr;
		pfram_data_valid <= pfrd_p1;
		pf_sel2 <= pfaddr_p1[0];
	end
	assign pfram_data = pf_sel2 ? pf_rd_odd : pf_rd_even;
	// one always_ff per array: 1 read + 1 write each = simple dual port
	always_ff @(posedge clk_sys) begin
		if (pfrd_p1) pf_rd_even <= {pfmem_even_hi[pfaddr_p1[12:1]],
									pfmem_even_lo[pfaddr_p1[12:1]]};
		// Byte-granular writes. `sn_pf` spans snoop_addr 0x800..0x17FF (4096
		// words, MAME's 0xd72000-0xd75fff) and the arrays are 4096 deep, but
		// the base 0x800 is not a multiple of the depth, so snoop_addr[11:0]
		// is not the relative index. `snoop_addr[11:0] ^ 12'h800` is
		// `snoop_addr - 12'h800` because 0x800 is half the 12-bit range;
		// without it the tilemap's two halves (the `(col & 0x40)` zone of
		// MAME's playfield_scan) swap. The alpha and object windows (bases
		// 0x1800, 0x2000) are multiples of their 1024-deep arrays and need no
		// correction.
		//
		// tb_video_frame loads pfmem_* by $readmemh and never exercises this
		// path; tb_pf_snoop.sv does.
		if (snoop_we && sn_pf) begin
			if (snoop_be[3]) pfmem_even_hi[snoop_addr[11:0] ^ 12'h800] <= snoop_wd[31:24];
			if (snoop_be[2]) pfmem_even_lo[snoop_addr[11:0] ^ 12'h800] <= snoop_wd[23:16];
		end
	end
	always_ff @(posedge clk_sys) begin
		if (pfrd_p1) pf_rd_odd <= {pfmem_odd_hi[pfaddr_p1[12:1]],
								   pfmem_odd_lo[pfaddr_p1[12:1]]};
		if (snoop_we && sn_pf) begin
			if (snoop_be[1]) pfmem_odd_hi[snoop_addr[11:0] ^ 12'h800] <= snoop_wd[15:8];
			if (snoop_be[0]) pfmem_odd_lo[snoop_addr[11:0] ^ 12'h800] <= snoop_wd[7:0];
		end
	end

	// object-list mirror and alpha RAM (2048 x 16, zero at power-up:
	// scroll/bank stay at reset defaults), split by entry parity and byte
	// lane like the playfield
	(* ramstyle = "M10K" *) logic [7:0] objm_even_hi [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] objm_even_lo [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] objm_odd_hi  [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] objm_odd_lo  [0:1023];
	integer obi;
	initial for (obi = 0; obi < 1024; obi = obi + 1) begin
		objm_even_hi[obi] = 8'd0; objm_even_lo[obi] = 8'd0;
		objm_odd_hi[obi]  = 8'd0; objm_odd_lo[obi]  = 8'd0;
	end

	(* ramstyle = "M10K" *) logic [7:0] alpham_even_hi [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] alpham_even_lo [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] alpham_odd_hi  [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] alpham_odd_lo  [0:1023];
	integer ai;
	initial begin
		for (ai = 0; ai < 1024; ai = ai + 1) begin
			alpham_even_hi[ai] = 8'd0; alpham_even_lo[ai] = 8'd0;
			alpham_odd_hi[ai]  = 8'd0; alpham_odd_lo[ai]  = 8'd0;
		end
	end

	// Simulation: load the alpha scene gen_video_golden.py renders, so the
	// pixel-exact comparison covers the alpha layer. Split by entry parity;
	// runs after the zero-fill above, so the scene overrides it.
`ifdef AGT_SIM_ALPHA_SCENE
	logic [15:0] al_init_even [0:1023];
	logic [15:0] al_init_odd  [0:1023];
	integer ali;
	initial begin
		$readmemh("mem/alpha_scene_even.hex", al_init_even);
		$readmemh("mem/alpha_scene_odd.hex",  al_init_odd);
		for (ali = 0; ali < 1024; ali = ali + 1) begin
			alpham_even_hi[ali] = al_init_even[ali][15:8];
			alpham_even_lo[ali] = al_init_even[ali][7:0];
			alpham_odd_hi[ali]  = al_init_odd[ali][15:8];
			alpham_odd_lo[ali]  = al_init_odd[ali][7:0];
		end
	end
`endif
	logic ard_p1;
	logic [10:0] aaddr_p1;
	logic        al_sel2;
	logic [15:0] al_rd_even, al_rd_odd;
	always_ff @(posedge clk_sys) begin
		ard_p1 <= alpharam_rd;
		aaddr_p1 <= alpharam_addr;
		alpharam_data_valid <= ard_p1;
		al_sel2 <= aaddr_p1[0];
	end
	assign alpharam_data = al_sel2 ? al_rd_odd : al_rd_even;
	// Census of the hflip bit arriving on the snoop bus, counted before any
	// mirror. Own always_ff so the counters have one driver and a reset.
	wire ob_w0_qual = snoop_we && sn_ob && snoop_be[3] &&
					  snoop_addr[1:0] == 2'b00;

	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			dbg_ob_hf_set <= 16'd0; dbg_ob_wr <= 16'd0;
		// Only entry word 0: snoop_addr is a 32-bit word index and an entry
		// is four words, so snoop_addr[1:0] == 0 is word 0, whose high
		// halfword (snoop_wd[31:16]) is the code/hflip word. Bit 31 of the
		// other words is not hflip.
		end else if (ob_w0_qual) begin
			if (dbg_ob_wr != 16'hFFFF) dbg_ob_wr <= dbg_ob_wr + 16'd1;
			if (snoop_wd[31] && dbg_ob_hf_set != 16'hFFFF)
				dbg_ob_hf_set <= dbg_ob_hf_set + 16'd1;
		end
	end

	// One qualifier for the counters above and the pulses, so they cannot
	// disagree.
	assign dbg_ob_w0_pulse = ob_w0_qual;
	assign dbg_ob_hf_pulse = ob_w0_qual && snoop_wd[31];

	always_ff @(posedge clk_sys) begin
		// object list: byte-granular, like the other two windows
		if (snoop_we && sn_ob) begin
			if (snoop_be[3]) objm_even_hi[snoop_addr[9:0]] <= snoop_wd[31:24];
			if (snoop_be[2]) objm_even_lo[snoop_addr[9:0]] <= snoop_wd[23:16];
			if (snoop_be[1]) objm_odd_hi[snoop_addr[9:0]]  <= snoop_wd[15:8];
			if (snoop_be[0]) objm_odd_lo[snoop_addr[9:0]]  <= snoop_wd[7:0];
		end
		if (ard_p1) al_rd_even <= {alpham_even_hi[aaddr_p1[10:1]],
								   alpham_even_lo[aaddr_p1[10:1]]};
		// Byte-granular: a 68020 MOVE.B yields a single byte enable (be8_of
		// in agt_main_memmap), and the game rewrites the tilemap every frame
		// (e.g. rolling credits), so no partial write may be dropped.
		if (snoop_we && sn_al) begin
			if (snoop_be[3]) alpham_even_hi[snoop_addr[9:0]] <= snoop_wd[31:24];
			if (snoop_be[2]) alpham_even_lo[snoop_addr[9:0]] <= snoop_wd[23:16];
		end
	end
	always_ff @(posedge clk_sys) begin
		if (ard_p1) al_rd_odd <= {alpham_odd_hi[aaddr_p1[10:1]],
								  alpham_odd_lo[aaddr_p1[10:1]]};
		if (snoop_we && sn_al) begin
			if (snoop_be[1]) alpham_odd_hi[snoop_addr[9:0]] <= snoop_wd[15:8];
			if (snoop_be[0]) alpham_odd_lo[snoop_addr[9:0]] <= snoop_wd[7:0];
		end
	end

	// Windowed tile ROM (24KB + 3-window remapper). TILEROM_USED is the real
	// content (3 x 8KB windows). The array is declared at 32768, the power of
	// two tile_bram_addr (15 bits) addresses, and the init file is padded to
	// match (avoids Critical Warning 127005).
	localparam int TILEROM_USED  = 24576;
	localparam int TILEROM_BYTES = 32768;
	// Simulation only: the demo tile artwork is tb_video_frame's tile source.
	// Hardware fetches tiles from SDRAM; without the define, reads return 0.
`ifdef AGT_SIM_DEMO_SCENE
	logic [7:0] tilemem [0:TILEROM_BYTES-1];
	initial $readmemh("mem/demo_primrage_tilerom.hex", tilemem);
`endif

	// remap the 3 windows into the array; out of window -> invalid
	logic [14:0] tile_bram_addr;
	logic        tile_addr_valid;
	always_comb begin
		tile_addr_valid = 1'b0;
		tile_bram_addr  = 15'd0;
		if (tile_rom_addr < 22'h002000) begin
			tile_bram_addr  = {2'b00, tile_rom_addr[12:0]};
			tile_addr_valid = 1'b1;
		end else if (tile_rom_addr >= 22'h100000 && tile_rom_addr < 22'h102000) begin
			tile_bram_addr  = {2'b01, tile_rom_addr[12:0]};
			tile_addr_valid = 1'b1;
		end else if (tile_rom_addr >= 22'h200000 && tile_rom_addr < 22'h202000) begin
			tile_bram_addr  = {2'b10, tile_rom_addr[12:0]};
			tile_addr_valid = 1'b1;
		end
	end

	logic trd_p1, trd3_p1, tvalid_p1;
	logic [14:0] taddr_p1;
	always_ff @(posedge clk_sys) begin
		trd_p1  <= tile_rom_rd | tile_rom_rd3;   // either kind of request
		trd3_p1 <= tile_rom_rd3;
		taddr_p1 <= tile_bram_addr;
		tvalid_p1 <= tile_addr_valid;
		tile_rom_data_valid <= trd_p1;
`ifdef AGT_SIM_DEMO_SCENE
		if (trd_p1) tile_rom_data <= tvalid_p1 ? tilemem[taddr_p1] : 8'd0;
`else
		tile_rom_data <= 8'd0;
`endif
		// Three-plane (burst3) response. taddr_p1's top two bits select the
		// plane bank (00=L, 01=M, 10=H); a burst3 request targets the L
		// address, so M and H are the same offset in banks 01 and 10. Word
		// order matches the SDRAM burst: {L, M, H}, each a 32-bit pair.
`ifdef AGT_SIM_DEMO_SCENE
		if (trd3_p1) tile_rom_data96 <= tvalid_p1
			? {tilemem[{2'b00, taddr_p1[12:2], 2'd0}], tilemem[{2'b00, taddr_p1[12:2], 2'd1}],
			   tilemem[{2'b00, taddr_p1[12:2], 2'd2}], tilemem[{2'b00, taddr_p1[12:2], 2'd3}],
			   tilemem[{2'b01, taddr_p1[12:2], 2'd0}], tilemem[{2'b01, taddr_p1[12:2], 2'd1}],
			   tilemem[{2'b01, taddr_p1[12:2], 2'd2}], tilemem[{2'b01, taddr_p1[12:2], 2'd3}],
			   tilemem[{2'b10, taddr_p1[12:2], 2'd0}], tilemem[{2'b10, taddr_p1[12:2], 2'd1}],
			   tilemem[{2'b10, taddr_p1[12:2], 2'd2}], tilemem[{2'b10, taddr_p1[12:2], 2'd3}]}
			: 96'd0;
`else
		tile_rom_data96 <= 96'd0;
`endif

		// The word pair containing the address, composed as the SDRAM tile
		// port does, so the decoder's WIDE_FETCH path behaves identically on
		// either memory. The pair base is the address with its low two bits
		// cleared.
`ifdef AGT_SIM_DEMO_SCENE
		if (trd_p1) tile_rom_data32 <= tvalid_p1
			? {tilemem[{taddr_p1[14:2], 2'd0}], tilemem[{taddr_p1[14:2], 2'd1}],
			   tilemem[{taddr_p1[14:2], 2'd2}], tilemem[{taddr_p1[14:2], 2'd3}]}
			: 32'd0;
`else
		tile_rom_data32 <= 32'd0;
`endif
	end

	// colorram + init replay FSM
	localparam int INIT_ENTRIES = 577;
	logic [9:0] init_idx;
	// Demo palette, replayed into colorram at boot (simulation only). The
	// game writes its own palette; this only tints the scene until it does.
`ifdef AGT_SIM_DEMO_SCENE
	localparam bit FIXTURE = 1'b1;
	logic [39:0] init_table [0:INIT_ENTRIES-1];
	initial $readmemh("mem/demo_primrage_colorram_init.hex", init_table);
	wire  [39:0] init_entry = init_table[init_idx];
`else
	localparam bit FIXTURE = 1'b0;
	wire  [39:0] init_entry = 40'd0;
`endif

	logic [18:0] wr_byte_addr;
	logic [15:0] wr_data;
	logic        wr_en;

	logic init_owns_port;
	wire        crw_en   = init_owns_port ? wr_en        : cpu_cr_wr;
	wire [18:0] crw_addr = init_owns_port ? wr_byte_addr : cpu_cr_addr;
	wire [15:0] crw_data = init_owns_port ? wr_data      : cpu_cr_data;

	agt_colorram u_colorram (
		.clk(clk_sys), .rst_n(rst_n),
		.wr_byte_addr(crw_addr), .wr_data(crw_data), .wr_en(crw_en),
		.cram_addr(cram_addr), .cram_data(cram_data), .color_latch(color_latch),
		.pen_addr_r(pen_addr_r), .pen_addr_g(pen_addr_g), .pen_addr_b(pen_addr_b),
		.pen_data_r(pen_data_r), .pen_data_g(pen_data_g), .pen_data_b(pen_data_b)
	);

	typedef enum logic [1:0] { INIT_RUN, INIT_GAP, INIT_DONE } init_state_t;
	init_state_t init_state;

	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			init_state <= INIT_RUN;
			init_idx <= 10'd0;
			wr_en <= 1'b0;
			video_rst_n <= 1'b0;
			init_owns_port <= 1'b1;
		end else begin
			wr_en <= 1'b0;
			unique case (init_state)
				INIT_RUN: begin
					if (!FIXTURE || !pal_fixture_en) begin
						// no replay: still release video and hand the port
						// over
						init_state <= INIT_DONE;
					end else begin
						wr_byte_addr <= init_entry[34:16];
						wr_data      <= init_entry[15:0];
						wr_en        <= 1'b1;
						init_state   <= INIT_GAP;
					end
				end
				INIT_GAP: begin
					if (init_idx == INIT_ENTRIES[9:0] - 1'b1) begin
						init_state <= INIT_DONE;
					end else begin
						init_idx   <= init_idx + 10'd1;
						init_state <= INIT_RUN;
					end
				end
				INIT_DONE: begin
					video_rst_n <= 1'b1;    // palette ready -- let video run
					init_owns_port <= 1'b0; // hand the write port to the CPU
				end
				default: init_state <= INIT_DONE;
			endcase
		end
	end

endmodule
