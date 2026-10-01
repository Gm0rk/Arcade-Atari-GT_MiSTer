// agt_playfield_line_render.sv -- render one scanline of playfield pixels
//
// Given the target scanline's scroll state (xscroll/yscroll/color_bank/
// tile_bank from agt_scanline_scroll, called once per scanline in order),
// renders the 336 visible pixels into an external line buffer, mixing in the
// alpha (an_line) and motion-object (mo_line) pixels via agt_colormix_primrage.
//
// 43 tile columns are fetched, not 42: unless xscroll is a multiple of 8 the
// window starts partway into a tile (xscroll & 7), so 336 pixels can span 43
// tiles. Pixel k of the 344-pixel run lands at column k - (xscroll & 7);
// pixels outside the window are dropped.
//
// The pipeline (tile fetch, decode, combine, colormix) is far slower than a
// pixel per pixel clock, so agt_video renders line N+1 into a back buffer
// while line N is displayed.

module agt_playfield_line_render #(
	parameter int VISIBLE_WIDTH = 336,
	// Tile rows fetched as one three-plane burst. A parameter so both benches
	// can run at the same setting.
	parameter bit USE_BURST3 = 1'b1) (
	// Debug: palette-chain witnesses, from agt_colormix_primrage
	output logic [15:0] dbg_mo_cram, dbg_mo_latch,
	output logic [23:0] dbg_mo_rgb,
	output logic [15:0] dbg_mo_hits,
	output logic [15:0] dbg_mo_nz,
	output logic [15:0] dbg_blk_cra, dbg_blk_cram, dbg_blk_latch,
	// Debug: probe pixel position (screen x, screen y) and its chain
	input  logic [8:0]  probe_x,
	input  logic [8:0]  probe_y,
	output logic [15:0] dbg_prb_mo, dbg_prb_pf, dbg_prb_cra, dbg_prb_cram,
	// Playfield census, free-running (agt_video takes the per-frame delta):
	//   dbg_pf_opaque -- playfield pixels with a non-zero pen (pf[5:0] != 0)
	//   dbg_pf_pixels -- playfield pixels the mixer looked at
	output logic [15:0] dbg_pf_opaque,
	output logic [15:0] dbg_pf_pixels,
	// Tilemap content:
	//   dbg_pf_ent  -- the last non-zero tilemap entry read
	//   dbg_pf_code -- the tile code that entry produced
	output logic [15:0] dbg_pf_ent,
	output logic [15:0] dbg_pf_code,
	// Range of opaque playfield pixel values, published at frame_tick.
	output logic [15:0] dbg_pf_min,
	output logic [15:0] dbg_pf_max,
	// {color_bank, tile_bank} in force when dbg_pf_code was formed
	output logic [15:0] dbg_pf_bank_at,
	// One complete tile fetch, all fields from the same transaction: the
	// tilemap entry, the banks in force, the code, the six ROM bytes and the
	// first two pens. Captured on the first tile of scanline probe_y, so it is
	// the same tile every frame.
	output logic [15:0] dbg_tx_ent, dbg_tx_code, dbg_tx_bank,
	// Which tilemap cell that fetch read, and the yscroll that chose its row.
	output logic [15:0] dbg_tx_addr,     // {3'b0, pfram_addr[12:0]} at the fetch
	output logic [15:0] dbg_tx_yscroll,  // {7'b0, yscroll[8:0]} at the fetch
	output logic [47:0] dbg_tx_bytes,
	output logic [11:0] dbg_tx_pix01,
	input  logic        frame_tick,     // vblank edge, from agt_video
	output logic [13:0] dbg_mo_cra,
	input  logic clk,
	input  logic rst_n,

	input  logic        start,          // 1-cycle pulse: render one scanline
	input  logic [8:0]  target_scanline,

	// Alpha pen buffer, written by agt_alpha_line_render. Screen-indexed,
	// because the alpha layer is screen-aligned while this renderer is scrolled.
	// That pass completes before lr_start, so the buffer is stable for the
	// whole pixel loop.
	input  logic        an_wr_valid,
	input  logic [8:0]  an_wr_col,
	input  logic [7:0]  an_wr_pen,
	input  logic [9:0]  xscroll,        // from agt_scanline_scroll, for this line
	input  logic [8:0]  yscroll,
	input  logic [4:0]  color_bank,
	input  logic [3:0]  tile_bank,
	input  logic [15:0] color_latch,    // passed through to colormix

	// playfield RAM port (shared with agt_playfield_addr)
	output logic [12:0] pfram_addr,
	output logic        pfram_rd,
	input  logic [15:0] pfram_data,
	input  logic        pfram_data_valid,

	// tile ROM port (shared with agt_tile_decode)
	output logic [21:0] tile_rom_addr,
	output logic        tile_rom_rd,
	output logic        tile_rom_rd3,      // three-plane burst request
	input  logic [7:0]  tile_rom_data,
	input  logic [31:0] tile_rom_data32,
	input  logic [95:0] tile_rom_data96,   // {L,M,H} from one access
	// Motion-object line for the line being rendered, written entry by entry.
	// mo_line_ready goes high once the whole line has been fetched; until then
	// the mixer gets 0, not a line that is half this scanline and half the last.
	input  logic         mo_line_ready,
	output logic [15:0]  dbg_mo_sel,   // debug: MO pixel fed to the mixer
	input  logic         mo_wr_valid,
	input  logic [8:0]   mo_wr_col,
	input  logic [15:0]  mo_wr_value,
	input  logic        tile_rom_data_valid,

	// colour RAM lookups (agt_colormix_primrage's cram/pen ports)
	output logic [13:0] cram_addr,
	input  logic [15:0] cram_data,
	output logic [14:0] pen_addr_r,
	output logic [14:0] pen_addr_g,
	output logic [14:0] pen_addr_b,
	input  logic [23:0] pen_data_r,
	input  logic [23:0] pen_data_g,
	input  logic [23:0] pen_data_b,

	// line buffer write port (external, one entry per visible column)
	output logic        line_wr_valid,
	output logic [8:0]  line_wr_col,      // 0 .. VISIBLE_WIDTH-1
	output logic [23:0] line_wr_rgb,

	output logic busy,
	output logic done
);

	localparam int N_TILES = 43;
	localparam int WIDE_WIDTH = N_TILES * 8;  // 344

	logic pfa_start, pfa_busy, pfa_done;
	logic [6:0] pfa_col;
	logic [5:0] pfa_row;
	logic [15:0] pfa_tile_code;
	logic        pfa_tile_valid;
	logic [47:0] td_row_bytes;
	logic [11:0] td_row_pix01;
	logic        tx_arm, tx_done;  // probe-tile capture, once per frame
	logic        td_pixels_valid;
	logic [15:0] pfa_ent_hold;     // the entry, held one cycle to pair with its code
	logic [2:0] pfa_tile_color;
	logic pfa_tile_hflip;

	agt_playfield_addr u_pfaddr (
		.clk(clk), .rst_n(rst_n),
		.start(pfa_start), .col(pfa_col), .row(pfa_row), .tile_bank(tile_bank),
		.pfram_addr(pfram_addr), .pfram_rd(pfram_rd),
		.pfram_data(pfram_data), .pfram_data_valid(pfram_data_valid),
		.tile_valid(pfa_tile_valid), .tile_code(pfa_tile_code), .tile_color(pfa_tile_color), .tile_hflip(pfa_tile_hflip),
		.busy(pfa_busy), .done(pfa_done)
	);

	logic td_start, td_busy, td_done;
	logic [2:0] td_row;
	logic [5:0] td_px0, td_px1, td_px2, td_px3, td_px4, td_px5, td_px6, td_px7;

	// WIDE_FETCH: 3 accesses per tile row instead of 6. At L=12 that is
	// 3x14 = 42 cycles against ~51 of per-tile work, so the lookahead hides it;
	// six byte reads (84 cycles) would not fit.
	agt_tile_decode #(.ROM_ADDR_WIDTH(22), .WIDE_FETCH(1'b1), .USE_BURST3(USE_BURST3)) u_tiledecode (
		.clk(clk), .rst_n(rst_n),
		.start(td_start), .code(pfa_tile_code), .row(td_row),
		.dbg_row_bytes(td_row_bytes), .dbg_row_pix01(td_row_pix01),
		.rom_addr(tile_rom_addr), .rom_rd(tile_rom_rd), .rom_rd3(tile_rom_rd3),
		.rom_data(tile_rom_data), .rom_data32(tile_rom_data32),
		.rom_data96(tile_rom_data96),
		.rom_data_valid(tile_rom_data_valid),
		.pixels_valid(td_pixels_valid), .pixel0(td_px0), .pixel1(td_px1), .pixel2(td_px2), .pixel3(td_px3),
		.pixel4(td_px4), .pixel5(td_px5), .pixel6(td_px6), .pixel7(td_px7),
		.busy(td_busy), .done(td_done)
	);

	// One-tile-row lookahead. Fetching a tile row is independent of combining
	// the previous tile's 8 pixels, so this sub-FSM fetches row N+1 while the
	// main FSM combines row N, hiding the memory latency instead of paying it.
	//
	// It alone drives pfa_start/td_start; the main FSM only raises line_active
	// and takes queue entries.
	typedef enum logic [1:0] { PF_IDLE, PF_ADDR, PF_DEC, PF_READY } pf_state_t;
	pf_state_t  pf_state;
	// Two-slot queue: a fetch gets two tiles of emission (~102 cycles) to
	// overlap, so a late fetch is absorbed instead of stalling the emitter. At
	// L=12 the fetch (3x14 = 42 cycles) keeps up with ~51 cycles of emission
	// per tile on average, so a deeper queue gains nothing.
	localparam int QDEPTH = 2;
	logic [5:0] q_pixels [0:QDEPTH-1][0:7];
	logic [2:0] q_color  [0:QDEPTH-1];
	logic [1:0] q_wr, q_rd;          // wraps at QDEPTH
	// Separate fill/take counters so each has one driver: the prefetcher owns
	// q_filled, the emitter owns q_taken.
	logic [5:0] q_filled, q_taken;
	wire  [5:0] q_count = q_filled - q_taken;
	logic       line_active;
	logic       line_active_d;
	// The tile colour travels with its pixels: agt_playfield_addr's tile_color
	// already belongs to the next prefetched row while this one is combined.
	logic [2:0] cur_color;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pf_state <= PF_IDLE;
			pfa_start <= 1'b0; td_start <= 1'b0;
			// Reset in the same always block as the capture below: two drivers pass
			// iverilog but Quartus rejects them (Error 10028). See
			// tools/check_multidriver.py.
			dbg_pf_ent <= 16'd0; dbg_pf_code <= 16'd0; pfa_ent_hold <= 16'd0;
			dbg_pf_bank_at <= 16'd0;
			tx_arm <= 1'b0; tx_done <= 1'b0;
			dbg_tx_ent <= 16'd0; dbg_tx_code <= 16'd0; dbg_tx_bank <= 16'd0;
			dbg_tx_bytes <= 48'd0; dbg_tx_pix01 <= 12'd0;
			dbg_tx_addr <= 16'd0; dbg_tx_yscroll <= 16'd0;
			q_wr <= 2'd0; q_filled <= 6'd0; fetch_idx <= 6'd0;
			line_active_d <= 1'b0;
		end else begin
			pfa_start <= 1'b0;
			td_start  <= 1'b0;
			// Last non-zero tilemap entry and the code agt_playfield_addr decodes from
			// it (including tile_bank). tile_code is registered from pfram_data on the
			// valid, so on that cycle it still holds the previous tile's code: hold the
			// entry one cycle and pair on tile_valid.
			if (pfram_data_valid) pfa_ent_hold <= pfram_data;
			if (pfa_tile_valid && pfa_ent_hold != 16'd0) begin
				dbg_pf_ent     <= pfa_ent_hold;
				dbg_pf_code    <= pfa_tile_code;
				dbg_pf_bank_at <= {7'd0, color_bank, tile_bank};
			end
			// Re-armed on any other scanline; captures the first tile of probe_y, then
			// holds until that scanline comes round again.
			if (target_scanline != probe_y) begin
				tx_arm <= 1'b1; tx_done <= 1'b0;
			end else if (tx_arm && !tx_done && pfa_tile_valid) begin
				tx_arm       <= 1'b0;
				dbg_tx_ent   <= pfa_ent_hold;
				dbg_tx_code  <= pfa_tile_code;
				dbg_tx_bank  <= {7'd0, color_bank, tile_bank};
				dbg_tx_addr  <= {3'd0, pfram_addr};
				dbg_tx_yscroll <= {7'd0, yscroll};
			end else if (!tx_arm && !tx_done && td_pixels_valid) begin
				tx_done      <= 1'b1;
				dbg_tx_bytes <= td_row_bytes;
				dbg_tx_pix01 <= td_row_pix01;
			end
			// fetch_idx counts tiles fetched for this line, so it restarts every line.
			line_active_d <= line_active;
			if (line_active && !line_active_d) fetch_idx <= 6'd0;
			unique case (pf_state)
				// Autonomous: refill whenever the queue has room and tiles remain. No
				// handshake with the emitter, so there is no take-and-go race.
				PF_IDLE: if (line_active && q_count < QDEPTH[5:0] &&
							 fetch_idx < N_TILES[5:0]) begin
							 pfa_col   <= (base_tile_col + {1'b0, fetch_idx}) & 7'h7f;
							 fetch_idx <= fetch_idx + 6'd1;
							 pfa_start <= 1'b1;
							 pf_state  <= PF_ADDR;
						 end
				PF_ADDR: if (pfa_done) begin
							 td_start <= 1'b1;
							 pf_state <= PF_DEC;
						 end
				PF_DEC:  if (td_done) begin
							 q_pixels[q_wr][0] <= td_px0; q_pixels[q_wr][1] <= td_px1;
							 q_pixels[q_wr][2] <= td_px2; q_pixels[q_wr][3] <= td_px3;
							 q_pixels[q_wr][4] <= td_px4; q_pixels[q_wr][5] <= td_px5;
							 q_pixels[q_wr][6] <= td_px6; q_pixels[q_wr][7] <= td_px7;
							 q_color[q_wr] <= pfa_tile_color;
							 q_wr     <= (q_wr == QDEPTH-1) ? 2'd0 : q_wr + 2'd1;
							 q_filled <= q_filled + 6'd1;
							 pf_state <= PF_IDLE;      // free to fetch again
						 end
				PF_READY: pf_state <= PF_IDLE;   // unused
			endcase
		end
	end

	logic [5:0] fetch_idx;

	// 336 x 8 alpha pens and 336 x 16 MO pixels. Quartus does not infer RAM
	// from an asynchronous read, so the read is two-stage (registered address,
	// registered data). The main FSM registers the address for pixel N one
	// cycle early, wherever it decides pixel N comes next (out of ST_PRIME, out
	// of ST_NEXT, or looping in ST_PIX_COLORMIX_WAIT): no added cycles.
	// MLAB with no_rw_check makes a same-cycle same-address read undefined.
	// Safe because none reaches the mixer: an_line is filled before lr_start,
	// and mo_line reads 0 until mo_line_ready (tb/rdw_probe_linerender.sv
	// checks this).
	(* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  an_line [0:VISIBLE_WIDTH-1];
	(* ramstyle = "MLAB, no_rw_check" *) logic [15:0] mo_line [0:VISIBLE_WIDTH-1];
	// Stage 1, set by the main FSM one cycle before the pixel reaches
	// ST_PIX_COMBINE: the address and its two bounds flags.
	logic [8:0] an_addr_r;
	logic       an_oob_r;   // negative or >= VISIBLE_WIDTH: use 0
	logic       an_neg_r;   // negative only (for the probe)
	// Stage 2: the registered read, in the same always_ff as the write (the
	// simple dual-port shape). mo_line_ready is sampled live here; it is a
	// per-line level. an_addr_r/an_oob_r/an_neg_r are reset in the main FSM's
	// block, where they are driven; resetting them here too would add a second
	// driver.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			an_sel_r <= 8'd0; mo_sel_r <= 16'd0;
		end else begin
			if (an_wr_valid) an_line[an_wr_col] <= an_wr_pen;
			if (mo_wr_valid) mo_line[mo_wr_col] <= mo_wr_value;
			an_sel_r <= an_oob_r ? 8'd0 : an_line[an_addr_r];
			mo_sel_r <= (an_oob_r || !mo_line_ready) ? 16'd0 : mo_line[an_addr_r];
		end
	end

	logic [5:0] combine_raw_pixel;
	logic [15:0] combine_pf_pixel;
	agt_playfield_pixel_combine u_combine (
		.raw_pixel(combine_raw_pixel), .tile_color(cur_color),
		.color_bank(color_bank), .pf_pixel(combine_pf_pixel)
	);

	// Registered read data of an_line/mo_line (driven in the block above), with
	// a reset so no X reaches the mixer, which reads them combinationally.
	logic [15:0] mo_sel_r;
	assign dbg_mo_sel = mo_sel_r;
	logic [7:0] an_sel_r;
	logic cm_start, cm_busy, cm_done;
	logic cm_probe;      // this pixel is the probe pixel
	// Census counters: free-running 16-bit; agt_video takes a wrap-safe
	// per-frame delta.
	logic [15:0] pfc_op, pfc_pix;
	assign dbg_pf_opaque = pfc_op;
	assign dbg_pf_pixels = pfc_pix;
	logic [15:0] pf_min_acc, pf_max_acc;
	logic [23:0] cm_rgb;
	agt_colormix_primrage u_colormix (
		.dbg_mo_cram(dbg_mo_cram), .dbg_mo_latch(dbg_mo_latch),
		.dbg_mo_rgb(dbg_mo_rgb), .dbg_mo_hits(dbg_mo_hits), .dbg_mo_cra(dbg_mo_cra),
		.dbg_mo_nz(dbg_mo_nz),
		.dbg_blk_cra(dbg_blk_cra), .dbg_blk_cram(dbg_blk_cram), .dbg_blk_latch(dbg_blk_latch),
		.probe(cm_probe), .dbg_prb_mo(dbg_prb_mo), .dbg_prb_pf(dbg_prb_pf),
		.dbg_prb_cra(dbg_prb_cra), .dbg_prb_cram(dbg_prb_cram),
		.clk(clk), .rst_n(rst_n),
		.start(cm_start), .an_pixel({8'd0, an_sel_r}), .pf_pixel(combine_pf_pixel), .mo_pixel(mo_sel_r),
		.color_latch(color_latch),
		.cram_addr(cram_addr), .cram_data(cram_data),
		.pen_addr_r(pen_addr_r), .pen_addr_g(pen_addr_g), .pen_addr_b(pen_addr_b),
		.pen_data_r(pen_data_r), .pen_data_g(pen_data_g), .pen_data_b(pen_data_b),
		.rgb_valid(), .rgb(cm_rgb),
		.busy(cm_busy), .done(cm_done)
	);

	// No pre-window buffer: the scroll shift is applied as each pixel is stored,
	// which avoids a 336-cycle copy pass per line.

	// Main FSM: takes tile rows from the queue, mixes each pixel, writes the line
	logic [5:0] tile_idx;          // 0..42
	logic [2:0] pix_in_tile;       // 0..7, which of the current tile's 8 pixels
	logic [5:0] cur_raw_pixels [0:7];
	logic [6:0] base_tile_col;     // xscroll >> 3, 7 bits (0-127 wrap)
	logic [2:0] xscroll_frac;      // xscroll & 7
	logic [8:0] effective_y;       // target_scanline + yscroll, mod 512 (64 tile rows x 8)

	typedef enum logic [3:0] {
		ST_IDLE,
		ST_PRIME,          // wait for the first tile row to arrive
		ST_NEXT,           // take the next prefetched row
		ST_PIX_COMBINE, ST_PIX_COLORMIX_WAIT
	} state_t;
	state_t state;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0;
			cm_start <= 1'b0; cm_probe <= 1'b0;
			pfc_op <= 16'd0; pfc_pix <= 16'd0;
			pf_min_acc <= 16'hFFFF; pf_max_acc <= 16'd0;
			dbg_pf_min <= 16'hFFFF; dbg_pf_max <= 16'd0;
			q_rd <= 2'd0; q_taken <= 6'd0; line_active <= 1'b0;
			line_wr_valid <= 1'b0;
			an_addr_r <= 9'd0; an_oob_r <= 1'b1; an_neg_r <= 1'b0;
		end else begin
			done <= 1'b0;
			cm_start <= 1'b0; cm_probe <= 1'b0;
			line_wr_valid <= 1'b0;
			// Publish last frame's range and restart; same always block as the
			// accumulators, so each has one driver.
			if (frame_tick) begin
				dbg_pf_min <= pf_min_acc; dbg_pf_max <= pf_max_acc;
				pf_min_acc <= 16'hFFFF;   pf_max_acc <= 16'd0;
			end

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy <= 1'b1;
						tile_idx <= 6'd0;
						base_tile_col <= xscroll[9:3];
						xscroll_frac <= xscroll[2:0];
						effective_y <= (target_scanline + yscroll) & 9'h1ff;
						// row selects are constant across the line, so they
						// are set once here instead of per tile
						pfa_row <= ((target_scanline + yscroll) & 9'h1ff) >> 3;
						td_row  <= (target_scanline + yscroll) & 9'h007;
						line_active <= 1'b1;   // prefetcher starts filling
						state     <= ST_PRIME;
					end
				end

				// The first tile of the line has nothing to overlap with: one unavoidable
				// stall per line.
				ST_PRIME: if (q_count != 6'd0) begin
					logic [9:0] next_shift;
					cur_raw_pixels[0] <= q_pixels[q_rd][0]; cur_raw_pixels[1] <= q_pixels[q_rd][1];
					cur_raw_pixels[2] <= q_pixels[q_rd][2]; cur_raw_pixels[3] <= q_pixels[q_rd][3];
					cur_raw_pixels[4] <= q_pixels[q_rd][4]; cur_raw_pixels[5] <= q_pixels[q_rd][5];
					cur_raw_pixels[6] <= q_pixels[q_rd][6]; cur_raw_pixels[7] <= q_pixels[q_rd][7];
					cur_color   <= q_color[q_rd];
					q_rd        <= (q_rd == QDEPTH-1) ? 2'd0 : q_rd + 2'd1;
					q_taken     <= q_taken + 6'd1;
					tile_idx    <= 6'd0;
					pix_in_tile <= 3'd0;
					// Pixel 0's an_line/mo_line address, one cycle early, so the
					// registered read is ready in ST_PIX_COMBINE
					next_shift  = {1'b0, 9'd0} - {7'd0, xscroll_frac};
					an_addr_r   <= next_shift[8:0];
					an_neg_r    <= next_shift[9];
					an_oob_r    <= next_shift[9] || (next_shift[8:0] >= VISIBLE_WIDTH[8:0]);
					state <= ST_PIX_COMBINE;
				end

				// The next row was prefetched during the last pixel loop, so the queue is
				// normally non-empty and this costs a single cycle.
				ST_NEXT: if (q_count != 6'd0) begin
					logic [8:0] next_raw_k;
					logic [9:0] next_shift;
					cur_raw_pixels[0] <= q_pixels[q_rd][0]; cur_raw_pixels[1] <= q_pixels[q_rd][1];
					cur_raw_pixels[2] <= q_pixels[q_rd][2]; cur_raw_pixels[3] <= q_pixels[q_rd][3];
					cur_raw_pixels[4] <= q_pixels[q_rd][4]; cur_raw_pixels[5] <= q_pixels[q_rd][5];
					cur_raw_pixels[6] <= q_pixels[q_rd][6]; cur_raw_pixels[7] <= q_pixels[q_rd][7];
					cur_color   <= q_color[q_rd];
					q_rd        <= (q_rd == QDEPTH-1) ? 2'd0 : q_rd + 2'd1;
					q_taken     <= q_taken + 6'd1;
					tile_idx    <= tile_idx + 6'd1;
					pix_in_tile <= 3'd0;
					// this tile's first pixel, address one cycle early
					next_raw_k  = {tile_idx + 6'd1, 3'd0};
					next_shift  = {1'b0, next_raw_k} - {7'd0, xscroll_frac};
					an_addr_r   <= next_shift[8:0];
					an_neg_r    <= next_shift[9];
					an_oob_r    <= next_shift[9] || (next_shift[8:0] >= VISIBLE_WIDTH[8:0]);
					state <= ST_PIX_COMBINE;
				end

				ST_PIX_COMBINE: begin
					// an_sel_r/mo_sel_r are already valid here: their address was
					// registered one cycle ago by the state that sent this pixel here.
					combine_raw_pixel <= cur_raw_pixels[pix_in_tile];
					cm_start <= 1'b1;
					// census of what the playfield offers
					pfc_pix <= pfc_pix + 16'd1;                 // wraps; delta taken at the top
					if (combine_pf_pixel[5:0] != 6'd0) begin
						pfc_op <= pfc_op + 16'd1;
						// running min/max of opaque values
						if (combine_pf_pixel < pf_min_acc) pf_min_acc <= combine_pf_pixel;
						if (combine_pf_pixel > pf_max_acc) pf_max_acc <= combine_pf_pixel;
					end
					// Probe: an_addr_r/an_neg_r are this pixel's column and sign
					// (registered last cycle).
					cm_probe <= !an_neg_r && (an_addr_r == probe_x) &&
								(target_scanline == probe_y);
					// cm_done is a 1-cycle pulse last asserted >= 3 cycles ago,
					// so no gap state is needed before waiting on it
					state <= ST_PIX_COLORMIX_WAIT;
				end
				ST_PIX_COLORMIX_WAIT: begin
					if (cm_done) begin
						// scroll shift applied at store time: raw pixel k
						// (of the 344-wide pre-window) lands at line column
						// k - xscroll_frac; the first xscroll_frac pixels
						// and any column >= VISIBLE_WIDTH fall outside the
						// window and are dropped
						logic [8:0] raw_k;
						logic [9:0] shifted;
						raw_k   = {tile_idx, pix_in_tile};
						shifted = {1'b0, raw_k} - {7'd0, xscroll_frac};
						if (!shifted[9] && shifted[8:0] < VISIBLE_WIDTH[8:0]) begin
							line_wr_valid <= 1'b1;
							line_wr_col   <= shifted[8:0];
							line_wr_rgb   <= cm_rgb;
						end
						// advance the loop in the same cycle the pixel is emitted
						if (pix_in_tile == 3'd7) begin
							if (tile_idx == N_TILES[5:0] - 1'b1) begin
								busy        <= 1'b0;
								done        <= 1'b1;
								line_active <= 1'b0;
								state       <= ST_IDLE;
							end else begin
								state <= ST_NEXT;
							end
						end else begin
							logic [8:0] next_raw_k;
							logic [9:0] next_shift;
							// next pixel of the same tile, address one cycle early
							next_raw_k  = {tile_idx, pix_in_tile + 3'd1};
							next_shift  = {1'b0, next_raw_k} - {7'd0, xscroll_frac};
							an_addr_r   <= next_shift[8:0];
							an_neg_r    <= next_shift[9];
							an_oob_r    <= next_shift[9] || (next_shift[8:0] >= VISIBLE_WIDTH[8:0]);
							pix_in_tile <= pix_in_tile + 3'd1;
							state <= ST_PIX_COMBINE;
						end
					end
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
