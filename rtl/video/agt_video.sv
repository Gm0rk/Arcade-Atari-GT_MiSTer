// agt_video.sv -- Atari GT / Primal Rage video: timing, line render pipeline,
// MO bitmap and debug overlays.
//
// Two clock domains:
//   pixel_clk (~7.16 MHz) runs only agt_video_timing, the front-buffer
//     display read, the buffer swap and the overlays.
//   clk_sys runs the whole render pipeline: agt_scanline_scroll, the alpha
//     pass, agt_playfield_line_render, agt_mo_vram and agt_rle_renderer.
// A line takes 2000+ cycles to render (43 tile fetches and 344 per-pixel
// colormix passes, each a multi-cycle FSM) against 456 pixel_clk cycles per
// scanline, so clk_sys must run at roughly 5x pixel_clk or more.
//
// The pipeline renders one line ahead into a double-buffered line memory.
// Which buffer is in front crosses domains by a 4-phase handshake, so no pulse
// has to survive the crossing and any clock ratio works:
//   - clk_sys raises cdc_req (held) when a line's render completes.
//   - pixel_clk sees it through a 2-flop chain and, as the next visible line
//     starts, swaps the buffers and raises cdc_ack (held).
//   - clk_sys sees cdc_ack through its own 2-flop chain, drops cdc_req and
//     starts the next line.
//   - pixel_clk sees cdc_req fall and drops cdc_ack.

module agt_video #(
	// Set from the top, so a stale copy of this file is a Quartus error rather
	// than a truncated dbg_words connection.
	parameter int DBG_NSLOTS = 12,
	// Build stamp width in BCD digits. Set from the top for the same reason as
	// DBG_NSLOTS: a stale copy with a 32-bit dbg_build_id fails to elaborate
	// rather than keeping the stamp's low 32 bits.
	parameter int BUILD_DIGITS = 9,
	// 1 drops the second MO target (TMO), which Primal Rage does not use, so
	// the design fits in M10K; mo_alt_wr_count counts writes aimed at it.
	parameter int MO_VRAM_TARGETS = 2,
	parameter bit MO_VRAM_DOUBLE  = 1,   // two MO buffers (0 = single)
	// A parameter here, not a constant in agt_playfield_line_render, so both
	// benches can be run at the same setting from the command line.
	parameter bit USE_BURST3 = 1'b1,
	// Scanline at which the pipeline re-renders line 0 for the coming frame
	// (see the resync below). 3 lines of lead: a line takes at most 2,573 of
	// the 3,648 cycles per scanline (tb_video_frame).
	parameter int LINE0_FETCH_LINE = 259,
	// 0 builds no overlay (the release build, Arcade-Atari-GT.qpf): `rgb` is
	// the picture, and dbg_bits, dbg_words and dbg_build_id drive nothing, so
	// synthesis removes the counters behind them. 1 (the debug build) draws
	// the status blocks and the hex text as the enables say.
	parameter bit DBG_OVERLAY = 1'b1,
	// 0 builds no SignalTap taps (`tap_*`, below): they are `noprune`, so
	// synthesis keeps them unless they are not generated. The top level sets
	// it with DBG_OVERLAY: both 1 in the debug build, both 0 in the release.
	parameter bit DBG_TAPS = 1'b1,
	// `line_late` rises when the line in progress has taken this many clk_sys
	// cycles (of the ~3,550 a line may take before the swap repeats one). An
	// uncontended line takes at most 2,573 (tb_video_frame), so a line past
	// this is being held up by the memory.
	parameter int LATE_AT = 2400
) (
	// 1 = re-render line 0 at vblank start; 0 = at LINE0_FETCH_LINE, late in
	// vblank (the default; OSD `Line 0 Fetch`). Static, synchronised below.
	input  logic line0_fetch_vblank,
	input  logic pixel_clk,   // ~7.16 MHz: timing and display read only
	input  logic clk_sys,     // render pipeline; ~5x pixel_clk or more
	input  logic rst_n,       // already synchronised to both domains

	// playfield RAM port (clk_sys domain)
	output logic [15:0] mo_alt_wr_count,   // writes aimed at the absent MO buffer
	output logic [12:0] pfram_addr,
	output logic        pfram_rd,
	input  logic [15:0] pfram_data,
	input  logic        pfram_data_valid,

	// tile ROM port (clk_sys domain)
	output logic [21:0] tile_rom_addr,
	output logic        tile_rom_rd,
	output logic        tile_rom_rd3,
	input  logic [7:0]  tile_rom_data,
	input  logic [31:0] tile_rom_data32,
	input  logic [95:0] tile_rom_data96,
	input  logic        tile_rom_data_valid,

	// chars ROM port for the alpha layer
	// Separate from tile_rom: chars are 4bpp packed MSB in a flat 128KB
	// region, tiles are three planes 1MB apart, so sharing a port would need a
	// mux and gain nothing.
	output logic [16:0] charrom_addr,
	output logic        charrom_rd,
	input  logic [7:0]  charrom_data,
	input  logic [31:0] charrom_data32,
	input  logic        charrom_data_valid,

	// alpha RAM port (clk_sys domain)
	output logic [10:0] alpharam_addr,
	output logic        alpharam_rd,
	input  logic [15:0] alpharam_data,
	input  logic        alpharam_data_valid,

	// colorram lookups (clk_sys domain)
	output logic [13:0] cram_addr,
	input  logic [15:0] cram_data,
	input  logic [15:0] color_latch,
	output logic [14:0] pen_addr_r,
	output logic [14:0] pen_addr_g,
	output logic [14:0] pen_addr_b,
	input  logic [23:0] pen_data_r,
	input  logic [23:0] pen_data_g,
	input  logic [23:0] pen_data_b,

	// pixel output (pixel_clk domain)
	output logic [23:0] rgb,

	// Status block overlay (drawn below), 8 blocks per row:
	//   row 0 = [7:0], row 1 = [15:8]: status bits, bit 0 leftmost
	//   rows 2-3 = [31:16], rows 4-5 = [47:32]: 16-bit values, MSB leftmost
	// Bit meanings are listed where the top level builds dbg_bits. Bit 7 is a
	// heartbeat toggling every 20,000,000 retired instructions (~8.7 s at
	// ~2.29M/s): a STAT word that never shows it low is the bad sign.
	input  logic [47:0] dbg_bits,
	// DBG_NSLOTS 32-bit values, drawn as hex by agt_dbg_text below the blocks.
	input  logic [DBG_NSLOTS*32-1:0] dbg_words,
	// Build stamp for the overlay's top row, constant, BUILD_DIGITS BCD
	// digits.
	input  logic [BUILD_DIGITS*4-1:0] dbg_build_id,
	// Overlay enables (OSD Debug page), clk_sys domain; synchronised below.
	// Unused when DBG_OVERLAY is 0.
	input  logic         dbg_blocks_en,
	input  logic         dbg_text_en,
	// Low until the game writes its own colorram. While low the game picture
	// is blanked but the debug overlays are not: they are all there is to look
	// at during a boot that takes seconds.
	input  logic         game_owns_screen,
	// sprite (MO) path
	input  logic         mogo_pulse,
	output logic [15:0]  dbg_mogo_deferred,   // MOGO requests seen
	output logic [15:0]  dbg_erase_sweeps,
	// Where sprite pixels land relative to the beam, per frame, latched at
	// vblank, >>2 (up to 80,640 writes a frame overflow 16 bits).
	//   behind = written to a row the raster has already scanned this frame.
	//            Single-buffered, that pixel is never displayed: the next
	//            vblank's sweep clears it before the beam comes back.
	//   ahead  = written in vblank, or to a row the beam has not reached.
	output logic [15:0]  dbg_wr_behind,
	output logic [15:0]  dbg_wr_ahead,
	input  logic         mo_erase_full_frame,
	input  logic         mo_frame_select, // CONTROL_FRAME
	// Buffer a sweep clears: the old FRAME bit for a control-write erase, the
	// current one for the vblank erase; valid with mo_erase_pulse.
	input  logic         mo_erase_frame,
	// Object-list snapshot copy in progress (agt_demo_memories). The render
	// waits for it as it does for erase_busy.
	input  logic         mo_list_snap_busy,
	input  logic         mo_erase_pulse,     // CONTROL_ERASE
	input  logic         mo_ctrl_wr,         // any control write
	input  logic         mo_erase_to_bottom, // 1: vblank-site erase, 0: control-write
	output logic         rnd_wr_valid_dbg,   // sprite pixel write (a port, so SignalTap sees it)
	output logic [10:0]  obj_rd_addr,
	output logic         obj_rd,
	input  logic [15:0]  obj_rd_data,
	input  logic         obj_rd_valid,
	// object-list debug taps for the overlay
	output logic [15:0]  dbg_obj_w0,
	output logic [15:0]  dbg_obj_w4,
	output logic         dbg_obj_valid,
	output logic [7:0]   dbg_obj_rejects,
	output logic [15:0]  dbg_obj_starts,
	output logic [15:0]  dbg_obj_full,
	output logic [8:0]   dbg_obj_lastidx,
	output logic [15:0]  dbg_obj_examined_pf,
	output logic [15:0]  dbg_obj_emitted_pf,
	output logic [15:0]  dbg_obj_examined,
	output logic [15:0]  dbg_obj_emitted,
	output logic [15:0]  dbg_stage_ot,
	output logic [15:0]  dbg_stage_bl,
	output logic         dbg_blit_meas,
	output logic         dbg_blit_wait,
	output logic         dbg_blit_emit,
	output logic [23:0]  dbg_st_obj, dbg_st_tbl, dbg_st_blt,
	// per-object attribute witnesses from agt_rle_renderer
	output logic [15:0]  dbg_obj_zscale, dbg_obj_offscr,
	output logic [15:0]  dbg_mo_cram, dbg_mo_latch,
	output logic [23:0]  dbg_mo_rgb,
	output logic [15:0]  dbg_mo_hits,
	output logic [15:0]  dbg_mo_nz,
	output logic [15:0]  dbg_blk_cra, dbg_blk_cram, dbg_blk_latch,
	input  logic [8:0]   probe_x, probe_y,                            // pixel sampled by dbg_prb_*
	output logic [15:0]  dbg_prb_mo, dbg_prb_pf, dbg_prb_cra, dbg_prb_cram,
	// render pass and object statistics from agt_rle_renderer
	output logic [15:0]  dbg_start_lost, dbg_pass_done,
	output logic [15:0]  dbg_pc_hits, dbg_pc_misses,
	output logic [15:0]  dbg_big_x, dbg_big_y, dbg_sml_x, dbg_sml_y,
	output logic [15:0]  dbg_big_code, dbg_big_width,
	output logic [15:0]  dbg_big_w0, dbg_big_w1,
	output logic [15:0] dbg_hf_w0,
	output logic [15:0] dbg_hf_live,
	output logic [15:0] dbg_hf_first,
	output logic [15:0]  dbg_hflip_cnt, dbg_obj_cnt,
	// Per-frame write losses, latched at the vblank edge, >>2 like MOWB, so
	// the numbers are directly comparable.
	output logic [15:0]  dbg_drop_alt,
	output logic [15:0]  dbg_drop_erase,
	// out-of-range writes, which the loss counters above do not see
	output logic [15:0]  dbg_drop_oobx,      // row ok, column out of range, >>2
	output logic [15:0]  dbg_drop_ooby,      // row out of range, >>2
	output logic signed [15:0] dbg_wr_xmin,  // raw signed wr_x range, last frame
	output logic signed [15:0] dbg_wr_xmax,
	// the first object of each pass whose horizontal span runs away
	output logic [15:0]  dbg_wild_code,
	output logic [15:0]  dbg_wild_scale,
	output logic signed [15:0] dbg_wild_draw_x,
	output logic [15:0]  dbg_wild_width,
	output logic [15:0]  dbg_wild_hdr,
	output logic [15:0]  dbg_wild_mwidth,
	output logic [13:0]  dbg_mo_cra,
	output logic [15:0]  dbg_scale_or,   dbg_scale_and,
	output logic [15:0]  dbg_mo_sel,   // MO pixel at the mixer
	// does a render pass finish before the raster leaves vblank?
	output logic         dbg_render_busy,
	output logic         dbg_in_vblank,
	// Per-line pacing on hardware (tb_video_frame prints the same as PACING):
	// worst case in clk_sys cycles since reset for the whole line (ss_start to
	// lr_done) and for each pass. A scanline is 3,648 cycles and the swap is
	// quantised to scanlines, so a worst line over ~3,550 (budget minus the
	// handshake) repeats a line, and a run of them is a 2x stretch.
	output logic [15:0]  dbg_line_worst,
	output logic [15:0]  dbg_scroll_worst,
	output logic [15:0]  dbg_alpha_worst,
	output logic [15:0]  dbg_render_worst,
	// Where the overruns happen (D-648), clk_sys, since reset. At each
	// render_overrun the line the renderer is late with (next_render_line):
	//   dbg_ovr_where = {lowest such line, highest, the last one,
	//                    overruns while the sprite renderer was drawing (sat)}
	//   dbg_ovr_pass  = {that last late line's alpha pass, its render pass},
	//                   each in 16-cycle steps (FF: 4,080 or more)
	// Before the first overrun: where = FF00 0000 (lowest > highest), pass 0.
	output logic [31:0]  dbg_ovr_where,
	output logic [15:0]  dbg_ovr_pass,
	output logic [23:0]  rle_rom_waddr,
	output logic         rle_rom_rd,
	input  logic [15:0]  rle_rom_data,
	input  logic         rle_rom_data_valid,

	// Scroll telemetry: the live scroll-pass values, to check the alpharam mux
	// on hardware (it can only misbehave if al_busy rises during a scroll
	// read, which simulation's fixed pass order never produces).
	output logic [9:0]   dbg_xscroll,
	output logic [8:0]   dbg_yscroll,
	output logic         dbg_ss_done,
	output logic [8:0]   dbg_ss_line,
	output logic [8:0]   dbg_vcount,        // raster line, for the control-write trace
	output logic        hsync,
	output logic        vsync,
	output logic        hblank,
	output logic        vblank,   // also exported as dbg_in_vblank
	output logic        pixel_active,

	output logic         render_overrun,  // pixel_clk domain
	// clk_sys: the line being rendered has run LATE_AT cycles and is not done.
	// The top level uses it to put the line renderer's fetches first at the
	// SDRAM (agt_sdram `tile_first`).
	output logic         line_late
);

	localparam int VISIBLE_WIDTH  = 336;
	localparam int VISIBLE_HEIGHT = 240;

	// pixel_clk domain: timing, display read, buffer swap
	logic [8:0] hcount, vcount;
	logic frame_start, line_start;

	agt_video_timing #(
		.H_TOTAL(456), .H_VISIBLE(VISIBLE_WIDTH),
		.V_TOTAL(262), .V_VISIBLE(VISIBLE_HEIGHT)
	) u_timing (
		.clk(pixel_clk), .rst_n(rst_n),
		.hcount(hcount), .vcount(vcount),
		.hblank(hblank), .vblank(vblank), .hsync(hsync), .vsync(vsync),
		.pixel_active(pixel_active), .frame_start(frame_start), .line_start(line_start)
	);

	// CDC: cdc_req, clk_sys -> pixel_clk
	logic cdc_req;                       // driven in clk_sys
	logic [1:0] cdc_req_sync_chain;
	wire  cdc_req_sync = cdc_req_sync_chain[1];
	always_ff @(posedge pixel_clk or negedge rst_n) begin
		if (!rst_n) cdc_req_sync_chain <= 2'b00;
		else cdc_req_sync_chain <= {cdc_req_sync_chain[0], cdc_req};
	end

	// swap and ack
	// Swaps happen only on entry to visible lines. The line-0 resync renders
	// line 0 during vblank; this gate holds it through vblank so it swaps in
	// exactly at vcount = 0. Allowing vblank swaps would render and swap away
	// the following lines too, before display resumes.
	logic cdc_ack;
	logic front_is_a;

	// the line that begins on the next cycle (see the swap-timing note below)
	localparam logic [8:0] H_LAST = 9'd455;    // H_TOTAL-1
	wire [8:0] swap_line = (vcount == 9'd261) ? 9'd0 : vcount + 9'd1;

	always_ff @(posedge pixel_clk or negedge rst_n) begin
		if (!rst_n) begin
			front_is_a <= 1'b1;
			cdc_ack <= 1'b0;
			render_overrun <= 1'b0;
		end else begin
			render_overrun <= 1'b0;

			// Decide the swap on the last hcount of the previous line, so
			// front_is_a has flipped when pixel 0 is served. Deciding at
			// hcount == 0 shows column 0 of each swapped line from the old
			// front buffer.
			if ((hcount == H_LAST) && (swap_line < VISIBLE_HEIGHT[8:0])) begin
				if (cdc_req_sync && !cdc_ack) begin
					front_is_a <= ~front_is_a;
					cdc_ack <= 1'b1;
				end else if (!cdc_req_sync) begin
					render_overrun <= 1'b1;
				end
			end

			if (cdc_ack && !cdc_req_sync) begin
				cdc_ack <= 1'b0;
			end
		end
	end

	// when line 0 is fetched
	// The resync re-renders line 0 (scroll, alpha, render), and its scroll
	// pass reads line 0's scroll entry (alpha words 0x30, 0x31). Line 0's X/Y
	// scroll are also the sky's: no other sky line has a valid entry, so the
	// registers carry from line 0 down to the horizon. On floor stages Primal
	// Rage rewrites line 0's entry every vblank (0x245C8 writes the Y word,
	// the builder at 0x2465C ends with the X word), 1-17 lines after vblank
	// starts. Read at vblank start (line 240), the backdrop gets the previous
	// frame's scroll while the floor lines get the new one, and a bar opens at
	// the horizon when the camera rises. MAME's scanline_update reads line 0's
	// group at scanline 0, after the handler. So by default the resync fires
	// at LINE0_FETCH_LINE: after the game's vblank work (done by line 257),
	// with 3 lines of lead. The line-0 render made at the end of the previous
	// frame is dropped.
	logic [1:0] l0v_sync;
	always_ff @(posedge pixel_clk or negedge rst_n)
		if (!rst_n) l0v_sync <= 2'b00;
		else        l0v_sync <= {l0v_sync[0], line0_fetch_vblank};
	wire l0_vblank_mode = l0v_sync[1];

	// CDC: line-0 resync, pixel_clk -> clk_sys, toggle-based.
	// The trigger is vblank's start (l0_vblank_mode) or the first cycle of
	// LINE0_FETCH_LINE. It must lead vcount = 0: triggering on frame_start
	// gives line 0's render no lead time, so every frame would start a line
	// late.
	localparam logic [8:0] L0_FETCH = 9'(LINE0_FETCH_LINE);
	logic resync_toggle;
	logic vblank_prev;
	logic [8:0] vcount_prev;
	wire  resync_now = l0_vblank_mode ? (vblank && !vblank_prev)
									  : (vcount == L0_FETCH && vcount_prev != L0_FETCH);
	always_ff @(posedge pixel_clk or negedge rst_n) begin
		if (!rst_n) begin
			resync_toggle <= 1'b0;
			vblank_prev <= 1'b0;
			vcount_prev <= 9'd0;
		end else begin
			if (resync_now) resync_toggle <= ~resync_toggle;
			vblank_prev <= vblank;
			vcount_prev <= vcount;
		end
	end
	logic [1:0] frame_start_sync_chain;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) frame_start_sync_chain <= 2'b00;
		else frame_start_sync_chain <= {frame_start_sync_chain[0], resync_toggle};
	end
	logic frame_start_sync_prev;
	wire  frame_start_pulse_sys = frame_start_sync_chain[1] != frame_start_sync_prev;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) frame_start_sync_prev <= 1'b0;
		else frame_start_sync_prev <= frame_start_sync_chain[1];
	end

	// double-buffered line memory (true dual-clock dual-port)
	// Depth is the range hcount[8:0] addresses (512), not the visible width,
	// so reads at hcount >= 336 stay in range and every M10K word has an
	// initial value (Quartus Critical Warning 127005 otherwise).
	localparam int LINE_BUF_DEPTH = 512;
	// Pinned to M10K: if block memory runs short, Quartus must then say so in
	// memory terms instead of silently turning the arrays into registers and
	// failing the fit on LABs (Error 170012).
	(* ramstyle = "M10K" *) logic [23:0] line_buf_a [0:LINE_BUF_DEPTH-1];
	(* ramstyle = "M10K" *) logic [23:0] line_buf_b [0:LINE_BUF_DEPTH-1];
	integer init_i;
	initial begin
		for (init_i = 0; init_i < LINE_BUF_DEPTH; init_i = init_i + 1) begin
			line_buf_a[init_i] = 24'd0;
			line_buf_b[init_i] = 24'd0;
		end
	end

	// status block overlay
	// Six rows of 8 colour blocks, each 16 px wide and 12 lines tall, in the
	// top-left corner: green = 1, dark red = 0 (layout at the dbg_bits port).
	// Drawn whenever enabled, all red if dbg_bits is 0, so it is never
	// ambiguous whether the overlay is there.
	wire [23:0] pixel_raw = front_is_a ? line_buf_a[hcount[8:0]]
									   : line_buf_b[hcount[8:0]];
	// game_owns_screen crosses from clk_sys. It latches once and never clears,
	// so a metastable cycle can only reveal the picture one frame early.
	reg gos_s1, gos_s2;
	always_ff @(posedge pixel_clk) begin
		gos_s1 <= game_owns_screen;
		gos_s2 <= gos_s1;
	end
	wire [23:0] pixel_rgb = gos_s2 ? pixel_raw : 24'd0;
	// The overlays, debug build only (DBG_OVERLAY above).
	generate if (DBG_OVERLAY) begin : g_overlay
		// The enables come from `status[]` in clk_sys and change only when the
		// user moves an OSD entry, so a two-flop synchroniser is enough; a
		// metastable cycle would at worst blink one pixel of a debug overlay.
		reg blocks_en_s1, blocks_en_s2, text_en_s1, text_en_s2;
		always_ff @(posedge pixel_clk) begin
			blocks_en_s1 <= dbg_blocks_en;  blocks_en_s2 <= blocks_en_s1;
			text_en_s1   <= dbg_text_en;    text_en_s2   <= text_en_s1;
		end

		wire        dbg_zone  = blocks_en_s2 && (vcount < 9'd72) && (hcount < 9'd128);
		logic [2:0] dbg_row;
		always_comb
			if      (vcount < 9'd12) dbg_row = 3'd0;
			else if (vcount < 9'd24) dbg_row = 3'd1;
			else if (vcount < 9'd36) dbg_row = 3'd2;
			else if (vcount < 9'd48) dbg_row = 3'd3;
			else if (vcount < 9'd60) dbg_row = 3'd4;
			else                     dbg_row = 3'd5;
		wire [2:0]  dbg_index = hcount[6:4];            // 16px per block
		logic       dbg_val;
		always_comb case (dbg_row)
			3'd0: dbg_val = dbg_bits[{1'b0, dbg_index}];
			3'd1: dbg_val = dbg_bits[{1'b1, dbg_index}];
			3'd2: dbg_val = dbg_bits[16 + {1'b1, ~dbg_index}];   // [31:24]
			3'd3: dbg_val = dbg_bits[16 + {1'b0, ~dbg_index}];   // [23:16]
			3'd4: dbg_val = dbg_bits[32 + {1'b1, ~dbg_index}];   // [47:40]
			3'd5: dbg_val = dbg_bits[32 + {1'b0, ~dbg_index}];   // [39:32]
			default: dbg_val = 1'b0;
		endcase
		wire [23:0] dbg_rgb   = dbg_val ? 24'h00FF00 : 24'h400000;

		// hex text overlay
		// dbg_words comes from clk_sys. It is latched once per frame, in vblank,
		// when the raster is nowhere near the text box, so a changing value (the
		// PC) is never drawn half old and half new.
		reg [DBG_NSLOTS*32-1:0] dbg_words_q;
		always_ff @(posedge pixel_clk) if (vcount == 9'd250) dbg_words_q <= dbg_words;

		localparam int TXT_X0 = 8;
		localparam int TXT_W  = 104;                      // 13 columns x 8 px
		localparam int TXT_Y0 = 72;
		localparam int TXT_H  = (DBG_NSLOTS + 1) * 8;     // build stamp + NSLOTS
		wire txt_on;
		// The build stamp takes lines 72..79, between the block overlay (which
		// ends at 72) and the slot rows. The box is the TXT_* constants above,
		// shared with txt_zone so the two cannot diverge. BUILD_DIGITS is passed
		// down so a stale agt_dbg_text fails to elaborate instead of truncating
		// the stamp.
		agt_dbg_text #(.NSLOTS(DBG_NSLOTS), .BUILD_DIGITS(BUILD_DIGITS),
					   .X0(TXT_X0), .Y0(TXT_Y0)) u_dbg_text (
			.hcount(hcount), .vcount(vcount),
			.dbg_words(dbg_words_q), .build_id(dbg_build_id), .text_on(txt_on)
		);
		// The text box is drawn on a dark backdrop so it stays legible over the
		// playfield.
		//
		// One definition of the box, used both to place agt_dbg_text and to gate
		// the mixer: a separate gate here could silently discard rows the module
		// draws, which tb_dbg_text (testing the module alone) cannot see.
		wire txt_zone = text_en_s2 &&
						(vcount >= TXT_Y0[8:0]) && (vcount < (TXT_Y0 + TXT_H)) &&
						(hcount >= TXT_X0[8:0]) && (hcount < (TXT_X0 + TXT_W));

		assign rgb = dbg_zone ? dbg_rgb
				   : txt_zone ? (txt_on ? 24'hFFFFFF : 24'h000000)
							  : pixel_rgb;
	end else begin : g_no_overlay
		assign rgb = pixel_rgb;
	end endgenerate

	// clk_sys domain: render pipeline

	// CDC: cdc_ack, pixel_clk -> clk_sys
	logic [1:0] cdc_ack_sync_chain;
	wire  cdc_ack_sync = cdc_ack_sync_chain[1];
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) cdc_ack_sync_chain <= 2'b00;
		else cdc_ack_sync_chain <= {cdc_ack_sync_chain[0], cdc_ack};
	end

	// Scroll fetcher and alpha pass both read alpha RAM, never at the same
	// time: the sequencer runs scroll, then alpha, then render. Muxing on
	// al_busy keeps that ordering explicit rather than arbitrating.
	wire [10:0] ss_aram_addr, al_aram_addr;
	wire        ss_aram_rd,   al_aram_rd;
	assign alpharam_addr = al_busy ? al_aram_addr : ss_aram_addr;
	assign alpharam_rd   = al_busy ? al_aram_rd   : ss_aram_rd;

	// Declared above every instance that connects to them: a forward reference
	// in a port connection makes a 1-bit implicit net in some tools.
	logic ss_start, ss_busy, ss_done;
	logic [8:0] ss_scanline;
	logic [9:0] xscroll;
	logic [8:0] yscroll;
	logic [4:0] color_bank;
	logic [3:0] tile_bank;

	wire               rnd_wr_valid, rnd_wr_target;
	wire signed [15:0] rnd_wr_x, rnd_wr_y;
	wire [15:0]        rnd_wr_value;

	// motion objects
	// The MO bitmap is filled once per frame by the MOGO-triggered renderer
	// and read back here one line at a time. MOGO almost always arrives in
	// vblank, so the render pass and this read-back do not normally overlap.

	// Power-up values. The reset branch runs only on a clock edge; before the
	// first one these registers are X in simulation, and an X selector on this
	// path reaches the mixer, indexes the palette and makes the whole frame
	// undefined. An FPGA powers up defined; simulation needs these.
	logic [8:0] mo_fetch_x;
	// the scanline this fetch is filling
	logic [8:0] mo_fetch_y;
	// The VRAM read is a pipeline: rd_value at cycle n holds the word for the
	// address presented at n-1, while rd_valid at n answers the request at
	// n-2. With mo_fetch_x advancing on each valid, the data belongs to
	// mo_fetch_x as it was one cycle ago, so the line-buffer write column is a
	// one-cycle-delayed copy of it. Keep this alignment here, on the consumer
	// side: moving it into agt_mo_vram defeats RAM inference (Quartus 276003).
	logic [8:0] mo_wr_col_q;
	logic       mo_fetching;
	logic       mo_line_ready;

	initial begin
		mo_fetch_x    = 9'd0;
		mo_fetch_y    = 9'd0;
		mo_wr_col_q   = 9'd0;
		mo_fetching   = 1'b0;
		mo_line_ready = 1'b0;
	end

	// clk_sys: this module has no `clk`, and naming one would create an
	// implicit net stuck at zero.
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			mo_fetch_x <= 9'd0; mo_fetch_y <= 9'd0; mo_wr_col_q <= 9'd0;
			mo_fetching <= 1'b0; mo_line_ready <= 1'b0;
		end else begin
			// Refill the line buffer at the start of each scanline, before the
			// playfield pass needs it. The write column trails the fetch
			// pointer by one cycle, matching the VRAM's data latency (see
			// above).
			mo_wr_col_q <= mo_fetch_x;
			if (ss_start) begin
				mo_fetch_x <= 9'd0; mo_fetching <= 1'b1; mo_line_ready <= 1'b0;
				// ss_scanline is already the line about to be processed: the
				// sequencer sets it in the same cycle it raises ss_start and
				// holds it until the next ST_START_SCROLL, long after this
				// fetch ends.
				mo_fetch_y <= ss_scanline;
			end
			else if (mo_fetching) begin
				if (movram_rd_valid) begin
					// The pointer holds at 335; the pass ends only once column
					// 335 is written, one valid later (its data is a cycle
					// behind). Ending on the pointer would skip that final
					// write.
					if (mo_fetch_x != 9'd335) mo_fetch_x <= mo_fetch_x + 9'd1;
					if (mo_wr_col_q == 9'd335) begin
						mo_fetching <= 1'b0; mo_line_ready <= 1'b1;
					end
				end
			end
		end
	end

	wire        movram_rd_req = mo_fetching;
	wire [15:0] movram_rd_value;
	wire        movram_rd_valid;

	// hold the render until the erase sweep finishes
	// Keeps MAME's order (erase, then render): a MOGO arriving mid-sweep is
	// latched and reissued after erase_busy falls, instead of drawing into a
	// buffer that is still being cleared.
	logic mo_erase_busy;
	logic mogo_pending;
	logic mogo_start;
	logic mo_erase_busy_d;
	// The render target, latched at the MOGO request. MAME's sort_and_render
	// draws into (~m_control_bits & FRAME) as of the write that raised MOGO
	// (`bitmap_index = (~m_control_bits & CONTROL_FRAME) >> 2`,
	// atarirle.cpp:471) and finishes atomically. Ours runs for most of a
	// frame, so the target must not follow the live bit if the game (or an OSD
	// toggle) flips it mid-pass. Captured on the request, not the reissue, so
	// a deferral does not change which buffer is drawn.
	logic mo_render_frame;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) mo_render_frame <= 1'b1;
		else if (mogo_pulse) mo_render_frame <= ~mo_frame_select;
	end
	// Two holds: the erase sweep and the object-list snapshot copy, which
	// starts on the same mogo_pulse and takes ~1,026 cycles. The copy always
	// runs, so every MOGO is latched and reissued on the edge where the last
	// hold drops.
	wire mo_hold = mo_erase_busy | mo_list_snap_busy;
	logic mo_hold_d;
	assign mogo_start = mogo_pending && mo_hold_d && !mo_hold;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			mogo_pending <= 1'b0; mo_erase_busy_d <= 1'b0; mo_hold_d <= 1'b0;
			dbg_mogo_deferred <= 16'd0;
		end else begin
			mo_erase_busy_d <= mo_erase_busy;
			mo_hold_d       <= mo_hold;
			if (mogo_pulse) begin
				mogo_pending <= 1'b1;
				if (dbg_mogo_deferred != 16'hFFFF)
					dbg_mogo_deferred <= dbg_mogo_deferred + 16'd1;
			end else if (mogo_start) begin
				mogo_pending <= 1'b0;
			end
		end
	end

	agt_mo_vram #(.SCREEN_WIDTH(336), .SCREEN_HEIGHT(240),
				  .TARGETS(MO_VRAM_TARGETS), .DOUBLE(MO_VRAM_DOUBLE)) u_mo_vram (
		.clk(clk_sys), .rst_n(rst_n),
		.wr_valid(rnd_wr_valid), .wr_x(rnd_wr_x), .wr_y(rnd_wr_y),
		.wr_value(rnd_wr_value), .wr_vram_target(rnd_wr_target),
		.alt_wr_count(mo_alt_wr_count),
		.dbg_wr_drop_alt(mo_drop_alt_s), .dbg_wr_drop_erase(mo_drop_erase_s),
		// Ignored while single-buffered. The game's CONTROL_FRAME bit is wired
		// all the way to this port, so double-buffering is a change in
		// agt_mo_vram alone.
		.frame_select(mo_frame_select),
		.wr_frame(mo_render_frame),          // ~FRAME at the MOGO request
		.erase_frame(mo_erase_frame),        // old FRAME (ctrl) / FRAME (vblank)
		// start_erase is the ERASE control bit, not the render trigger:
		// separate bits in the same latch field, treated independently by
		// MAME. erase_busy must hold off the render: agt_mo_vram drops (does
		// not delay) sprite writes during a sweep, the sweep is up to 80,640
		// cycles, and MOGO normally fires in vblank, just when the erase runs.
		// In MAME, control_write() erases and then calls sort_and_render().
		.start_erase(mo_erase_pulse), .erase_busy(mo_erase_busy), .erase_done(),
		// The erase span: the vblank site clears to the bottom and resets the
		// watermark; the control-write site stops at the current raster line.
		.erase_line(vcount), .erase_to_bottom(mo_erase_to_bottom),
		.ctrl_wr(mo_ctrl_wr), .erase_full_frame(mo_erase_full_frame),
		.dbg_sweeps(dbg_erase_sweeps),
		.rd_req(movram_rd_req),
		.rd_x({7'd0, mo_fetch_x}), .rd_y({7'd0, mo_fetch_y}),
		.rd_vram_target(1'b0),          // MO layer; Primal Rage does not use TMO
		.rd_value(movram_rd_value), .rd_valid(movram_rd_valid));

	assign rnd_wr_valid_dbg = rnd_wr_valid;

	// SignalTap taps
	// rnd_wr_* are internal wires between the renderer and the MO VRAM;
	// Quartus merges and renames them, so the Node Finder cannot offer them.
	// These registered copies drive nothing, so they need both attributes:
	// `preserve` stops a register being merged or minimised away, `noprune`
	// stops one with no fan-out being removed. Filter the Node Finder on
	// `tap_` (under agt_video:u_video, in the generate block g_taps). 65
	// flip-flops, debug build only (DBG_TAPS). tap_wr_x/tap_wr_y are the
	// signed values as the renderer emits them, before any bounds test.
	generate if (DBG_TAPS) begin : g_taps
		(* preserve, noprune *) logic               tap_wr_valid;
		(* preserve, noprune *) logic signed [15:0] tap_wr_x, tap_wr_y;
		(* preserve, noprune *) logic        [15:0] tap_wr_value;
		(* preserve, noprune *) logic               tap_wr_target;
		(* preserve, noprune *) logic               tap_in_bounds;
		(* preserve, noprune *) logic               tap_erasing;
		(* preserve, noprune *) logic        [8:0]  tap_vcount;
		(* preserve, noprune *) logic               tap_vblank;
		(* preserve, noprune *) logic               tap_mogo_start;
		(* preserve, noprune *) logic               tap_render_busy;
		(* preserve, noprune *) logic               tap_erase_busy;
		always_ff @(posedge clk_sys) begin
			tap_wr_valid    <= rnd_wr_valid;
			tap_wr_x        <= rnd_wr_x;
			tap_wr_y        <= rnd_wr_y;
			tap_wr_value    <= rnd_wr_value;
			tap_wr_target   <= rnd_wr_target;
			// agt_mo_vram's bounds test, recomputed here only as a tap; the module
			// uses its own copy
			tap_in_bounds   <= (rnd_wr_x >= 16'sd0) && (rnd_wr_x < 16'sd336) &&
							   (rnd_wr_y >= 16'sd0) && (rnd_wr_y < 16'sd240);
			tap_erasing     <= mo_erase_busy;
			tap_vcount      <= vcount;
			tap_vblank      <= vblank;
			tap_mogo_start  <= mogo_start;
			tap_render_busy <= dbg_render_busy;
			tap_erase_busy  <= mo_erase_busy;
		end
	end endgenerate

	// out-of-range sprite writes
	// MOWB gates on the row only (wrb_in_rows); agt_mo_vram requires the
	// column too, and both MODR fields are gated on its bounds test, so a
	// write with a good y and a bad x is counted nowhere else. At MOWB's >>2
	// scale:
	//   oobx -- row in range, column out of range
	//   ooby -- row out of range
	// The min/max of the raw signed x over the frame shows whether the
	// coordinates are merely clipped or wildly wrong.
	logic [17:0] oobx_cnt, ooby_cnt;
	logic        oob_vbl_d;
	logic signed [15:0] xmin_acc, xmax_acc;
	wire wrb_in_cols = (rnd_wr_x >= 16'sd0) && (rnd_wr_x < 16'sd336);
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			oobx_cnt <= '0; ooby_cnt <= '0; oob_vbl_d <= 1'b0;
			xmin_acc <= 16'sh7FFF; xmax_acc <= 16'sh8000;
			dbg_drop_oobx <= 16'd0; dbg_drop_ooby <= 16'd0;
			dbg_wr_xmin   <= 16'd0; dbg_wr_xmax   <= 16'd0;
		end else begin
			oob_vbl_d <= vblank;
			if (vblank && !oob_vbl_d) begin
				dbg_drop_oobx <= oobx_cnt[17:2];
				dbg_drop_ooby <= ooby_cnt[17:2];
				dbg_wr_xmin   <= xmin_acc;
				dbg_wr_xmax   <= xmax_acc;
				oobx_cnt <= '0; ooby_cnt <= '0;
				xmin_acc <= 16'sh7FFF; xmax_acc <= 16'sh8000;
			end else if (rnd_wr_valid) begin
				if (!wrb_in_rows) begin
					if (ooby_cnt != 18'h3FFFF) ooby_cnt <= ooby_cnt + 18'd1;
				end else if (!wrb_in_cols) begin
					if (oobx_cnt != 18'h3FFFF) oobx_cnt <= oobx_cnt + 18'd1;
				end
				if (rnd_wr_x < xmin_acc) xmin_acc <= rnd_wr_x;
				if (rnd_wr_x > xmax_acc) xmax_acc <= rnd_wr_x;
			end
		end
	end

	// lost sprite writes
	// MOWB counts writes requested; these count the two ways a requested write
	// never reaches the buffer. Same frame boundary, same >>2, so:
	//   MOWB.behind + MOWB.ahead  -  MODR.alt  -  MODR.erase  =  landed
	logic mo_drop_alt_s, mo_drop_erase_s;
	logic [17:0] dra_cnt, dre_cnt;
	logic        dr_vbl_d;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			dra_cnt <= '0; dre_cnt <= '0; dr_vbl_d <= 1'b0;
			dbg_drop_alt <= 16'd0; dbg_drop_erase <= 16'd0;
		end else begin
			dr_vbl_d <= vblank;
			if (vblank && !dr_vbl_d) begin
				dbg_drop_alt   <= dra_cnt[17:2];
				dbg_drop_erase <= dre_cnt[17:2];
				dra_cnt <= '0; dre_cnt <= '0;
			end else begin
				if (mo_drop_alt_s   && dra_cnt != 18'h3FFFF) dra_cnt <= dra_cnt + 18'd1;
				if (mo_drop_erase_s && dre_cnt != 18'h3FFFF) dre_cnt <= dre_cnt + 18'd1;
			end
		end
	end

	// beam-relative sprite-write counters
	// vcount and vblank are pixel-domain, sampled here in clk_sys as
	// erase_line is. A multi-bit sample can be briefly inconsistent at a count
	// boundary; for a per-frame debug total that is noise in the last digit.
	logic [17:0] wrb_cnt, wra_cnt;
	logic        wrb_vbl_d;
	wire         wrb_in_rows = (rnd_wr_y >= 16'sd0) && (rnd_wr_y < 16'sd240);
	wire         wrb_behind  = !vblank && (rnd_wr_y[8:0] < vcount);
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			wrb_cnt <= '0; wra_cnt <= '0; wrb_vbl_d <= 1'b0;
			dbg_wr_behind <= 16'd0; dbg_wr_ahead <= 16'd0;
		end else begin
			wrb_vbl_d <= vblank;
			if (vblank && !wrb_vbl_d) begin            // frame boundary: latch, restart
				dbg_wr_behind <= wrb_cnt[17:2];
				dbg_wr_ahead  <= wra_cnt[17:2];
				wrb_cnt <= '0; wra_cnt <= '0;
			end else if (rnd_wr_valid && wrb_in_rows) begin
				if (wrb_behind) begin
					if (wrb_cnt != 18'h3FFFF) wrb_cnt <= wrb_cnt + 18'd1;
				end else begin
					if (wra_cnt != 18'h3FFFF) wra_cnt <= wra_cnt + 18'd1;
				end
			end
		end
	end

	agt_rle_renderer #(.OBJRAM_ADDR_WIDTH(11), .RLE_ROM_ADDR_WIDTH(24)) u_rle_renderer (
		.clk(clk_sys), .rst_n(rst_n),
		.start(mogo_start),
		.object_count(16'd12000),       // count_objects() on the real region
		.cliprect_left(16'sd0),
		.objram_addr(obj_rd_addr), .objram_rd(obj_rd),
		.objram_data(obj_rd_data), .objram_data_valid(obj_rd_valid),
		.rle_rom_addr(rle_rom_waddr), .rle_rom_rd(rle_rom_rd),
		.rle_rom_data(rle_rom_data), .rle_rom_data_valid(rle_rom_data_valid),
		.wr_valid(rnd_wr_valid), .wr_x(rnd_wr_x), .wr_y(rnd_wr_y),
		.wr_value(rnd_wr_value), .wr_vram_target(rnd_wr_target),
		.dbg_obj_w0(dbg_obj_w0), .dbg_obj_w4(dbg_obj_w4),
		.dbg_obj_valid(dbg_obj_valid), .dbg_obj_rejects(dbg_obj_rejects),
		.dbg_obj_starts(dbg_obj_starts), .dbg_obj_full(dbg_obj_full),
		.dbg_obj_lastidx(dbg_obj_lastidx),
		.dbg_obj_examined(dbg_obj_examined), .dbg_obj_emitted(dbg_obj_emitted),
		.dbg_obj_examined_pf(dbg_obj_examined_pf), .dbg_obj_emitted_pf(dbg_obj_emitted_pf),
		.dbg_stage_ot(dbg_stage_ot), .dbg_stage_bl(dbg_stage_bl),
		.dbg_blit_meas(dbg_blit_meas),
		.dbg_blit_wait(dbg_blit_wait), .dbg_blit_emit(dbg_blit_emit),
		.dbg_st_obj(dbg_st_obj), .dbg_st_tbl(dbg_st_tbl), .dbg_st_blt(dbg_st_blt),
		.dbg_obj_zscale(dbg_obj_zscale), .dbg_obj_offscr(dbg_obj_offscr),
		.dbg_scale_or(dbg_scale_or), .dbg_scale_and(dbg_scale_and),
		.render_busy(dbg_render_busy), .render_done(), .object_hflip_skipped(),
		.dbg_wild_code(dbg_wild_code),   .dbg_wild_scale(dbg_wild_scale),
		.dbg_wild_draw_x(dbg_wild_draw_x), .dbg_wild_width(dbg_wild_width),
		.dbg_wild_hdr(dbg_wild_hdr), .dbg_wild_mwidth(dbg_wild_mwidth),
		.dbg_start_lost(dbg_start_lost), .dbg_pass_done(dbg_pass_done),
		.dbg_pc_hits(dbg_pc_hits), .dbg_pc_misses(dbg_pc_misses),
		.dbg_big_x(dbg_big_x), .dbg_big_y(dbg_big_y),
		.dbg_sml_x(dbg_sml_x), .dbg_sml_y(dbg_sml_y),
		.dbg_big_code(dbg_big_code), .dbg_big_width(dbg_big_width),
		.dbg_big_w0(dbg_big_w0), .dbg_big_w1(dbg_big_w1),
		.dbg_hf_live(dbg_hf_live), .dbg_hf_first(dbg_hf_first), .dbg_hf_w0(dbg_hf_w0),
		.dbg_hflip_cnt(dbg_hflip_cnt), .dbg_obj_cnt(dbg_obj_cnt));

	assign dbg_in_vblank = vblank;
	assign dbg_xscroll = xscroll;
	assign dbg_yscroll = yscroll;
	assign dbg_ss_done = ss_done;
	assign dbg_ss_line = ss_scanline;
	assign dbg_vcount  = vcount;

	agt_scanline_scroll u_scanscroll (
		.clk(clk_sys), .rst_n(rst_n),
		.start(ss_start), .scanline(ss_scanline),
		.alpharam_addr(ss_aram_addr), .alpharam_rd(ss_aram_rd),
		.alpharam_data(alpharam_data), .alpharam_data_valid(alpharam_data_valid),
		.xscroll(xscroll), .yscroll(yscroll), .color_bank(color_bank), .tile_bank(tile_bank),
		.busy(ss_busy), .done(ss_done)
	);

	// alpha layer pass
	// Runs between the scroll fetch and the line render, and must complete
	// before the pixel loop starts, which reads its output.
	logic al_start, al_busy, al_done;
	logic [8:0] al_scanline;
	wire        al_pen_valid;
	wire [8:0]  al_pen_col;
	wire [7:0]  al_pen_value;

	agt_alpha_line_render u_alpha (
		.clk(clk_sys), .rst_n(rst_n),
		.start(al_start), .target_scanline(al_scanline),
		.alpharam_addr(al_aram_addr), .alpharam_rd(al_aram_rd),
		.alpharam_data(alpharam_data), .alpharam_data_valid(alpharam_data_valid),
		.charrom_addr(charrom_addr), .charrom_rd(charrom_rd),
		.charrom_data(charrom_data), .charrom_data32(charrom_data32),
		.charrom_data_valid(charrom_data_valid),
		.pen_valid(al_pen_valid), .pen_col(al_pen_col), .pen_value(al_pen_value),
		.busy(al_busy), .done(al_done)
	);

	logic lr_start, lr_busy, lr_done;
	logic [8:0] lr_target_scanline;
	logic lr_wr_valid;
	logic [8:0] lr_wr_col;
	logic [23:0] lr_wr_rgb;

	agt_playfield_line_render #(.VISIBLE_WIDTH(VISIBLE_WIDTH), .USE_BURST3(USE_BURST3)) u_linerender (
		.dbg_mo_cram(dbg_mo_cram), .dbg_mo_latch(dbg_mo_latch),
		.dbg_mo_rgb(dbg_mo_rgb), .dbg_mo_hits(dbg_mo_hits), .dbg_mo_cra(dbg_mo_cra),
		.dbg_mo_nz(dbg_mo_nz),
		.dbg_blk_cra(dbg_blk_cra), .dbg_blk_cram(dbg_blk_cram), .dbg_blk_latch(dbg_blk_latch),
		.probe_x(probe_x), .probe_y(probe_y),
		.dbg_pf_opaque(), .dbg_pf_pixels(),
		.dbg_pf_ent(), .dbg_pf_code(),
		.dbg_pf_min(), .dbg_pf_max(), .frame_tick(1'b0),  // output unused, input tied off
		.dbg_pf_bank_at(),
		.dbg_tx_ent(), .dbg_tx_code(), .dbg_tx_bank(),
		.dbg_tx_addr(), .dbg_tx_yscroll(),
		.dbg_tx_bytes(), .dbg_tx_pix01(),
		.dbg_prb_mo(dbg_prb_mo), .dbg_prb_pf(dbg_prb_pf), .dbg_prb_cra(dbg_prb_cra), .dbg_prb_cram(dbg_prb_cram),
		.clk(clk_sys), .rst_n(rst_n),
		.start(lr_start), .target_scanline(lr_target_scanline),
		.an_wr_valid(al_pen_valid), .an_wr_col(al_pen_col),
		.an_wr_pen(al_pen_value),
		.xscroll(xscroll), .yscroll(yscroll), .color_bank(color_bank), .tile_bank(tile_bank),
		.color_latch(color_latch),
		.pfram_addr(pfram_addr), .pfram_rd(pfram_rd),
		.pfram_data(pfram_data), .pfram_data_valid(pfram_data_valid),
		.tile_rom_addr(tile_rom_addr), .tile_rom_rd(tile_rom_rd),
		.tile_rom_rd3(tile_rom_rd3), .tile_rom_data96(tile_rom_data96),
		// one entry per cycle as it arrives, into an array the line renderer
		// owns; no wide vector anywhere on this path
		.dbg_mo_sel(dbg_mo_sel),
		.mo_line_ready(mo_line_ready),
		.mo_wr_valid(movram_rd_valid && mo_fetching),
		.mo_wr_col(mo_wr_col_q),
		.mo_wr_value(movram_rd_value),
		.tile_rom_data(tile_rom_data), .tile_rom_data32(tile_rom_data32),
		.tile_rom_data_valid(tile_rom_data_valid),
		.cram_addr(cram_addr), .cram_data(cram_data),
		.pen_addr_r(pen_addr_r), .pen_addr_g(pen_addr_g), .pen_addr_b(pen_addr_b),
		.pen_data_r(pen_data_r), .pen_data_g(pen_data_g), .pen_data_b(pen_data_b),
		.line_wr_valid(lr_wr_valid), .line_wr_col(lr_wr_col), .line_wr_rgb(lr_wr_rgb),
		.busy(lr_busy), .done(lr_done)
	);

	// The render targets clk_sys's own copy of the back-buffer select, not a
	// second CDC of front_is_a: clk_sys flips it when it sees the ack for
	// pixel_clk's flip, so the two stay in lockstep by construction.
	logic render_target_is_a;

	always_ff @(posedge clk_sys) begin
		if (lr_wr_valid) begin
			if (render_target_is_a) line_buf_a[lr_wr_col] <= lr_wr_rgb;
			else                    line_buf_b[lr_wr_col] <= lr_wr_rgb;
		end
	end

	typedef enum logic [2:0] {
		ST_INIT, ST_START_SCROLL, ST_WAIT_SCROLL, ST_WAIT_ALPHA, ST_WAIT_RENDER,
		ST_HOLD_REQ, ST_WAIT_ACK_CLEAR
	} state_t;
	state_t state;

	logic [8:0] next_render_line;
	logic discard_render;   // set at resync if a stale pass is in flight

	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_INIT;
			ss_start <= 1'b0;
			al_start <= 1'b0;
			lr_start <= 1'b0;
			cdc_req <= 1'b0;
			next_render_line <= 9'd0;
			discard_render <= 1'b0;
			render_target_is_a <= 1'b0;  // front_is_a starts at 1: first render goes to B
		end else begin
			ss_start <= 1'b0;
			al_start <= 1'b0;
			lr_start <= 1'b0;

			unique case (state)
				ST_INIT: begin
					next_render_line <= 9'd0;
					ss_scanline <= 9'd0;
					ss_start <= 1'b1;
					state <= ST_WAIT_SCROLL;
				end

				ST_START_SCROLL: begin
					ss_scanline <= next_render_line;
					ss_start <= 1'b1;
					state <= ST_WAIT_SCROLL;
				end
				ST_WAIT_SCROLL: begin
					if (ss_done) begin
						if (discard_render) begin
							// stale scroll pass for a pre-resync line: skip
							// it, restart at line 0
							discard_render <= 1'b0;
							state <= ST_START_SCROLL;
						end else begin
							// alpha first: the pixel loop reads its output, so
							// it must be complete before lr_start
							al_scanline <= next_render_line;
							al_start    <= 1'b1;
							state       <= ST_WAIT_ALPHA;
						end
					end
				end
				ST_WAIT_ALPHA: begin
					if (al_done) begin
						lr_target_scanline <= next_render_line;
						lr_start <= 1'b1;
						state <= ST_WAIT_RENDER;
					end
				end
				ST_WAIT_RENDER: begin
					if (lr_done) begin
						if (discard_render) begin
							// stale render finishing after a resync: don't
							// offer it for swap; the line-0 pass overwrites
							// the same back buffer
							discard_render <= 1'b0;
							state <= ST_START_SCROLL;
						end else begin
							cdc_req <= 1'b1;
							state <= ST_HOLD_REQ;
						end
					end
				end

				ST_HOLD_REQ: begin
					if (cdc_ack_sync) begin
						cdc_req <= 1'b0;
						render_target_is_a <= ~render_target_is_a;
						if (next_render_line == VISIBLE_HEIGHT[8:0] - 1'b1)
							next_render_line <= 9'd0;
						else
							next_render_line <= next_render_line + 9'd1;
						state <= ST_WAIT_ACK_CLEAR;
					end
				end
				ST_WAIT_ACK_CLEAR: begin
					if (!cdc_ack_sync) begin
						state <= ST_START_SCROLL;
					end
				end

				default: state <= ST_START_SCROLL;
			endcase

			// line-0 resync, after the case so it overrides same-cycle
			// assignments. Retarget the pipeline to line 0 with lead time, and
			// keep work for the old cadence from leaking through:
			//  - holding a completed render: drop the request (the line-0
			//    render overwrites the same back buffer);
			//  - a scroll/render pass in flight: mark it for discard on
			//    completion (the submodule FSMs can't be safely aborted, but
			//    their results can be ignored);
			//  - ack in flight: let the handshake close normally; only the
			//    line numbering needs correcting.
			if (frame_start_pulse_sys) begin
				next_render_line <= 9'd0;
				if (state == ST_HOLD_REQ && !cdc_ack_sync) begin
					cdc_req <= 1'b0;
					state <= ST_START_SCROLL;
				end else if (state == ST_START_SCROLL || state == ST_WAIT_SCROLL || state == ST_WAIT_RENDER) begin
					discard_render <= 1'b1;
				end
			end
		end
	end

	// per-line pacing counters
	// Saturating, clk_sys, since reset (an OSD reset zeroes them). Timers
	// restart on ss_start; each pass is closed by its done in the state
	// waiting for it, so a discarded (post-resync) pass is still measured and
	// a stray done in any other state is ignored. line_worst >= every pass
	// worst.
	logic [15:0] t_line, t_pass;
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			t_line <= 16'd0; t_pass <= 16'd0;
			dbg_line_worst <= 16'd0;  dbg_scroll_worst <= 16'd0;
			dbg_alpha_worst <= 16'd0; dbg_render_worst <= 16'd0;
			line_late <= 1'b0;
		end else begin
			line_late <= (state == ST_WAIT_SCROLL || state == ST_WAIT_ALPHA
						  || state == ST_WAIT_RENDER) && (t_line >= LATE_AT[15:0]);
			if (t_line != 16'hFFFF) t_line <= t_line + 16'd1;
			if (t_pass != 16'hFFFF) t_pass <= t_pass + 16'd1;
			if (ss_start) begin
				t_line <= 16'd0; t_pass <= 16'd0;
			end
			if (state == ST_WAIT_SCROLL && ss_done) begin
				if (t_pass > dbg_scroll_worst) dbg_scroll_worst <= t_pass;
				t_pass <= 16'd0;
			end
			if (state == ST_WAIT_ALPHA && al_done) begin
				if (t_pass > dbg_alpha_worst) dbg_alpha_worst <= t_pass;
				t_pass <= 16'd0;
			end
			if (state == ST_WAIT_RENDER && lr_done) begin
				if (t_pass > dbg_render_worst) dbg_render_worst <= t_pass;
				if (t_line > dbg_line_worst)   dbg_line_worst   <= t_line;
			end
		end
	end

	// Where the overruns happen (D-648; the ports' comment has the layout).
	// render_overrun is a one-pixel_clk pulse (8 clk_sys cycles; the clocks
	// are one PLL's, 8:1): two flops and an edge. At that edge the renderer is
	// still on the late line: next_render_line moves only on the swap's ack,
	// which the missed swap has not given. The line's passes are latched when
	// it completes; if it had completed (ST_HOLD_REQ: the handshake missed
	// by a hair), at once.
	logic [2:0]  ovr_s;
	logic [15:0] t_alpha_ln, t_render_ln;   // this line's passes, as they close
	logic        late_pend;
	logic [7:0]  ovr_lo, ovr_hi, ovr_last, ovr_spr, late_al, late_lr;
	wire         ovr_hit  = ovr_s[1] && !ovr_s[2];
	wire  [7:0]  ovr_line = next_render_line[7:0];   // 0-239
	function automatic logic [7:0] step16(input logic [15:0] t);
		step16 = (t[15:12] != 4'd0) ? 8'hFF : t[11:4];
	endfunction
	always_ff @(posedge clk_sys or negedge rst_n) begin
		if (!rst_n) begin
			ovr_s <= 3'd0;
			t_alpha_ln <= 16'd0; t_render_ln <= 16'd0; late_pend <= 1'b0;
			ovr_lo <= 8'hFF; ovr_hi <= 8'd0; ovr_last <= 8'd0; ovr_spr <= 8'd0;
			late_al <= 8'd0; late_lr <= 8'd0;
		end else begin
			ovr_s <= {ovr_s[1:0], render_overrun};
			if (state == ST_WAIT_ALPHA && al_done) t_alpha_ln <= t_pass;
			if (state == ST_WAIT_RENDER && lr_done) begin
				t_render_ln <= t_pass;
				if (late_pend) begin
					late_al   <= step16(t_alpha_ln);
					late_lr   <= step16(t_pass);
					late_pend <= 1'b0;
				end
			end
			if (ovr_hit) begin
				if (ovr_line < ovr_lo) ovr_lo <= ovr_line;
				if (ovr_line > ovr_hi) ovr_hi <= ovr_line;
				ovr_last <= ovr_line;
				if (dbg_render_busy && ovr_spr != 8'hFF) ovr_spr <= ovr_spr + 8'd1;
				if (state == ST_HOLD_REQ) begin
					late_al   <= step16(t_alpha_ln);
					late_lr   <= step16(t_render_ln);
				end else if (state == ST_WAIT_RENDER && lr_done) begin
					late_al   <= step16(t_alpha_ln);   // completing this cycle
					late_lr   <= step16(t_pass);
					late_pend <= 1'b0;
				end else begin
					late_pend <= 1'b1;
				end
			end
		end
	end
	assign dbg_ovr_where = {ovr_lo, ovr_hi, ovr_last, ovr_spr};
	assign dbg_ovr_pass  = {late_al, late_lr};

endmodule
