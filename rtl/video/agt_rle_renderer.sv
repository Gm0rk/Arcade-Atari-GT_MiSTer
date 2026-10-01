// agt_rle_renderer.sv -- Atari GT / Primal Rage RLE object render orchestrator
//
// Chains the RLE pipeline stages into one MOGO-triggered render pass:
//   agt_rle_objlist -> agt_rle_objtable -> agt_rle_blit -> agt_mo_pixel_combine
// producing a per-pixel write stream (wr_valid/wr_x/wr_y/wr_value/
// wr_vram_target) for an externally instantiated agt_mo_vram.sv.
//
// objlist scans far faster than objtable+blit can draw, so its
// consumer_ready is pulsed once per object, only after that object's blit
// has finished. objtable and blit share the "rle" ROM port but are never
// active at once (objtable finishes before blit starts), so a plain mux
// suffices.

module agt_rle_renderer #(
	parameter int OBJRAM_ADDR_WIDTH  = 11,
	parameter int RLE_ROM_ADDR_WIDTH = 24
) (
	input  logic clk,
	input  logic rst_n,

	input  logic        start,            // MOGO trigger: begin a full render pass
	input  logic [15:0] object_count,
	input  logic signed [15:0] cliprect_left,

	// object list RAM port (to the internal objlist)
	output logic [OBJRAM_ADDR_WIDTH-1:0] objram_addr,
	output logic                         objram_rd,
	input  logic [15:0]                  objram_data,
	input  logic                         objram_data_valid,

	// shared "rle" gfx ROM port (objtable or blit)
	output logic [RLE_ROM_ADDR_WIDTH-1:0] rle_rom_addr,
	output logic                          rle_rom_rd,
	input  logic [15:0]                   rle_rom_data,
	input  logic                          rle_rom_data_valid,

	// per-pixel write stream (to an external agt_mo_vram)
	output logic               wr_valid,
	output logic signed [15:0] wr_x,
	output logic signed [15:0] wr_y,
	output logic [15:0]        wr_value,
	output logic                wr_vram_target,

	// objlist debug taps, forwarded to the overlay
	output logic [15:0] dbg_obj_w0,
	output logic [15:0] dbg_obj_w4,
	output logic        dbg_obj_valid,
	output logic [7:0]  dbg_obj_rejects,
	output logic [15:0] dbg_obj_starts,
	output logic [15:0] dbg_obj_full,
	output logic [8:0]  dbg_obj_lastidx,
	output logic [15:0] dbg_obj_examined_pf,   // per pass
	output logic [15:0] dbg_obj_emitted_pf,    // per pass
	output logic [15:0] dbg_obj_examined,
	output logic [15:0] dbg_obj_emitted,
	// renderer stage completions (saturating)
	output logic [15:0] dbg_stage_ot,   // objtable lookups completed
	output logic [15:0] dbg_stage_bl,   // blit passes completed
	output logic        dbg_blit_meas,  // live blit state flags
	output logic        dbg_blit_wait,
	output logic        dbg_blit_emit,
	// per-stage cycle counts, live within the current pass
	output logic [23:0]  dbg_st_obj,   // ST_WAIT_OBJECT
	output logic [23:0]  dbg_st_tbl,   // ST_WAIT_OBJTABLE
	output logic [23:0]  dbg_st_blt,   // ST_WAIT_BLIT
	// Attributes of accepted objects, within the current pass:
	//   dbg_obj_zscale : scale == 0            -> nothing to draw
	//   dbg_obj_offscr : y outside 0..239      -> drawn off the visible area
	// dbg_scale_or/dbg_scale_and give the range of scale and cannot saturate.
	output logic [15:0]  dbg_obj_zscale,
	output logic [15:0]  dbg_obj_offscr,
	output logic [15:0]  dbg_scale_or,
	output logic [15:0]  dbg_scale_and,
	output logic render_busy,
	output logic render_done,          // 1-cycle pulse: full object list pass complete
	output logic object_hflip_skipped, // 1-cycle pulse: object skipped (hflip unsupported)
	// The first object of each pass whose horizontal span leaves the screen by
	// more than a plausible clip. These are the whole input set of
	// `pix_x = draw_x + xpix`, so tools/dump_rle_object.py can decode the
	// object from the ROM offline.
	output logic [15:0] dbg_wild_code,
	output logic [15:0] dbg_wild_scale,
	output logic signed [15:0] dbg_wild_draw_x,
	output logic [15:0] dbg_wild_width,
	// For the same object: the first row-header word the blit was given and the
	// unscaled width it measured from the words that followed.
	output logic [15:0] dbg_wild_hdr,
	output logic [15:0] dbg_wild_mwidth,
	// A start that arrives while a pass is still running is ignored (only
	// ST_IDLE looks at `start`), so that frame is not rendered:
	// {starts ignored because busy, passes completed}.
	output logic [15:0] dbg_start_lost,
	output logic [15:0] dbg_pass_done,
	output logic [15:0] dbg_pc_hits, dbg_pc_misses,   // blit prescan cache hits/misses
	// Widest object of each pass: {hflip, code[14:0]}, its scaled width, its
	// object-list position and raw words 0/1 (tools/dump_rle_object.py shows
	// which way its art faces). A second witness gives the position of the
	// widest object under 300 px, since the widest is often a full-width
	// backdrop.
	output logic [15:0] dbg_big_x,      // object-list x of the widest object
	output logic [15:0] dbg_big_y,
	output logic [15:0] dbg_sml_x,      // ...and of the widest under 300 px
	output logic [15:0] dbg_sml_y,
	output logic [15:0] dbg_big_code,
	output logic [15:0] dbg_big_width,
	// raw object-RAM words of the widest object
	output logic [15:0] dbg_big_w0,
	output logic [15:0] dbg_big_w1,
	// hflip census: dbg_hf_* straight from agt_rle_objlist, then per pass
	// {objects accepted with hflip = 1, objects accepted in total}.
	output logic [15:0] dbg_hf_w0,
	output logic [15:0] dbg_hf_live,
	output logic [15:0] dbg_hf_first,
	output logic [15:0] dbg_hflip_cnt,
	output logic [15:0] dbg_obj_cnt
);

	logic objlist_object_valid, objlist_consumer_ready;
	logic [14:0] ol_code;
	logic [15:0] ol_color;
	logic signed [15:0] ol_x, ol_y;
	logic [15:0] ol_scale;
	logic ol_hflip, ol_vram_target;
	logic [15:0] ol_w0, ol_w1;
	logic [15:0] cur_w0, cur_w1;
	logic objlist_busy, objlist_done;

	agt_rle_objlist #(.OBJRAM_ADDR_WIDTH(OBJRAM_ADDR_WIDTH)) u_objlist (
		.clk(clk), .rst_n(rst_n),
		.start(start), .object_count(object_count), .cliprect_left(cliprect_left),
		.objram_addr(objram_addr), .objram_rd(objram_rd),
		.objram_data(objram_data), .objram_data_valid(objram_data_valid),
		.object_valid(objlist_object_valid), .consumer_ready(objlist_consumer_ready),
		.code(ol_code), .color(ol_color), .x(ol_x), .y(ol_y), .scale(ol_scale),
		.hflip(ol_hflip), .vram_target(ol_vram_target),
		.dbg_emit_w0(ol_w0), .dbg_emit_w1(ol_w1),
		.dbg_hf_live(dbg_hf_live), .dbg_hf_first(dbg_hf_first), .dbg_hf_w0(dbg_hf_w0),
		.dbg_w0(dbg_obj_w0), .dbg_w4(dbg_obj_w4),
		.dbg_valid(dbg_obj_valid), .dbg_rejects(dbg_obj_rejects),
		.dbg_starts(dbg_obj_starts), .dbg_full_scans(dbg_obj_full),
		.dbg_last_idx(dbg_obj_lastidx),
		.dbg_examined(dbg_obj_examined), .dbg_emitted(dbg_obj_emitted),
		.dbg_examined_pf(dbg_obj_examined_pf), .dbg_emitted_pf(dbg_obj_emitted_pf),
		.busy(objlist_busy), .done(objlist_done)
	);

	// per-object fields, held for the whole objtable+blit sequence
	logic [14:0] cur_code;
	logic [15:0] cur_color;
	logic signed [15:0] cur_x, cur_y;
	logic [15:0] cur_scale;
	logic cur_hflip, cur_vram_target;

	// objtable and blit completion counts, saturating
	logic [15:0] dbg_ot_done, dbg_bl_done;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin dbg_ot_done <= 16'd0; dbg_bl_done <= 16'd0; end
		else begin
			if (objtable_done && dbg_ot_done != 16'hFFFF)
				dbg_ot_done <= dbg_ot_done + 16'd1;
			if (blit_done    && dbg_bl_done != 16'hFFFF)
				dbg_bl_done <= dbg_bl_done + 16'd1;
		end
	end
	assign dbg_stage_ot = dbg_ot_done;
	assign dbg_stage_bl = dbg_bl_done;

	logic objtable_start, objtable_done;
	logic signed [15:0] ot_xoffs, ot_yoffs;
	logic [2:0] ot_table_sel;
	logic [RLE_ROM_ADDR_WIDTH-1:0] ot_data_offset;
	logic ot_valid;
	logic [RLE_ROM_ADDR_WIDTH-1:0] objtable_rom_addr;
	logic objtable_rom_rd;

	agt_rle_objtable #(.ROM_ADDR_WIDTH(RLE_ROM_ADDR_WIDTH)) u_objtable (
		.clk(clk), .rst_n(rst_n),
		.start(objtable_start), .code({1'b0, cur_code}),
		.rom_addr(objtable_rom_addr), .rom_rd(objtable_rom_rd),
		.rom_data(rle_rom_data), .rom_data_valid(rle_rom_data_valid),
		.done(objtable_done), .xoffs(ot_xoffs), .yoffs(ot_yoffs),
		.table_sel(ot_table_sel), .data_offset(ot_data_offset), .valid(ot_valid)
	);

	logic blit_start, blit_busy, blit_done, blit_hflip_unsupported;
	// blit debug outputs, for the wild and widest-object witnesses
	logic               blit_setup_valid;
	logic signed [15:0] blit_draw_x;
	logic        [15:0] blit_scaled_width;
	logic        [15:0] blit_first_hdr, blit_meas_width;
	logic               wild_captured;
	logic [15:0]        big_width_acc;
	logic [15:0]        sml_width_acc;   // widest under 300
	logic [15:0]        hflip_acc, obj_acc;
	// "wild" = the object's horizontal span leaves the screen by more than a
	// full screen width in either direction. A sprite half off the edge is
	// normal; this catches only a runaway.
	wire signed [16:0] wild_end = $signed({blit_draw_x[15], blit_draw_x}) +
								  $signed({1'b0, blit_scaled_width});
	wire wild_now = blit_setup_valid &&
					((blit_draw_x < -16'sd336) || (wild_end > 17'sd672));
	logic [RLE_ROM_ADDR_WIDTH-1:0] blit_rom_addr;
	logic blit_rom_rd;
	logic blit_pix_valid;
	logic signed [15:0] blit_pix_x, blit_pix_y;
	logic [5:0] blit_pix_value;

	agt_rle_blit #(.ROM_ADDR_WIDTH(RLE_ROM_ADDR_WIDTH)) u_blit (
		.clk(clk), .rst_n(rst_n),
		.start(blit_start),
		.xoffs(ot_xoffs), .yoffs(ot_yoffs), .table_sel(ot_table_sel),
		.data_offset(ot_data_offset), .hdr_valid(ot_valid),
		.code(cur_code),
		.pos_x(cur_x), .pos_y(cur_y), .scale(cur_scale), .hflip(cur_hflip),
		.rom_addr(blit_rom_addr), .rom_rd(blit_rom_rd),
		.rom_data(rle_rom_data), .rom_data_valid(rle_rom_data_valid),
		.pix_valid(blit_pix_valid), .pix_x(blit_pix_x), .pix_y(blit_pix_y), .pix_value(blit_pix_value),
		.dbg_pc_hits(dbg_pc_hits), .dbg_pc_misses(dbg_pc_misses),
		.dbg_in_meas(dbg_blit_meas),
		.dbg_in_wait(dbg_blit_wait), .dbg_emitting(dbg_blit_emit),
		.busy(blit_busy), .done(blit_done), .hflip_unsupported(blit_hflip_unsupported),
		.dbg_setup_valid(blit_setup_valid), .dbg_draw_x(blit_draw_x),
		.dbg_scaled_width(blit_scaled_width),
		.dbg_first_hdr(blit_first_hdr), .dbg_meas_width(blit_meas_width)
	);

	// objtable owns the ROM port in ST_START_OBJTABLE/ST_WAIT_OBJTABLE, the
	// blit otherwise.
	logic objtable_active;
	assign objtable_active = (state == ST_START_OBJTABLE) || (state == ST_WAIT_OBJTABLE);

	assign rle_rom_addr = objtable_active ? objtable_rom_addr : blit_rom_addr;
	assign rle_rom_rd   = objtable_active ? objtable_rom_rd   : blit_rom_rd;

	logic [15:0] combined_pixel;
	// m_rle_bpp[8] = {4,5,5,5,6,6,6,6}, indexed by table_sel (atarirle.cpp
	// build_rle_tables()). ot_table_sel is stable for the whole blit, so this
	// needs no latch.
	wire [2:0] cur_bpp = (ot_table_sel == 3'd0) ? 3'd4
					   : (ot_table_sel <= 3'd3) ? 3'd5
												: 3'd6;
	agt_mo_pixel_combine u_combine (
		.raw_pixel_value(blit_pix_value), .color(cur_color), .bpp(cur_bpp),
		.mo_pixel(combined_pixel)
	);

	// pixel write stream: blit's pixels through the combinational combine,
	// registered (one cycle latency)
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_valid <= 1'b0;
		end else begin
			wr_valid <= blit_pix_valid;
			wr_x <= blit_pix_x;
			wr_y <= blit_pix_y;
			wr_value <= combined_pixel;
			wr_vram_target <= cur_vram_target;
		end
	end

	// Top-level FSM: each object through objtable, then blit
	typedef enum logic [2:0] {
		ST_IDLE, ST_WAIT_OBJECT, ST_START_OBJTABLE, ST_WAIT_OBJTABLE,
		ST_START_BLIT, ST_WAIT_BLIT
	} state_t;
	state_t state;

	// dbg_st_*: cycles spent in each wait state, reset at ST_IDLE, so a value
	// read mid-pass shows where the current pass is stuck, even one that never
	// completes.

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			render_busy <= 1'b0;
			render_done <= 1'b0;
			object_hflip_skipped <= 1'b0;
			objlist_consumer_ready <= 1'b0;
			objtable_start <= 1'b0;
			blit_start <= 1'b0;
			wild_captured <= 1'b0;
			dbg_wild_code <= 16'd0; dbg_wild_scale <= 16'd0;
			dbg_wild_draw_x <= 16'sd0; dbg_wild_width <= 16'd0;
			dbg_wild_hdr <= 16'd0; dbg_wild_mwidth <= 16'd0;
			dbg_start_lost <= 16'd0; dbg_pass_done <= 16'd0;
			dbg_big_code <= 16'd0; dbg_big_width <= 16'd0; big_width_acc <= 16'd0;
			dbg_big_x <= 16'd0; dbg_big_y <= 16'd0;
			dbg_sml_x <= 16'd0; dbg_sml_y <= 16'd0; sml_width_acc <= 16'd0;
			dbg_big_w0 <= 16'd0; dbg_big_w1 <= 16'd0;
			dbg_hflip_cnt <= 16'd0; dbg_obj_cnt <= 16'd0;
			hflip_acc <= 16'd0; obj_acc <= 16'd0;
		end else begin
			// Widest object of the running pass. No width filter: Primal Rage's
			// backgrounds are motion objects, so the widest is often a backdrop.
			if (blit_setup_valid && blit_scaled_width > big_width_acc) begin
				big_width_acc <= blit_scaled_width;
				dbg_big_x     <= cur_x;
				dbg_big_y     <= cur_y;
				dbg_big_code  <= {cur_hflip, cur_code[14:0]};
				dbg_big_width <= blit_scaled_width;
				dbg_big_w0    <= cur_w0;
				dbg_big_w1    <= cur_w1;
			end
			// Second witness: the widest object under 300 px, excluding a full-width
			// backdrop.
			if (blit_setup_valid && blit_scaled_width > sml_width_acc &&
				blit_scaled_width < 16'd300) begin
				sml_width_acc <= blit_scaled_width;
				dbg_sml_x     <= cur_x;
				dbg_sml_y     <= cur_y;
			end

			// A start while busy is a lost frame. Counted outside the case so it is
			// independent of the state.
			if (start && state != ST_IDLE && dbg_start_lost != 16'hFFFF)
				dbg_start_lost <= dbg_start_lost + 16'd1;
			render_done <= 1'b0;
			object_hflip_skipped <= 1'b0;
			objlist_consumer_ready <= 1'b0;
			objtable_start <= 1'b0;
			blit_start <= 1'b0;

			// First wild object of the pass, held (not latest-wins) until the next
			// pass starts, so the reading is stable.
			if (wild_now && !wild_captured) begin
				wild_captured   <= 1'b1;
				dbg_wild_code   <= {1'b1, cur_code[14:0]};  // top bit = captured
				dbg_wild_scale  <= cur_scale;
				dbg_wild_draw_x <= blit_draw_x;
				dbg_wild_width  <= blit_scaled_width;
				dbg_wild_hdr    <= blit_first_hdr;
				dbg_wild_mwidth <= blit_meas_width;
			end

			unique case (state)
				ST_IDLE: begin
					dbg_st_obj <= 24'd0;
					dbg_obj_zscale <= 16'd0; dbg_obj_offscr <= 16'd0;
					dbg_scale_or <= 16'd0; dbg_scale_and <= 16'hFFFF; dbg_st_tbl <= 24'd0; dbg_st_blt <= 24'd0;
					if (start) begin
						render_busy <= 1'b1;
						wild_captured   <= 1'b0;               // new pass
						big_width_acc   <= 16'd0;              // new pass (outputs hold last pass's answer)
						sml_width_acc   <= 16'd0;              // same
						// publish the pass that just ended, restart
						dbg_hflip_cnt   <= hflip_acc;
						dbg_obj_cnt     <= obj_acc;
						hflip_acc       <= 16'd0;
						obj_acc         <= 16'd0;
						dbg_wild_code   <= 16'd0; dbg_wild_scale <= 16'd0;
						dbg_wild_draw_x <= 16'sd0; dbg_wild_width <= 16'd0;
						dbg_wild_hdr <= 16'd0; dbg_wild_mwidth <= 16'd0;
						state <= ST_WAIT_OBJECT;
					end
				end

				// objlist runs on its own; wait for either a new object or the
				// end of the pass
				ST_WAIT_OBJECT: begin
					if (dbg_st_obj != 24'hFFFFFF) dbg_st_obj <= dbg_st_obj + 24'd1;
					if (objlist_object_valid) begin
						// attributes of every accepted object, as it is accepted
						if (ol_scale == 16'd0 && dbg_obj_zscale != 16'hFFFF)
							dbg_obj_zscale <= dbg_obj_zscale + 16'd1;
						if ((ol_y < 16'sd0 || ol_y > 16'sd239) &&
							dbg_obj_offscr != 16'hFFFF)
							dbg_obj_offscr <= dbg_obj_offscr + 16'd1;
						dbg_scale_or  <= dbg_scale_or  | ol_scale;
						dbg_scale_and <= dbg_scale_and & ol_scale;
						cur_code        <= ol_code;
						cur_color       <= ol_color;
						cur_x           <= ol_x;
						cur_y           <= ol_y;
						cur_scale       <= ol_scale;
						cur_hflip       <= ol_hflip;
						cur_w0          <= ol_w0;
						cur_w1          <= ol_w1;
						// count every object this pass accepts
						if (obj_acc != 16'hFFFF) obj_acc <= obj_acc + 16'd1;
						if (ol_hflip && hflip_acc != 16'hFFFF)
							hflip_acc <= hflip_acc + 16'd1;
						cur_vram_target <= ol_vram_target;
						state           <= ST_START_OBJTABLE;
					end else if (objlist_done) begin
						render_busy <= 1'b0;
						render_done <= 1'b1;
						if (dbg_pass_done != 16'hFFFF) dbg_pass_done <= dbg_pass_done + 16'd1;
						state       <= ST_IDLE;
					end
				end

				ST_START_OBJTABLE: begin
					objtable_start <= 1'b1;
					state <= ST_WAIT_OBJTABLE;
				end
				ST_WAIT_OBJTABLE: begin
					if (dbg_st_tbl != 24'hFFFFFF) dbg_st_tbl <= dbg_st_tbl + 24'd1;
					if (objtable_done) state <= ST_START_BLIT;
				end

				ST_START_BLIT: begin
					blit_start <= 1'b1;
					state <= ST_WAIT_BLIT;
				end
				ST_WAIT_BLIT: begin
					if (dbg_st_blt != 24'hFFFFFF) dbg_st_blt <= dbg_st_blt + 24'd1;
					if (blit_done) begin
						if (blit_hflip_unsupported)
							object_hflip_skipped <= 1'b1;
						objlist_consumer_ready <= 1'b1;  // let objlist advance
						state <= ST_WAIT_OBJECT;
					end
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
