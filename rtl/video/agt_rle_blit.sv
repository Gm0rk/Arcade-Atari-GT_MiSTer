// agt_rle_blit.sv -- RLE motion-object scale/decode engine.
//
// Given an object's header (from agt_rle_objtable.sv: xoffs, yoffs,
// table_sel, data_offset) and its object-list placement (position, scale,
// hflip), runs atarirle.cpp's prescan_rle() + draw_rle_zoom(), streaming out
// (x, y, pixel_value) for every opaque destination pixel.
//
// Measure: the scale math needs the unscaled width and height first:
//   scaled_width  = round((xscale16 * width)  >> 16, min 1)
//   scaled_height = round((yscale16 * height) >> 16, min 1)
//   dx = (width  << 16) / scaled_width     -- source step per dest column
//   dy = (height << 16) / scaled_height    -- source step per dest row
// Neither is in the header; they emerge only by walking the compressed rows
// to their terminator (entry_count == 0), as prescan_rle() does. MAME
// prescans every object once at init; here a per-code prescan cache keeps
// the results and a miss runs the measure pass.
//
// Row walk (validated against rle_golden.py's draw_object()): `row_start`
// advances only when moving to a new source row (zooming out skips rows,
// zooming in revisits one row for several destination lines). Each
// destination line decodes the row at `row_start` afresh, with its own
// sourcex/rle_end/xpix; only the seek step consumes rows.
//
// Placement, from atarirle.cpp's draw_rle() (lines 519-550):
//   scaled_xoffs = (raw_scale * info.xoffs) >> 12
//   scaled_yoffs = (raw_scale * info.yoffs) >> 12
//   x -= scaled_xoffs;  y -= scaled_yoffs;
// With xscale16 = raw_scale << 4 (exact), (xscale16 * xoffs) >> 16 gives the
// same value for every input, negative xoffs included. cliprect.left() is
// not added here: sort_and_render() applies it before draw_rle(), so it
// belongs in pos_x.
//
// hflip: MAME's draw_rle_zoom_hflip differs from draw_rle_zoom only in the
// destination, which starts at the right edge and decrements (*dest--), with
// the clip edges trading roles. Decode order and `sourcex += dx` are
// unchanged, so there is one decode path and a direction flag on pix_x.
// draw_rle() also measures the x offset from the right edge (see
// ST_SETUP_POSITION).
//
// `hflip_unsupported` never pulses. tb_rle_renderer counts its pulses
// (object_hflip_skipped) and fails on any, a guard against reintroducing a
// skip path.

module agt_rle_blit #(
	parameter int ROM_ADDR_WIDTH = 24
) (
	input  logic        clk,
	input  logic        rst_n,

	// start a new object
	input  logic        start,
	// header fields, from agt_rle_objtable.sv
	input  logic signed [15:0]        xoffs,
	input  logic signed [15:0]        yoffs,
	input  logic [2:0]                table_sel,
	input  logic [ROM_ADDR_WIDTH-1:0] data_offset,
	input  logic                      hdr_valid,
	// Object code, used only as the prescan cache key: the measured
	// width/height depend only on this code's RLE data, which is ROM (MAME
	// likewise runs prescan_rle once per object in device_start() and then
	// reads m_info[code]).
	input  logic [14:0]               code,
	// object-list placement fields
	input  logic signed [15:0] pos_x,
	input  logic signed [15:0] pos_y,
	input  logic [15:0]        scale,      // raw field; xscale16 = scale << 4
	input  logic               hflip,

	// generic synchronous ROM port (shared "rle" region)
	output logic [ROM_ADDR_WIDTH-1:0] rom_addr,
	output logic                      rom_rd,
	// Profiling strobes. dbg_in_wait is gated on state, not on `rom_rd` (a
	// one-cycle pulse), so it covers the whole stall. dbg_in_meas is high
	// during the MEASURE traversal, which walks an object's whole RLE stream
	// to compute width/height before drawing it.
	output logic [15:0]               dbg_pc_hits,    // prescan cache
	output logic [15:0]               dbg_pc_misses,
	output logic                      dbg_in_meas,
	output logic                      dbg_in_wait,
	// and the cycles that actually emit a pixel
	output logic                      dbg_emitting,
	input  logic [15:0]               rom_data,
	input  logic                      rom_data_valid,

	// decoded pixel stream (destination coords + raw palette index)
	output logic               pix_valid,
	// Placement witness: pulsed for one cycle at the end of ST_SETUP_POSITION
	// with the start column and the scaled width. With `code` and `scale` from
	// the renderer that is every input of `pix_x = draw_x + xpix`, so a wild x
	// can be traced to its object and decoded offline from the ROM.
	output logic               dbg_setup_valid,
	output logic signed [15:0] dbg_draw_x,
	output logic        [15:0] dbg_scaled_width,
	// The object's first row-header word as received from the ROM path, and
	// the unscaled width the MEASURE pass computed: compared offline against
	// the ROM, they show whether the words the blit was given were wrong.
	output logic        [15:0] dbg_first_hdr,
	output logic        [15:0] dbg_meas_width,
	output logic signed [15:0] pix_x,
	output logic signed [15:0] pix_y,
	output logic [5:0]         pix_value,

	output logic        busy,
	output logic        done,               // 1-cycle pulse, object fully processed
	output logic        hflip_unsupported   // always 0 (bench guard, see header)
);

	// byte decode submodule (combinational)
	logic [7:0] dec_byte_in;
	logic [4:0] dec_run_count;
	logic [5:0] dec_pixel_value;
	agt_rle_decode u_decode (
		.byte_in     (dec_byte_in),
		.table_sel   (table_sel),
		.run_count   (dec_run_count),
		.pixel_value (dec_pixel_value)
	);

	// divider submodule
	logic        div_start, div_busy, div_done;
	logic [31:0] div_dividend, div_divisor, div_quotient;
	agt_div32 u_div (
		.clk(clk), .rst_n(rst_n), .start(div_start),
		.dividend(div_dividend), .divisor(div_divisor),
		.busy(div_busy), .done(div_done),
		.quotient(div_quotient), .remainder()
	);

	typedef enum logic [5:0] {
		ST_IDLE,
		ST_CHECK_VALID,
		ST_PC_LOOKUP,          // one cycle for the cache read to land
		// measure phase (mirrors prescan_rle's row-scan loop)
		ST_MEAS_ROW_HDR, ST_MEAS_ROW_WAIT, ST_MEAS_ROW_CHECK,
		ST_MEAS_WORD_HDR, ST_MEAS_WORD_WAIT, ST_MEAS_WORD_DECODE,
		// setup phase
		ST_SETUP_MULT,
		ST_SETUP_DIV_X_START, ST_SETUP_DIV_X_WAIT,
		ST_SETUP_DIV_Y_START, ST_SETUP_DIV_Y_WAIT,
		ST_SETUP_POSITION,
		// draw phase
		ST_DRAW_SCANLINE_START,
		ST_SEEK_ROW_HDR, ST_SEEK_ROW_WAIT, ST_SEEK_ROW_ADVANCE,
		ST_ROW_DECODE_HDR, ST_ROW_DECODE_HDR_WAIT, ST_ROW_DECODE_INIT,
		ST_ROW_WORD_FETCH, ST_ROW_WORD_WAIT, ST_ROW_SIB_WAIT,
		ST_ROW_DECODE_LOW, ST_ROW_EMIT_LOW,
		ST_ROW_DECODE_HIGH, ST_ROW_EMIT_HIGH,
		ST_ROW_WORD_DONE,
		ST_SCANLINE_DONE,
		ST_DONE, ST_SKIP_HFLIP
	} state_t;

	state_t state;

	// object-wide registers
	logic [ROM_ADDR_WIDTH-1:0] mptr;           // measure-phase read pointer
	logic [15:0] width, height;                // measured, unscaled

	// Prescan cache. Direct-mapped, 512 entries keyed on code[8:0], tag
	// code[14:9]. One packed word, so it infers as a single simple-dual-port
	// M10K (2 blocks, 512x39 in 256x40 mode) rather than several sliced arrays.
	//
	//   entry = {valid, tag[5:0], height[15:0], width[15:0]}   = 39 bits
	//
	// Codes collide over 512 slots, but a frame draws only tens of distinct
	// codes, and a collision costs a MEASURE pass, never a wrong answer, since
	// the tag is checked. The RLE region is ROM, written once at download, so
	// an entry cannot go stale. The `initial` loop clears the valid bits
	// (Quartus emits a MIF from it, as agt_demo_memories does for its mirrors).
	localparam int PC_IDX_BITS = 9;
	localparam int PC_ENTRIES  = 1 << PC_IDX_BITS;
	(* ramstyle = "M10K" *) logic [38:0] pcache [0:PC_ENTRIES-1];
	integer pci;
	initial for (pci = 0; pci < PC_ENTRIES; pci = pci + 1) pcache[pci] = 39'd0;

	logic [PC_IDX_BITS-1:0] pc_idx;
	logic [5:0]             pc_tag;
	assign pc_idx = code[PC_IDX_BITS-1:0];
	assign pc_tag = code[14:PC_IDX_BITS];

	// prescan cache hit/miss counters (saturating)
	logic [15:0] pc_hits, pc_misses;
	assign dbg_pc_hits   = pc_hits;
	assign dbg_pc_misses = pc_misses;
	logic [38:0] pc_rd;                 // registered read, one cycle after idx
	logic        pc_hit;
	assign pc_hit = pc_rd[38] && (pc_rd[37:32] == pc_tag_q);
	logic [5:0]  pc_tag_q;              // the tag that pc_rd was fetched for

	always_ff @(posedge clk) begin
		pc_rd    <= pcache[pc_idx];
		pc_tag_q <= pc_tag;
		if (pc_wr) pcache[pc_wr_idx] <= pc_wr_data;
	end
	logic                   pc_wr;
	logic [PC_IDX_BITS-1:0] pc_wr_idx;
	logic [38:0]            pc_wr_data;
	logic        first_hdr_seen;               // dbg_first_hdr captured
	logic [15:0] tempwidth;                    // running width for the current row
	logic [15:0] rows_counted;

	logic [15:0] entry_count;                  // current row's word count (post-inversion)
	logic [15:0] raw_word;                     // last word fetched from ROM

	logic [31:0] xscale16, yscale16;           // scale << 4, widened
	logic [31:0] scaled_width, scaled_height;  // rounded, min-1 clamped
	logic [31:0] dx, dy;                       // source-space step per dest column/row

	logic signed [31:0] draw_x, draw_y;        // final screen placement (top-left)

	logic [ROM_ADDR_WIDTH-1:0] row_start;       // persistent row pointer (draw phase)
	logic [15:0] current_row, target_row;
	logic [31:0] sourcey;
	logic [15:0] dest_y;

	// One-word prefetch with a single consumer: `rom_data_valid` is read only
	// by the capture below, and every draw state reads pf_have/pf_data and
	// nothing else, so a response can never be taken by one path while another
	// waits for it (which hangs the draw loop).
	//
	// agt_tile_sdram's single-outstanding rule (one `rd_pending` latch, no
	// queue) holds by construction: a read is issued only when pf_pending is
	// clear, and only the capture clears it.
	logic [15:0]               pf_data;
	logic                      pf_have;      // pf_data holds an unconsumed word
	logic                      pf_pending;   // a read is in flight

	// Pair prefetch. A 32-bit last-pair cache in front of this port at the top
	// level (`rle_wpair = rle_rom_waddr[23:1]`) answers a request for the
	// sibling of the word it holds in one cycle with no SDRAM transaction, so
	// only every other word is a round trip. A one-word prefetch overlaps that
	// miss with only one word's processing.
	//
	// So on receiving the even word of a pair, fetch its sibling at once (a
	// hit), stash it, and issue the next pair's read straight away: the miss
	// then overlaps the processing of both words.
	//
	//   before   A arrives -> req A+1(hit) -> work A -> A+1 -> req A+2(MISS) -> work A+1 -> WAIT
	//   after    A arrives -> req A+1(hit) -> stash -> req A+2(MISS) -> work A -> work A+1 -> WAIT
	//
	// Still single-outstanding: the sibling's response is consumed before the
	// next request is issued, and `rom_data_valid` still has one consumer.
	// A row may start on an odd word; that word falls back to the one-word
	// schedule, and from the next word on the stream stays aligned.
`ifdef RLE_PAIR_PREFETCH_OFF
	localparam bit PAIR_PREFETCH = 1'b0;   // bench A/B only: one-word prefetch
`else
	localparam bit PAIR_PREFETCH = 1'b1;   // shipped config
`endif
	logic [15:0]               sib_data;     // the pair sibling, fetched early
	logic                      sib_valid;    // sib_data holds an unconsumed word

	logic [ROM_ADDR_WIDTH-1:0] rowbase;         // this scanline's row pointer (local copy)
	logic [15:0] remaining;                     // words left to decode in this row
	logic [31:0] sourcex, rle_end;
	logic [15:0] xpix;

	assign dbg_in_meas  = (state == ST_MEAS_ROW_HDR)
					   || (state == ST_MEAS_ROW_WAIT)
					   || (state == ST_MEAS_ROW_CHECK)
					   || (state == ST_MEAS_WORD_HDR)
					   || (state == ST_MEAS_WORD_WAIT)
					   || (state == ST_MEAS_WORD_DECODE);
	assign dbg_in_wait  = (state == ST_MEAS_ROW_WAIT)
					   || (state == ST_MEAS_WORD_WAIT)
					   || (state == ST_SEEK_ROW_WAIT)
					   || (state == ST_ROW_DECODE_HDR_WAIT)
					   || (state == ST_ROW_WORD_WAIT)
					   // SIB_WAIT counts as wait: its cycles
					   // replace WORD_WAIT cycles, so leaving it
					   // out would hide wait, not remove it.
					   || (state == ST_ROW_SIB_WAIT);
	assign dbg_emitting = pix_valid;

	// ROM requests pulse rom_rd for one cycle, one outstanding at a time;
	// *_WAIT states hold until rom_data_valid (no assumed fixed latency, the
	// same contract as agt_rle_objtable.sv).

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pc_hits <= 16'd0; pc_misses <= 16'd0;
			state <= ST_IDLE;
			busy <= 1'b0;
			done <= 1'b0;
			hflip_unsupported <= 1'b0;
			pix_valid <= 1'b0;
			rom_rd <= 1'b0;
			div_start <= 1'b0;
			dbg_setup_valid <= 1'b0;
			dbg_draw_x <= 16'sd0; dbg_scaled_width <= 16'd0;
			dbg_first_hdr <= 16'd0; dbg_meas_width <= 16'd0; first_hdr_seen <= 1'b0;
			pc_wr <= 1'b0;
			pf_have <= 1'b0; pf_pending <= 1'b0;
			sib_valid <= 1'b0;
		end else begin
			done <= 1'b0;
			pc_wr <= 1'b0;      // one-shot, re-asserted only on a fill

			// the only consumer of rom_data_valid in the draw loop
			if (pf_pending && rom_data_valid) begin
				pf_data    <= rom_data;
				pf_have    <= 1'b1;
				pf_pending <= 1'b0;
			end
			hflip_unsupported <= 1'b0;
			pix_valid <= 1'b0;
			rom_rd <= 1'b0;
			div_start <= 1'b0;
			dbg_setup_valid <= 1'b0;

			unique case (state)

				ST_IDLE: begin
					if (start) begin
						busy <= 1'b1;
						state <= ST_CHECK_VALID;
					end
				end

				ST_CHECK_VALID: begin
					if (!hdr_valid) begin
						busy <= 1'b0;
						done <= 1'b1;
						state <= ST_IDLE;
					end else begin
						mptr <= data_offset;
						width <= 16'd0;
						rows_counted <= 16'd0;
						first_hdr_seen <= 1'b0;
						// pc_idx/pc_tag are combinational on `code`,
						// stable since `start`; the registered read
						// issued this cycle lands next cycle.
						state <= ST_PC_LOOKUP;
					end
				end

				// Cache probe. Hit: take width/height and go straight to the
				// scale math, skipping MEASURE. Miss: measure.
				ST_PC_LOOKUP: begin
					// Saturate, do not wrap: a wrapped hit count
					// going down looks like a cache getting worse.
					if (pc_hit) begin
						if (pc_hits   != 16'hFFFF) pc_hits   <= pc_hits   + 16'd1;
					end else begin
						if (pc_misses != 16'hFFFF) pc_misses <= pc_misses + 16'd1;
					end
					if (pc_hit) begin
						width  <= pc_rd[15:0];
						height <= pc_rd[31:16];
						state  <= ST_SETUP_MULT;
					end else begin
						state  <= ST_MEAS_ROW_HDR;
					end
				end

				// MEASURE: row-scan loop, mirrors prescan_rle
				ST_MEAS_ROW_HDR: begin
					rom_addr <= mptr;
					rom_rd   <= 1'b1;
					state    <= ST_MEAS_ROW_WAIT;
				end
				ST_MEAS_ROW_WAIT: begin
					if (rom_data_valid) begin
						raw_word <= rom_data;
						mptr     <= mptr + 1'b1;
						state    <= ST_MEAS_ROW_CHECK;
					end
				end
				ST_MEAS_ROW_CHECK: begin
					// apply the bit-15 "inverted entries" convention fresh each
					// time (MAME's self-modifying-ROM trick is a software cache
					// optimization with no hardware equivalent)
					logic [15:0] ec;
					ec = raw_word[15] ? (raw_word ^ 16'hFFFF) : raw_word;
					if (!first_hdr_seen) begin
						first_hdr_seen <= 1'b1;
						dbg_first_hdr  <= raw_word;                  // as received, pre-inversion
					end
					if (ec == 16'd0) begin
						height <= rows_counted;
						// Fill the cache on the way out of MEASURE,
						// from the values being committed (`width` is
						// final, `rows_counted` is the height being
						// latched), not from registers that update a
						// cycle later.
						pc_wr      <= 1'b1;
						pc_wr_idx  <= pc_idx;
						pc_wr_data <= {1'b1, pc_tag, rows_counted, width};
						state  <= ST_SETUP_MULT;
					end else begin
						entry_count <= ec;
						tempwidth   <= 16'd0;
						state       <= ST_MEAS_WORD_HDR;
					end
				end
				ST_MEAS_WORD_HDR: begin
					rom_addr <= mptr;
					rom_rd   <= 1'b1;
					state    <= ST_MEAS_WORD_WAIT;
				end
				ST_MEAS_WORD_WAIT: begin
					if (rom_data_valid) begin
						raw_word <= rom_data;
						mptr     <= mptr + 1'b1;
						state    <= ST_MEAS_WORD_DECODE;
					end
				end
				ST_MEAS_WORD_DECODE: begin
					// decode_byte is combinational (u_decode); it is fed the
					// low byte then the high byte on two cycles of this state
					// (meas_phase, see the dec_byte_in mux).
					if (meas_phase == 1'b0) begin
						tempwidth   <= tempwidth + {11'd0, dec_run_count};
						meas_phase  <= 1'b1;
					end else begin
						tempwidth  <= tempwidth + {11'd0, dec_run_count};
						meas_phase <= 1'b0;
						remaining  <= entry_count - 16'd1;
						if (entry_count == 16'd1) begin
							rows_counted <= rows_counted + 16'd1;
							if (tempwidth + {11'd0, dec_run_count} > width)
								width <= tempwidth + {11'd0, dec_run_count};
							state <= ST_MEAS_ROW_HDR;
						end else begin
							entry_count <= entry_count - 16'd1;
							state <= ST_MEAS_WORD_HDR;
						end
					end
				end

				// SETUP: scale math
				ST_SETUP_MULT: begin
					xscale16 <= {12'd0, scale} << 4;
					yscale16 <= {12'd0, scale} << 4;  // single combined scale
					state    <= ST_SETUP_DIV_X_START;
				end
				ST_SETUP_DIV_X_START: begin
					// scaled_width = max(1, (xscale16*width + 0x7fff) >> 16)
					// The rounding constant is added to the full product before
					// truncating. "Truncate, then add 1 if bit 15 was set"
					// differs when the low 16 bits are exactly 0x8000 (the
					// code=15 scale=0x800 vector: 10 instead of 9).
					logic [47:0] prod;
					logic [47:0] rounded;
					logic [31:0] sw;
					prod    = xscale16 * {16'd0, width};
					rounded = prod + 48'h7fff;
					sw      = rounded[47:16];
					scaled_width <= (sw == 32'd0) ? 32'd1 : sw;
					div_dividend <= {16'd0, width} << 16;
					div_divisor  <= (sw == 32'd0) ? 32'd1 : sw;
					div_start    <= 1'b1;
					state        <= ST_SETUP_DIV_X_WAIT;
				end
				ST_SETUP_DIV_X_WAIT: begin
					if (div_done) begin
						dx    <= div_quotient;
						state <= ST_SETUP_DIV_Y_START;
					end
				end
				ST_SETUP_DIV_Y_START: begin
					// scaled_height = max(1, (yscale16*height + 0x7fff) >> 16)
					// -- rounded on the full product, as scaled_width above.
					logic [47:0] prod;
					logic [47:0] rounded;
					logic [31:0] sh;
					prod    = yscale16 * {16'd0, height};
					rounded = prod + 48'h7fff;
					sh      = rounded[47:16];
					scaled_height <= (sh == 32'd0) ? 32'd1 : sh;
					div_dividend  <= {16'd0, height} << 16;
					div_divisor   <= (sh == 32'd0) ? 32'd1 : sh;
					div_start     <= 1'b1;
					state         <= ST_SETUP_DIV_Y_WAIT;
				end
				ST_SETUP_DIV_Y_WAIT: begin
					if (div_done) begin
						dy    <= div_quotient;
						state <= ST_SETUP_POSITION;
					end
				end
				ST_SETUP_POSITION: begin
					// atarirle.cpp draw_rle() (lines 519-550):
					// scaled_xoffs = (raw_scale*xoffs)>>12 == (xscale16*xoffs)>>16
					// exactly, since xscale16 = raw_scale<<4 loses no bits.
					logic signed [47:0] xo, yo;
					logic signed [47:0] wo;
					xo = $signed(xscale16) * $signed({{16{xoffs[15]}}, xoffs});
					yo = $signed(yscale16) * $signed({{16{yoffs[15]}}, yoffs});
					// hflip, atarirle.cpp draw_rle():
					//     if (hflip) scaled_xoffs = ((xscale*info.width)>>12) - scaled_xoffs;
					// The offset is measured from the object's right edge.
					// `width` is the measured width, still valid from MEASURE.
					wo = $signed(xscale16) * $signed({32'd0, width});
					draw_x <= hflip
							? $signed({{16{pos_x[15]}}, pos_x}) - ((wo >>> 16) - (xo >>> 16))
							: $signed({{16{pos_x[15]}}, pos_x}) - (xo >>> 16);
					draw_y <= $signed({{16{pos_y[15]}}, pos_y}) - (yo >>> 16);

					// placement witness, on the cycle it is decided
					dbg_setup_valid  <= 1'b1;
					dbg_draw_x       <= hflip
							? ($signed({{16{pos_x[15]}}, pos_x}) - ((wo >>> 16) - (xo >>> 16)))
							: ($signed({{16{pos_x[15]}}, pos_x}) - (xo >>> 16));
					dbg_scaled_width <= scaled_width[15:0];
					dbg_meas_width   <= width;                       // unscaled

					row_start   <= data_offset;
					current_row <= 16'd0;
					sourcey     <= dy >> 1;
					dest_y      <= 16'd0;
					state       <= ST_DRAW_SCANLINE_START;
				end

				// DRAW: per-scanline row seek + decode
				ST_DRAW_SCANLINE_START: begin
					target_row <= sourcey[31:16];
					if (current_row < sourcey[31:16])
						state <= ST_SEEK_ROW_HDR;
					else begin
						rowbase <= row_start;
						state   <= ST_ROW_DECODE_HDR;
					end
				end
				ST_SEEK_ROW_HDR: begin
					rom_addr <= row_start;
					rom_rd   <= 1'b1;
					state    <= ST_SEEK_ROW_WAIT;
				end
				ST_SEEK_ROW_WAIT: begin
					if (rom_data_valid) begin
						raw_word <= rom_data;
						state    <= ST_SEEK_ROW_ADVANCE;
					end
				end
				ST_SEEK_ROW_ADVANCE: begin
					logic [15:0] ec;
					ec = raw_word[15] ? (raw_word ^ 16'hFFFF) : raw_word;
					row_start   <= row_start + 1'b1 + {{(ROM_ADDR_WIDTH-16){1'b0}}, ec};
					current_row <= current_row + 16'd1;
					if ((current_row + 16'd1) < target_row)
						state <= ST_SEEK_ROW_HDR;
					else begin
						rowbase <= row_start + 1'b1 + {{(ROM_ADDR_WIDTH-16){1'b0}}, ec};
						state   <= ST_ROW_DECODE_HDR;
					end
				end
				ST_ROW_DECODE_HDR: begin
					rom_addr <= rowbase;
					rom_rd   <= 1'b1;
					state    <= ST_ROW_DECODE_HDR_WAIT;
				end
				ST_ROW_DECODE_HDR_WAIT: begin
					if (rom_data_valid) begin
						raw_word <= rom_data;
						state    <= ST_ROW_DECODE_INIT;
					end
				end
				ST_ROW_DECODE_INIT: begin
					logic [15:0] ec;
					ec = raw_word[15] ? (raw_word ^ 16'hFFFF) : raw_word;
					remaining <= ec;
					rowbase   <= rowbase + 1'b1;
					sourcex   <= dx >> 1;
					rle_end   <= 32'd0;
					xpix      <= 16'd0;
					pf_have    <= 1'b0;          // new row: nothing
					pf_pending <= 1'b0;          // carried over
					sib_valid  <= 1'b0;          // nor a stashed sibling
					if (ec == 16'd0) state <= ST_SCANLINE_DONE;
					else             state <= ST_ROW_WORD_FETCH;
				end
				ST_ROW_WORD_FETCH: begin
					rom_addr   <= rowbase;
					rom_rd     <= 1'b1;
					pf_pending <= 1'b1;          // the capture owns it
					state      <= ST_ROW_WORD_WAIT;
				end
				ST_ROW_WORD_WAIT: begin
					// Waits on pf_have, never on rom_data_valid: if the
					// capture took the response in the cycle this state was
					// entered, pf_have is already set and it proceeds at once.
					// The stash is checked first; when it holds this word, the
					// next pair's read is already in flight, so nothing is
					// issued here. The low byte's `rle_end` is advanced here,
					// in the take cycle.
					if (PAIR_PREFETCH && sib_valid) begin
						raw_word  <= sib_data;
						rowbase   <= rowbase + 1'b1;
						sib_valid <= 1'b0;
						rle_end   <= rle_end + ({16'd0, dec_run_count} << 16);
						state     <= ST_ROW_EMIT_LOW;
					end else if (pf_have) begin
						raw_word <= pf_data;
						rowbase  <= rowbase + 1'b1;
						pf_have  <= 1'b0;
						// launch word N+1 while the EMIT states drain word N
						if (remaining > 16'd1) begin
							rom_addr   <= rowbase + 1'b1;
							rom_rd     <= 1'b1;
							pf_pending <= 1'b1;
						end
						// `rowbase` is the address of the word just
						// received. When it is even, `rowbase + 1` is its
						// sibling in the same 32-bit pair and the request
						// above hits the top level's pair cache, so collect
						// it now and issue the next pair's miss from there.
						rle_end <= rle_end + ({16'd0, dec_run_count} << 16);
						if (PAIR_PREFETCH && !rowbase[0] && (remaining > 16'd1))
							state <= ST_ROW_SIB_WAIT;
						else
							state <= ST_ROW_EMIT_LOW;
					end
				end
				// Collect the sibling (a hit, one cycle) and launch the next
				// pair. One state per pair, in place of the ST_ROW_WORD_WAIT
				// cycle the sibling would otherwise cost after this word.
				ST_ROW_SIB_WAIT: begin
					if (pf_have) begin
						sib_data  <= pf_data;
						sib_valid <= 1'b1;
						pf_have   <= 1'b0;
						// `rowbase` already advanced past the word in
						// `raw_word`, so `rowbase + 1` is the next pair.
						if (remaining > 16'd2) begin
							rom_addr   <= rowbase + 1'b1;
							rom_rd     <= 1'b1;
							pf_pending <= 1'b1;
						end
						state <= ST_ROW_EMIT_LOW;
					end
				end
				// Unreachable; kept so the state enum does not shift.
				ST_ROW_DECODE_LOW: begin
					// dec_byte_in is muxed to raw_word[7:0] in this state
					// (see always_comb); dec_run_count/dec_pixel_value are
					// combinationally valid this same cycle.
					rle_end <= rle_end + ({16'd0, dec_run_count} << 16);
					state   <= ST_ROW_EMIT_LOW;
				end
				ST_ROW_EMIT_LOW: begin
					if (skip_ok && sourcex < rle_end && xpix < scaled_width[15:0]) begin
						// whole transparent run in one cycle (1:1 only)
						sourcex <= rle_end + 32'h0000_8000;
						xpix    <= xpix + {8'd0, dec_run_count_latched};
					end else if (sourcex < rle_end && xpix < scaled_width[15:0]) begin
						if (dec_pixel_value_latched != 6'd0) begin
							pix_valid <= 1'b1;
							// hflip: destination starts at the right edge and
							// decrements (*dest--); source stepping is
							// unchanged (sourcex += dx in both).
							pix_x     <= hflip
									   ? draw_x[15:0] + scaled_width[15:0] - 16'd1
													  - $signed({10'd0, xpix})
									   : draw_x[15:0] + $signed({10'd0, xpix});
							pix_y     <= draw_y[15:0] + $signed({10'd0, dest_y});
							pix_value <= dec_pixel_value_latched;
						end
						sourcex <= sourcex + dx;
						xpix    <= xpix + 16'd1;
					end else begin
						// The high byte's run is added here, on the
						// cycle the low run runs out: the mux already
						// presents raw_word[15:8] and the latch fires on
						// this same edge.
						rle_end <= rle_end + ({16'd0, dec_run_count} << 16);
						state   <= ST_ROW_EMIT_HIGH;
					end
				end
				// Unreachable; kept so the state enum does not shift.
				ST_ROW_DECODE_HIGH: begin
					rle_end <= rle_end + ({16'd0, dec_run_count} << 16);
					state   <= ST_ROW_EMIT_HIGH;
				end
				ST_ROW_EMIT_HIGH: begin
					if (skip_ok && sourcex < rle_end && xpix < scaled_width[15:0]) begin
						// whole transparent run in one cycle (1:1 only)
						sourcex <= rle_end + 32'h0000_8000;
						xpix    <= xpix + {8'd0, dec_run_count_latched};
					end else if (sourcex < rle_end && xpix < scaled_width[15:0]) begin
						if (dec_pixel_value_latched != 6'd0) begin
							pix_valid <= 1'b1;
							// hflip: destination starts at the right edge and
							// decrements (*dest--); source stepping is
							// unchanged (sourcex += dx in both).
							pix_x     <= hflip
									   ? draw_x[15:0] + scaled_width[15:0] - 16'd1
													  - $signed({10'd0, xpix})
									   : draw_x[15:0] + $signed({10'd0, xpix});
							pix_y     <= draw_y[15:0] + $signed({10'd0, dest_y});
							pix_value <= dec_pixel_value_latched;
						end
						sourcex <= sourcex + dx;
						xpix    <= xpix + 16'd1;
					end else begin
						// end of word (ST_ROW_WORD_DONE's work, folded
						// into the exit cycle)
					if (remaining == 16'd1) begin
						pf_have    <= 1'b0;      // row over: drop any prefetch
						pf_pending <= 1'b0;      // still in flight
						sib_valid  <= 1'b0;      // and any stash
						state <= ST_SCANLINE_DONE;
					end else begin
						remaining <= remaining - 16'd1;
						// The word may already be here or still in
						// flight; either way go to WAIT, which reads
						// pf_have, so no routing decision is made on a
						// stale flag. If it is here, WAIT costs one
						// cycle instead of a round trip.
						state     <= ST_ROW_WORD_WAIT;
					end

					end
				end
				// Unreachable; kept so the state enum does not shift.
				ST_ROW_WORD_DONE: begin
					if (remaining == 16'd1) begin
						pf_have    <= 1'b0;      // row over: drop any prefetch
						pf_pending <= 1'b0;      // still in flight
						sib_valid  <= 1'b0;      // and any stash
						state <= ST_SCANLINE_DONE;
					end else begin
						remaining <= remaining - 16'd1;
						// The word may already be here or still in
						// flight; either way go to WAIT, which reads
						// pf_have, so no routing decision is made on a
						// stale flag. If it is here, WAIT costs one
						// cycle instead of a round trip.
						state     <= ST_ROW_WORD_WAIT;
					end
				end
				ST_SCANLINE_DONE: begin
					sourcey <= sourcey + dy;
					dest_y  <= dest_y + 16'd1;
					if ((dest_y + 16'd1) >= scaled_height[15:0]) begin
						busy  <= 1'b0;
						done  <= 1'b1;
						state <= ST_IDLE;
					end else
						state <= ST_DRAW_SCANLINE_START;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

	// Terms for the folded states. `emit_run_done` is the exit condition of
	// both emit loops, so the exit cycle can do the next piece of work rather
	// than spend a state on one add. `word_take` is the cycle a word is
	// consumed, from the stash or from the prefetch.
	wire emit_run_done = !(sourcex < rle_end && xpix < scaled_width[15:0]);
	wire word_take     = (state == ST_ROW_WORD_WAIT)
					  && ((PAIR_PREFETCH && sib_valid) || pf_have);

	// byte-decode muxing and 2-phase tracking (shared by measure and draw)
	logic meas_phase;   // 0 = low byte, 1 = high byte (measure loop)
	logic [5:0] dec_pixel_value_latched;

	always_comb begin
		unique case (state)
			ST_MEAS_WORD_DECODE: dec_byte_in = meas_phase ? raw_word[15:8] : raw_word[7:0];
			// The low byte is decoded in the cycle the word is taken.
			// `raw_word` is being assigned that cycle, so the decode has to
			// see the incoming word, not the register.
			ST_ROW_WORD_WAIT:    dec_byte_in = (PAIR_PREFETCH && sib_valid)
											 ? sib_data[7:0] : pf_data[7:0];
			// The high byte is decoded in the EMIT_LOW exit cycle. The mux
			// is held here for the whole loop; only the latch below fires,
			// and only on the exit.
			ST_ROW_EMIT_LOW:     dec_byte_in = raw_word[15:8];
			// unreachable, kept so the enum numbering (and
			// probe_blit_hist's map) does not shift
			ST_ROW_DECODE_LOW:   dec_byte_in = raw_word[7:0];
			ST_ROW_DECODE_HIGH:  dec_byte_in = raw_word[15:8];
			default:             dec_byte_in = 8'd0;
		endcase
	end

	// Latch the decoded value for the multi-cycle emit loop: dec_pixel_value is
	// combinational off dec_byte_in, which is correct only in the decode cycle.
	// The run count too, for the transparent-run skip below.
	logic [7:0] dec_run_count_latched;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			dec_pixel_value_latched <= 6'd0;
			dec_run_count_latched   <= 8'd0;
		// Latched on the cycle the byte is decoded: the word-take cycle for
		// the low byte, the EMIT_LOW exit for the high.
		end else if (word_take || (state == ST_ROW_EMIT_LOW && emit_run_done)) begin
			dec_pixel_value_latched <= dec_pixel_value;
			dec_run_count_latched   <= dec_run_count;
		end
	end

	// A transparent run at 1:1 scale is skipped in one cycle instead of one
	// cycle per pixel; the emit loop otherwise steps every pixel of every run,
	// drawn or not. The scene backgrounds are drawn at 1:1.
	//
	// 1:1 only: a run takes ceil((rle_end - sourcex) / dx) steps, a division
	// in general. At 1:1 it is exactly the run count, and the loop leaves
	// sourcex at exactly rle_end + (dx >> 1), since sourcex starts each row at
	// dx >> 1 and each run advances it by run_count << 16. So the skip matches
	// the stepped result bit for bit at 1:1 and is not taken otherwise.
	//
	// xpix may overshoot scaled_width: stepping stops at xpix == scaled_width
	// mid-run, the skip jumps past it. Either way every later EMIT this row
	// fails `xpix < scaled_width` and draws nothing, and both reset at the next
	// row, so the pixels written are identical. `tb_rle_blit` and
	// `tb_rle_renderer` check exactly that.
	wire skip_ok = (dx == 32'h0001_0000) && (dec_pixel_value_latched == 6'd0);

endmodule
