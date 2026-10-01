// agt_rle_objlist.sv -- Atari GT / Primal Rage object-list priority sequencer
//
// Reads the 256-entry x 8-word object list from shared RAM and emits decoded,
// validated objects (code/color/x/y/scale/hflip/vram_target) in exactly the
// order of MAME's sort_and_render() (atarirle.cpp:450-511), for
// agt_rle_objtable.sv -> agt_rle_blit.sv.
//
// Field layout (atarigt.cpp modesc, lines 837-852, decoded with
// sprite_parameter::set() in atarirle.cpp lines 1030-1060):
//   word0[14:0]  = code        (15 bits)
//   word0[15]    = hflip
//   word1[11:4]  = color (raw) (8 bits)
//   word1[11:9]  = priority (raw) (3 bits); overlaps color's top 3 bits.
//                  That is the real layout, and the merge reproduces it.
//   word1[15]    = vram_target (selects MO vs TMO channel)
//   word2[15:6]  = xpos (raw)  (10 bits)
//   word3[15:6]  = ypos (raw)  (10 bits)
//   word4[15:0]  = scale       (full word, unsigned)
//   word6[7:0]   = order       (8 bits)
//   word5 and word7 are unused (all-zero masks) and never read.
//
// Objects are prepended to a bucket per `order`, and buckets are walked for
// order = 1..255, so within a bucket objects come out in descending slot
// order. order == 0 disables the entry. Stage 1 reads word6 of all 256 slots
// and builds the buckets; stage 2 walks each chain and fetches words 0-4.
//
// As sort_and_render(): skip if scale == 0 or code >= object_count; x/y are
// sign-extended from 10 bits and cliprect_left is added to x;
// color = (raw_color << 4) | (raw_priority << 12). MAME's hilite_index debug
// aid is not implemented.

module agt_rle_objlist #(
	parameter int OBJRAM_ADDR_WIDTH = 11   // 256 entries x 8 words = 2048 words
) (
	input  logic        clk,
	input  logic        rst_n,

	input  logic        start,                // 1-cycle pulse: begin a full pass (MOGO)
	input  logic [15:0] object_count,         // valid code range is [0, object_count)
	input  logic signed [15:0] cliprect_left, // 0 for Primal Rage

	// synchronous object-list RAM port
	output logic [OBJRAM_ADDR_WIDTH-1:0] objram_addr,
	output logic                         objram_rd,
	input  logic [15:0]                  objram_data,
	input  logic                         objram_data_valid,

	// Decoded, validated object stream (feeds objtable -> blit). The consumer
	// is much slower than the scan, so after object_valid the sequencer holds
	// until consumer_ready before moving on. Rejected entries need no handoff.
	output logic               object_valid,    // 1-cycle pulse
	input  logic                consumer_ready, // must pulse after object_valid to advance
	output logic [14:0]        code,
	output logic [15:0]        color,          // merged (raw_color<<4)|(raw_priority<<12)
	output logic signed [15:0] x,
	output logic signed [15:0] y,
	output logic [15:0]        scale,
	output logic               hflip,
	output logic                vram_target,

	// Debug taps (free in synthesis if unconnected): the first entry examined
	// this pass and how many the pass rejected; the raw words 0 and 1 of the
	// object being emitted, latched with the emit.
	output logic [15:0] dbg_emit_w0,
	output logic [15:0] dbg_emit_w1,
	output logic [15:0] dbg_w0,
	output logic [15:0] dbg_w4,
	output logic        dbg_valid,
	output logic [7:0]  dbg_rejects,
	// Scan completion: dbg_full_scans counts passes that reached slot 255,
	// dbg_starts passes begun. If they diverge the scan is being cut short, and
	// dbg_last_idx says where it stopped.
	output logic [15:0] dbg_starts,
	output logic [15:0] dbg_full_scans,
	output logic [8:0]  dbg_last_idx,
	// hflip census, per pass: stage 1 reads word0 of every slot on the way past
	// (256 extra reads), before the order != 0 filter and the accept guard, so
	// it shows which entries carry hflip, not only how many accepted objects do.
	output logic [15:0] dbg_hf_live,    // slots (of 256) with word0 bit 15 set
	output logic [15:0] dbg_hf_first,   // index of the first such slot, else 0x1FF
	// word0 of that first slot. A slot still holding the bulk-init pattern has
	// bit 15 set too; its code (>= 12000, rejected by the accept guard) tells
	// it from a real flip.
	output logic [15:0] dbg_hf_w0,
	output logic [15:0] dbg_examined,   // cumulative, saturating: entries whose order was non-zero
	output logic [15:0] dbg_emitted,    // of those, how many passed validation
	// The same two counts per pass; the cumulative pair cannot give a rate.
	output logic [15:0] dbg_examined_pf,
	output logic [15:0] dbg_emitted_pf,
	output logic        busy,
	output logic        done                // 1-cycle pulse: full pass complete
);

	localparam int NUM_SLOTS = 256;

	// order field for all 256 slots, filled in stage 1
	logic [7:0] order_buf [0:NUM_SLOTS-1];

	// Bucketed linked list, as atarirle.cpp does it in O(256 + objects):
	//     sort_entry[objnum].next = list_head[order];   // PREPEND
	//     list_head[order]        = &sort_entry[objnum];
	// list_head[o] holds the most recently inserted slot for order o and
	// next_slot[s] chains to the previously inserted one; built during stage 1.
	//
	// Prepend order matters: inserting slots 0..255 ascending and prepending
	// makes each chain come out in descending slot order (objlist_ref.csv:
	// bucket 255 holds slots {4,6,7} and must emit 7, 6, 4). Reversed,
	// overlapping sprites z-fight the wrong way.
	//
	// head_valid[o] distinguishes "bucket empty" from "bucket holds slot 0".
	//
	// Deliberately not pinned: ST_SCAN_WAIT reads list_head[o]/head_valid[o] and
	// writes the same address in the same cycle, and the chain needs the old
	// head. Quartus refuses an explicit MLAB for that, and `no_rw_check` would
	// give the new value, chaining every bucket to itself: never add it here.
	// Unpinned, Quartus picks MLAB or registers; both are correct.
	// tools/check_ram_pins.py lists them as an exception, and
	// check_ram_summary.py's DEMOTED TO LOGIC is benign for these two.
	logic [7:0] list_head  [0:255];
	logic       head_valid [0:255];
	// MLAB, no_rw_check: both are read combinationally in the walk and written
	// only in ST_SCAN_WAIT, a different state of the same FSM, so a read never
	// meets a write. The pin stops synthesis silently demoting them to logic.
	(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] next_slot  [0:NUM_SLOTS-1];
	(* ramstyle = "MLAB, no_rw_check" *) logic       next_valid [0:NUM_SLOTS-1];
	logic [8:0] init_idx;

	logic [8:0] scan_idx;         // 0-256, for stage 1
	logic [8:0] target_order;     // 1-255, outer loop of stage 2
	logic [8:0] hf_cnt;           // in-progress census
	logic [8:0] hf_first;         // 9'h1FF until a set bit is seen
	logic [15:0] hf_w0;           // word0 of that first slot
	logic [15:0] ex_cnt, em_cnt;  // in-progress per-pass counts
	logic [8:0] slot;             // current slot in the bucket chain

	logic [15:0] w0, w1, w2, w3, w4; // fetched entry words (w5, w7 never read)
	logic        dbg_seen;           // first entry of this pass captured
	logic [2:0]  fetch_step;         // which word of the entry is being fetched

	typedef enum logic [3:0] {
		ST_IDLE,
		ST_CLEAR_HEADS,
		ST_HFCEN_REQ, ST_HFCEN_WAIT,
		ST_SCAN_REQ, ST_SCAN_WAIT,
		ST_PROCESS_CHECK_SLOT, ST_NEXT_IN_BUCKET,
		ST_FETCH_REQ, ST_FETCH_WAIT,
		ST_DECODE,
		ST_WAIT_CONSUMER,
		ST_DONE
	} state_t;
	state_t state;

	wire [OBJRAM_ADDR_WIDTH-1:0] entry_base = {slot[7:0], 3'b000};  // slot * 8

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0;
			done <= 1'b0;
			dbg_w0 <= 16'd0; dbg_w4 <= 16'd0; dbg_valid <= 1'b0;
			dbg_emit_w0 <= 16'd0; dbg_emit_w1 <= 16'd0;
			dbg_rejects <= 8'd0; dbg_seen <= 1'b0;
			dbg_starts <= 16'd0; dbg_full_scans <= 16'd0; dbg_last_idx <= 9'd0;
			dbg_examined <= 16'd0; dbg_emitted <= 16'd0;
			dbg_examined_pf <= 16'd0; dbg_emitted_pf <= 16'd0;
			ex_cnt <= 16'd0; em_cnt <= 16'd0;
			dbg_hf_live <= 16'd0; dbg_hf_first <= 16'h01FF;
			dbg_hf_w0 <= 16'd0;
			hf_cnt <= 9'd0; hf_first <= 9'h1FF; hf_w0 <= 16'd0;
			object_valid <= 1'b0;
			objram_rd <= 1'b0;
		end else begin
			done <= 1'b0;
			object_valid <= 1'b0;
			objram_rd <= 1'b0;

			unique case (state)

				ST_IDLE: begin
					if (start) begin
						// cleared per pass, so each pass reports its own first entry
						dbg_seen    <= 1'b0;
						dbg_rejects <= 8'd0;
						ex_cnt      <= 16'd0;
						em_cnt      <= 16'd0;
						hf_cnt      <= 9'd0;
						hf_first    <= 9'h1FF;
						hf_w0       <= 16'd0;
						// Count starts, and keep how far the previous pass got:
						// if it was cut short by this start, dbg_last_idx says where.
						dbg_starts   <= dbg_starts + 16'd1;
						dbg_last_idx <= scan_idx;
						busy <= 1'b1;
						scan_idx <= 9'd0;
						// 256 cycles to clear the bucket heads before the scan
						// builds them.
						init_idx <= 9'd0;
						state <= ST_CLEAR_HEADS;
					end
				end

				ST_CLEAR_HEADS: begin
					head_valid[init_idx[7:0]] <= 1'b0;
					if (init_idx == 9'd255) state <= ST_HFCEN_REQ;
					else init_idx <= init_idx + 9'd1;
				end

				// Stage 1, per slot: read word0 (hflip census), then word6 (order).
				ST_HFCEN_REQ: begin
					objram_addr <= {scan_idx[7:0], 3'b000};  // slot*8 + 0
					objram_rd   <= 1'b1;
					state       <= ST_HFCEN_WAIT;
				end
				ST_HFCEN_WAIT: begin
					if (objram_data_valid) begin
						if (objram_data[15]) begin
							if (hf_cnt != 9'd511) hf_cnt <= hf_cnt + 9'd1;
							if (hf_first == 9'h1FF) begin
								hf_first <= scan_idx;
								hf_w0    <= objram_data[15:0];
							end
						end
						state <= ST_SCAN_REQ;
					end
				end

				ST_SCAN_REQ: begin
					objram_addr <= {scan_idx[7:0], 3'b110};  // scan_idx*8 + 6
					objram_rd   <= 1'b1;
					state       <= ST_SCAN_WAIT;
				end
				ST_SCAN_WAIT: begin
					if (objram_data_valid) begin
						order_buf[scan_idx[7:0]] <= objram_data[7:0];
						// Prepend this slot onto its order bucket, in ascending
						// scan order, as atarirle.cpp does. Order 0 is disabled
						// and never chained.
						if (objram_data[7:0] != 8'd0) begin
							next_slot [scan_idx[7:0]] <= list_head [objram_data[7:0]];
							next_valid[scan_idx[7:0]] <= head_valid[objram_data[7:0]];
							list_head [objram_data[7:0]] <= scan_idx[7:0];
							head_valid[objram_data[7:0]] <= 1'b1;
						end
						if (scan_idx == 9'd255) begin
							dbg_full_scans <= dbg_full_scans + 16'd1;
							// All 256 word0s are in: slot N's census read precedes
							// its word6 read.
							dbg_hf_live  <= {7'd0, hf_cnt};
							dbg_hf_first <= {7'd0, hf_first};
							dbg_hf_w0    <= hf_w0;
							target_order <= 9'd1;
							state        <= ST_PROCESS_CHECK_SLOT;
						end else begin
							scan_idx <= scan_idx + 9'd1;
							state    <= ST_HFCEN_REQ;
						end
					end
				end

				// Stage 2: follow list_head[order] down its chain, visiting only the
				// objects that exist, in descending slot order.
				ST_PROCESS_CHECK_SLOT: begin
					if (head_valid[target_order[7:0]]) begin
						slot       <= {1'b0, list_head[target_order[7:0]]};
						fetch_step <= 3'd0;
						state      <= ST_FETCH_REQ;
					end else if (target_order == 9'd255) begin
						busy  <= 1'b0;
						done  <= 1'b1;
						// Latch the per-pass counts. The outer loop has two exits
						// and both must do it, or one publishes a stale figure.
						dbg_examined_pf <= ex_cnt;
						dbg_emitted_pf  <= em_cnt;
						state <= ST_IDLE;
					end else begin
						target_order <= target_order + 9'd1;
						state        <= ST_PROCESS_CHECK_SLOT;
					end
				end

				// After an object is emitted, step to the next link in this
				// bucket; when the chain ends, move to the next order.
				ST_NEXT_IN_BUCKET: begin
					if (next_valid[slot[7:0]]) begin
						slot       <= {1'b0, next_slot[slot[7:0]]};
						fetch_step <= 3'd0;
						state      <= ST_FETCH_REQ;
					end else if (target_order == 9'd255) begin
						busy  <= 1'b0;
						done  <= 1'b1;
						// Latch the per-pass counts (the loop's other exit).
						dbg_examined_pf <= ex_cnt;
						dbg_emitted_pf  <= em_cnt;
						state <= ST_IDLE;
					end else begin
						target_order <= target_order + 9'd1;
						state        <= ST_PROCESS_CHECK_SLOT;
					end
				end

				// Fetch words 0-4 of this slot's entry (word6 was read in stage 1).
				ST_FETCH_REQ: begin
					case (fetch_step)
						3'd0: objram_addr <= entry_base + 11'd0;
						3'd1: objram_addr <= entry_base + 11'd1;
						3'd2: objram_addr <= entry_base + 11'd2;
						3'd3: objram_addr <= entry_base + 11'd3;
						3'd4: objram_addr <= entry_base + 11'd4;
						default: objram_addr <= entry_base;
					endcase
					objram_rd <= 1'b1;
					state     <= ST_FETCH_WAIT;
				end
				ST_FETCH_WAIT: begin
					if (objram_data_valid) begin
						unique case (fetch_step)
							3'd0: w0 <= objram_data;
							3'd1: w1 <= objram_data;
							3'd2: w2 <= objram_data;
							3'd3: w3 <= objram_data;
							3'd4: w4 <= objram_data;
							default: ;
						endcase
						if (fetch_step == 3'd4)
							state <= ST_DECODE;
						else begin
							fetch_step <= fetch_step + 3'd1;
							state      <= ST_FETCH_REQ;
						end
					end
				end

				ST_DECODE: begin
					logic [14:0] c_code;
					logic [15:0] c_scale;
					logic [7:0]  raw_color;
					logic [2:0]  raw_priority;
					logic [9:0]  raw_x, raw_y;
					logic signed [15:0] sx, sy;
					logic was_valid;

					c_code       = w0[14:0];
					c_scale      = w4;
					raw_color    = w1[11:4];
					raw_priority = w1[11:9];
					raw_x        = w2[15:6];
					raw_y        = w3[15:6];
					was_valid    = (c_scale != 16'd0) && ({1'b0, c_code} < object_count);

					// Debug: the first entry examined each pass, raw word0 and
					// word4 plus the verdict.
					if (!dbg_seen) begin
						dbg_w0    <= w0;
						dbg_w4    <= w4;
						dbg_valid <= was_valid;
						dbg_seen  <= 1'b1;
					end
					if (!was_valid) dbg_rejects <= dbg_rejects + 8'd1;

					// Cumulative and saturating: a saturated counter is
					// obviously pinned, a wrapped one silently plausible.
					if (dbg_examined != 16'hFFFF)
						dbg_examined <= dbg_examined + 16'd1;
					if (was_valid && dbg_emitted != 16'hFFFF)
						dbg_emitted <= dbg_emitted + 16'd1;
					// per pass, also saturating
					if (ex_cnt != 16'hFFFF) ex_cnt <= ex_cnt + 16'd1;
					if (was_valid && em_cnt != 16'hFFFF)
						em_cnt <= em_cnt + 16'd1;

					if (was_valid) begin
						sx = raw_x[9] ? $signed({6'b111111, raw_x}) : $signed({6'b000000, raw_x});
						sy = raw_y[9] ? $signed({6'b111111, raw_y}) : $signed({6'b000000, raw_y});

						code         <= c_code;
						color        <= ({8'd0, raw_color} << 4) | ({13'd0, raw_priority} << 12);
						x            <= sx + cliprect_left;
						y            <= sy;
						scale        <= c_scale;
						hflip        <= w0[15];
						dbg_emit_w0  <= w0;
						dbg_emit_w1  <= w1;
						vram_target  <= w1[15];
						object_valid <= 1'b1;
						state        <= ST_WAIT_CONSUMER;
					end else begin
						// Rejected (scale == 0 or code >= object_count): step
						// to the next link in this bucket.
						state <= ST_NEXT_IN_BUCKET;
					end
				end

				// Hold until the objtable/blit pipeline has taken the object just
				// emitted (object_valid has already dropped by now).
				ST_WAIT_CONSUMER: begin
					if (consumer_ready) state <= ST_NEXT_IN_BUCKET;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
