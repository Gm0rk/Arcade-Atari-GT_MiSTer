// agt_mo_vram.sv -- motion-object frame buffers (MO and TMO).
//
// Holds the rendered motion-object bitmaps: MO, which Primal Rage's colormix
// reads, and TMO, T-MEK's layer, modeled for completeness. MO is
// double-buffered on MAME's CONTROL_FRAME convention (atarirle.h): display
// reads buffer `(control_bits & FRAME) >> 2` while rendering writes the
// other, so drawing the next frame never disturbs scanout. TMO is
// single-buffered.
//
// Erase clears only a scanline span, as atarirle.cpp's partial erase does,
// not the whole buffer. All arrays are full-frame block RAM (M10K).

module agt_mo_vram #(
	parameter int SCREEN_WIDTH  = 336,
	parameter int SCREEN_HEIGHT = 240,
	// TARGETS = 2 keeps the TMO buffer (the default, and the shape the benches
	// verify). TARGETS = 1 degenerates it to one word to save block RAM.
	// MAME allocates the second bitmap whenever the object descriptor's vram
	// field has a non-zero mask, and atarigt's does, so whether TARGETS = 1 is
	// safe is a question about the game: writes aimed at the missing buffer are
	// counted in alt_wr_count, and a count that stays zero says it is unused.
	parameter int TARGETS = 2,
	// DOUBLE = 1 keeps two MO buffers (MAME's model, 158 more M10K).
	// DOUBLE = 0 degenerates the second array to one word and routes every
	// frame/erase/read select to buffer 0 (single-buffered), so a fitter
	// refusal is a one-parameter fallback at the top level.
	parameter bit DOUBLE  = 1
) (
	input  logic clk,
	input  logic rst_n,

	// write port: one pixel at a time, from agt_mo_pixel_combine
	input  logic               wr_valid,
	input  logic signed [15:0] wr_x,
	input  logic signed [15:0] wr_y,
	input  logic [15:0]        wr_value,
	input  logic                wr_vram_target,   // 0=MO, 1=TMO

	// writes aimed at the second buffer while TARGETS==1 (saturating)
	output logic [15:0]         alt_wr_count,
	// Dropped-write strobes, countable per frame at the top:
	//   drop_alt   -- in bounds, but aimed at the TMO buffer, which is one
	//                 dummy word at TARGETS=1.
	//   drop_erase -- in bounds, aimed at the MO buffer a sweep owns this cycle.
	// requested (MOWB behind+ahead) = drop_alt + drop_erase + landed.
	output logic                dbg_wr_drop_alt,
	output logic                dbg_wr_drop_erase,

	// frame control (CONTROL_FRAME bit), MAME's model (atarirle.h / atarirle.cpp):
	//   display reads   m_vram[..][ (control_bits & FRAME) >> 2 ]   = frame_select
	//   render writes   m_vram[..][ (~control_bits & FRAME) >> 2 ]  = wr_frame
	//   erase clears    the FRAME buffer as of the bits in force when the
	//                   request was made                            = erase_frame
	// `frame_select` is the live bit and selects the read (front) buffer.
	input  logic frame_select,
	// `wr_frame` is the render target, latched by the caller at the MOGO request
	// (~frame_select at that moment) and held for the whole pass, so a render
	// that outlives the next flip does not follow the bit into the other buffer
	// mid-sprite.
	input  logic wr_frame,
	// `erase_frame` is the buffer a sweep clears, sampled with `start_erase`:
	// the old bits' FRAME for a control-write erase (`oldbits & CONTROL_FRAME`
	// in control_write), the current one for the vblank site. The flip write
	// erases the buffer that was displayed and the render then draws into that
	// same buffer (~new == old), so agt_video's erase/render interlock applies
	// there; otherwise erase and render touch different arrays.
	input  logic erase_frame,

	// erase: clears a scanline span of one MO buffer (and TMO)
	input  logic start_erase,
	// atarirle.cpp never clears the whole buffer. Both erase sites clear only
	// [m_partial_scanline+1 .. some bottom] -- the region already scanned out
	// -- and vblank_callback then resets m_partial_scanline to -1:
	//
	//   control_write():    sety(max(top, partial+1), min(bottom, scanline))
	//   vblank_callback():  sety(max(top, partial+1), bottom)
	//
	// A sweep takes priority over pixel writes to its buffer, so a whole-buffer
	// sweep would destroy sprite pixels drawn meanwhile.
	//
	// erase_to_bottom distinguishes the two sites: the control-write erase
	// stops at the current line, the vblank erase runs to the last line.
	input  logic [8:0] erase_line,        // current raster line
	// erase_to_bottom is the vblank rising edge itself, not "a vblank erase".
	// Pulse it every vblank; it resets the watermark on its own (MAME:
	// `m_partial_scanline = -1`, unconditional). Assert start_erase in the
	// same cycle only when the erase bit is set.
	input  logic       erase_to_bottom,   // 1 = vblank site, 0 = control-write
	// Pulse on every control write, erase bit or not: MAME's control_write ends
	// with `m_partial_scanline = scanline;`, outside the
	// `if ((oldbits & CONTROL_ERASE))` guard, so the watermark always advances.
	input  logic       ctrl_wr,
	// Full-frame erase override: ignore the watermark and clear the whole
	// visible buffer at the vblank site, once per frame. Not MAME-accurate
	// (MAME's erase is a partial span), but a known-good clear: it separates a
	// fault in the span logic from one upstream of the buffer.
	input  logic       erase_full_frame,
	// Port contract: `ctrl_wr` and `start_erase` for the same control write must
	// arrive in the same cycle. The span is computed from the watermark as it
	// stands when `start_erase` is sampled, then the watermark advances to
	// `erase_line`: MAME's order (erase on the old m_partial_scanline, then
	// `m_partial_scanline = scanline`). If ctrl_wr arrives first, every
	// control-write span is degenerate and the buffer is never cleared;
	// `tb_mo_erase +mutant=3` reproduces that skew and must fail.

	output logic erase_busy,
	output logic [15:0] dbg_sweeps,     // sweeps that actually ran
	output logic erase_done,

	// read port: for colormix / display scanout
	input  logic                rd_req,
	input  logic signed [15:0]  rd_x,
	input  logic signed [15:0]  rd_y,
	input  logic                 rd_vram_target,
	output logic [15:0]         rd_value,
	output logic                rd_valid
);

	localparam int PIXELS = SCREEN_WIDTH * SCREEN_HEIGHT;
	localparam int ADDR_W = $clog2(PIXELS);

	// MO buffers: two arrays, each one write port with an erase/pixel
	// write-enable mux and one registered read, the shape that infers as M10K.
	// One array of 2*PIXELS indexed base+addr does not infer as RAM.
	//
	// Double-buffering matters because atarirle.cpp renders mid-frame by design
	// (screen().update_partial before sort_and_render) and the game picks the
	// target with CONTROL_FRAME: single-buffered, a pass that overruns vblank
	// writes into the buffer the raster is reading.
	//
	// ramstyle is an instruction, not a hint: without it Quartus may fall back
	// to registers when it cannot place the array (Error 276003); with it, a
	// design that does not fit fails in the fitter naming the resource.
	localparam int B1_PIXELS = DOUBLE ? PIXELS : 1;
	localparam int B1_AW     = DOUBLE ? ADDR_W : 1;
	(* ramstyle = "M10K" *) logic [15:0] mo_buf0 [0:PIXELS-1];
	(* ramstyle = "M10K" *) logic [15:0] mo_buf1 [0:B1_PIXELS-1];
	// the three selects, forced to buffer 0 when single-buffered
	wire sel_wr    = DOUBLE ? wr_frame     : 1'b0;
	wire sel_erase = DOUBLE ? erase_frame  : 1'b0;
	wire sel_front = DOUBLE ? frame_select : 1'b0;

	// Zero-init the buffers. Block RAM configures to zero on FPGA load, but in
	// simulation unwritten memory reads X. $readmemh from an all-zero file
	// covers both; a for-loop `initial` would exceed Quartus's 5000-iteration
	// cap (Error 10106), since PIXELS is 80,640.
	initial $readmemh("mem/mo_vram_zero.hex", mo_buf0);
	generate
		if (DOUBLE) initial $readmemh("mem/mo_vram_zero.hex", mo_buf1);
		else        initial mo_buf1[0] = 16'd0;
	endgenerate
	generate
		if (TARGETS == 2) initial $readmemh("mem/mo_vram_zero.hex", tmo_buf);
		else              initial tmo_buf[0] = 16'd0;
	endgenerate

	// write side
	wire wr_in_bounds = (wr_x >= 16'sd0) && (wr_x < SCREEN_WIDTH[15:0]) &&
						 (wr_y >= 16'sd0) && (wr_y < SCREEN_HEIGHT[15:0]);
	wire [ADDR_W-1:0] wr_addr = wr_y[15:0] * SCREEN_WIDTH[15:0] + wr_x[15:0];

	// Erase sweep FSM: sweeps a scanline span, not the whole buffer.
	// partial_scanline mirrors MAME's m_partial_scanline: how far down the frame
	// has already been erased. It advances with each control write and resets
	// to -1 (here 9'h1FF, an all-ones sentinel) at the vblank site.
	logic [ADDR_W-1:0] erase_addr;
	logic [ADDR_W-1:0] erase_last;      // inclusive end of the span
	logic erasing;
	logic erase_tgt;                    // which buffer this sweep clears
	// A non-degenerate request for the other buffer while a sweep runs cannot
	// extend that sweep. One is held and started when the sweep ends; dropping
	// it would leave stale rows. With the game's usual pattern this never
	// fires: after the flip's sweep, the later requests of the vblank are
	// degenerate.
	logic pend_valid, pend_tgt;
	logic [ADDR_W-1:0] pend_first, pend_last;
	logic [8:0] partial_scanline;       // 9'h1FF == MAME's -1
	// Sweeps that actually started. A request with a degenerate span does no
	// work, so with the top level's request count the pair reads
	// {requests, sweeps}, sweeps <= requests always.
	logic [15:0] sweeps_started;
	assign dbg_sweeps = sweeps_started;

	// First pixel of the span: (partial_scanline + 1) * WIDTH, the sentinel
	// meaning line 0. Both ends are clamped to the visible rectangle, as MAME:
	//     cliprect.sety(max(cliprect.top(),    m_partial_scanline + 1),
	//                   min(cliprect.bottom(), scanline));
	// MOGO fires mostly in vblank, where `erase_line` is past the last visible
	// line; unclamped, the sweep would run past the end of the array and leave
	// the watermark past the bottom, degenerating every later span.
	wire [8:0] LAST_LINE = SCREEN_HEIGHT[8:0] - 9'd1;
	wire [8:0] raw_top   = erase_full_frame ? 9'd0
						 : (partial_scanline == 9'h1FF) ? 9'd0
														: partial_scanline + 9'd1;
	wire [8:0] raw_bot   = (erase_full_frame || erase_to_bottom) ? LAST_LINE
																 : erase_line;
	wire [8:0] span_top  = raw_top;                                  // max(top,..)
	wire [8:0] span_bot  = (raw_bot > LAST_LINE) ? LAST_LINE : raw_bot;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			erasing <= 1'b0;
			erase_busy <= 1'b0;
			erase_done <= 1'b0;
			erase_addr <= '0;
			erase_last <= '0;
			erase_tgt  <= 1'b0;
			pend_valid <= 1'b0; pend_tgt <= 1'b0;
			pend_first <= '0;   pend_last <= '0;
			partial_scanline <= 9'h1FF;          // MAME's -1
			sweeps_started <= 16'd0;
		end else begin
			erase_done <= 1'b0;
			if (erasing) begin
				if (erase_addr == erase_last) begin
					if (pend_valid) begin
						// run the held request for the other buffer
						pend_valid <= 1'b0;
						erase_tgt  <= pend_tgt;
						erase_addr <= pend_first;
						erase_last <= pend_last;
						if (sweeps_started != 16'hFFFF)
							sweeps_started <= sweeps_started + 16'd1;
					end else begin
						erasing <= 1'b0;
						erase_busy <= 1'b0;
						erase_done <= 1'b1;
					end
				end else begin
					erase_addr <= erase_addr + 1'b1;
				end
				// A request arriving mid-sweep extends the span (never shortens it);
				// dropping it would leave a band MAME clears. Only a request for the same
				// buffer can extend; one for the other buffer is held (one deep, later
				// one wins).
				if (start_erase && span_bot >= span_top &&
					(!erase_full_frame || erase_to_bottom)) begin
					if (sel_erase == erase_tgt) begin
						if (((span_bot + 9'd1) * SCREEN_WIDTH[8:0] - 1'b1) > erase_last)
							erase_last <= (span_bot + 9'd1) * SCREEN_WIDTH[8:0] - 1'b1;
					end else begin
						pend_valid <= 1'b1;
						pend_tgt   <= sel_erase;
						pend_first <= span_top * SCREEN_WIDTH[8:0];
						pend_last  <= (span_bot + 9'd1) * SCREEN_WIDTH[8:0] - 1'b1;
					end
				end
			end else if (start_erase &&
						 (!erase_full_frame || erase_to_bottom)) begin
				// An empty span (span_bot < span_top) clears nothing, like MAME's
				// degenerate rectangle.
				if (span_bot >= span_top) begin
					erasing    <= 1'b1;
					erase_busy <= 1'b1;
					erase_tgt  <= sel_erase;
					if (sweeps_started != 16'hFFFF)
						sweeps_started <= sweeps_started + 16'd1;
					erase_addr <= span_top * SCREEN_WIDTH[8:0];
					erase_last <= (span_bot + 9'd1) * SCREEN_WIDTH[8:0] - 1'b1;
				end
			end

			// The watermark advances on every control write and on the vblank site,
			// whether or not a sweep ran (the last statement of MAME's control_write,
			// outside the erase guard). It is clamped: MAME stores the raw scanline and
			// clamps on use, but this compares directly, so a watermark past the last
			// line would freeze every later span. The vblank site resets it
			// unconditionally: MAME's vblank_callback erases only with CONTROL_ERASE
			// set but always ends with `m_partial_scanline = -1`. A sweep still needs
			// `start_erase`.
			if (start_erase || ctrl_wr || erase_to_bottom)
				partial_scanline <= erase_to_bottom ? 9'h1FF
								  : ((erase_line > LAST_LINE) ? LAST_LINE
															  : erase_line);
		end
	end

	// memory writes: an erase sweep takes priority over pixel writes
	always_ff @(posedge clk) begin
		// Plain conditional writes, which infer a write-enable. Writing the array
		// back into itself on the other branch:
		//     mo_buf0[a] <= (cond) ? 16'd0 : mo_buf0[a];
		// is a same-address read-modify-write that defeats BRAM inference.
		// One block per array, each with its own enable: a sweep on one buffer does
		// not block pixel writes to the other. A pixel write aimed at the buffer
		// being swept is dropped; that is the flip case, and the erase/render
		// interlock in agt_video keeps it from arising.
		if (erasing && erase_tgt == 1'b0) begin
			mo_buf0[erase_addr] <= 16'd0;
		end else if (wr_valid && wr_in_bounds && !wr_vram_target && sel_wr == 1'b0) begin
			mo_buf0[wr_addr] <= wr_value;
		end
	end
	wire [B1_AW-1:0] b1_wr_a = DOUBLE ? wr_addr[B1_AW-1:0]    : {B1_AW{1'b0}};
	wire [B1_AW-1:0] b1_er_a = DOUBLE ? erase_addr[B1_AW-1:0] : {B1_AW{1'b0}};
	always_ff @(posedge clk) begin
		if (DOUBLE && erasing && erase_tgt == 1'b1) begin
			mo_buf1[b1_er_a] <= 16'd0;
		end else if (DOUBLE && wr_valid && wr_in_bounds && !wr_vram_target && sel_wr == 1'b1) begin
			mo_buf1[b1_wr_a] <= wr_value;
		end
	end

	// TMO buffer. Not hidden behind a generate, which would give the two
	// channels different read latencies: when TARGETS == 1 the array
	// degenerates to a single entry with its writes gated off, so it reads
	// zero (transparent) through the same code path and latency.
	localparam int TMO_PIXELS = (TARGETS == 2) ? PIXELS : 1;
	localparam int TMO_AW     = (TARGETS == 2) ? ADDR_W : 1;

	(* ramstyle = "M10K" *) logic [15:0] tmo_buf [0:TMO_PIXELS-1];

	wire [TMO_AW-1:0] tmo_wr_a = (TARGETS == 2) ? wr_addr[TMO_AW-1:0]
												: {TMO_AW{1'b0}};
	wire [TMO_AW-1:0] tmo_rd_a = (TARGETS == 2) ? rd_addr[TMO_AW-1:0]
												: {TMO_AW{1'b0}};
	wire [TMO_AW-1:0] tmo_er_a = (TARGETS == 2) ? erase_addr[TMO_AW-1:0]
												: {TMO_AW{1'b0}};

	// TMO stays single-buffered: it is T-MEK's layer, unused by Primal Rage
	// (which ships TARGETS=1), and a second TMO buffer is another 158 M10K.
	always_ff @(posedge clk) begin
		if (erasing) begin
			tmo_buf[tmo_er_a] <= 16'd0;
		end else if (TARGETS == 2 && wr_valid && wr_in_bounds
					 && wr_vram_target) begin
			tmo_buf[tmo_wr_a] <= wr_value;
		end
	end

	// The same conditions the write blocks above use, exported from here so the
	// top level cannot drift from them.
	assign dbg_wr_drop_alt   = wr_valid && wr_in_bounds && wr_vram_target &&
							   (TARGETS == 1);
	assign dbg_wr_drop_erase = wr_valid && wr_in_bounds && !wr_vram_target &&
							   erasing && (erase_tgt == sel_wr);

	// Counts writes lost at TARGETS==1: shows whether a game needs TMO.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) alt_wr_count <= 16'd0;
		else if (TARGETS == 1 && wr_valid && wr_in_bounds && wr_vram_target
				 && alt_wr_count != 16'hFFFF)
			alt_wr_count <= alt_wr_count + 16'd1;
	end

	// read side
	wire rd_in_bounds = (rd_x >= 16'sd0) && (rd_x < SCREEN_WIDTH[15:0]) &&
						 (rd_y >= 16'sd0) && (rd_y < SCREEN_HEIGHT[15:0]);
	// Reads are bounds-checked like writes: an out-of-range or not-yet-valid
	// scanline would index past the array and return X, which propagates
	// through the mixer and makes the whole frame undefined.
	logic [ADDR_W-1:0] rd_addr;
	always_comb begin
		rd_addr = {ADDR_W{1'b0}};
		if (rd_in_bounds) rd_addr = rd_y[15:0] * SCREEN_WIDTH[15:0] + rd_x[15:0];
	end
	logic rd_req_d;
	logic rd_in_bounds_d;

	// The read path shape is load-bearing. Quartus infers an M10K read port
	// only from a clocked block containing nothing but the array read: no
	// reset, no output mux. Anything else is an asynchronous read
	// (Info 276007, "uninferred due to asynchronous read logic"), and an
	// uninferred buffer is 80,640 x 16 flip-flops (Error 276003).
	//
	// So selection and zeroing sit after the memory, in combinational logic,
	// with their qualifiers delayed to land on the same edge. Read contract:
	//     rd_value(n) = word for the address presented at n-1
	//     rd_valid(n) = answer to the request made at n-2
	// tb_mo_vram and tb_mo_linebuf hold that contract to the cycle.
	logic [15:0] mo_q0, mo_q1, tmo_q;
	wire [B1_AW-1:0] b1_rd_a = DOUBLE ? rd_addr[B1_AW-1:0] : {B1_AW{1'b0}};

	// A read enable, which M10K supports natively (rden) and Quartus still
	// infers, keeps rd_value holding its last word after the request falls
	// rather than dropping to zero; tb_mo_vram checks that hold.
	always_ff @(posedge clk) begin
		if (rd_req_d) begin
			// display reads the front buffer; atarirle.h:
			//   vram(idx) = m_vram[idx][(control_bits & FRAME) >> 2]
			// Both arrays are read every time with the same enable and address, so
			// each keeps its canonical shape; the front selection is made on the
			// registered outputs below, travelling with the data.
			mo_q0 <= mo_buf0[rd_addr];
			mo_q1 <= mo_buf1[b1_rd_a];
			tmo_q <= tmo_buf[tmo_rd_a];
		end
	end

	// qualifiers, delayed to arrive with the memory's output
	logic q_req, q_inb_d, q_inb, q_tgt, q_front;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_req_d       <= 1'b0;
			rd_in_bounds_d <= 1'b0;
			rd_valid       <= 1'b0;
			q_req          <= 1'b0;
			q_inb_d        <= 1'b0;
			q_inb          <= 1'b0;
			q_tgt          <= 1'b0;
			q_front        <= 1'b0;
		end else begin
			rd_req_d       <= rd_req;
			rd_in_bounds_d <= rd_in_bounds;
			rd_valid       <= rd_req_d;
			// captured only when the memory is, so the selection travels
			// with its data and holds with it
			if (rd_req_d) begin
				q_req   <= 1'b1;
				q_inb_d <= rd_in_bounds_d;
				q_inb   <= rd_in_bounds;
				q_tgt   <= rd_vram_target;
				q_front <= sel_front;       // display reads FRAME (atarirle.h vram())
			end
		end
	end

	// if, not a ternary: rd_y comes from the scanline, which is X before the
	// first line, and an X selector makes a ternary X regardless of both arms.
	always_comb begin
		rd_value = 16'd0;
		if (q_req && q_inb_d && q_inb) begin
			if (!q_tgt) begin
				if (q_front) rd_value = mo_q1;
				else         rd_value = mo_q0;
			end else        rd_value = tmo_q;
		end
	end

endmodule
