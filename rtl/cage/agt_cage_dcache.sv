// agt_cage_dcache.sv -- CAGE data cache in front of cageram in SDRAM
//
// 2-way set-associative, write-back, LRU, 4-word lines, SETS sets (256 by
// default: 2,048 words). Sits between agt_c31's memory port and cageram as
// board glue, downstream of what the core issues. The C3x DMA reads through
// the same port, so it goes through this cache too and stays coherent.
//
// tools/cage31.py's DCache is the reference model. It counts rather than
// stores, so tb_cage_dcache checks the hit/miss/write-back sequence against
// the model and the returned words against a reference array.
//
// Address split, with SB = log2(SETS) set bits and TB = 14 - SB tag bits:
//   addr[15:0]      cageram word address, 16K words
//   addr[1:0]       word within the line
//   addr[SB+1:2]    set                 (256: addr[9:2])
//   addr[15:SB+2]   tag                 (256: addr[15:10], 6 bits)
// SETS = 128 builds the 1,024-word cache, for benches that compare shapes.
//
// Every array has one read address and one write address, the shape that
// infers as memory (an M10K has one read port). rd_ix is the single data read
// address, muxed by state. The victim line is copied into wbuf before the
// write-back burst, so the burst never touches the arrays and tolerates any
// m_ack timing. Only `valid` is reset: a line with valid=0 is never matched,
// so its tag and dirty bits are never read. {dirty, tag} lives in reset-free
// arrays, since an array with an asynchronous reset cannot be a RAM.
//
// Data is two arrays, one per way, read in the same cycle and selected after
// the tag compare, so a read hit takes two cycles.
//
// Only the accept edge reads c_addr. After it the tags are read at r_set, so
// the master may change c_addr mid-transaction (e.g. a core held in reset).
`default_nettype none

module agt_cage_dcache #(
	parameter int SETS = 256            // 256: 2,048 words
) (
	input  wire        clk,
	input  wire        rst_n,

	// CPU side. The master holds c_req until it sees c_ack, then drops it or
	// raises it again with a new address on the very next edge.
	input  wire [15:0] c_addr,
	input  wire        c_req,
	input  wire        c_we,
	input  wire [31:0] c_wdata,
	output wire  [31:0] c_rdata,           // a hit answers in S_LOOK itself
	output wire        c_ack,

	// memory side: one word per ack, LINE words per burst
	output logic [15:0] m_addr,
	output logic       m_req,
	output logic       m_we,
	output logic [31:0] m_wdata,
	input  wire  [31:0] m_rdata,
	input  wire        m_ack,

	// counters, for the bench and the overlay
	output logic [31:0] n_hit,
	output logic [31:0] n_miss,
	output logic [31:0] n_wb,

	// An access is in progress (accepted, not yet acked): agt_cage_release's
	// mem_busy. The core's d_req falls when the core is held, while this cache
	// may still be filling.
	output wire         busy
);

	localparam int LINE = 4;
	localparam int SB   = $clog2(SETS);     // set bits: 8 at 256
	localparam int TB   = 14 - SB;          // tag bits: 6 at 256
	localparam int TW   = TB + 1;           // {dirty, tag} -- valid is separate
	localparam int DB   = SB + 2;           // a data array's index bits

	// `valid` lives in its own vectors and {dirty, tag} in reset-free arrays, so
	// the tags can be memory (see header). MLAB, no_rw_check: the tags are read
	// and written at the same set for the whole transaction, so every tag write
	// collides with a read, but no colliding read is consumed. The lookup uses
	// the read from S_IDLE's accept edge (never a tag-write edge) and the
	// write-back address uses reads from S_WBR (a miss, no tag write).
	(* ramstyle = "MLAB, no_rw_check" *) logic [TW-1:0] ctag0 [0:SETS-1];
	(* ramstyle = "MLAB, no_rw_check" *) logic [TW-1:0] ctag1 [0:SETS-1];
	logic [SETS-1:0] valid0, valid1;    // the only reset-cleared tag state
	// Data ways pinned to M10K: left unpinned, synthesis turns them into
	// flip-flops when block memory is near the device limit. No no_rw_check: a
	// read and a write of one word can meet (rd_ix vs wr_ix below), and the
	// RTL's behaviour there is OLD_DATA, which is what the inference gives.
	(* ramstyle = "M10K" *) logic [31:0] cdata0 [0:SETS*LINE-1];
	(* ramstyle = "M10K" *) logic [31:0] cdata1 [0:SETS*LINE-1];

	logic [SETS-1:0] lru;               // which way is the victim, per set

	wire [SB-1:0] a_set  = c_addr[SB+1:2];
	wire [TB-1:0] a_tag  = c_addr[15:SB+2];
	wire [1:0]    a_word = c_addr[1:0];

	logic [TW-1:0] q_tag0, q_tag1;
	logic          q_v0, q_v1;          // read alongside the tag, same cycle
	logic [31:0]   q_dat0, q_dat1;

	logic [SB-1:0] r_set;
	logic [TB-1:0] r_tag;
	logic [1:0]  r_word, r_beat;
	logic        r_we, r_way;
	logic [31:0] r_wdata;
	logic [31:0] wbuf [0:LINE-1];       // the victim line, out of the array

	typedef enum logic [2:0] {
		S_IDLE, S_LOOK, S_WBR, S_WB, S_FILL, S_DONE
	} state_e;
	state_e st;

	assign busy = (st != S_IDLE);

	wire hit0 = q_v0 && (q_tag0[TB-1:0] == r_tag);
	wire hit1 = q_v1 && (q_tag1[TB-1:0] == r_tag);
	wire hit  = hit0 || hit1;
	wire hit_way = hit1;

	// A hit answers in S_LOOK: the ack is combinational and the word is the
	// hitting way's RAM output. A read hit is two cycles (accept, look). A miss
	// acks from ack_q in the cycle after S_DONE, with the word in rdata_q.
	logic        ack_q;
	logic [31:0] rdata_q;
	wire         look_hit = (st == S_LOOK) && hit;
	assign c_ack   = ack_q | look_hit;
	assign c_rdata = look_hit ? (hit_way ? q_dat1 : q_dat0) : rdata_q;

	// Tag read index: a_set in S_IDLE (the speculative lookup, consumed at the
	// accept edge), r_set once a request is taken, so c_addr need only be valid
	// at the accept. Tag read and write addresses stay equal for the whole
	// transaction, and the only read consumed is the accept edge's.
	wire [SB-1:0] t_ix = (st == S_IDLE) ? a_set : r_set;

	wire [TW-1:0] vq      = lru[r_set] ? q_tag1 : q_tag0;
	wire          v_valid = lru[r_set] ? q_v1   : q_v0;
	wire          v_dirty = v_valid && vq[TW-1];

	wire [DB-1:0] dix = {r_set, r_word};

	// the single data read address, muxed by state
	logic [DB-1:0] rd_ix;
	always_comb begin
		unique case (st)
			S_LOOK:  rd_ix = {r_set, 2'd0};          // first victim word
			S_WBR:   rd_ix = {r_set, r_beat + 2'd1}; // the next one
			default: rd_ix = {a_set, a_word};        // speculative lookup
		endcase
	end

	// the single write port per way
	logic          we0, we1;
	logic [DB-1:0] wr_ix;
	logic [31:0] wr_dat;
	always_comb begin
		we0 = 1'b0; we1 = 1'b0;
		wr_ix  = dix;
		wr_dat = r_wdata;
		if (st == S_LOOK && hit && r_we) begin
			we0 = ~hit_way;
			we1 =  hit_way;
		end else if (st == S_FILL && m_ack) begin
			we0 = ~r_way;
			we1 =  r_way;
			wr_ix  = {r_set, r_beat};
			// The requested word goes in as the caller's value, not the fetched one,
			// which serves a missed write inside the fill with no extra state.
			wr_dat = (r_we && r_beat == r_word) ? r_wdata : m_rdata;
		end
	end

	// The single write port per tag array. Both writers target r_set, so there
	// is one write address; the state machine only sets `valid`.
	logic          tw0, tw1;
	logic [TW-1:0] t_wd;
	always_comb begin
		tw0 = 1'b0; tw1 = 1'b0;
		// A write hit only sets the dirty bit; `valid` is already 1 on a hit.
		t_wd = {1'b1, r_tag};
		if (st == S_LOOK && hit && r_we) begin
			tw0 = ~hit_way;
			tw1 =  hit_way;
		end else if (st == S_FILL && m_ack && r_beat == 2'(LINE - 1)) begin
			tw0  = ~r_way;
			tw1  =  r_way;
			t_wd = {r_we, r_tag};   // dirty iff the miss that filled it wrote
		end
	end

	always_ff @(posedge clk) begin
		q_tag0 <= ctag0[t_ix];
		q_tag1 <= ctag1[t_ix];
		q_v0   <= valid0[t_ix];
		q_v1   <= valid1[t_ix];
		if (tw0) ctag0[r_set] <= t_wd;
		if (tw1) ctag1[r_set] <= t_wd;
		q_dat0 <= cdata0[rd_ix];
		q_dat1 <= cdata1[rd_ix];
		if (we0) cdata0[wr_ix] <= wr_dat;
		if (we1) cdata1[wr_ix] <= wr_dat;
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			st     <= S_IDLE;
			ack_q  <= 1'b0;
			rdata_q <= 32'd0;
			m_req  <= 1'b0;
			m_we   <= 1'b0;
			n_hit  <= 32'd0;
			n_miss <= 32'd0;
			n_wb   <= 32'd0;
			lru    <= '0;
			// Only `valid` is cleared: the tag arrays must stay reset-free to be RAM.
			// A line with valid=0 is never matched, so its tag is never read.
			valid0 <= '0;
			valid1 <= '0;
		end else begin
			ack_q <= 1'b0;

			unique case (st)
			// Accept on c_req && !ack_q: in a miss's ack cycle a master that holds
			// c_req until it sees c_ack is still presenting the request just served.
			// After a hit there is no ack cycle, so the next request is taken at once.
			S_IDLE: if (c_req && !ack_q) begin
				r_set   <= a_set;
				r_tag   <= a_tag;
				r_word  <= a_word;
				r_we    <= c_we;
				r_wdata <= c_wdata;
				st      <= S_LOOK;
			end

			// Tags and both ways' data for the requested word are valid now; the write
			// port above has already taken the hit-write case.
			S_LOOK: begin
				if (hit) begin
					n_hit      <= n_hit + 32'd1;
					lru[r_set] <= ~hit_way;
					r_way      <= hit_way;
					if (r_we) begin
						// Data and {dirty, tag} are written by the write ports above; `valid` is
						// already 1 on a hit.
					end
					// the ack and the word are look_hit, above
					st    <= S_IDLE;
				end else begin
					n_miss <= n_miss + 32'd1;
					r_way  <= lru[r_set];
					r_beat <= 2'd0;
					if (v_dirty) begin
						// A dirty victim is written back before the fill, so the miss costs two
						// bursts (counted the same way in the model).
						n_wb <= n_wb + 32'd1;
						st   <= S_WBR;
					end else begin
						m_addr <= {r_tag, r_set, 2'd0};
						m_we   <= 1'b0;
						m_req  <= 1'b1;
						st     <= S_FILL;
					end
				end
			end

			// Read the victim line into wbuf, one word a cycle, before any bus cycle
			// starts. rd_ix is already one ahead.
			S_WBR: begin
				wbuf[r_beat] <= r_way ? q_dat1 : q_dat0;
				if (r_beat == 2'(LINE - 1)) begin
					m_addr  <= {vq[TB-1:0], r_set, 2'd0};
					// The burst starts at word 0, captured three cycles ago (q_dat now holds
					// word 3).
					m_wdata <= wbuf[0];
					m_we    <= 1'b1;
					m_req   <= 1'b1;
					r_beat  <= 2'd0;
					st      <= S_WB;
				end else begin
					r_beat <= r_beat + 2'd1;
				end
			end

			S_WB: if (m_ack) begin
				if (r_beat == 2'(LINE - 1)) begin
					m_addr <= {r_tag, r_set, 2'd0};
					m_we   <= 1'b0;
					m_req  <= 1'b1;
					r_beat <= 2'd0;
					st     <= S_FILL;
				end else begin
					r_beat  <= r_beat + 2'd1;
					m_addr  <= m_addr + 16'd1;
					m_wdata <= wbuf[r_beat + 2'd1];
				end
			end

			S_FILL: if (m_ack) begin
				// The array write is handled by the write port above.
				if (!r_we && r_beat == r_word) rdata_q <= m_rdata;
				if (r_beat == 2'(LINE - 1)) begin
					m_req <= 1'b0;
					// {dirty, tag} is written by the tag write port this cycle; only `valid`
					// is set here.
					if (r_way) valid1[r_set] <= 1'b1;
					else       valid0[r_set] <= 1'b1;
					lru[r_set] <= ~r_way;
					st <= S_DONE;
				end else begin
					r_beat <= r_beat + 2'd1;
					m_addr <= m_addr + 16'd1;
				end
			end

			S_DONE: begin
				ack_q <= 1'b1;
				st    <= S_IDLE;
			end
			endcase
		end
	end

endmodule

`default_nettype wire
