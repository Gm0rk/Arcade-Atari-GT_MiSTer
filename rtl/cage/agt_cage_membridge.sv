// agt_cage_membridge.sv -- the cache's memory port, from clk_dsp to cageram
// in SDRAM on clk_sys.
//
// `agt_cage_dcache` misses and write-backs leave on `m_*` in clk_dsp; cageram
// is in SDRAM behind `agt_sdram`'s `cage_*` port in clk_sys. The cache cannot
// drive that port itself: it holds `m_req` across a four-word line and moves
// `m_addr` in the ack cycle, and the port needs one low cycle between
// requests. This module speaks both protocols:
//   clk_dsp side, a slave to the cache: take = m_req && !m_ack && idle,
//   right for a master that may re-raise `req` with a new address in the
//   ack cycle (a `served` latch would deadlock on it).
//   clk_sys side, a master of the port (through agt_cage_portmux): hold
//   `p_req` until `p_ack`; the mux supplies the low cycle.
//
// LINE = 0: one word per crossing, both ways (`agt_cage_sbank`'s mode: it
// reads one word at a time from anywhere in 1M words), one word per port
// access (`p_line` low, data in bits [31:0])
//   request  clk_dsp -> clk_sys  {we, word address[AW-1:0], wdata}  49 bits at AW = 16
//   response clk_sys -> clk_dsp  {rdata}                           32 bits
// Both through `agt_cdc_hs`. A write is acked only after the SDRAM has acked
// it, so the master never runs ahead of memory.
//
// LINE = 1: one line per crossing, each way (the data cache)
//   request  {we, line address[AW-3:0], 4 words}   1 + (AW-2) + 128 bits
//   response {was a read, 4 words}                  129 bits
// The clk_sys side makes one port access per line (`p_line` high): the
// controller's burst reads or writes all four words on one row and acks once.
//   Writes are posted: words 0-2 are buffered (`wb`) and acked the cycle
//   after they are taken; word 3 is taken only when the request can be sent,
//   and sends the whole line. So a write-back costs the cache eight clk_dsp
//   cycles and its fill's request follows at once. Requests run strictly in
//   order on clk_sys, so a later read of a line always sees an earlier write
//   of it (the one ordering the cache relies on: evict a line, miss on it
//   again).
//   Reads: a read not in the read buffer sends the line's request and waits.
//   The response crossing's own output register (`r_data`) is the read
//   buffer (nothing else lands there until another request is sent); words
//   1-3 are served from it the cycle after they are asked for. The buffer
//   lives only while `m_req` is held: it is invalidated when `m_req` is low,
//   on any write, and when another line is requested, so a word is never
//   served from a copy older than its fill.
//   At most two requests are outstanding (`d_out`): the write-back and its
//   fill. clk_sys holds one running and one waiting (in the request
//   crossing's output register, `q_data`), and a third cannot arrive: it is
//   sent only after a response, and a response only after its request
//   finished. `n_ovr` counts a request arriving with one already waiting, or
//   a response sent into a busy channel; it must stay 0, and the benches
//   check it.
//   Contract: a master writes whole lines, words 0 to 3 in order, with
//   `m_req` held across them (agt_cage_dcache's write-back). Reads may be any
//   words; a line is fetched for the first, and the rest of that line come
//   from the buffer while `m_req` stays high.
//   `busy` is high while anything is outstanding. With this cache it never
//   decides agt_cage_release's `mem_busy` (a write-back is always followed by
//   a fill that queues behind it), but the release must not rely on that.
//
// `AW` is the word-address width: 16 (default) for cageram's 64K words, 20
// in `agt_cage_sbank` for the sound bank's 1M words. The SDRAM byte address
// is `BASE + word * 4` in either mode.
`default_nettype none

module agt_cage_membridge #(
	parameter logic [25:0] BASE = 26'h2B20000,     // SDR_BASE_CAGERAM
	parameter int          AW   = 16,              // word-address bits
	parameter bit          LINE = 1'b0             // 1: a line per crossing
) (
	// clk_dsp: agt_cage_dcache's miss / write-back port
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	input  wire  [AW-1:0] m_addr,
	input  wire         m_req,
	input  wire         m_we,
	input  wire  [31:0] m_wdata,
	output logic [31:0] m_rdata,
	output logic        m_ack,
	output wire         busy,          // clk_dsp: a request outstanding

	// clk_sys: one master of the cage port
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	output logic [25:0] p_addr,
	output logic        p_we,
	output wire         p_line,        // 1: a four-word line (LINE = 1)
	output logic [127:0] p_wdata,      // a word in [31:0], or a line, word k at [32k +: 32]
	output logic        p_req,
	input  wire         p_ack,
	input  wire  [127:0] p_rdata,

	// counters
	output logic [31:0] n_rd,          // clk_dsp: words read (served to the master)
	output logic [31:0] n_wr,          // clk_dsp: words written
	output logic [31:0] n_rdl,         // clk_dsp: line reads sent (0 in word mode)
	output logic [31:0] n_wrl,         // clk_dsp: line writes sent (0 in word mode)
	output logic [15:0] n_ovr          // clk_sys: must stay 0
);

generate if (!LINE) begin : g_word

	// LINE = 0: a word per crossing
	logic        d_wait;                 // a request is out; no new take
	wire         q_busy;                 // request channel still closing
	wire         r_pulse;
	wire  [31:0] r_data;

	wire take = m_req && !m_ack && !d_wait && !q_busy;

	assign busy   = d_wait;
	assign n_rdl  = 32'd0;
	assign n_wrl  = 32'd0;
	assign p_line = 1'b0;

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			d_wait  <= 1'b0;
			m_ack   <= 1'b0;
			m_rdata <= 32'd0;
			n_rd    <= 32'd0;
			n_wr    <= 32'd0;
		end else begin
			m_ack <= 1'b0;
			if (take) begin
				d_wait <= 1'b1;
				if (m_we) n_wr <= n_wr + 32'd1;
				else      n_rd <= n_rd + 32'd1;
			end
			if (r_pulse) begin
				m_rdata <= r_data;
				m_ack   <= 1'b1;
				d_wait  <= 1'b0;
			end
		end
	end

	wire         q_pulse;
	wire  [AW+32:0] q_data;
	logic        r_send;
	logic [31:0] r_hold;
	wire         r_busy;

	agt_cdc_hs #(.W(AW+33)) u_req (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(take), .s_data({m_we, m_addr, m_wdata}), .s_busy(q_busy),
		.d_clk(clk_sys), .d_rst_n(rst_sys_n),
		.d_pulse(q_pulse), .d_data(q_data)
	);

	agt_cdc_hs #(.W(32)) u_rsp (
		.s_clk(clk_sys), .s_rst_n(rst_sys_n),
		.s_send(r_send), .s_data(r_hold), .s_busy(r_busy),
		.d_clk(clk_dsp), .d_rst_n(rst_dsp_n),
		.d_pulse(r_pulse), .d_data(r_data)
	);

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			p_req   <= 1'b0;
			p_we    <= 1'b0;
			p_addr  <= 26'd0;
			p_wdata <= 128'd0;
			r_send  <= 1'b0;
			r_hold  <= 32'd0;
			n_ovr   <= 16'd0;
		end else begin
			r_send <= 1'b0;
			// the send happens in the cycle r_send is high: check busy then
			if (r_send && r_busy && n_ovr != 16'hFFFF) n_ovr <= n_ovr + 16'd1;
			if (q_pulse) begin
				p_we    <= q_data[AW+32];
				p_addr  <= BASE + {{(24-AW){1'b0}}, q_data[AW+31:32], 2'b00};
				p_wdata <= {96'd0, q_data[31:0]};
				p_req   <= 1'b1;
			end else if (p_req && p_ack) begin
				p_req  <= 1'b0;
				r_hold <= p_rdata[31:0];
				r_send <= 1'b1;
			end
		end
	end

end else begin : g_line

	// LINE = 1: a line per crossing
	localparam int LW = AW - 2;                  // line-address bits
	localparam int QW = 1 + LW + 128;            // request: {we, line, 4 words}
	localparam int RW = 1 + 128;                 // response: {was a read, 4 words}

	// The outputs are driven here by `assign` from these, so each has one
	// procedural driver in the module (the word branch's), which is what
	// tools/check_multidriver.py can see without unrolling the generate.
	logic        l_ack;
	logic [31:0] l_rdata, l_nrd, l_nwr;
	logic        l_preq, l_pwe, l_rsend;
	logic [25:0] l_paddr;
	logic [15:0] l_novr;
	assign m_ack   = l_ack;
	assign m_rdata = l_rdata;
	assign n_rd    = l_nrd;
	assign n_wr    = l_nwr;
	assign p_req   = l_preq;
	assign p_we    = l_pwe;
	assign p_addr  = l_paddr;
	assign n_ovr   = l_novr;

	// clk_dsp
	wire           q_busy;                       // request channel still closing
	wire           r_pulse;
	wire  [RW-1:0] r_data;                       // held after the pulse: the read buffer

	logic [95:0]   wb;                           // words 0-2 of the line being written
	logic [1:0]    d_out;                        // requests sent, not yet answered (0-2)
	logic          rd_wait;                      // a line read is out for the master
	logic [1:0]    rd_word;                      // ... and the word it asked for
	logic          rb_v;                         // r_data holds line rb_line
	logic [LW-1:0] rb_line;

	wire [LW-1:0] a_line = m_addr[AW-1:2];
	wire [1:0]    a_word = m_addr[1:0];
	wire          can_send = !q_busy && (d_out != 2'd2);
	wire          rb_hit   = rb_v && (rb_line == a_line);
	wire          open_req = m_req && !l_ack;

	wire t_wr  = open_req &&  m_we && ((a_word != 2'd3) || can_send);
	wire t_rdh = open_req && !m_we && !rd_wait &&  rb_hit;
	wire t_rdm = open_req && !m_we && !rd_wait && !rb_hit && can_send;
	wire t_wrl = t_wr && (a_word == 2'd3);       // the line write goes now
	wire send  = t_wrl || t_rdm;

	wire [QW-1:0] q_word = t_rdm ? {1'b0, a_line, 128'd0}
								 : {1'b1, a_line, m_wdata, wb};

	function automatic [31:0] pick(input [127:0] line, input [1:0] w);
		pick = line[32*w +: 32];
	endfunction

	assign busy = (d_out != 2'd0);

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			l_ack   <= 1'b0;
			l_rdata <= 32'd0;
			wb      <= 96'd0;
			d_out   <= 2'd0;
			rd_wait <= 1'b0;
			rd_word <= 2'd0;
			rb_v    <= 1'b0;
			rb_line <= '0;
			l_nrd   <= 32'd0;
			l_nwr   <= 32'd0;
			n_rdl   <= 32'd0;
			n_wrl   <= 32'd0;
		end else begin
			l_ack <= 1'b0;
			d_out <= d_out + (send ? 2'd1 : 2'd0) - (r_pulse ? 2'd1 : 2'd0);

			// the buffer lives only while the master holds the port
			if (!m_req) rb_v <= 1'b0;

			if (t_wr) begin
				case (a_word)
					2'd0: wb[31:0]  <= m_wdata;
					2'd1: wb[63:32] <= m_wdata;
					2'd2: wb[95:64] <= m_wdata;
					default: ;                   // word 3 goes with the send
				endcase
				l_ack <= 1'b1;
				rb_v  <= 1'b0;
				l_nwr <= l_nwr + 32'd1;
				if (t_wrl) n_wrl <= n_wrl + 32'd1;
			end

			if (t_rdh) begin
				l_rdata <= pick(r_data[127:0], a_word);
				l_ack   <= 1'b1;
				l_nrd   <= l_nrd + 32'd1;
			end

			if (t_rdm) begin
				rd_wait <= 1'b1;
				rd_word <= a_word;
				rb_line <= a_line;
				rb_v    <= 1'b0;                 // r_data is about to be replaced
				n_rdl   <= n_rdl + 32'd1;
			end

			// Responses arrive in request order; only a read's is delivered.
			// A write's replaces r_data, so it ends the buffer too (with
			// this cache none arrives while the buffer is live).
			if (r_pulse && !r_data[128]) rb_v <= 1'b0;
			if (r_pulse && r_data[128]) begin
				l_rdata <= pick(r_data[127:0], rd_word);
				l_ack   <= 1'b1;
				rd_wait <= 1'b0;
				rb_v    <= 1'b1;
				l_nrd   <= l_nrd + 32'd1;
			end
		end
	end

	// crossings
	wire           q_pulse;
	wire  [QW-1:0] q_data;                       // held after the pulse: the waiting request
	wire           r_busy;
	logic [127:0]  e_buf;                        // the running line: data out, then data in
	logic          e_rd;

	agt_cdc_hs #(.W(QW)) u_req (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(send), .s_data(q_word), .s_busy(q_busy),
		.d_clk(clk_sys), .d_rst_n(rst_sys_n),
		.d_pulse(q_pulse), .d_data(q_data)
	);

	agt_cdc_hs #(.W(RW)) u_rsp (
		.s_clk(clk_sys), .s_rst_n(rst_sys_n),
		.s_send(l_rsend), .s_data({e_rd, e_buf}), .s_busy(r_busy),
		.d_clk(clk_dsp), .d_rst_n(rst_dsp_n),
		.d_pulse(r_pulse), .d_data(r_data)
	);

	// clk_sys
	// e_act: a line is running (its port access, then its response).
	// e_fin: the access acked, the response owed.
	// pend:  a request is waiting in q_data.
	// e_buf holds the line to write, then the line read: word k at [32k +: 32].
	logic       e_act, e_fin, pend;

	wire start = !e_act && (pend || q_pulse);
	// the line's address, from the waiting request
	wire [25:0] q_addr = BASE + {{(24-AW){1'b0}}, q_data[QW-2 -: LW], 2'b00, 2'b00};

	assign p_wdata = e_buf;
	assign p_line  = 1'b1;

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			e_act   <= 1'b0;
			e_fin   <= 1'b0;
			e_rd    <= 1'b0;
			e_buf   <= 128'd0;
			pend    <= 1'b0;
			l_preq  <= 1'b0;
			l_pwe   <= 1'b0;
			l_paddr <= 26'd0;
			l_rsend <= 1'b0;
			l_novr  <= 16'd0;
		end else begin
			l_rsend <= 1'b0;
			if (l_rsend && r_busy && l_novr != 16'hFFFF) l_novr <= l_novr + 16'd1;

			// A request arrives: start it, or leave it waiting in q_data.
			// One arriving while another waits has just overwritten it: a
			// third outstanding request, which `d_out` rules out.
			if (q_pulse && pend && l_novr != 16'hFFFF) l_novr <= l_novr + 16'd1;
			if (q_pulse && !start) pend <= 1'b1;

			if (start) begin
				pend    <= 1'b0;
				e_act   <= 1'b1;
				e_fin   <= 1'b0;
				e_rd    <= !q_data[QW-1];
				e_buf   <= q_data[127:0];
				l_pwe   <= q_data[QW-1];
				l_paddr <= q_addr;
				l_preq  <= 1'b1;
			end else if (e_act && !e_fin) begin
				if (l_preq && p_ack) begin
					if (e_rd) e_buf <= p_rdata;
					l_preq <= 1'b0;
					e_fin  <= 1'b1;
				end
			end else if (e_fin && !r_busy) begin
				// the response carries e_buf as it is now; the next request
				// may start on the next edge
				l_rsend <= 1'b1;
				e_act   <= 1'b0;
				e_fin   <= 1'b0;
			end
		end
	end

end endgenerate

endmodule

`default_nettype wire
