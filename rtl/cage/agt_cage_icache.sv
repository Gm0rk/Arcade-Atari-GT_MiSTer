// agt_cage_icache.sv -- instruction cache beside agt_c31.
//
// A fetch through agt_cage_bus, the data cache, the clock crossing and SDRAM
// costs several clk_dsp cycles, far over the chip's ~2.65 per instruction.
// The program never writes its code, so fetches are served from this copy
// and the port is left for data.
//
// 1,024 words, direct mapped by the word address's low ten bits, one word a
// line, in two banks by address bit 0 (even/odd, 512 entries each). An entry
// is {valid, tag = addr[15:10], word}: 39 bits, two M10K per bank in x20
// mode.
//
// Two words a lookup: the core presents `la` every cycle, and one cycle
// later the cache answers for `la` (hit0/word0) and `la + 1` (hit1/word1).
// Consecutive words are in different banks, so this is one read of each.
// The second word lets the core take the next instruction in the cycle the
// current one retires: the PC is then one past the word looked up.
//
// agt_c31 (IC_EN = 1) takes word0 when the address looked up is its PC and
// word1 when that address plus one is; otherwise it fetches through the
// port. A wrong guess costs only the port path; a stale lookup is never used.
//
// Filling, by watching the core's memory port: a fetch the port answers
// (`p_ifetch`, a read, below 0x10000: cageram) is written into its bank in
// the ack cycle. A write accepted at a cageram address clears that entry, so
// the cache misses rather than serve an old word. DMA reads are not fetches
// and fill nothing.
//
// `flush` (agt_cage's `mem_rst_n` low: a download is rewriting cageram) and
// reset start a sweep clearing both banks, one entry of each per cycle (512
// cycles); it runs to the end even if `flush` falls first, with hit0/hit1
// held low throughout. Restarting the core alone does not flush: cageram's
// code is unchanged.
//
// Each bank has one read address, one write address and one write
// statement. A fill or clear and a lookup of the same entry in one cycle
// read whatever the M10K gives (`no_rw_check`; the Fitter reports the mixed-
// port read-during-write mode as "Don't care"); the simulator gives the old
// entry. This does happen: from a repeat block's last step the lookup is RS,
// and a store there into cageram at an address with RS's low ten bits clears
// RS's entry in that same cycle. So a read that met a write is refused:
// `col_e`/`col_o` register the collision and that half is not a hit; the
// core takes the port path instead. `tb/rdw_probe_icache.sv` counts these
// reads and, with `+poison`, turns each into a wrong hit (the worst case).
`default_nettype none

module agt_cage_icache (
	input  wire         clk,
	input  wire         rst_n,
	input  wire         flush,          // cageram is being rewritten

	// the core's lookup
	input  wire  [23:0] la,             // looked up every cycle
	output wire         hit0,           // for `la` as it was last cycle
	output wire  [31:0] word0,
	output wire         hit1,           // ...and for the word after it
	output wire  [31:0] word1,

	// the core's memory port, watched
	input  wire         p_req,
	input  wire         p_ack,
	input  wire         p_we,
	input  wire         p_ifetch,
	input  wire  [23:0] p_addr,
	input  wire  [31:0] p_rdata,

	// for the benches
	output logic [31:0] n_fill,
	output logic [31:0] n_inval,
	output logic        flushing
);

	(* ramstyle = "M10K, no_rw_check" *) logic [38:0] ic_even [0:511];
	(* ramstyle = "M10K, no_rw_check" *) logic [38:0] ic_odd  [0:511];

	logic [38:0] q_e, q_o;
	logic [23:0] qa;                    // the address the answer is for
	logic [8:0]  fcnt;

	// the two words: `la` and `la + 1`, one in each bank
	wire  [23:0] la1   = la + 24'd1;
	wire  [8:0]  ix_e  = la[0] ? la1[9:1] : la[9:1];
	wire  [8:0]  ix_o  = la[9:1];

	wire in_ram = (p_addr[23:16] == 8'd0);
	wire fill   = p_req && p_ack && p_ifetch && !p_we && in_ram && !flushing;
	wire inval  = p_req && p_ack && p_we && in_ram && !flushing;

	// the writers, muxed: the sweep (both banks), a fill or a clear (one)
	wire        w_any = flushing || fill || inval;
	wire [8:0]  w_ix  = flushing ? fcnt : p_addr[9:1];
	wire [38:0] w_d   = fill ? {1'b1, p_addr[15:10], p_rdata} : 39'd0;
	wire        we_e  = w_any && (flushing || !p_addr[0]);
	wire        we_o  = w_any && (flushing ||  p_addr[0]);

	always_ff @(posedge clk) begin
		if (we_e) ic_even[w_ix] <= w_d;
		q_e <= ic_even[ix_e];
	end
	always_ff @(posedge clk) begin
		if (we_o) ic_odd[w_ix] <= w_d;
		q_o <= ic_odd[ix_o];
	end
	always_ff @(posedge clk) qa <= la;

	// Did this edge's read of a bank meet a write of the same entry? Compared
	// before the register, so the hit path gains one AND input.
	logic col_e, col_o;
	always_ff @(posedge clk) begin
		col_e <= we_e && (w_ix == ix_e);
		col_o <= we_o && (w_ix == ix_o);
	end

	// the answer, for `qa` and `qa + 1`: bank by the address's bit 0
	wire  [23:0] qa1 = qa + 24'd1;
	wire  [38:0] e0  = qa[0] ? q_o : q_e;           // the entry for qa
	wire  [38:0] e1  = qa[0] ? q_e : q_o;           // ...for qa + 1
	wire         c0  = qa[0] ? col_o : col_e;       // its read met a write
	wire         c1  = qa[0] ? col_e : col_o;
	assign hit0  = e0[38] && (e0[37:32] == qa[15:10])  && (qa[23:16]  == 8'd0) && !flushing && !c0;
	assign hit1  = e1[38] && (e1[37:32] == qa1[15:10]) && (qa1[23:16] == 8'd0) && !flushing && !c1;
	assign word0 = e0[31:0];
	assign word1 = e1[31:0];

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			flushing <= 1'b1;           // power-on contents are not trusted
			fcnt     <= 9'd0;
			n_fill   <= 32'd0;
			n_inval  <= 32'd0;
		end else begin
			if (flush && !flushing) begin
				flushing <= 1'b1;
				fcnt     <= 9'd0;
			end else if (flushing) begin
				fcnt <= fcnt + 9'd1;
				if (fcnt == 9'd511 && !flush) flushing <= 1'b0;
			end
			if (fill)  n_fill  <= n_fill + 32'd1;
			if (inval) n_inval <= n_inval + 32'd1;
		end
	end

endmodule

`default_nettype wire
