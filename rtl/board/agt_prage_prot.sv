// agt_prage_prot.sv -- Primal Rage protection: Atari 136094-0004A XGA
//
// Transcribed from MAME's atarixga.cpp (atari_136094_0004a_device). The XGA
// is a Fibonacci LFSR cipher: the 16-bit ciphertext is loaded into the
// register and clocked n times, where n is looked up from the key byte
// selected by a bit permutation of the query index, and the feedback taps
// come from a per-character word the game writes before each query.
//
// Addresses are full 24-bit CPU addresses (MAME's offsets plus the 0xD80000
// base of the colorram window the chip is overlaid on):
//
//   DC4010  PR_SETKEY    read : begin key upload
//   DC4022  PR_DECIPHER  read : begin a query
//   DC4700  PR_STATUS    read : bit 15 = result ready
//   DCC7C0  PR_DONE0     read : end key upload, checksum path
//   DCC7C2  PR_RESULT    read : query result
//   DCC7C4  PR_DONE4     read : end key upload, boot path
//   DC7800  PR_DATA      write: key byte / ciphertext, 2048 words
//   DC8700  PR_CHAR0     write: character word, P1 path
//   DE4000  PR_CHAR1     write: character word, P2 path
//   DEC000  PR_CHAR2     write: character word, P2 path
//
// PR_CHAR0 lies inside the PR_DATA window [DC7800, DC8800). MAME's write16
// returns from the data-window branch whenever mode is SETKEY or DECIPHER,
// so DC8700 reaches set_character() only while mode is IDLE. Reproduced
// exactly in the write path.
//
// Deviation from MAME: its decipher() is instantaneous, so PR_STATUS always
// returns 0x8000. Here the LFSR needs up to 249 clocks plus 5 setup cycles,
// so bit 15 is gated on the engine being idle, which is what MAME documents
// the bit as meaning ("result ready") and what the game's spin loop expects
// (worst case ~254 clk). STATUS_ALWAYS_READY = 1 gives MAME's constant
// 0x8000, but the engine then races the CPU and PR_RESULT may be stale.
//
// Unknowns inherited from MAME:
//   * only five character words have known feedback taps; any other word
//     leaves taps unchanged, including the 0x8016 trailer on the P1 path
//   * 146 of 256 key bytes have no known clock count; those decipher with
//     n = 0, which returns the ciphertext unchanged
//
// tb_prage_prot.sv checks it against golden vectors from
// tools/xga_0004a_ref.py, transcribed from the same MAME source.
`default_nettype none

module agt_prage_prot (
	input  wire         clk,
	input  wire         rst_n,

	// one pulse per CPU colorram-window access; both halves in this cycle
	input  wire         acc_valid,
	input  wire         acc_we,
	input  wire  [23:0] h0_addr,     // first half (lower address)
	input  wire  [15:0] h0_wdata,
	input  wire         h1_valid,    // second half present (32-bit access)
	input  wire  [23:0] h1_addr,
	input  wire  [15:0] h1_wdata,

	// registered read substitutions: valid the cycle after acc_valid, held
	// until the next access
	output logic        sub0_valid,
	output logic [15:0] sub0_data,
	output logic        sub1_valid,
	output logic [15:0] sub1_data,

	// debug: 0 = IDLE, 1 = SETKEY, 2 = DECIPHER
	output logic [1:0]  prot_mode_out
);

	// 1: MAME-exact constant 0x8000 on PR_STATUS (see the header first)
	localparam bit STATUS_ALWAYS_READY = 1'b0;

	localparam logic [23:0] PR_SETKEY   = 24'hDC4010;
	localparam logic [23:0] PR_DECIPHER = 24'hDC4022;
	localparam logic [23:0] PR_STATUS   = 24'hDC4700;
	localparam logic [23:0] PR_DONE0    = 24'hDCC7C0;
	localparam logic [23:0] PR_RESULT   = 24'hDCC7C2;
	localparam logic [23:0] PR_DONE4    = 24'hDCC7C4;
	localparam logic [23:0] PR_DATA     = 24'hDC7800;
	localparam logic [23:0] PR_DATA_END = 24'hDC8800;   // exclusive
	localparam logic [23:0] PR_CHAR0    = 24'hDC8700;
	localparam logic [23:0] PR_CHAR1    = 24'hDE4000;
	localparam logic [23:0] PR_CHAR2    = 24'hDEC000;

	localparam logic [1:0] MODE_IDLE     = 2'd0;
	localparam logic [1:0] MODE_SETKEY   = 2'd1;
	localparam logic [1:0] MODE_DECIPHER = 2'd2;

	// chip state (MAME member names on the right)
	logic [1:0]  mode;          // m_mode
	logic [15:0] taps;          // m_taps
	logic [15:0] reply;         // m_reply

	assign prot_mode_out = mode;

	// key RAM: 2048 x 8, split even/odd so both access halves commit
	(* ramstyle = "M10K" *) logic [7:0] keyram_e [0:1023];
	(* ramstyle = "M10K" *) logic [7:0] keyram_o [0:1023];

	// The bank is selected by index bit 0, not by access half: a 16-bit access
	// can carry any index in half 0. In a 32-bit access idx1 == idx0 + 1, so the
	// halves always land in different banks and one write port per bank is
	// enough.
	logic        kwe_e, kwe_o;
	logic [9:0]  kwa_e, kwa_o;
	logic [7:0]  kwd_e, kwd_o;

	logic [9:0]  kr_a;
	logic [7:0]  kr_qe, kr_qo;

	always_ff @(posedge clk) begin
		if (kwe_e) keyram_e[kwa_e] <= kwd_e;
		if (kwe_o) keyram_o[kwa_o] <= kwd_o;
		kr_qe <= keyram_e[kr_a];
		kr_qo <= keyram_o[kr_a];
	end

	// kmap ROM: LFSR clock count per key byte, 0 = unknown
	logic [7:0] kmap [0:255];
	initial begin
		{kmap[0], kmap[1], kmap[2], kmap[3], kmap[4], kmap[5], kmap[6], kmap[7], kmap[8], kmap[9], kmap[10], kmap[11], kmap[12], kmap[13], kmap[14], kmap[15]} = 128'h000000000059177B00004F2700003D00;
		{kmap[16], kmap[17], kmap[18], kmap[19], kmap[20], kmap[21], kmap[22], kmap[23], kmap[24], kmap[25], kmap[26], kmap[27], kmap[28], kmap[29], kmap[30], kmap[31]} = 128'h000000000067006F00655700454B0000;
		{kmap[32], kmap[33], kmap[34], kmap[35], kmap[36], kmap[37], kmap[38], kmap[39], kmap[40], kmap[41], kmap[42], kmap[43], kmap[44], kmap[45], kmap[46], kmap[47]} = 128'h1B1D190000613F005F000000002D0000;
		{kmap[48], kmap[49], kmap[50], kmap[51], kmap[52], kmap[53], kmap[54], kmap[55], kmap[56], kmap[57], kmap[58], kmap[59], kmap[60], kmap[61], kmap[62], kmap[63]} = 128'h1F005B7D0000000000230053156D7900;
		{kmap[64], kmap[65], kmap[66], kmap[67], kmap[68], kmap[69], kmap[70], kmap[71], kmap[72], kmap[73], kmap[74], kmap[75], kmap[76], kmap[77], kmap[78], kmap[79]} = 128'h5D21000000000047006B2B1375330037;
		{kmap[80], kmap[81], kmap[82], kmap[83], kmap[84], kmap[85], kmap[86], kmap[87], kmap[88], kmap[89], kmap[90], kmap[91], kmap[92], kmap[93], kmap[94], kmap[95]} = 128'h0000697125554D007300310000003B00;
		{kmap[96], kmap[97], kmap[98], kmap[99], kmap[100], kmap[101], kmap[102], kmap[103], kmap[104], kmap[105], kmap[106], kmap[107], kmap[108], kmap[109], kmap[110], kmap[111]} = 128'h00000041005100000000000000000077;
		{kmap[112], kmap[113], kmap[114], kmap[115], kmap[116], kmap[117], kmap[118], kmap[119], kmap[120], kmap[121], kmap[122], kmap[123], kmap[124], kmap[125], kmap[126], kmap[127]} = 128'h000000632900112F0043004900003539;
		{kmap[128], kmap[129], kmap[130], kmap[131], kmap[132], kmap[133], kmap[134], kmap[135], kmap[136], kmap[137], kmap[138], kmap[139], kmap[140], kmap[141], kmap[142], kmap[143]} = 128'h0000006400220042006A200000001C00;
		{kmap[144], kmap[145], kmap[146], kmap[147], kmap[148], kmap[149], kmap[150], kmap[151], kmap[152], kmap[153], kmap[154], kmap[155], kmap[156], kmap[157], kmap[158], kmap[159]} = 128'h6600544A006C000058320000502C6000;
		{kmap[160], kmap[161], kmap[162], kmap[163], kmap[164], kmap[165], kmap[166], kmap[167], kmap[168], kmap[169], kmap[170], kmap[171], kmap[172], kmap[173], kmap[174], kmap[175]} = 128'h700000007C4862520026001200004000;
		{kmap[176], kmap[177], kmap[178], kmap[179], kmap[180], kmap[181], kmap[182], kmap[183], kmap[184], kmap[185], kmap[186], kmap[187], kmap[188], kmap[189], kmap[190], kmap[191]} = 128'h00006E0000382E0046007A3600760000;
		{kmap[192], kmap[193], kmap[194], kmap[195], kmap[196], kmap[197], kmap[198], kmap[199], kmap[200], kmap[201], kmap[202], kmap[203], kmap[204], kmap[205], kmap[206], kmap[207]} = 128'h0072000000001E0000005C00005E1A00;
		{kmap[208], kmap[209], kmap[210], kmap[211], kmap[212], kmap[213], kmap[214], kmap[215], kmap[216], kmap[217], kmap[218], kmap[219], kmap[220], kmap[221], kmap[222], kmap[223]} = 128'h00002444281400000074000000000000;
		{kmap[224], kmap[225], kmap[226], kmap[227], kmap[228], kmap[229], kmap[230], kmap[231], kmap[232], kmap[233], kmap[234], kmap[235], kmap[236], kmap[237], kmap[238], kmap[239]} = 128'h685600305A000000004E002A18000000;
		{kmap[240], kmap[241], kmap[242], kmap[243], kmap[244], kmap[245], kmap[246], kmap[247], kmap[248], kmap[249], kmap[250], kmap[251], kmap[252], kmap[253], kmap[254], kmap[255]} = 128'h4C00003A00341078003C16003E000000;
	end

	// key_offset(): bit permutation with inversions. A bijection over 0..2047
	// (tools/xga_0004a_ref.py --selftest), so every key byte is reachable and
	// none alias.
	function automatic logic [10:0] key_offset(input logic [10:0] i);
		key_offset = { i[10], i[9], i[8], ~i[6], i[7], i[1], ~i[0],
					   i[4], ~i[2], ~i[5], i[3] };
	endfunction

	// one Fibonacci step: (x << 1) | parity(x & taps)
	function automatic logic [15:0] lfsr_step(input logic [15:0] x,
											  input logic [15:0] t);
		lfsr_step = { x[14:0], ^(x & t) };
	endfunction

	function automatic logic in_data_window(input logic [23:0] a);
		in_data_window = (a >= PR_DATA) && (a < PR_DATA_END);
	endfunction

	// character word -> feedback taps; only five words are known
	function automatic logic [15:0] char_taps(input logic [15:0] w,
											  input logic [15:0] cur);
		case (w)
			16'h2694: char_taps = 16'hBCC8;   // Sauron, Diablo
			16'h6EE0: char_taps = 16'hAED5;   // Blizzard, Talon
			16'h34F7: char_taps = 16'h9D79;   // Chaos
			16'h32B9: char_taps = 16'hFD10;   // Vertigo
			16'h4D5A: char_taps = 16'h82A3;   // Armadon
			default:  char_taps = cur;        // incl. the 0x8016 P1 trailer
		endcase
	endfunction

	// Decipher engine. Sequential because the LFSR needs up to 125 clocks, plus
	// up to 124 more on the early-state retry path.
	localparam logic [2:0] E_IDLE = 3'd0, E_K1   = 3'd1, E_K2 = 3'd2,
						   E_RUN1 = 3'd3, E_EVAL = 3'd4, E_RUN2 = 3'd5,
						   E_DONE = 3'd6;

	logic [2:0]  e_state;
	logic [15:0] e_x, e_c, e_taps;
	logic [7:0]  e_cnt, e_cnt0;
	logic        e_early;
	logic        e_kbank;        // key_offset bit 0: bank holding the byte
	logic [7:0]  e_k, e_n;

	// Pending query. A later query supersedes an earlier one, as MAME's m_reply
	// does: only the last one before the PR_RESULT read is observable, and
	// PR_STATUS holds the CPU off until then.
	logic        rq_v;
	logic [10:0] rq_idx;
	logic [15:0] rq_c;
	logic [15:0] rq_taps;

	wire busy = rq_v || (e_state != E_IDLE);

	wire [15:0] e_nx = lfsr_step(e_x, e_taps);

	wire [10:0] ko = key_offset(rq_idx);

	// key byte and its clock count
	wire [7:0]  kb = e_kbank ? kr_qo : kr_qe;
	wire [7:0]  kn = kmap[kb];

	// Access path and engine share one always_ff so the working-mode locals walk
	// half 0 then half 1 in the order MAME processes them.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			mode       <= MODE_IDLE;
			taps       <= 16'd0;
			reply      <= 16'd0;
			sub0_valid <= 1'b0; sub0_data <= 16'd0;
			sub1_valid <= 1'b0; sub1_data <= 16'd0;
			kwe_e      <= 1'b0; kwe_o     <= 1'b0;
			kwa_e      <= 10'd0; kwd_e    <= 8'd0;
			kwa_o      <= 10'd0; kwd_o    <= 8'd0;
			kr_a       <= 10'd0;
			rq_v       <= 1'b0; rq_idx    <= 11'd0;
			rq_c       <= 16'd0; rq_taps  <= 16'd0;
			e_state    <= E_IDLE;
			e_x        <= 16'd0; e_c      <= 16'd0; e_taps <= 16'd0;
			e_cnt      <= 8'd0;  e_cnt0   <= 8'd0;
			e_early    <= 1'b0;  e_kbank  <= 1'b0;
			e_k        <= 8'd0;  e_n      <= 8'd0;
		end else begin
			kwe_e <= 1'b0;
			kwe_o <= 1'b0;

			// Decipher engine
			case (e_state)
				E_IDLE: if (rq_v) begin
					kr_a    <= ko[10:1];
					e_kbank <= ko[0];
					e_c     <= rq_c;
					e_taps  <= rq_taps;
					e_early <= 1'b0;
					e_state <= E_K1;
				end

				E_K1: e_state <= E_K2;      // kr_a settled, RAM read in flight

				E_K2: begin
					// key byte -> clock count. rq_v clears here, but the access block
					// runs after this case and re-asserts it if a new query arrived
					// this cycle, so nothing is dropped.
					e_k <= kb;
					e_n <= kn;
					if (e_c == 16'd0) begin
						e_x    <= 16'h0001;
						e_cnt  <= (kn == 8'd0) ? 8'd0 : (kn - 8'd1);
						e_cnt0 <= (kn == 8'd0) ? 8'd0 : (kn - 8'd1);
					end else begin
						e_x    <= e_c;
						e_cnt  <= kn;
						e_cnt0 <= kn;
					end
					rq_v    <= 1'b0;
					e_state <= E_RUN1;
				end

				E_RUN1: if (e_cnt == 8'd0) begin
					e_state <= E_EVAL;
				end else begin
					e_x   <= e_nx;
					e_cnt <= e_cnt - 8'd1;
					if (e_nx == 16'h0001) e_early <= 1'b1;
				end

				E_EVAL: begin
					if (!e_early) begin
						e_state <= E_DONE;
					end else if (e_x == 16'h0001) begin
						e_x     <= 16'h0000;
						e_state <= E_DONE;
					end else begin
						// retry from the raw ciphertext, one clock short
						e_x     <= e_c;
						e_cnt   <= (e_cnt0 == 8'd0) ? 8'd0 : (e_cnt0 - 8'd1);
						e_state <= E_RUN2;
					end
				end

				E_RUN2: if (e_cnt == 8'd0) begin
					e_state <= E_DONE;
				end else begin
					e_x   <= e_nx;
					e_cnt <= e_cnt - 8'd1;
				end

				E_DONE: begin
					reply   <= e_x;
					e_state <= E_IDLE;
				end

				default: e_state <= E_IDLE;
			endcase

			// CPU access
			if (acc_valid) begin
				logic [1:0]  wm;      // working mode
				logic [15:0] wt;      // working taps
				logic        s0v, s1v;
				logic [15:0] s0d, s1d;
				logic [10:0] idx0, idx1;
				logic        rqv;
				logic [10:0] rqi;
				logic [15:0] rqc;

				wm  = mode;
				wt  = taps;
				s0v = 1'b0; s0d = 16'd0;
				s1v = 1'b0; s1d = 16'd0;
				rqv = 1'b0; rqi = 11'd0; rqc = 16'd0;

				idx0 = (h0_addr[11:1] - PR_DATA[11:1]);
				idx1 = (h1_addr[11:1] - PR_DATA[11:1]);

				if (acc_we) begin
					// write16, half 0. Mode never changes on a write, so both
					// halves see the same mode; taps can change between them.
					if (in_data_window(h0_addr) && wm == MODE_SETKEY) begin
						if (idx0[0]) begin
							kwe_o <= 1'b1; kwa_o <= idx0[10:1]; kwd_o <= h0_wdata[7:0];
						end else begin
							kwe_e <= 1'b1; kwa_e <= idx0[10:1]; kwd_e <= h0_wdata[7:0];
						end
					end else if (in_data_window(h0_addr) && wm == MODE_DECIPHER) begin
						rqv = 1'b1; rqi = idx0; rqc = h0_wdata;
					end else if (h0_addr == PR_CHAR0 || h0_addr == PR_CHAR1 ||
								 h0_addr == PR_CHAR2) begin
						// reached only if the data-window branch did not claim the
						// access, i.e. mode == IDLE for CHAR0
						wt = char_taps(h0_wdata, wt);
					end

					// write16, half 1
					if (h1_valid) begin
						if (in_data_window(h1_addr) && wm == MODE_SETKEY) begin
							if (idx1[0]) begin
								kwe_o <= 1'b1; kwa_o <= idx1[10:1]; kwd_o <= h1_wdata[7:0];
							end else begin
								kwe_e <= 1'b1; kwa_e <= idx1[10:1]; kwd_e <= h1_wdata[7:0];
							end
						end else if (in_data_window(h1_addr) && wm == MODE_DECIPHER) begin
							rqv = 1'b1; rqi = idx1; rqc = h1_wdata;
						end else if (h1_addr == PR_CHAR0 || h1_addr == PR_CHAR1 ||
									 h1_addr == PR_CHAR2) begin
							wt = char_taps(h1_wdata, wt);
						end
					end
				end else begin
					// read16, half 0
					case (h0_addr)
						PR_SETKEY:   wm = MODE_SETKEY;
						PR_DECIPHER: wm = MODE_DECIPHER;
						PR_DONE0,
						PR_DONE4:    if (wm == MODE_SETKEY) wm = MODE_IDLE;
						PR_STATUS: begin
							s0v = 1'b1;
							s0d = (STATUS_ALWAYS_READY || !busy) ? 16'h8000 : 16'h0000;
						end
						PR_RESULT: if (wm == MODE_DECIPHER) begin
							s0v = 1'b1;
							s0d = reply;
							wm  = MODE_IDLE;
						end
						default: ;
					endcase

					// read16, half 1
					if (h1_valid) begin
						case (h1_addr)
							PR_SETKEY:   wm = MODE_SETKEY;
							PR_DECIPHER: wm = MODE_DECIPHER;
							PR_DONE0,
							PR_DONE4:    if (wm == MODE_SETKEY) wm = MODE_IDLE;
							PR_STATUS: begin
								s1v = 1'b1;
								s1d = (STATUS_ALWAYS_READY || !busy) ? 16'h8000 : 16'h0000;
							end
							PR_RESULT: if (wm == MODE_DECIPHER) begin
								s1v = 1'b1;
								s1d = reply;
								wm  = MODE_IDLE;
							end
							default: ;
						endcase
					end
				end

				mode       <= wm;
				taps       <= wt;
				sub0_valid <= s0v; sub0_data <= s0d;
				sub1_valid <= s1v; sub1_data <= s1d;

				if (rqv) begin
					rq_v    <= 1'b1;
					rq_idx  <= rqi;
					rq_c    <= rqc;
					rq_taps <= wt;       // taps written earlier in this access apply
				end
			end

		end
	end

	// Simulation check: the even/odd bank split needs the memmap to present the
	// second half of a 32-bit access at h0_addr + 2. Only key-RAM writes depend
	// on it, so only they are checked.
	// synthesis translate_off
	always @(posedge clk) begin
		if (rst_n && acc_valid && acc_we && h1_valid &&
			in_data_window(h0_addr) && in_data_window(h1_addr) &&
			(h1_addr !== h0_addr + 24'd2))
			$display("%t agt_prage_prot: CONTRACT VIOLATION h0=%06X h1=%06X",
					 $time, h0_addr, h1_addr);
	end
	// synthesis translate_on

endmodule

`default_nettype wire
