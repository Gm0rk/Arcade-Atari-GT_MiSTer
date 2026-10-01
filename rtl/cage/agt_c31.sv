// agt_c31.sv -- TMS320C31 DSP core for the CAGE sound board
//
// Implements tools/cage31.py, a transcription of MAME's 320c3x.cpp, which is
// the specification; tb/tb_c31_lockstep.sv compares the two instruction by
// instruction. The decode is generated from MAME's 2,048-entry dispatch table
// (tools/gen_c31_decode.py), so the core and the model always agree on which
// handler runs. An opcode with no handler decodes to OP_UNIMPL and halts.
//
// Multi-cycle, one memory port, no pipeline: the lockstep bench serves reads
// from the model's own transaction stream and checks address, order and
// kind, so the core issues its accesses in the model's order. With IC_EN,
// fetches that hit agt_cage_icache skip the port.
//
// The registers are one array of {exponent, mantissa} in MAME's layout
// (R0-R7, AR0-AR7, DP, IR0, IR1, BK, SP, ST, IE, IF, IOF, RS, RE, RC and
// scratch slots), because the golden state dump is that array.
//
// On chip: the DMA controller, the two timers, the serial port's timing (its
// data leaves on dac_* for agt_cage_dac), the peripheral file and IRAM.
//
// S_EOS, the end of a step, is a state of its own. It counts the model's
// cycles (1 per step, 3 more for a taken branch, 4 for a trap) and runs the
// model's last three lines
//     poll_timers(); poll_dma(); check_irqs()
// It must be a separate cycle: `retic` sets GIE and then asks for interrupts
// in the same step, and a non-blocking man[ST] write is not visible to logic
// on its own edge.
`default_nettype none

`include "c31_ops.svh"
`include "c31_decode.svh"

// BOOT_START = 1: come out of reset in the state cage31.py's boot_load
// leaves, since nothing on the board seeds the registers:
//     self.pc = entry            (the first block's destination, 0x001000)
//     self.r_man[ST] = 0
//     self.r_man[SP] = 0x0000FF  ("the program sets its own")
// and every other register zero, which the reset gives. The reset enters
// S_BOOT, which loads boot_pc and SP in one cycle and falls into S_FETCH. Not
// in the reset branch itself: loading a variable there is an asynchronous
// load, which Quartus builds from latches.
// BOOT_START = 0 never enters S_BOOT and ignores boot_pc; benches then seed
// the registers and PC after reset.
module agt_c31 #(
	parameter bit BOOT_START = 1'b0,
	// Fetch from agt_cage_icache when it hits, with what that enables (the take
	// in S_EOS, the hold, the fused end of step). 0 leaves ic_hit*/ic_word*
	// unread and fetches every word through the port.
	parameter bit IC_EN = 1'b0,
	// Wait in S_EXEC while `hold` is high (agt_cage_gov: the model's time is
	// ahead of real time). 0 leaves `hold` unread.
	parameter bit HOLD_EN = 1'b0
) (
	input  wire        clk,
	input  wire        rst_n,

	// memory port: one access at a time, req held until ack
	output logic [23:0] mem_addr,
	output logic        mem_req,
	output logic        mem_we,
	output logic [31:0] mem_wdata,
	input  wire  [31:0] mem_rdata,
	input  wire         mem_ack,
	// 1 during an instruction fetch. The golden stream records the same
	// distinction, so the bench checks the role of every access.
	output logic        mem_ifetch,

	// External interrupt lines from the board. irq_in[0] is agt_cage_comm's
	// dsp_irq0; nothing drives 3:1 (see the interrupts note below).
	input  wire  [3:0]  irq_in,

	// IOF's input flags INXF0 (bit 3) and INXF1 (bit 7), as cage.cpp drives
	// them: bit 3 is cpu_to_cage_ready (a command is waiting), bit 7
	// cage_to_cpu_ready (the DSP's reply has not been taken). Applied on each
	// edge of the line, as in the model: a program write to IOF keeps its value
	// until the line next moves. The program never names IOF as an operand;
	// these keep the register file equal to the model's, which the lockstep
	// compares every step. Tie to 0 if unused.
	input  wire  [1:0]  xf_in,

	// Boot write port: agt_cage_boot fills IRAM here while the core is held in
	// reset. Tie boot_we low if unused.
	input  wire         boot_we,
	input  wire  [10:0] boot_addr,
	input  wire  [31:0] boot_data,
	// Where S_BOOT puts the PC (agt_cage_boot's entry, via agt_cage_release).
	// Must be stable while rst_n is low and for the cycle after; unused unless
	// BOOT_START.
	input  wire  [23:0] boot_pc,

	// retire, for the lockstep bench
	output logic        insn_done,      // one cycle per retired instruction
	output logic [23:0] insn_pc,        // the PC it started at
	output logic        unimplemented,  // decoded OP_UNIMPL; core halts

	// What it halted on: ir and start_pc. Both are stable from the cycle
	// `unimplemented` rises until the next reset (S_HALT changes neither), so
	// another clock can capture them once the halt has crossed (agt_cage does,
	// for the overlay's CHLT). Leave unconnected elsewhere.
	output wire  [31:0] dbg_ir,
	output wire  [23:0] dbg_pc,
	// High in every S_FP cycle, for agt_cage's DSTL counter (the DSP's share of
	// time in the float ALU). Leave unconnected elsewhere.
	output wire         dbg_fp,

	// Real-time governor. `hold`: wait at the next instruction's S_EXEC (HOLD_EN
	// only). The instruction is fetched and nothing of it has run; board lines
	// still land in IF/IOF while it waits. Tie low if unused.
	// dbg_held: high in every cycle the core waited on it, for DSTL.
	input  wire         hold,
	output wire         dbg_held,

	// Serial port feed and model clock, for agt_cage_dac, which plays the words
	// in the DSP's own time. The words are the model's
	// `dac.append(rmem(addr) & 0xFFFF)`, one per DMA burst read, in order;
	// mcyc_add is the model's `cycles` increment for the step just ended, the
	// same count the timers and DMA run on. Leave unconnected if unused.
	output logic        dac_stb,      // one cycle per word the DMA reads
	output logic [15:0] dac_word,     // that word's low half
	output logic        dac_first,    // with dac_stb: the block's first word
	output logic        mcyc_stb,     // one cycle per end of step (S_EOS)
	output logic [4:0]  mcyc_add,     // the model cycles it earned, 1..16
	output wire  [24:0] dac_per,      // `serial_per_word or 32`, model cycles

	// Instruction cache lookup (IC_EN). ic_la is the address the next S_FETCH
	// will want: the PC, or RS when a repeat block is about to wrap.
	// agt_cage_icache answers a cycle later for ic_la (ic_hit0/ic_word0) and
	// ic_la + 1 (ic_hit1/ic_word1). A word is used only if its address is the PC
	// being fetched, and the fetch then makes no port access:
	//   - S_FETCH takes it and goes straight to S_EXEC;
	//   - S_EOS takes it at no cost when the retiring cycle's lookup is the PC
	//     (ic_la + 1 serves an instruction that was itself taken the cycle
	//     before, whose PC is one past the lookup);
	//   - S_RPTW takes RS's word on a repeat wrap.
	// ic_used pulses when a fetch is served here. When S_EOS takes it, ic_used
	// is high with insn_done and the PC has already moved past the word: the
	// step's PC is start_pc (the benches compare `ic_used ? start_pc : pc`).
	output wire  [23:0] ic_la,
	input  wire         ic_hit0,
	input  wire  [31:0] ic_word0,
	input  wire         ic_hit1,
	input  wire  [31:0] ic_word1,
	output logic        ic_used
);

	// register file, with MAME's indices
	localparam int R0  = 0,  AR0 = 8,  DP  = 16, IR0 = 17, IR1 = 18,
				   BK  = 19, SP  = 20, ST  = 21, IE  = 22, IF  = 23,
				   IOF = 24,
				   RS  = 25, RE  = 26, RC  = 27;

	logic [31:0] man [0:34];
	logic [7:0]  exp [0:34];

	// ST flag bits (320c3x.cpp)
	localparam int C_BIT = 0, V_BIT = 1, Z_BIT = 2, N_BIT = 3,
				   UF_BIT = 4, LV_BIT = 5, LUF_BIT = 6, RM_BIT = 8,
				   GIE_BIT = 13;

	// trap() jumps straight to 0x809FC0 + num, with no vector word: there is no
	// c31boot.bin in this machine, and the loader leaves a branch at each of
	// these addresses.
	localparam [23:0] VECTOR_BASE = 24'h809FC0;

	// AR0 as a five-bit value, for register-index sums: Quartus sizes a
	// `localparam int` by its value (8 -> four bits), not as 32 bits.
	localparam logic [4:0] AR0_R = 5'd8;

	// Register-file indices are five bits on purpose. man/exp are [0:34], so
	// Quartus warns (10027) that a five-bit index cannot reach every element.
	// That is the point: 32..34 are TEMP1..TEMP3, addressed only by the
	// constants T1/T2/T3 (the '31's register field is five bits), and synthesis
	// leaves them out of every variable-index write mux. Six-bit indices cost
	// ~330 ALMs. The warning is disabled in syn/c31.qsf (MESSAGE_DISABLE 10027),
	// and tools/check_c31_index_width.py asserts that no variable index can
	// reach 32..34.

	logic [23:0] pc;
	logic [31:0] ir;                    // the instruction word being executed
	logic [23:0] start_pc;
	logic [23:0] ic_la_q;           // the address the cache answers for
	logic [23:0] ic_la_q1;          // ...and the one after it
	assign dbg_ir = ir;
	assign dbg_pc = start_pc;

	// Deferred AR write-back (the model's ind_1_def): a parallel op computes its
	// pointer update but commits it only after both halves have read. A
	// one-entry queue, since writing early is the bug `_def` exists to avoid.
	logic        def_pend;
	logic [4:0]  def_reg;
	logic [31:0] def_val;

	// decode
	c31_op_e     opk;
	always_comb begin
		`C31_DECODE(ir[31:21], opk)
	end

	// operand fields shared by the two-operand forms
	wire [4:0]  f_dst   = ir[20:16];
	wire [4:0]  f_sreg  = ir[4:0];
	wire [15:0] f_imm16 = ir[15:0];
	wire [23:0] f_imm24 = ir[23:0];
	wire [7:0]  f_ind_o = ir[15:8];     // mod[4:0], reg[2:0]
	wire [7:0]  f_disp  = ir[7:0];
	wire [4:0]  f_cond  = ir[20:16];

	// Condition evaluation: 320c3x.cpp's condition_table, 0x80 entries indexed
	// by {LUF,LV,UF,N,Z,V,C}, each a 32-bit mask over the condition codes. Used
	// as is: c31_cond.svh is generated from MAME's source by gen_c31_decode.py.

	// A conditional load's condition is in its name, not in a field, so it comes
	// from the generated table, as does which handlers are conditional loads
	// (C31_IS_NAMED_COND, by the model's rule). cond_sel picks between the two.
	logic [4:0] named_cond;
	always_comb begin
		`C31_NAMED_COND(opk, named_cond)
	end
	wire is_named_cond = `C31_IS_NAMED_COND(opk);
	wire [4:0] cond_sel = is_named_cond ? named_cond : f_cond;

	// MAME's table, generated rather than re-derived, so it cannot differ from
	// the model. (MAME's LT/LE/GT/GE are N, N|Z, !N&!Z and !N.)
	`include "c31_cond.svh"
	wire cond_ok = C31_COND_TABLE[{cond_sel, man[ST][6:0]}];

	// Indirect addressing: the model's indirect(), one for one. `d` is the
	// displacement: the instruction's low byte for the ordinary forms, 1 for the
	// parallel forms. The write-back to ARn is immediate or deferred.
	function automatic [31:0] ind_addr(input [7:0] o, input [31:0] d,
									   input [31:0] cur, input [31:0] ir0,
									   input [31:0] ir1);
		logic [4:0]  mod;
		logic [31:0] step;
		begin
			mod  = o[7:3];
			step = (mod < 5'h08) ? d : ((mod < 5'h10) ? ir0 : ir1);
			unique case (mod[2:0])
				3'd0: ind_addr = (mod == 5'h18) ? cur : (cur + step);
				3'd1: ind_addr = cur - step;
				3'd2: ind_addr = cur + step;
				3'd3: ind_addr = cur - step;
				default: ind_addr = cur;         // post-modify: address is cur
			endcase
			// mod 0x00/0x01 are pre-add/pre-sub with no write-back; 0x18 is the plain
			// *ARn. ind_writes decides whether ARn moves at all.
		end
	endfunction

	function automatic ind_writes(input [7:0] o);
		logic [4:0] mod;
		begin
			mod = o[7:3];
			ind_writes = (mod != 5'h18) && (mod[2:0] >= 3'd2) &&
						 (mod <= 5'h17);
		end
	endfunction

	function automatic [31:0] ind_newar(input [7:0] o, input [31:0] d,
										input [31:0] cur, input [31:0] ir0,
										input [31:0] ir1);
		logic [4:0]  mod;
		logic [31:0] step;
		begin
			mod  = o[7:3];
			step = (mod < 5'h08) ? d : ((mod < 5'h10) ? ir0 : ir1);
			unique case (mod[2:0])
				3'd2, 3'd4: ind_newar = cur + step;
				3'd3, 3'd5: ind_newar = cur - step;
				default:    ind_newar = cur;     // circular modes not modelled
			endcase
		end
	endfunction

	// Integer flag helpers. `op_ldi`: setreg, then, only when the destination
	// is R0-R7, clear N/Z/V/UF and OR in N and Z (an AR load touches no flags).
	// `op_logic` is gated the same way.
	localparam logic [31:0] M_NZVUF  = (32'd1 << N_BIT) | (32'd1 << Z_BIT) |
									   (32'd1 << V_BIT) | (32'd1 << UF_BIT);
	localparam logic [31:0] M_NZCVUF = M_NZVUF | (32'd1 << C_BIT);

	function automatic [31:0] ldi_flags(input [31:0] st, input [4:0] dreg,
										input [31:0] r);
		begin
			if (dreg < 5'd8) begin
				ldi_flags            = st & ~M_NZVUF;
				ldi_flags[N_BIT]     = r[31];
				ldi_flags[Z_BIT]     = (r == 32'd0);
			end else begin
				ldi_flags = st;
			end
		end
	endfunction

	// `op_cmpi` / `op_subi` (no borrow): clear N/Z/C/V/UF, then C from an
	// unsigned borrow, V from OVERFLOW_SUB, LV |= V, N and Z from the result.
	function automatic [31:0] sub_flags(input [31:0] st, input [31:0] s1,
										input [31:0] s2, input [31:0] r);
		logic v;
		logic [31:0] ovf;
		begin
			ovf                = (s1 ^ s2) & (s1 ^ r);
			v                  = ovf[31];
			sub_flags          = st & ~M_NZCVUF;
			sub_flags[C_BIT]   = (s2 > s1);
			sub_flags[V_BIT]   = v;
			sub_flags[LV_BIT]  = sub_flags[LV_BIT] | v;
			sub_flags[N_BIT]   = r[31];
			sub_flags[Z_BIT]   = (r == 32'd0);
		end
	endfunction

	// `op_addi`: clear N/Z/C/V/UF, C from an unsigned carry out (`s1 > res`),
	// V from OVERFLOW_ADD, LV |= V, then N and Z. The caller gates on dreg < 8.
	function automatic [31:0] add_flags(input [31:0] st, input [31:0] s1,
										input [31:0] s2, input [31:0] r);
		logic v;
		logic [31:0] ovf;
		begin
			ovf               = (s1 ^ r) & (s2 ^ r);
			v                 = ovf[31];
			add_flags         = st & ~M_NZCVUF;
			add_flags[C_BIT]  = (s1 > r);
			add_flags[V_BIT]  = v;
			add_flags[LV_BIT] = add_flags[LV_BIT] | v;
			add_flags[N_BIT]  = r[31];
			add_flags[Z_BIT]  = (r == 32'd0);
		end
	endfunction

	// `op_lsh`: the count is a 7-bit signed field of the source, negative
	// meaning shift right. C is the last bit shifted out, only for counts within
	// +/-32. The guards bound the count, so the shift amount is narrowed to six
	// unsigned bits (a signed shift amount draws Quartus warning 10764).
	function automatic [31:0] lsh_res(input [31:0] src, input [6:0] cnt);
		logic signed [7:0] c;
		logic [7:0]        na;
		logic [5:0]        n;
		begin
			c  = {{1{cnt[6]}}, cnt};
			na = c[7] ? (-c) : c;
			n  = na[5:0];
			if (c < 0) lsh_res = (c >= -8'sd31) ? (src >> n) : 32'd0;
			else       lsh_res = (c <= 8'sd31)  ? (src << n) : 32'd0;
		end
	endfunction

	// `op_ash`: the same 7-bit signed count, arithmetic shift. Each branch goes
	// through a signed intermediate on purpose: inline in a wider expression,
	// Verilog makes the left operand unsigned and `>>>` silently becomes a
	// logical shift.
	function automatic [31:0] ash_res(input [31:0] src, input [6:0] cnt);
		logic signed [7:0]  c;
		logic [7:0]         na;
		logic [5:0]         n;
		logic signed [31:0] s, r;
		begin
			c  = {{1{cnt[6]}}, cnt};
			na = c[7] ? (-c) : c;
			n  = na[5:0];
			s  = $signed(src);
			if (c < 0) r = (c >= -8'sd31) ? (s >>> n) : (s >>> 6'd31);
			else       r = (c <= 8'sd31)  ? (s <<< n) : 32'sd0;
			ash_res = r;
		end
	endfunction

	// The flags differ from lsh's in one place only: for a count below -32,
	// op_ash takes the carry from the sign bit and op_lsh sets none. Elsewhere
	// they are the same expression (for 32 or fewer places, `& 1` cannot tell an
	// arithmetic shift from a logical one).
	function automatic [31:0] ash_flags(input [31:0] st, input [31:0] src,
										input [6:0] cnt, input [31:0] r);
		logic signed [7:0] c;
		logic [7:0]        na;
		logic [5:0]        nm;
		logic [31:0]       t;
		logic cb;
		begin
			c  = {{1{cnt[6]}}, cnt};
			na = c[7] ? (-c - 8'sd1) : (c - 8'sd1);
			nm = na[5:0];
			cb = 1'b0;
			if (c < 0 && c >= -8'sd32)      begin t = src >> nm; cb = t[0];  end
			else if (c < 0)                 cb = src[31];
			else if (c > 0 && c <= 8'sd32)  begin t = src << nm; cb = t[31]; end
			ash_flags         = st & ~M_NZCVUF;
			ash_flags[N_BIT]  = r[31];
			ash_flags[Z_BIT]  = (r == 32'd0);
			ash_flags[C_BIT]  = cb;
		end
	endfunction

	function automatic [31:0] lsh_flags(input [31:0] st, input [31:0] src,
										input [6:0] cnt, input [31:0] r);
		logic signed [7:0] c;
		logic [7:0]        na;
		logic [5:0]        nm;
		logic [31:0]       t;
		logic cb;
		begin
			c  = {{1{cnt[6]}}, cnt};
			na = c[7] ? (-c - 8'sd1) : (c - 8'sd1);
			nm = na[5:0];
			cb = 1'b0;
			if (c < 0 && c >= -8'sd32)      begin t = src >> nm; cb = t[0];  end
			else if (c > 0 && c <= 8'sd32)  begin t = src << nm; cb = t[31]; end
			lsh_flags         = st & ~M_NZCVUF;
			lsh_flags[N_BIT]  = r[31];
			lsh_flags[Z_BIT]  = (r == 32'd0);
			lsh_flags[C_BIT]  = cb;
		end
	endfunction

	// `op_mpyi`: a 24 x 24 signed multiply kept to 32 bits, with V/LV set when
	// it did not fit. Clears NZVUF but not the carry.
	function automatic [47:0] mpyi_prod(input [31:0] a, input [31:0] b);
		begin
			mpyi_prod = $signed({{24{a[23]}}, a[23:0]}) *
						$signed({{24{b[23]}}, b[23:0]});
		end
	endfunction

	function automatic [31:0] mpyi_flags(input [31:0] st, input [47:0] p);
		logic fits;
		begin
			fits = (p[47:31] == 17'h00000) || (p[47:31] == 17'h1FFFF);
			mpyi_flags         = st & ~M_NZVUF;
			mpyi_flags[N_BIT]  = p[31];
			mpyi_flags[Z_BIT]  = (p[31:0] == 32'd0);
			if (!fits) begin
				mpyi_flags[V_BIT]  = 1'b1;
				mpyi_flags[LV_BIT] = 1'b1;
			end
		end
	endfunction

	// `tstb`: `ST &= ~NZVUF; or_nz(res)` with no destination, so no `dreg < 8`
	// gate.
	function automatic [31:0] tst_flags(input [31:0] st, input [31:0] r);
		begin
			tst_flags        = st & ~M_NZVUF;
			tst_flags[N_BIT] = r[31];
			tst_flags[Z_BIT] = (r == 32'd0);
		end
	endfunction

	// `op_absi`: the magnitude, with V/LV when it does not fit (only 0x80000000,
	// whose negation is itself). OVM saturation is not modelled, as for every
	// integer op here: the program never sets OVM.
	function automatic [31:0] absi_res(input [31:0] s);
		begin
			absi_res = s[31] ? (32'd0 - s) : s;
		end
	endfunction
	function automatic [31:0] absi_flags(input [31:0] st, input [31:0] r);
		begin
			absi_flags        = st & ~M_NZVUF;
			absi_flags[N_BIT] = r[31];
			absi_flags[Z_BIT] = (r == 32'd0);
			if (r == 32'h8000_0000) begin
				absi_flags[V_BIT]  = 1'b1;
				absi_flags[LV_BIT] = 1'b1;
			end
		end
	endfunction

	// The C3x extended float: an 8-bit exponent and a 32-bit two's-complement
	// mantissa with the implicit bit folded into the sign bit, so
	//     value = ((sign_extend32(stored) ^ 0x80000000) / 2**31) * 2**exp
	// +1.0 is {man 0x00000000, exp 0} and -1.0 is {0x80000000, -1}, because the
	// negative mantissa range is [-2,-1). The conversions below are long2fp,
	// short2fp and fp2long transcribed, not re-derived.

	// `long2fp`: a 32-bit memory word is exponent in the top byte and the
	// mantissa's top 24 bits below.
	function automatic [39:0] long2fp(input [31:0] v);
		begin
			long2fp = {v[31:24], v[23:0], 8'd0};   // {exp, man}
		end
	endfunction

	// `short2fp`: the 16-bit immediate form. 0x8000 encodes zero, which is
	// {man 0, exp -128}, not a tiny negative number.
	function automatic [39:0] short2fp(input [15:0] v);
		begin
			if (v == 16'h8000) short2fp = {8'h80, 32'd0};
			else               short2fp = {{{4{v[15]}}, v[15:12]},
										   v[11:0], 20'd0};
		end
	endfunction

	// `fp2long`, the way back out to memory.
	function automatic [31:0] fp2long(input [7:0] e, input [31:0] m);
		begin
			fp2long = {e, m[31:8]};
		end
	endfunction

	// `or_nzf`: N from the mantissa's sign bit, Z from exp == -128 (the
	// representation of zero). No `dreg < 8` gate: a float destination is R0-R7
	// by construction.
	function automatic [31:0] ldf_flags(input [31:0] st, input [7:0] e,
										input [31:0] m);
		begin
			ldf_flags         = st & ~M_NZVUF;
			ldf_flags[N_BIT]  = m[31];
			ldf_flags[Z_BIT]  = (e == 8'h80);
		end
	endfunction

	// The float ALU (addf, subf, mpyf and the conversions) is agt_c31_fp.sv, a
	// combinational module so tb_c31_float can drive it directly: the lockstep
	// never reaches underflow, exponent overflow, exact cancellation or a
	// 32-place alignment. Operands come from registers and the result is read a
	// cycle later in S_FP.
	localparam [2:0] FP_ADD = 3'd0, FP_SUB = 3'd1, FP_MPY  = 3'd2,
					 FP_NEG = 3'd3, FP_I2F = 3'd4, FP_F2I  = 3'd5,
					 FP_F2IQ = 3'd6;
	logic [2:0]  fp_op;
	logic [7:0]  fa_e, fb_e;
	logic [31:0] fa_m, fb_m;
	logic [5:0]  fp_dst;
	logic        fp_manonly;    // `fix`: integer result, exponent left alone
	logic [1:0]  fp_stage;      // mpyaddf runs the ALU twice
	logic [7:0]  fp_re;
	logic [31:0] fp_rm, fp_rst;

	agt_c31_fp u_fp (
		.op(fp_op),
		.e1(fa_e), .m1(fa_m), .e2(fb_e), .m2(fb_m),
		.st_i(man[ST]),
		.ro_e(fp_re), .ro_m(fp_rm), .st_o(fp_rst)
	);

	// mpyaddf_3's bookkeeping uses TEMP1..TEMP3 (slots 32-34) as the model does:
	// a parallel multiply-add needs three values to outlive a cycle, and the
	// same slots keep the dataflow one for one with `_mpy_par`. The '31 has no
	// such registers.
	localparam int T1 = 32, T2 = 33, T3 = 34;
	logic [4:0] ma_dm, ma_da;
	logic [2:0] ma_r1, ma_r2;

	// state machine
	typedef enum logic [4:0] {
		S_FETCH, S_FWAIT, S_EXEC, S_RD, S_WR, S_HALT,
		S_RPTW,          // the repeat-block wrap: a step with no fetch
		S_PAR_RD, S_PAR_WR,
		S_RET,           // retsc: the popped PC is arriving
		S_STI2,          // stisti: the second of two writes
		S_DMA,           // the DMA controller's burst
		S_3RD1,          // three-operand: the first indirect source
		S_3RD2X,         // ...indind's second
		S_3RD2,          // ...indreg's register half
		S_EOS,           // end of step: cycles, poll_dma, check_irqs
		S_TRAP,          // ...and if one was taken, the push
		S_TRAPW,
		S_FP,            // the float ALU's result is on the wires
		S_MA1,           // mpyaddf: the first indirect source
		S_MA2,           // ...and the second
		S_BOOT           // the loader's start state (BOOT_START only)
	} state_e;
	state_e st_q;
	assign dbg_fp = (st_q == S_FP);
	assign dbg_held = HOLD_EN && hold && (st_q == S_EXEC);

	// Instruction cache lookup address: what the next S_FETCH wants, one cycle
	// ahead. Every state before an S_FETCH leaves the PC as S_FETCH will read
	// it, except the repeat wrap, which moves the PC to RS on the edge it enters
	// S_FETCH. `rpt_end` (the PC is one past the block with RM set: the next
	// step is the wrap) and `rpt_goes` (RC - 1 >= 0: the wrap returns to RS)
	// hold from the cycle the block's last instruction is taken until S_RPTW, so
	// the lookup is RS in all of them and S_RPTW's word is in hand when it runs.
	// A lookup that turns out wrong is simply not used.
	wire rpt_end  = man[ST][RM_BIT] && ({8'd0, pc} == man[RE] + 32'd1);
	wire rpt_goes = !(man[RC] - 32'd1 >= 32'h8000_0000);
	wire ic_wrap_to_rs = rpt_end && rpt_goes;
	assign ic_la = ic_wrap_to_rs ? man[RS][23:0] : pc;
	always_ff @(posedge clk) ic_la_q  <= ic_la;
	always_ff @(posedge clk) ic_la_q1 <= ic_la + 24'd1;
	// The PC's word is either half of the answer: word 0 when the lookup was the
	// PC, word 1 when it was the word before the PC.
	wire        ic_sel0 = ic_hit0 && (ic_la_q  == pc);
	wire        ic_sel1 = ic_hit1 && (ic_la_q1 == pc);
	wire [31:0] ic_word = ic_sel0 ? ic_word0 : ic_word1;
	wire ic_take = IC_EN && (ic_sel0 || ic_sel1) && (pc[23:16] == 8'd0);
	// S_EXEC's next state, blocking, so the retire can be derived from it.
	state_e nx;

	// What S_RD should do with the word when it arrives. The direct and
	// indirect forms all read the same way and differ only here.
	typedef enum logic [3:0] {
		RK_LDI, RK_ADDI, RK_CMPI, RK_LDCOND, RK_POP, RK_LDF, RK_POPF,
		// The read feeds the float ALU. The operation and the operand slot are
		// latched at decode, so one kind covers mpyf/addf/subf/cmpf (the word is
		// operand B, the destination register operand A) and negf/fix (the word is
		// the only operand).
		RK_FOP,
		// subi/subri/mpyi/and/tstb from memory; `float` of a memory integer (the
		// word goes to the ALU raw, not long2fp'd); and a conditional float load
		// (long2fp, no flags).
		RK_SUBI, RK_SUBRI, RK_MPYI, RK_AND, RK_TSTB, RK_FOPI, RK_LDFC,
		// `negi` from memory, `op_subi(dreg31, 0, word)`. The four bits are full.
		RK_NEGI
	} rdkind_e;
	rdkind_e     rk;
	logic        rk_mem_a;

	logic        retire;        // set by whichever state finishes the step
	// Set in the cycle a delayed branch decodes. dly_cnt and dly_pend are
	// non-blocking and still zero on that edge, so without this the branch would
	// retire on its own and the group count four lines where the model has one.
	logic        dly_start;
	// The C3x DMA controller: a peripheral of the DSP, so it lives here and
	// shares the memory port. MAME does the whole transfer the instant the
	// enable is written and only delays the completion interrupt, so the burst
	// runs contiguously inside the instruction whose write enabled it, with
	// nothing executing (the real chip steals cycles gradually).
	// Only the registers the core uses are shadowed, not the full 256-word I/O
	// file (8,192 flip-flops).
	localparam int IO_BASE = 24'h808000;
	localparam [7:0] DMA_GLOBAL_CTL = 8'h00, DMA_SOURCE = 8'h04,
					 DMA_DEST       = 8'h06, DMA_COUNT  = 8'h08,
					 SPORT_GLOBAL_CTL = 8'h40, SPORT_RX_CTL = 8'h43,
					 SPORT_TIMER_PERIOD = 8'h46;
	// SPORT_RX_CTL: a plain read-back register; the program reads it and nothing
	// here uses it.
	logic [31:0] sport_rx;
	logic [31:0] dma_ctl, dma_src, dma_dst, dma_cnt;
	logic        dma_on;            // `self.dma_on`: edge-detect the enable
	logic [31:0] dma_left;          // words still to read in this burst
	logic [31:0] dma_addr;          // the walking source address
	logic [23:0] dma_fin;           // `self.dma_final`, latched at the end
	logic        dma_first;         // the burst's next word is its first

	// dma_dst shadows DMA_DEST for read-back only (per_rdata).
	wire dma_inc = dma_ctl[4];

	logic        is_io, is_dmar;
	logic [31:0] nctl, nsrc, ncnt;

	// The step clock and the DMA's completion interrupt. The transfer is
	// instantaneous; DINT0 is not: MAME schedules it `serial_per_word * count`
	// cycles out and raises it from poll_dma. So the core keeps the model's
	// cycle count, a step clock rather than a hardware clock:
	//
	//     +1   every step()          (the fetch)
	//     +3   a taken branch, call, retsc, retic, rptb -- inside the handler
	//     +4   trap()
	//     +0   a repeat-block wrap, which returns before the increment
	//
	// cyc_pend carries the extras to the S_EOS that spends them: trap()'s 4 are
	// added after that step's poll_dma and belong to the next step, and the wrap
	// step spends nothing. Four bits: a trap's 4 plus the handler's first `br`
	// (3) is 7, exactly the three-bit maximum, so one bit of headroom.
	logic [3:0]  cyc_pend;

	// The deadline is a countdown rather than `cycles` against an absolute
	// `dma_due`: the same arithmetic, with no 2^32 wrap to reason about.
	logic [31:0] dma_cd;
	logic        dma_pend;          // `self.dma_due is not None`
	logic        dma_skip;          // the arming step's own poll: not ours

	// The two timers (poll_timers): the same countdown and arming-step `skip`
	// as the DMA, but a fire reloads from the current cycle, because
	// poll_timers calls update_timer again and that re-arms from `cycles`.
	localparam [7:0] TIMER0_CTL = 8'h20, TIMER0_COUNTER = 8'h24,
					 TIMER0_PERIOD = 8'h28,
					 TIMER1_CTL = 8'h30, TIMER1_COUNTER = 8'h34,
					 TIMER1_PERIOD = 8'h38;
	logic [31:0] tmr_ctl [0:1];
	logic [31:0] tmr_per [0:1];
	logic [32:0] tmr_cd  [0:1];     // 2 x a 32-bit period needs 33 bits
	logic        tmr_pend[0:1];
	logic        tmr_skip[0:1];
	logic        tmr_hit;
	logic [31:0] ntctl, ntper;

	// `update_serial()`, from the two SPORT registers that feed it. The third
	// write that triggers it in the model (SPORT_TIMER_CTL, 0x44) changes none
	// of its inputs, so it is not shadowed.
	logic [31:0] sport_ctl, sport_per;
	wire [2:0]  sp_clk    = sport_ctl[2] ? 3'd4 : 3'd2;         // H1 cycles
	// `bit_clk = clk if period == 0 else clk * period`, and period == 0 with the
	// internal clock bit clear is MAME's `attotime::never`.
	wire        sp_never  = (sport_per[15:0] == 16'd0) && !sport_ctl[2];
	wire [18:0] sp_bitclk = (sport_per[15:0] == 16'd0)
							  ? {16'd0, sp_clk}
							  : (sport_per[15:0] * sp_clk);
	wire [5:0]  sp_bits   = {(3'd1 + {1'b0, sport_ctl[19:18]}), 3'd0};  // 8*(w+1)
	wire [24:0] sp_word   = sp_bitclk * sp_bits;
	// `per = self.serial_per_word or 32`: None and zero both mean 32.
	wire [24:0] sp_per_eff = (sp_never || sp_word == 25'd0) ? 25'd32 : sp_word;
	assign dac_per = sp_per_eff;

	// The internal bus. The state machine drives i_* and reads i_rdata/i_ack;
	// the module port carries what is left after the on-chip responders (the
	// peripheral file and IRAM) take their share. On a real '31 both are inside
	// the package and no bus cycle leaves for them.
	logic [23:0] i_addr;
	logic        i_req, i_we, i_ifetch;
	logic [31:0] i_wdata;

	// The peripheral file: 808000..8080FF, decoded on the top sixteen bits as
	// the write path does.
	wire         acc_per  = (i_addr[23:8] == IO_BASE[23:8]);

	// IRAM, 809800..809FFF: the '31's own RAM, 2,048 x 32 = eight M10K.
	// The read is registered: an asynchronous read would infer 65,536
	// flip-flops instead of memory. A read therefore acks a cycle late, which
	// the req/ack port tolerates, and the model counts steps, not wait states.
	// Pinned to M10K: left unpinned, synthesis may demote it to flip-flops when
	// block memory is near the device limit. tools/check_ram_pins.py checks that
	// every array in mem/ram_baseline.txt is pinned.
	localparam [23:0] IRAM_BASE = 24'h809800;
	(* ramstyle = "M10K" *) logic [31:0] iram [0:2047];
	logic [31:0] iram_q;
	logic        iram_pend;
	wire         acc_iram = (i_addr[23:11] == IRAM_BASE[23:11]);
	wire  [10:0] iram_ix  = i_addr[10:0];

	// Boot write port: agt_cage_boot (rtl/board) writes the boot table's IRAM
	// blocks here while the core is held in reset, so the write cannot live in
	// the rst_n branch. The array has its own reset-free block with one muxed
	// write address, which keeps it inferring as memory (one write and one read
	// port; an array with an asynchronous reset is not a RAM).
	wire        ir_we  = boot_we
					   || (rst_n && acc_iram && i_req && i_we);
	wire [10:0] ir_wix = boot_we ? boot_addr  : iram_ix;
	wire [31:0] ir_wd  = boot_we ? boot_data  : i_wdata;

	always_ff @(posedge clk) begin
		iram_q <= iram[iram_ix];
		if (ir_we) iram[ir_wix] <= ir_wd;
	end

	always_ff @(posedge clk) begin
		if (!rst_n) iram_pend <= 1'b0;
		else        iram_pend <= acc_iram && i_req && !i_we && !iram_pend;
	end

	wire         acc_int  = acc_per || acc_iram;

	assign mem_addr   = i_addr;
	assign mem_we     = i_we;
	assign mem_wdata  = i_wdata;
	assign mem_ifetch = i_ifetch;
	assign mem_req    = i_req && !acc_int;

	// `io_read`'s one live case: DMA_GLOBAL_CTL reads back with bits 3:2 forced
	// to 11 when the channel is enabled and 00 when it is not --
	//     (io[0] & ~0xC) | 0xC   if (io[0] & 3) == 3 and io[COUNT] != 0
	// e.g. 0xE13 disabled, 0xE1F enabled.
	wire         dma_en_rd = (dma_ctl[1:0] == 2'b11) && (dma_cnt != 32'd0);
	logic [31:0] per_rdata;
	always_comb begin
		// Every offset the core holds answers with its register; any other offset
		// reads 0.
		unique case (i_addr[7:0])
			DMA_GLOBAL_CTL:     per_rdata = {dma_ctl[31:4], {2{dma_en_rd}},
											 dma_ctl[1:0]};
			DMA_SOURCE:         per_rdata = dma_src;
			DMA_DEST:           per_rdata = dma_dst;
			DMA_COUNT:          per_rdata = dma_cnt;
			SPORT_GLOBAL_CTL:   per_rdata = sport_ctl;
			SPORT_RX_CTL:       per_rdata = sport_rx;
			SPORT_TIMER_PERIOD: per_rdata = sport_per;
			TIMER0_CTL:         per_rdata = tmr_ctl[0];
			TIMER0_PERIOD:      per_rdata = tmr_per[0];
			TIMER1_CTL:         per_rdata = tmr_ctl[1];
			TIMER1_PERIOD:      per_rdata = tmr_per[1];
			default:            per_rdata = 32'd0;
		endcase
	end

	// A write lands in one cycle on either responder, like the external side.
	// A read of the peripheral file is combinational over registers; a read of
	// IRAM waits one cycle for the block RAM's registered output.
	wire [31:0] i_rdata = acc_iram ? iram_q
						: acc_per  ? per_rdata
								   : mem_rdata;
	wire        i_ack   = acc_iram ? (i_we ? i_req : iram_pend)
						: acc_per  ? i_req
								   : mem_ack;

	// Interrupts. Of the model's irq_state only bits 3:0 are ever read
	// (`IF |= irq_state & 0x0F`), and those are the board's lines IRQ0-3, so
	// the core keeps no irq_state register: the lines are the irq_in port.
	// irq_in[0] is agt_cage_comm's dsp_irq0 ("pulses when a new command is
	// posted", cage.cpp's set_irq_line(0, ASSERT_LINE)). Nothing on this board
	// drives INT1..3 (cage.cpp raises line 0 only); the port is four bits to
	// match IF's IRQ0-3.

	logic [4:0]  trap_num;

	// the lines' previous values, for the edge events below
	logic [3:0]  irq_prev;
	logic [1:0]  xf_prev;

	// End-of-step take (IC_EN): S_EOS goes straight to S_EXEC when the cache
	// has the PC's word. The lookup was made in the retiring cycle, so the guard
	// refuses it whenever the retire itself moved the PC (a branch, the end of a
	// delayed group), and S_FETCH takes the word instead. Not on a repeat wrap:
	// the end of step goes to S_RPTW for that.
	wire ic_rpt_wrap = rpt_end;
	wire ic_take_eos = ic_take && !ic_rpt_wrap;

	// The hold (IC_EN). Without S_FETCH, the cycle after S_EOS is the next
	// instruction's S_EXEC. A board line that changes in that cycle (IRQ0-3,
	// INXF0/1) lands in IF/IOF only at its end, so S_EXEC would read them a
	// cycle stale, and an instruction writing IF would overwrite the new bit.
	// So S_EXEC waits one cycle whenever a line changed: the edge lands, then
	// the instruction runs, as the model's next step would see it.
	wire ic_hold = IC_EN && ((irq_in != irq_prev) || (xf_in != xf_prev));

	// end-of-step scratch
	logic        eos_done;          // this step retires an instruction
	logic        eos_retic;         // ...and it was a taken `retic`
	logic        grp_end, rpt_step, dma_fire, take_irq;
	// Fused end of step. cyc_wr: a handler added model cycles this cycle (a
	// branch, call, retic, rptb, rpts); fuse_now: the end of step runs in this
	// retiring cycle; done_v/retic_v: eos_done/eos_retic for whichever cycle
	// ends the step.
	logic        cyc_wr, fuse_ok, fuse_now, done_v, retic_v, mem_ok;
	// rptw_took: S_RPTW took the next word this cycle; fuse_st, fuse_dst: the
	// fused end of step's state and destination tests.
	logic        rptw_took, fuse_st, fuse_dst;
	logic [31:0] cyc_add, if_next, vpre, vpost, vsel;
	logic [41:0] per_n;
	logic [3:0]  which_i;
	// Three-operand sources. three_srcs reads them in a fixed order (the first
	// through a deferred pointer, the second immediate) and do_three commits the
	// deferred one after both, as ldisti and stisti do.
	logic        retic;
	logic [31:0] s3_a;
	typedef enum logic [3:0] { T3_CMPI, T3_MPYI, T3_OR, T3_FLT,
							   T3_CMPIR, T3_ADDIR,
							   T3_LSH, T3_ASH,
							   T3_SUBIR, T3_AND, T3_ANDN,      // indreg
							   T3_ADDRI                        // regind
							   } t3kind_e;
	t3kind_e     t3;
	logic [39:0] fp;
	logic [47:0] prod;
	logic [31:0] rd_val;
	logic [31:0] tmp;
	logic [31:0] ea;
	// absf/rnd scratch, computed here, not in the float ALU
	logic [31:0] fx_m, fx_st;
	logic [7:0]  fx_e;

	// Delayed branches. `execute_delayed(newpc)` runs three more steps and only
	// then moves the PC. Modelled as a counter: dly_cnt counts the slots still
	// to retire and dly_pc holds the target (dly_take is 0 when the condition
	// failed, which still burns the three slots).
	logic [1:0]  dly_cnt;
	logic        dly_take;
	logic        dly_pend;
	logic [23:0] dly_pc;

	// S_RPTW takes the word after the wrap. The next step starts at RS (or, on
	// the last pass, the word after the block), and the lookup in the cycle
	// before S_RPTW was for exactly that address, so S_RPTW takes it as S_FETCH
	// would, a state sooner. Not inside a delayed group: a wrap there is one of
	// the group's slots, and the group's end may move the PC on this edge.
	// That lookup is made while the block's last step is still in flight, so a
	// store by that step to an address with RS's low ten bits can clear RS's
	// entry in the cycle it is read. agt_cage_icache refuses a read that met a
	// write (no hit), and S_FETCH then fetches the word as for any miss.
	wire [23:0] rptw_pc   = rpt_goes ? man[RS][23:0] : pc;
	wire        rptw_take = IC_EN && ic_hit0 && (ic_la_q == rptw_pc)
							&& (rptw_pc[23:16] == 8'd0)
							&& (dly_cnt == 2'd0) && !dly_pend;

	// src3 of a parallel store, latched at decode: the model reads it before
	// the load half writes, and the two can name the same register.
	logic [31:0] par_src3;
	// `ldfstf` is ldisti's shape with a float load half: the word is long2fp'd
	// into {exp, man} rather than written to the mantissa raw.
	logic        par_flt;
	// `mpyf3stf`: the "load" half is a multiply. The word is long2fp'd into
	// operand B, register (op>>19)&7 is operand A, and the product goes through
	// S_FP after the store.
	logic        par_mpy;

	// The model's `delayed` after an rpts: _rpts sets `self.delayed = True`, and
	// only the end of the next execute_delayed clears it, so from an rpts until
	// the next delayed branch has run its three slots, check_irqs recognises an
	// interrupt and does not take it. (MAME clears it when the repeat ends;
	// cage31.py is the specification here.) The delay-slot half of `delayed` is
	// eos_done, below.
	logic        rpts_dly;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (int i = 0; i < 35; i = i + 1) begin
				man[i] <= 32'd0;
				exp[i] <= 8'd0;
			end
			pc            <= 24'd0;
			st_q          <= BOOT_START ? S_BOOT : S_FETCH;
			insn_done     <= 1'b0;
			unimplemented <= 1'b0;
			i_req       <= 1'b0;
			i_we        <= 1'b0;
			i_ifetch    <= 1'b0;
			def_pend      <= 1'b0;
			dly_cnt       <= 2'd0;
			dly_take      <= 1'b0;
			dly_pend      <= 1'b0;
			dma_ctl       <= 32'd0;
			dma_src       <= 32'd0;
			dma_dst       <= 32'd0;
			dma_cnt       <= 32'd0;
			dma_on        <= 1'b0;
			retic         <= 1'b0;
			dma_pend      <= 1'b0;
			dma_skip      <= 1'b0;
			dma_cd        <= 32'd0;
			dma_fin       <= 24'd0;
			sport_ctl     <= 32'd0;
			sport_per     <= 32'd0;
			sport_rx      <= 32'd0;
			cyc_pend      <= 4'd0;
			trap_num      <= 5'd0;
			irq_prev      <= 4'd0;
			xf_prev       <= 2'd0;
			eos_done      <= 1'b0;
			eos_retic     <= 1'b0;
			fp_stage      <= 2'd0;
			fp_op         <= 3'd0;
			fp_manonly    <= 1'b0;
			rk_mem_a      <= 1'b0;
			par_mpy       <= 1'b0;
			rpts_dly      <= 1'b0;
			dma_first     <= 1'b0;
			dac_stb       <= 1'b0;
			ic_used       <= 1'b0;
			dac_word      <= 16'd0;
			dac_first     <= 1'b0;
			mcyc_stb      <= 1'b0;
			mcyc_add      <= 5'd0;
			for (int i = 0; i < 2; i = i + 1) begin
				tmr_ctl[i]  <= 32'd0;
				tmr_per[i]  <= 32'd0;
				tmr_cd[i]   <= 33'd0;
				tmr_pend[i] <= 1'b0;
				tmr_skip[i] <= 1'b0;
			end
		end else begin
			insn_done <= 1'b0;
			dac_stb   <= 1'b0;
			mcyc_stb  <= 1'b0;
			ic_used   <= 1'b0;
			retire     = 1'b0;
			dly_start  = 1'b0;
			rpt_step   = 1'b0;
			cyc_wr     = 1'b0;
			fuse_now   = 1'b0;
			rptw_took  = 1'b0;
			grp_end    = 1'b0;      // read by the end of step below
			if_next    = man[IF];

			// The initial assertion. cage31.py's set_irq_line(line, True) does, when
			// the line is driven:
			//     self.irq_state    |= mask
			//     self.r_man[IF]    |= mask          <-- this
			//     self.r_man[IF]    |= self.irq_state & 0x0F
			//     self.idling        = False
			// The three `if_next | irq_in` sites below are the re-assertions (after a
			// taken interrupt, a timer interrupt and a DMA completion). This block runs
			// every cycle, so the OR is a superset of the model's event: a level on
			// irq_in keeps the bit set while the line is asserted, as the '31's IRQ0-3
			// do. With irq_in tied low it ORs zero.
			if_next    = if_next | {28'd0, irq_in};

			// if_next is only committed in S_EOS, so a line that rises and falls inside
			// one step (a command posted and read by the same polling step) would be
			// lost. The model sets the IF bit the moment the line rises, and only the
			// program or a taken interrupt clears it, so the rise is committed here at
			// once. (An instruction writing IF on that same edge wins.)
			irq_prev <= irq_in;
			if ((irq_in & ~irq_prev) != 4'd0)
				man[IF] <= man[IF] | {28'd0, irq_in & ~irq_prev};

			// IOF's INXF0/INXF1 follow the board's lines on each edge: host_command /
			// io_iof_clear_in (bit 3) and the mailbox write / host_read_mailbox (bit 7).
			xf_prev <= xf_in;
			if (xf_in[0] != xf_prev[0]) man[IOF][3] <= xf_in[0];
			if (xf_in[1] != xf_prev[1]) man[IOF][7] <= xf_in[1];

			unique case (st_q)
			S_FETCH: begin
				start_pc <= pc;
				// MAME checks the repeat block before the fetch and returns without
				// fetching, so a wrap is a step() with no memory op and no instruction.
				// The golden state file has a line for it, so the core retires one too.
				if (man[ST][RM_BIT] && ({8'd0, pc} == man[RE] + 32'd1)) begin
					st_q <= S_RPTW;
				end else if (ic_take) begin
					// The instruction cache has this PC's word: what S_FWAIT would do with the
					// port's, a state sooner.
					ir       <= ic_word;
					pc       <= pc + 24'd1;
					ic_used  <= 1'b1;
					st_q     <= S_EXEC;
				end else begin
					i_addr   <= pc;
					i_req    <= 1'b1;
					i_we     <= 1'b0;
					i_ifetch <= 1'b1;
					st_q       <= S_FWAIT;
				end
			end

			// The wrap itself: decrement RC, then test the new value --
			// `RC -= 1; if (int32)RC >= 0 goto RS; else clear RM`.
			S_RPTW: begin
				man[RC] <= man[RC] - 32'd1;
				if ($signed(man[RC] - 32'd1) >= 0)
					pc <= man[RS][23:0];
				else
					man[ST] <= man[ST] & ~(32'd1 << RM_BIT);
				// The model returns from this branch of step() before `cycles += 1` and
				// before all three polls, so a wrap costs no cycle and cannot take an
				// interrupt. rpt_step routes it past S_EOS.
				rpt_step = 1'b1;
				retire = 1'b1;
				st_q      <= S_FETCH;
				// The next step's word, from the lookup made for it the cycle before. The
				// retire block below sends this to S_EXEC.
				if (rptw_take) begin
					start_pc <= rptw_pc;
					ir       <= ic_word0;
					pc       <= rptw_pc + 24'd1;
					ic_used  <= 1'b1;
					rptw_took = 1'b1;
				end
			end

			S_FWAIT: if (i_ack) begin
				i_req    <= 1'b0;
				i_ifetch <= 1'b0;
				ir         <= i_rdata;
				pc         <= pc + 24'd1;
				st_q       <= S_EXEC;
			end

			// Wait while a board line changed this cycle (ic_hold: run next cycle, once
			// the edge has landed in IF/IOF) or the governor says the model is ahead of
			// real time. Nothing of the instruction has run, and the model counts none
			// of these cycles.
			S_EXEC: if (ic_hold || (HOLD_EN && hold)) begin
				st_q <= S_EXEC;
			end else begin
				// The next state is a blocking variable and the retire follows from it: a
				// handler that has not moved on by the end of this state is done.
				nx = S_FETCH;

				unique case (opk)
				OP_BR_IMM: begin
					pc <= f_imm24;
					cyc_pend <= cyc_pend + 4'd3; cyc_wr = 1'b1;   // `self.cycles += 3`
					nx  = S_FETCH;
				end

				OP_LDI_IMM: begin
					man[f_dst] <= {{16{f_imm16[15]}}, f_imm16};
					man[ST]    <= ldi_flags(man[ST], f_dst,
											{{16{f_imm16[15]}}, f_imm16});
				end

				// `ldiu` is the conditional load with condition U (sign-extended, no flags),
				// in the conditional group below.

				OP_LDI_REG: begin
					man[f_dst] <= man[f_sreg];
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, man[f_sreg]);
				end

				OP_CMPI_IMM: begin
					tmp = man[f_dst] - {{16{f_imm16[15]}}, f_imm16};
					// CMPI always writes flags: there is no destination register to gate on.
					man[ST] <= sub_flags(man[ST], man[f_dst],
										 {{16{f_imm16[15]}}, f_imm16}, tmp);
				end

				OP_SUBI_IMM: begin
					tmp = man[f_dst] - {{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], man[f_dst],
											 {{16{f_imm16[15]}}, f_imm16}, tmp);
				end

				OP_BRC_IMM: begin
					if (cond_ok) begin
						pc <= pc + {{8{f_imm16[15]}}, f_imm16};
						cyc_pend <= cyc_pend + 4'd3; cyc_wr = 1'b1;
					end
				end

				OP_BRCD_IMM: begin
					// `execute_delayed`: three more steps run, and only then does the PC move.
					// The target is `pc + 2 + disp` where brc_imm's is `pc + disp`: the extra 2
					// is the '31's delay-slot accounting. A failing condition still burns the
					// three slots; only the PC write is conditional.
					if (dly_cnt != 2'd0) begin
						// A delayed branch inside a delay slot nests in the model. Not modelled;
						// flagged as unimplemented.
						unimplemented <= 1'b1;
						nx           = S_HALT;
					end else begin
						dly_cnt   <= 2'd3;
						dly_pend  <= 1'b1;
						dly_take  <= cond_ok;
						dly_start  = 1'b1;
						// `retire` is still set below: it counts the branch itself as the first of
						// the four steps the model folds into one line.
						dly_pc   <= pc + 24'd2 + {{8{f_imm16[15]}}, f_imm16};
					end
				end

				OP_RPTB_IMM: begin
					man[RS]     <= {8'd0, pc};
					man[RE]     <= {8'd0, f_imm24};
					man[ST]     <= man[ST] | (32'd1 << RM_BIT);
					cyc_pend    <= cyc_pend + 4'd3; cyc_wr = 1'b1;
				end

				OP_LDI_DIR: begin
					rk       <= RK_LDI;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_LDI_IND: begin
					rk <= RK_LDI;
					ea = ind_addr(f_ind_o, {24'd0, f_disp}, man[AR0 + f_ind_o[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_LDISTI: begin
					// LDI || STI. `par_op_store`, in order:
					//   1. src3 = r_man[(op>>16)&7]        -- latched now,
					//      because the load half may write the same register
					//   2. src2 = rmem(ind_1_def(op, op))  -- read,  AR from
					//      op[7:0], displacement 1, write-back deferred
					//   3. r_man[(op>>22)&7] = src2        -- no flags
					//   4. wmem(ind_1(op, op>>8), src3)    -- write, AR from
					//      op[15:8], displacement 1, write-back immediate
					//   5. update_def()                    -- commit (2)
					// The deferred write lands last, so if both halves name the same ARn the
					// deferred value wins.
					par_flt  <= 1'b0;
					par_mpy  <= 1'b0;
					par_src3 <= man[{2'd0, ir[18:16]}];
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[2:0]};
						def_val  <= ind_newar(ir[7:0], 32'd1,
											  man[AR0 + ir[2:0]],
											  man[IR0], man[IR1]);
					end else begin
						def_pend <= 1'b0;
					end
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_PAR_RD;
				end

				OP_NOP_REG: ;                       // `i_nop_reg`: pass

				OP_CMPI_REG: begin
					tmp = man[f_dst] - man[f_sreg];
					man[ST] <= sub_flags(man[ST], man[f_dst], man[f_sreg],
										 tmp);
				end

				// AND / ANDN / OR with an immediate. `src_int(mode="imm")` is `s16(op)`:
				// sign-extended even for the logical ops, as in MAME.
				OP_AND_IMM: begin
					tmp = man[f_dst] & {{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				OP_ANDN_IMM: begin
					tmp = man[f_dst] & ~{{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				OP_OR_IMM: begin
					tmp = man[f_dst] | {{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end

				// Conditional loads touch no flags: MAME assigns the register directly
				// rather than through op_ldi, because the condition it just tested lives in
				// those flags.
				OP_LDIEQ_IMM, OP_LDINE_IMM, OP_LDIHI_IMM, OP_LDIGE_IMM,
				OP_LDIU_IMM, OP_LDIHS_IMM, OP_LDILO_IMM: begin
					if (cond_ok)
						man[f_dst] <= {{16{f_imm16[15]}}, f_imm16};
				end

				OP_CALL_IMM: begin
					// SP is pre-incremented, the return address is the PC already past the
					// call, and only then does the PC move.
					man[SP]   <= man[SP] + 32'd1;
					i_addr  <= man[SP][23:0] + 24'd1;
					i_wdata <= {8'd0, pc};
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					pc        <= f_imm24;
					cyc_pend  <= cyc_pend + 4'd3; cyc_wr = 1'b1;
					nx       = S_WR;
				end

				OP_RETSC_REG: begin
					// Conditional: when it does not take, there is no memory access at all
					// (the golden stream checks it).
					if (cond_ok) begin
						i_addr <= man[SP][23:0];
						i_req  <= 1'b1;
						i_we   <= 1'b0;
						cyc_pend <= cyc_pend + 4'd3; cyc_wr = 1'b1;
						nx      = S_RET;
					end
				end

				OP_STI_DIR: begin
					i_addr  <= {man[DP][7:0], f_imm16};
					i_wdata <= man[f_dst];
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end

				OP_STI_IND: begin
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr  <= ea[23:0];
					i_wdata <= man[f_dst];
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end

				OP_ADDI_IMM: begin
					tmp = man[f_dst] + {{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], man[f_dst],
											 {{16{f_imm16[15]}}, f_imm16},
											 tmp);
				end
				OP_ADDI_REG: begin
					tmp = man[f_dst] + man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], man[f_dst],
											 man[f_sreg], tmp);
				end
				OP_ADDI3_REGREG: begin
					// `three_srcs("regreg")` is r_man[(op>>8)&31] and r_man[op&31]: two
					// registers, no memory.
					tmp = man[ir[12:8]] + man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], man[ir[12:8]],
											 man[f_sreg], tmp);
				end
				OP_OR_REG: begin
					tmp = man[f_dst] | man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end

				OP_LSH_IMM: begin
					tmp = lsh_res(man[f_dst], f_imm16[6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= lsh_flags(man[ST], man[f_dst],
											 f_imm16[6:0], tmp);
				end
				OP_LSH3_REGREG: begin
					tmp = lsh_res(man[ir[12:8]], man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= lsh_flags(man[ST], man[ir[12:8]],
											 man[f_sreg][6:0], tmp);
				end

				OP_MPYI_IMM: begin
					prod = mpyi_prod(man[f_dst],
									 {{16{f_imm16[15]}}, f_imm16});
					man[f_dst] <= prod[31:0];
					if (f_dst < 5'd8)
						man[ST] <= mpyi_flags(man[ST], prod);
				end
				OP_MPYI3_REGREG: begin
					prod = mpyi_prod(man[ir[12:8]], man[f_sreg]);
					man[f_dst] <= prod[31:0];
					if (f_dst < 5'd8)
						man[ST] <= mpyi_flags(man[ST], prod);
				end

				OP_BRC_REG: begin
					if (cond_ok) begin
						pc <= man[f_sreg][23:0];
						cyc_pend <= cyc_pend + 4'd3; cyc_wr = 1'b1;
					end
				end
				OP_BRCD_REG: begin
					if (dly_cnt != 2'd0) begin
						unimplemented <= 1'b1;
						nx           = S_HALT;
					end else begin
						// The register form takes its target as-is; only the immediate form
						// carries the +2.
						dly_cnt   <= 2'd3;
						dly_pend  <= 1'b1;
						dly_take  <= cond_ok;
						dly_pc    <= man[f_sreg][23:0];
						dly_start  = 1'b1;
					end
				end

				OP_PUSH: begin
					man[SP]   <= man[SP] + 32'd1;
					i_addr  <= man[SP][23:0] + 24'd1;
					i_wdata <= man[f_dst];
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end
				OP_POP: begin
					rk       <= RK_POP;
					i_addr <= man[SP][23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_ADDI_DIR: begin
					rk       <= RK_ADDI;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				OP_CMPI_DIR: begin
					rk       <= RK_CMPI;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				OP_LDILS_DIR: begin
					// A conditional load always reads; only the write-back is conditional (the
					// model's own comment). So the access happens whatever the flags say.
					rk       <= RK_LDCOND;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_STISTI: begin
					// STI || STI: two writes and no load. The first pointer is deferred and the
					// second immediate, and the deferred commit lands last, as in ldisti.
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[10:8]};
						def_val  <= ind_newar(ir[15:8], 32'd1,
											  man[AR0 + ir[10:8]],
											  man[IR0], man[IR1]);
					end else begin
						def_pend <= 1'b0;
					end
					i_addr  <= ea[23:0];
					i_wdata <= man[{2'd0, ir[18:16]}];
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_STI2;
				end

				OP_ANDN_REG: begin
					tmp = man[f_dst] & ~man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end

				OP_LDF_IMM: begin
					fp = short2fp(f_imm16);
					exp[{2'd0, ir[18:16]}] <= fp[39:32];
					man[{2'd0, ir[18:16]}] <= fp[31:0];
					man[ST] <= ldf_flags(man[ST], fp[39:32], fp[31:0]);
				end

				OP_LDF_DIR: begin
					rk       <= RK_LDF;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_STF_IND: begin
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr  <= ea[23:0];
					i_wdata <= fp2long(exp[{2'd0, ir[18:16]}],
										 man[{2'd0, ir[18:16]}]);
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end

				OP_PUSHF: begin
					man[SP]   <= man[SP] + 32'd1;
					i_addr  <= man[SP][23:0] + 24'd1;
					i_wdata <= fp2long(exp[{2'd0, ir[18:16]}],
										 man[{2'd0, ir[18:16]}]);
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end

				// Three-operand indirect modes. three_srcs uses ind_1, displacement one,
				// not the instruction's low byte as ind_d does for the single-operand forms.
				OP_CMPI3_INDIND: begin
					t3 <= T3_CMPI;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[10:8]};
						def_val  <= ind_newar(ir[15:8], 32'd1,
											  man[AR0 + ir[10:8]],
											  man[IR0], man[IR1]);
					end else def_pend <= 1'b0;
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD1;
				end

				OP_MPYI3_INDREG, OP_OR3_INDREG: begin
					// indreg: the first source is indirect, the second a register, and nothing
					// is deferred.
					t3 <= (opk == OP_MPYI3_INDREG) ? T3_MPYI : T3_OR;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;      // second source is man[ir[4:0]]
				end

				OP_XOR_IMM: begin
					tmp = man[f_dst] ^ {{16{f_imm16[15]}}, f_imm16};
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end

				// The single-operand indirect forms use ind_d: the displacement is the
				// instruction's low byte.
				OP_ADDI_IND, OP_CMPI_IND, OP_LDF_IND: begin
					rk <= (opk == OP_ADDI_IND) ? RK_ADDI :
						  (opk == OP_CMPI_IND) ? RK_CMPI : RK_LDF;
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_STF_DIR: begin
					i_addr  <= {man[DP][7:0], f_imm16};
					i_wdata <= fp2long(exp[{2'd0, ir[18:16]}],
										 man[{2'd0, ir[18:16]}]);
					i_req   <= 1'b1;
					i_we    <= 1'b1;
					nx       = S_WR;
				end

				OP_POPF: begin
					rk       <= RK_POPF;
					i_addr <= man[SP][23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				OP_RETIC_REG: begin
					// As retsc, plus setting GIE, and then check_irqs() from inside the
					// handler: the one place the model asks for interrupts before that step's
					// poll_timers/poll_dma rather than after. eos_retic carries that to S_EOS,
					// which then sees the IF value from before the polls.
					if (cond_ok) begin
						i_addr  <= man[SP][23:0];
						i_req   <= 1'b1;
						i_we    <= 1'b0;
						cyc_pend  <= cyc_pend + 4'd3; cyc_wr = 1'b1;
						nx       = S_RET;
						retic     <= 1'b1;
						eos_retic <= 1'b1;
					end
				end

				// Float ALU forms. do_three's float path converts both sources with long2fp
				// into TEMP1/TEMP2 whatever the mode; the register forms read the registers
				// directly, so those writes are not transcribed (the bench does not compare
				// slots 32-34).
				OP_MPYF3_REGREG, OP_SUBF3_REGREG,
				OP_ADDF3_REGREG: begin
					fp_op      <= (opk == OP_MPYF3_REGREG) ? FP_MPY :
								  (opk == OP_ADDF3_REGREG) ? FP_ADD : FP_SUB;
					fa_e       <= exp[{2'd0, ir[10:8]}];
					fa_m       <= man[{2'd0, ir[10:8]}];
					fb_e       <= exp[{2'd0, ir[2:0]}];
					fb_m       <= man[{2'd0, ir[2:0]}];
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end

				// `mpyf(dreg7, dreg7, s)`: the destination is also the first source, as
				// do_single does for each of addf/subf/mpyf.
				OP_MPYF_REG, OP_ADDF_REG, OP_SUBF_REG: begin
					fp_op      <= (opk == OP_MPYF_REG) ? FP_MPY :
								  (opk == OP_ADDF_REG) ? FP_ADD : FP_SUB;
					fa_e       <= exp[{2'd0, ir[18:16]}];
					fa_m       <= man[{2'd0, ir[18:16]}];
					fb_e       <= exp[{2'd0, ir[2:0]}];
					fb_m       <= man[{2'd0, ir[2:0]}];
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end

				// `_copy(dreg7, op & 7)` then clear NZVUF and or_nzf. No arithmetic, so no
				// ALU pass.
				OP_LDF_REG: begin
					exp[{2'd0, ir[18:16]}] <= exp[{2'd0, ir[2:0]}];
					man[{2'd0, ir[18:16]}] <= man[{2'd0, ir[2:0]}];
					man[ST] <= ldf_flags(man[ST], exp[{2'd0, ir[2:0]}],
												  man[{2'd0, ir[2:0]}]);
				end

				// indreg: source 1 is indirect (ind_1, displacement one), source 2 a
				// register. Nothing deferred.
				OP_MPYF3_INDREG: begin
					t3 <= T3_FLT;
					fp_manonly <= 1'b0;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// regind: the other way round, and the indirect field is the instruction's
				// low byte here, not bits 15:8.
				OP_ADDF3_REGIND: begin
					t3 <= T3_FLT;
					fp_manonly <= 1'b0;
					def_pend <= 1'b0;
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0]))
						man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
														man[AR0 + ir[2:0]],
														man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// MPYF || ADDF, variant 3: two indirect reads with the first pointer
				// deferred, then
				//     mpyf(TEMP3, TEMP1, r1)
				//     addf(d_a,  r2,    TEMP2)
				//     _copy(d_m, TEMP3)
				// in that order, so the flags that survive are the add's: the multiply's
				// V/UF are cleared by the add while its LV/LUF stay, being sticky.
				OP_MPYADDF_3: begin
					fp_manonly <= 1'b0;
					ma_dm <= {4'd0, ir[23]};            // R0 or R1
					ma_da <= {3'd0, 1'b1, ir[22]};      // R2 or R3
					ma_r1 <= ir[21:19];
					ma_r2 <= ir[18:16];
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[10:8]};
						def_val  <= ind_newar(ir[15:8], 32'd1,
											  man[AR0 + ir[10:8]],
											  man[IR0], man[IR1]);
					end else def_pend <= 1'b0;
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_MA1;
				end

				// `ash`: the arithmetic shift. The flags are lsh's everywhere except a
				// count below -32, where ash takes the carry from the sign bit and lsh none.
				OP_ASH_IMM: begin
					tmp = ash_res(man[f_dst], f_imm16[6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ash_flags(man[ST], man[f_dst],
											 f_imm16[6:0], tmp);
				end

				// `callc` is `call` with a condition and a register target. When it does
				// not take there is no memory access at all.
				OP_CALLC_REG: begin
					if (cond_ok) begin
						man[SP]   <= man[SP] + 32'd1;
						i_addr  <= man[SP][23:0] + 24'd1;
						i_wdata <= {8'd0, pc};
						i_req   <= 1'b1;
						i_we    <= 1'b1;
						pc        <= man[f_sreg][23:0];
						cyc_pend  <= cyc_pend + 4'd3; cyc_wr = 1'b1;
						nx       = S_WR;
					end
				end

				OP_AND3_REGREG: begin
					tmp = man[ir[12:8]] & man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end

				// `float`: the integer goes into the destination's mantissa and is
				// converted in place, so the ALU gets it as operand 1 with no exponent.
				// `src_int("reg")` is `r_man[op & 31]`, five bits, not the three a float
				// source would use.
				OP_FLOAT_REG: begin
					fp_op      <= FP_I2F;
					fa_e       <= 8'd0;
					fa_m       <= man[f_sreg];
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx        = S_FP;
				end

				// `fix`: a float in, an integer out. The destination is a full five-bit
				// register whose exponent is left alone (`setreg(dreg31, r_man[TEMP1])`
				// takes the mantissa only). The flags are written only when the destination
				// is R0-R7; FP_F2IQ is the flagless variant.
				OP_FIX_REG: begin
					fp_op      <= (f_dst < 5'd8) ? FP_F2I : FP_F2IQ;
					fa_e       <= exp[{2'd0, ir[2:0]}];
					fa_m       <= man[{2'd0, ir[2:0]}];
					fp_dst     <= {1'b0, f_dst};
					fp_manonly <= 1'b1;
					fp_stage   <= 2'd0;
					nx        = S_FP;
				end

				// The float single-operand direct forms. `cmpf` is a subf into TEMP2: it
				// exists for its flags and its result is thrown away. addf/subf are
				// `addf(dreg7, dreg7, s)` and `subf(dreg7, dreg7, s)`.
				OP_MPYF_DIR, OP_CMPF_DIR, OP_ADDF_DIR, OP_SUBF_DIR: begin
					rk         <= RK_FOP;
					rk_mem_a   <= 1'b0;
					fp_op      <= (opk == OP_MPYF_DIR) ? FP_MPY :
								  (opk == OP_ADDF_DIR) ? FP_ADD : FP_SUB;
					fa_e       <= exp[{2'd0, ir[18:16]}];
					fa_m       <= man[{2'd0, ir[18:16]}];
					fp_dst     <= (opk == OP_CMPF_DIR) ? 6'd33       // TEMP2
													   : {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					i_addr   <= {man[DP][7:0], f_imm16};
					i_req    <= 1'b1;
					i_we     <= 1'b0;
					nx        = S_RD;
				end

				// ...and the float single-operand indirect forms, which use ind_d: the
				// displacement is the instruction's low byte. negf and fix take the word as
				// their only operand; addf/subf/mpyf take it as the second, with the
				// destination register as the first.
				OP_ADDF_IND, OP_SUBF_IND, OP_MPYF_IND,
				OP_NEGF_IND, OP_FIX_IND, OP_CMPF_IND: begin
					rk         <= RK_FOP;
					rk_mem_a   <= (opk == OP_NEGF_IND) || (opk == OP_FIX_IND);
					fp_op      <= (opk == OP_ADDF_IND) ? FP_ADD :
								  (opk == OP_SUBF_IND) ? FP_SUB :
								  (opk == OP_CMPF_IND) ? FP_SUB :
								  (opk == OP_MPYF_IND) ? FP_MPY :
								  (opk == OP_NEGF_IND) ? FP_NEG :
								  ((f_dst < 5'd8) ? FP_F2I : FP_F2IQ);
					fa_e       <= exp[{2'd0, ir[18:16]}];
					fa_m       <= man[{2'd0, ir[18:16]}];
					// `cmpf` is a subf into TEMP2: it exists for its flags and its result is
					// thrown away.
					fp_dst     <= (opk == OP_FIX_IND)  ? {1'b0, f_dst} :
								  (opk == OP_CMPF_IND) ? 6'd33
													   : {3'd0, ir[18:16]};
					fp_manonly <= (opk == OP_FIX_IND);
					fp_stage   <= 2'd0;
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				// `cmpi3` indreg: flags only, no destination write.
				// `addi3` indreg: the same read, an ordinary destination.
				OP_CMPI3_INDREG, OP_ADDI3_INDREG: begin
					t3 <= (opk == OP_CMPI3_INDREG) ? T3_CMPIR : T3_ADDIR;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// Handlers the game's commands reach, grouped by shape (lockstepped against
				// tools/gen_c31_cosim.py: the real 68020 program driving cage31.py).

				// Conditional loads (the condition is in the name).
				// `ldi<cond>_reg`: `setreg(dreg31, r_man[op & 31])` if the condition holds.
				// No flags, no memory.
				OP_LDIHI_REG, OP_LDINE_REG, OP_LDILT_REG,
				OP_LDIHS_REG: begin
					if (cond_ok)
						man[f_dst] <= man[f_sreg];
				end
				// `ldi<cond>_dir`: the read happens whatever the condition (the model's
				// `val = src_int(...)` comes first); only the write-back is conditional.
				// Same read kind as ldils_dir.
				OP_LDINE_DIR, OP_LDIEQ_DIR: begin
					rk     <= RK_LDCOND;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				// `ldi<cond>_ind`: likewise always read, through ind_d.
				OP_LDINE_IND, OP_LDILT_IND: begin
					rk <= RK_LDCOND;
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				// `ldf<cond>_imm`: `short2fp(dreg7, op)` if the condition holds. No flags.
				OP_LDFGE_IMM, OP_LDFGT_IMM, OP_LDFLT_IMM: begin
					if (cond_ok) begin
						fp = short2fp(f_imm16);
						exp[{2'd0, ir[18:16]}] <= fp[39:32];
						man[{2'd0, ir[18:16]}] <= fp[31:0];
					end
				end
				// `ldf<cond>_dir`: the float form evaluates the condition first, and a false
				// one makes no access at all, the opposite of ldi<cond> (the model's own
				// comment: the difference is a real memory access). The golden stream
				// checks it.
				OP_LDFLT_DIR, OP_LDFGE_DIR: begin
					if (cond_ok) begin
						rk     <= RK_LDFC;
						i_addr <= {man[DP][7:0], f_imm16};
						i_req  <= 1'b1;
						i_we   <= 1'b0;
						nx      = S_RD;
					end
				end

				// Integer, single operand.
				// `op_ash(dreg31, dst, r_man[op & 31])`: the count is the source register's
				// low seven bits, signed.
				OP_ASH_REG: begin
					tmp = ash_res(man[f_dst], man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ash_flags(man[ST], man[f_dst],
											 man[f_sreg][6:0], tmp);
				end
				// `op_lsh(dreg31, dst, r_man[op & 31])`: ASH_REG's shape with the logical
				// shift.
				OP_LSH_REG: begin
					tmp = lsh_res(man[f_dst], man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= lsh_flags(man[ST], man[f_dst],
											 man[f_sreg][6:0], tmp);
				end
				OP_ABSI_REG: begin
					tmp = absi_res(man[f_sreg]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= absi_flags(man[ST], tmp);
				end
				// `negi` is `op_subi(dreg31, 0, src)`.
				OP_NEGI_REG: begin
					tmp = 32'd0 - man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], 32'd0, man[f_sreg], tmp);
				end
				// `subri` is `op_subi(dreg31, src, dst)`: reversed.
				OP_SUBRI_IMM: begin
					tmp = {{16{f_imm16[15]}}, f_imm16} - man[f_dst];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST],
											 {{16{f_imm16[15]}}, f_imm16},
											 man[f_dst], tmp);
				end
				// `tstb`: flags from `dst & src`, no destination write and no `dreg < 8`
				// gate.
				OP_TSTB_IMM: begin
					man[ST] <= tst_flags(man[ST],
										 man[f_dst] & {{16{f_imm16[15]}}, f_imm16});
				end
				// and_ind uses RK_AND, as and_dir does.
				OP_SUBI_IND, OP_SUBRI_IND, OP_MPYI_IND,
				OP_AND_IND, OP_NEGI_IND: begin
					rk <= (opk == OP_SUBI_IND)  ? RK_SUBI  :
						  (opk == OP_SUBRI_IND) ? RK_SUBRI :
						  (opk == OP_AND_IND)   ? RK_AND   :
						  (opk == OP_NEGI_IND)  ? RK_NEGI  : RK_MPYI;
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				OP_AND_DIR, OP_TSTB_DIR: begin
					rk     <= (opk == OP_AND_DIR) ? RK_AND : RK_TSTB;
					i_addr <= {man[DP][7:0], f_imm16};
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end

				// Integer, three operand.
				// `op_ash(dreg31, s1, s2)`: s1 is shifted, s2 is the count.
				OP_ASH3_REGREG: begin
					tmp = ash_res(man[ir[12:8]], man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ash_flags(man[ST], man[ir[12:8]],
											 man[f_sreg][6:0], tmp);
				end
				// `op_subi(dreg31, s1, s2)` with `three_srcs("regreg")`: src1 is bits 12:8,
				// src2 bits 4:0, and it is src1 - src2.
				OP_SUBI3_REGREG: begin
					tmp = man[ir[12:8]] - man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], man[ir[12:8]],
											 man[f_sreg], tmp);
				end
				OP_XOR3_REGREG: begin
					tmp = man[ir[12:8]] ^ man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				// indreg: the shifted value is the indirect word and the count the
				// register (three_srcs returns (mem, reg)).
				OP_LSH3_INDREG, OP_ASH3_INDREG: begin
					t3 <= (opk == OP_LSH3_INDREG) ? T3_LSH : T3_ASH;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// Float.
				// `cmpf` is `subf(TEMP2, dreg7, s)`: flags only.
				OP_CMPF_REG: begin
					fp_op      <= FP_SUB;
					fa_e       <= exp[{2'd0, ir[18:16]}];
					fa_m       <= man[{2'd0, ir[18:16]}];
					fb_e       <= exp[{2'd0, ir[2:0]}];
					fb_m       <= man[{2'd0, ir[2:0]}];
					fp_dst     <= 6'd33;                  // TEMP2
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end
				// The float immediate forms: `short2fp` into TEMP1, then the op
				// with the destination as the first operand (cmpf into TEMP2).
				OP_CMPF_IMM, OP_MPYF_IMM, OP_ADDF_IMM, OP_SUBF_IMM: begin
					fp = short2fp(f_imm16);
					fp_op      <= (opk == OP_MPYF_IMM) ? FP_MPY :
								  (opk == OP_ADDF_IMM) ? FP_ADD : FP_SUB;
					fa_e       <= exp[{2'd0, ir[18:16]}];
					fa_m       <= man[{2'd0, ir[18:16]}];
					fb_e       <= fp[39:32];
					fb_m       <= fp[31:0];
					fp_dst     <= (opk == OP_CMPF_IMM) ? 6'd33
													   : {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end
				// `float_ind`: `float_reg`'s ALU pass with a memory source (the
				// word is an integer).
				OP_FLOAT_IND: begin
					rk         <= RK_FOPI;
					fp_op      <= FP_I2F;
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_RD;
				end
				// `subf3` indreg: `subf(dreg7, TEMP1, op & 7)`, memory minus
				// register. `mpyf3_indreg`'s read and slots, with the subtract.
				OP_SUBF3_INDREG: begin
					t3 <= T3_FLT;
					fp_manonly <= 1'b0;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// LDF || STF: `par_op_store` with kind ("ldf", True), ldisti's
				// sequence with `par_flt` choosing the float load half. src3 is
				// fp2long(r[(op>>16)&7]), latched now.
				OP_LDFSTF: begin
					par_flt  <= 1'b1;
					par_mpy  <= 1'b0;
					par_src3 <= fp2long(exp[{2'd0, ir[18:16]}],
										man[{2'd0, ir[18:16]}]);
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[2:0]};
						def_val  <= ind_newar(ir[7:0], 32'd1,
											  man[AR0 + ir[2:0]],
											  man[IR0], man[IR1]);
					end else begin
						def_pend <= 1'b0;
					end
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_PAR_RD;
				end

				// The rest of the handlers the program can reach, as found by
				// tools/scan_c31_handlers.py (which walks the program's control
				// flow and the code addresses its tables hold). Each is
				// lockstepped against a directed program on the model
				// (tools/gen_c31_directed.py, run by tb_c31_cosim).

				// Integer, register.
				// `op_subi(dreg31, dst, src)`
				OP_SUBI_REG: begin
					tmp = man[f_dst] - man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], man[f_dst], man[f_sreg], tmp);
				end
				// `op_logic(dreg31, dst & src)` / `dst ^ src`
				OP_AND_REG: begin
					tmp = man[f_dst] & man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				OP_XOR_REG: begin
					tmp = man[f_dst] ^ man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				// `op_subc`, the division step: an unsigned compare, no flags.
				OP_SUBC_REG: begin
					if (man[f_dst] >= man[f_sreg])
						tmp = ((man[f_dst] - man[f_sreg]) << 1) | 32'd1;
					else
						tmp = man[f_dst] << 1;
					man[f_dst] <= tmp;
				end

				// Repeat single. `_rpts(r_man[op & 31])`: RC = count, RS = RE =
				// the PC already past the rpts, RM on, 3 cycles, and
				// `self.delayed = True` (`rpts_dly`, at its declaration). The
				// wrap is rptb's (S_RPTW), so the one instruction runs RC + 1
				// times.
				OP_RPTS_REG: begin
					man[RC]  <= man[f_sreg];
					man[RS]  <= {8'd0, pc};
					man[RE]  <= {8'd0, pc};
					man[ST]  <= man[ST] | (32'd1 << RM_BIT);
					cyc_pend <= cyc_pend + 4'd3; cyc_wr = 1'b1;
					rpts_dly <= 1'b1;
				end

				// Integer, three operand, one source in memory.
				// indreg: `three_srcs` returns (word via ind_1(op, op>>8),
				// r_man[op & 31]); nothing deferred.
				OP_SUBI3_INDREG, OP_AND3_INDREG, OP_ANDN3_INDREG: begin
					t3 <= (opk == OP_SUBI3_INDREG) ? T3_SUBIR :
						  (opk == OP_AND3_INDREG)  ? T3_AND   : T3_ANDN;
					def_pend <= 1'b0;
					ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[15:8]))
						man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
														 man[AR0 + ir[10:8]],
														 man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end
				// regind: (r_man[(op>>8) & 31], word via ind_1(op, op)); the
				// pointer is the instruction's low byte, as for `addf3_regind`.
				OP_ADDI3_REGIND: begin
					t3 <= T3_ADDRI;
					def_pend <= 1'b0;
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0]))
						man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
														man[AR0 + ir[2:0]],
														man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// Conditional float loads.
				// `ldf<cond>_reg`: `_copy(dreg7, op & 7)` if the condition
				// holds. No flags.
				OP_LDFLT_REG, OP_LDFGT_REG: begin
					if (cond_ok) begin
						exp[{2'd0, ir[18:16]}] <= exp[{2'd0, ir[2:0]}];
						man[{2'd0, ir[18:16]}] <= man[{2'd0, ir[2:0]}];
					end
				end
				// `ldf<cond>_ind`: the pointer update happens either way
				// (`self.ind_d(op, op >> 8)`), the read only when the condition
				// holds, unlike `ldi<cond>_ind`, which always reads.
				OP_LDFLT_IND, OP_LDFGT_IND: begin
					ea = ind_addr(f_ind_o, {24'd0, f_disp},
								  man[AR0 + f_ind_o[2:0]], man[IR0], man[IR1]);
					if (ind_writes(f_ind_o))
						man[AR0 + f_ind_o[2:0]] <=
							ind_newar(f_ind_o, {24'd0, f_disp},
									  man[AR0 + f_ind_o[2:0]],
									  man[IR0], man[IR1]);
					if (cond_ok) begin
						rk     <= RK_LDFC;
						i_addr <= ea[23:0];
						i_req  <= 1'b1;
						i_we   <= 1'b0;
						nx      = S_RD;
					end
				end

				// Float, register.
				// `negf(dreg7, op & 7)`: `negf_ind`'s ALU pass with no read.
				OP_NEGF_REG: begin
					fp_op      <= FP_NEG;
					fa_e       <= exp[{2'd0, ir[2:0]}];
					fa_m       <= man[{2'd0, ir[2:0]}];
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end
				// `op_absf(dreg7, op & 7)`: zero reads as man 0; a negative
				// mantissa is negated, except 0x80000000 (-1 x 2^e), whose
				// magnitude is +1 x 2^(e+1): man 0, exp + 1. N/Z/V/UF cleared,
				// then N and Z from the result.
				OP_ABSF_REG: begin
					fx_e = exp[{2'd0, ir[2:0]}];
					fx_m = man[{2'd0, ir[2:0]}];
					if (fx_e == 8'h80)
						fx_m = 32'd0;
					else if (fx_m[31]) begin
						if (fx_m != 32'h8000_0000) fx_m = 32'd0 - fx_m;
						else begin
							fx_m = 32'd0;
							fx_e = fx_e + 8'd1;      // s8(fexp + 1): wraps
						end
					end
					exp[{2'd0, ir[18:16]}] <= fx_e;
					man[{2'd0, ir[18:16]}] <= fx_m;
					man[ST] <= ldf_flags(man[ST], fx_e, fx_m);
				end
				// `rnd`: round the mantissa to 24 bits. Only N, V and UF are
				// cleared (Z survives); N is set from the result, and UF and LUF
				// for an exponent of -128. At the top of the range the exponent
				// steps up; at exponent 127 it saturates with V and LV.
				OP_RND_REG: begin
					fx_e  = exp[{2'd0, ir[2:0]}];
					fx_m  = man[{2'd0, ir[2:0]}];
					fx_st = man[ST] & ~((32'd1 << N_BIT) | (32'd1 << V_BIT) |
										(32'd1 << UF_BIT));
					if ($signed(fx_m) < $signed(32'h7FFF_FF80)) begin
						fx_m = (fx_m + 32'h80) & 32'hFFFF_FF00;
						fx_st[N_BIT] = fx_m[31];
						if (fx_e == 8'h80) begin
							fx_st[UF_BIT]  = 1'b1;
							fx_st[LUF_BIT] = 1'b1;
						end
					end else if ($signed(fx_e) < $signed(8'sd127)) begin
						fx_m = (fx_m + 32'h80) & 32'h7FFF_FF00;
						fx_e = fx_e + 8'd1;
						fx_st[N_BIT] = fx_m[31];
						if (fx_e == 8'h80) begin
							fx_st[UF_BIT]  = 1'b1;
							fx_st[LUF_BIT] = 1'b1;
						end
					end else begin
						fx_m = 32'h7FFF_FF00;
						fx_st[V_BIT]  = 1'b1;
						fx_st[LV_BIT] = 1'b1;
					end
					exp[{2'd0, ir[18:16]}] <= fx_e;
					man[{2'd0, ir[18:16]}] <= fx_m;
					man[ST] <= fx_st;
				end
				// `lde`: the exponent only; a zero exponent (-128) zeroes the
				// mantissa as well. No flags.
				OP_LDE_REG: begin
					exp[{2'd0, ir[18:16]}] <= exp[{2'd0, ir[2:0]}];
					if (exp[{2'd0, ir[2:0]}] == 8'h80)
						man[{2'd0, ir[18:16]}] <= 32'd0;
				end
				// `subrf_imm`: `subf(dreg7, TEMP1, dreg7)`, the immediate minus
				// the register.
				OP_SUBRF_IMM: begin
					fp = short2fp(f_imm16);
					fp_op      <= FP_SUB;
					fa_e       <= fp[39:32];
					fa_m       <= fp[31:0];
					fb_e       <= exp[{2'd0, ir[18:16]}];
					fb_m       <= man[{2'd0, ir[18:16]}];
					fp_dst     <= {3'd0, ir[18:16]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
					nx      = S_FP;
				end
				// `mpyf3` regind: `mpyf(dreg7, (op >> 8) & 7, TEMP2)`, the word
				// from the low byte's pointer. `addf3_regind`'s read and slots,
				// with the multiply.
				OP_MPYF3_REGIND: begin
					t3 <= T3_FLT;
					fp_manonly <= 1'b0;
					def_pend <= 1'b0;
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0]))
						man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
														man[AR0 + ir[2:0]],
														man[IR0], man[IR1]);
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_3RD2;
				end

				// MPYF3 || STF: `par_op_store` with ("mpyf3", True). ldfstf's
				// sequence, but the load half is a multiply: the word times
				// register (op >> 19) & 7 into (op >> 22) & 7, with mpyf's
				// flags, in S_FP after the store (the product touches no
				// register the store uses).
				OP_MPYF3STF: begin
					par_flt  <= 1'b0;
					par_mpy  <= 1'b1;
					par_src3 <= fp2long(exp[{2'd0, ir[18:16]}],
										man[{2'd0, ir[18:16]}]);
					ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
								  man[IR0], man[IR1]);
					if (ind_writes(ir[7:0])) begin
						def_pend <= 1'b1;
						def_reg  <= AR0_R + {2'd0, ir[2:0]};
						def_val  <= ind_newar(ir[7:0], 32'd1,
											  man[AR0 + ir[2:0]],
											  man[IR0], man[IR1]);
					end else begin
						def_pend <= 1'b0;
					end
					i_addr <= ea[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					nx      = S_PAR_RD;
				end

				default: begin
					unimplemented <= 1'b1;
					nx           = S_HALT;
				end
				endcase

				// Everything that finishes inside S_EXEC retires here; handlers
				// that issue an access retire in their own state, and S_HALT
				// never retires.
				st_q <= nx;
				if (nx == S_FETCH) retire = 1'b1;
			end

			S_RD: if (i_ack) begin
				i_req <= 1'b0;
				unique case (rk)
				RK_LDI: begin
					man[f_dst] <= i_rdata;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, i_rdata);
				end
				RK_ADDI: begin
					tmp = man[f_dst] + i_rdata;
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], man[f_dst],
											 i_rdata, tmp);
				end
				RK_CMPI: begin
					tmp = man[f_dst] - i_rdata;
					man[ST] <= sub_flags(man[ST], man[f_dst], i_rdata, tmp);
				end
				RK_LDCOND: begin
					// no flags, and the write-back is the conditional part
					if (cond_ok) man[f_dst] <= i_rdata;
				end
				RK_LDF: begin
					fp = long2fp(i_rdata);
					exp[{2'd0, ir[18:16]}] <= fp[39:32];
					man[{2'd0, ir[18:16]}] <= fp[31:0];
					man[ST] <= ldf_flags(man[ST], fp[39:32], fp[31:0]);
				end
				RK_POPF: begin
					fp = long2fp(i_rdata);
					man[SP] <= man[SP] - 32'd1;
					exp[{2'd0, ir[18:16]}] <= fp[39:32];
					man[{2'd0, ir[18:16]}] <= fp[31:0];
					man[ST] <= ldf_flags(man[ST], fp[39:32], fp[31:0]);
				end
				RK_POP: begin
					man[f_dst] <= i_rdata;
					man[SP]    <= man[SP] - 32'd1;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, i_rdata);
				end
				// The word is long2fp'd and lands in whichever operand slot the
				// handler said. Everything else the ALU needs was latched at
				// decode.
				RK_FOP: begin
					if (rk_mem_a) begin
						fa_e <= i_rdata[31:24];
						fa_m <= {i_rdata[23:0], 8'd0};
					end else begin
						fb_e <= i_rdata[31:24];
						fb_m <= {i_rdata[23:0], 8'd0};
					end
				end
				RK_FOPI: begin                  // `float`: the word raw
					fa_e <= 8'd0;
					fa_m <= i_rdata;
				end
				RK_LDFC: begin                  // ldf<cond>: no flags
					fp = long2fp(i_rdata);
					exp[{2'd0, ir[18:16]}] <= fp[39:32];
					man[{2'd0, ir[18:16]}] <= fp[31:0];
				end
				RK_SUBI: begin                  // op_subi(d, dst, mem)
					tmp = man[f_dst] - i_rdata;
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], man[f_dst], i_rdata, tmp);
				end
				RK_SUBRI: begin                 // op_subi(d, mem, dst)
					tmp = i_rdata - man[f_dst];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], i_rdata, man[f_dst], tmp);
				end
				RK_MPYI: begin
					prod = mpyi_prod(man[f_dst], i_rdata);
					man[f_dst] <= prod[31:0];
					if (f_dst < 5'd8)
						man[ST] <= mpyi_flags(man[ST], prod);
				end
				RK_AND: begin
					tmp = man[f_dst] & i_rdata;
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				RK_TSTB: begin
					man[ST] <= tst_flags(man[ST], man[f_dst] & i_rdata);
				end
				RK_NEGI: begin                  // op_subi(d, 0, mem)
					tmp = 32'd0 - i_rdata;
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], 32'd0, i_rdata, tmp);
				end
				endcase
				if (rk == RK_FOP || rk == RK_FOPI) begin
					st_q   <= S_FP;
				end else begin
					retire = 1'b1;
					st_q   <= S_FETCH;
				end
			end

			// indind: the first source has arrived; issue the second from the
			// instruction's low byte, with an immediate write-back.
			S_3RD1: if (i_ack) begin
				s3_a <= i_rdata;
				ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
							  man[IR0], man[IR1]);
				if (ind_writes(ir[7:0]))
					man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
													man[AR0 + ir[2:0]],
													man[IR0], man[IR1]);
				i_addr <= ea[23:0];
				i_req  <= 1'b1;
				st_q     <= S_3RD2X;
			end

			// Both sources in hand: apply the op, then commit the deferred
			// pointer (`do_three` calls `update_def()` after `three_srcs`).
			S_3RD2X: if (i_ack) begin
				i_req <= 1'b0;
				tmp = s3_a - i_rdata;
				man[ST] <= sub_flags(man[ST], s3_a, i_rdata, tmp);
				if (def_pend) man[def_reg] <= def_val;
				def_pend <= 1'b0;
				retire = 1'b1;
				st_q   <= S_FETCH;
			end

			// indreg: the indirect source has arrived, the other is a
			// register, and nothing was deferred.
			S_3RD2: if (i_ack) begin
				i_req <= 1'b0;
				unique case (t3)
				T3_MPYI: begin
					prod = mpyi_prod(i_rdata, man[f_sreg]);
					man[f_dst] <= prod[31:0];
					if (f_dst < 5'd8) man[ST] <= mpyi_flags(man[ST], prod);
				end
				// Float three-operand forms. Which slot the memory word lands in
				// is the whole difference between indreg and regind. addf is
				// symmetric but subf3/cmpf3 (same path in the model) are not, so
				// the slots are transcribed rather than commuted.
				T3_FLT: begin
					fp_op    <= (opk == OP_MPYF3_INDREG) ? FP_MPY :
								(opk == OP_MPYF3_REGIND) ? FP_MPY :
								(opk == OP_SUBF3_INDREG) ? FP_SUB : FP_ADD;
					fp_dst   <= {3'd0, ir[18:16]};
					fp_stage <= 2'd0;
					// indreg (memory first): mpyf3 and subf3
					if (opk == OP_MPYF3_INDREG || opk == OP_SUBF3_INDREG) begin
						fa_e <= i_rdata[31:24];
						fa_m <= {i_rdata[23:0], 8'd0};
						fb_e <= exp[{2'd0, ir[2:0]}];
						fb_m <= man[{2'd0, ir[2:0]}];
					end else begin      // OP_ADDF3_REGIND, OP_MPYF3_REGIND
						fa_e <= exp[{2'd0, ir[10:8]}];
						fa_m <= man[{2'd0, ir[10:8]}];
						fb_e <= i_rdata[31:24];
						fb_m <= {i_rdata[23:0], 8'd0};
					end
				end
				// `op_cmpi(s1, s2)`: flags only, so no `dreg < 8` gate.
				T3_CMPIR: begin
					tmp = i_rdata - man[f_sreg];
					man[ST] <= sub_flags(man[ST], i_rdata, man[f_sreg], tmp);
				end
				T3_ADDIR: begin
					tmp = i_rdata + man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], i_rdata,
											 man[f_sreg], tmp);
				end
				// lsh3/ash3 indreg: the word is shifted by the register's low
				// seven bits.
				T3_LSH: begin
					tmp = lsh_res(i_rdata, man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= lsh_flags(man[ST], i_rdata,
											 man[f_sreg][6:0], tmp);
				end
				T3_ASH: begin
					tmp = ash_res(i_rdata, man[f_sreg][6:0]);
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ash_flags(man[ST], i_rdata,
											 man[f_sreg][6:0], tmp);
				end
				// `subi3` indreg: word minus register
				T3_SUBIR: begin
					tmp = i_rdata - man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= sub_flags(man[ST], i_rdata,
											 man[f_sreg], tmp);
				end
				T3_AND: begin
					tmp = i_rdata & man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				T3_ANDN: begin
					tmp = i_rdata & ~man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				// `addi3` regind: the register (bits 12:8) is the first
				// operand, the word the second
				T3_ADDRI: begin
					tmp = man[ir[12:8]] + i_rdata;
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= add_flags(man[ST], man[ir[12:8]],
											 i_rdata, tmp);
				end
				default: begin                      // T3_OR
					tmp = i_rdata | man[f_sreg];
					man[f_dst] <= tmp;
					if (f_dst < 5'd8)
						man[ST] <= ldi_flags(man[ST], f_dst, tmp);
				end
				endcase
				if (t3 == T3_FLT) begin
					st_q   <= S_FP;
				end else begin
					retire = 1'b1;
					st_q   <= S_FETCH;
				end
			end

			// The float ALU's result is on the wires this cycle. One pass for
			// an ordinary float op; two for `mpyaddf`, a multiply and an add
			// in one instruction.
			S_FP: begin
				// `fix` writes the mantissa only: its result is an integer, and
				// `setreg(dreg31, r_man[TEMP1])` leaves the exponent alone.
				if (!fp_manonly) exp[fp_dst] <= fp_re;
				man[fp_dst] <= fp_rm;
				man[ST]     <= fp_rst;
				if (fp_stage == 2'd1) begin
					// the multiply landed in TEMP3; now `addf(d_a, r2, TEMP2)`
					fp_op    <= FP_ADD;
					fa_e     <= exp[{2'd0, ma_r2}];
					fa_m     <= man[{2'd0, ma_r2}];
					fb_e     <= exp[T2];
					fb_m     <= man[T2];
					fp_dst   <= {1'b0, ma_da};
					fp_stage <= 2'd2;
					st_q     <= S_FP;
				end else begin
					if (fp_stage == 2'd2) begin
						// `self._copy(d_m, TEMP3)`, after the add, so the add's
						// flags survive; the deferred pointer lands last of all
						// (`update_def()` ends `_mpy_par`). ma_dm is five bits
						// and indexes unpadded: a six-bit index could name
						// TEMP1..TEMP3, which no variable index may reach.
						exp[ma_dm] <= exp[T3];
						man[ma_dm] <= man[T3];
						if (def_pend) man[def_reg] <= def_val;
						def_pend <= 1'b0;
					end
					retire = 1'b1;
					st_q   <= S_FETCH;
				end
			end

			// mpyaddf's first source has arrived; issue the second, which
			// uses the instruction's low byte and an immediate write-back.
			S_MA1: if (i_ack) begin
				exp[T1] <= i_rdata[31:24];
				man[T1] <= {i_rdata[23:0], 8'd0};
				ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
							  man[IR0], man[IR1]);
				if (ind_writes(ir[7:0]))
					man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
													man[AR0 + ir[2:0]],
													man[IR0], man[IR1]);
				i_addr <= ea[23:0];
				i_req  <= 1'b1;
				st_q     <= S_MA2;
			end

			// Both in hand. Variant 3 pairs them (TEMP1, r1, r2, TEMP2), so
			// the multiply is TEMP1 x r1 into TEMP3.
			S_MA2: if (i_ack) begin
				i_req  <= 1'b0;
				exp[T2]  <= i_rdata[31:24];
				man[T2]  <= {i_rdata[23:0], 8'd0};
				fp_op    <= FP_MPY;
				fa_e     <= exp[T1];
				fa_m     <= man[T1];
				fb_e     <= exp[{2'd0, ma_r1}];
				fb_m     <= man[{2'd0, ma_r1}];
				fp_dst   <= 6'd34;                  // TEMP3
				fp_stage <= 2'd1;
				st_q     <= S_FP;
			end

			// stisti's second write, then the deferred pointer commits
			S_STI2: if (i_ack) begin
				ea = ind_addr(ir[7:0], 32'd1, man[AR0 + ir[2:0]],
							  man[IR0], man[IR1]);
				if (ind_writes(ir[7:0]))
					man[AR0 + ir[2:0]] <= ind_newar(ir[7:0], 32'd1,
													man[AR0 + ir[2:0]],
													man[IR0], man[IR1]);
				i_addr  <= ea[23:0];
				i_wdata <= man[{2'd0, ir[24:22]}];
				i_req   <= 1'b1;
				i_we    <= 1'b1;
				st_q      <= S_PAR_WR;
			end

			S_PAR_RD: if (i_ack) begin
				i_req <= 1'b0;
				// the load half: no flags (MAME writes r_man directly);
				// `ldfstf` long2fp's into {exp, man}
				if (par_mpy) begin
					// `mpyf3stf`: operands for S_FP, no register written
					// yet: mpyf(dreg, (op >> 19) & 7, TEMP1)
					fp_op      <= FP_MPY;
					fa_e       <= exp[{2'd0, ir[21:19]}];
					fa_m       <= man[{2'd0, ir[21:19]}];
					fb_e       <= i_rdata[31:24];
					fb_m       <= {i_rdata[23:0], 8'd0};
					fp_dst     <= {3'd0, ir[24:22]};
					fp_manonly <= 1'b0;
					fp_stage   <= 2'd0;
				end else if (par_flt) begin
					fp = long2fp(i_rdata);
					exp[{2'd0, ir[24:22]}] <= fp[39:32];
					man[{2'd0, ir[24:22]}] <= fp[31:0];
				end else begin
					man[{2'd0, ir[24:22]}] <= i_rdata;
				end
				ea = ind_addr(ir[15:8], 32'd1, man[AR0 + ir[10:8]],
							  man[IR0], man[IR1]);
				if (ind_writes(ir[15:8]))
					man[AR0 + ir[10:8]] <= ind_newar(ir[15:8], 32'd1,
													 man[AR0 + ir[10:8]],
													 man[IR0], man[IR1]);
				i_addr  <= ea[23:0];
				i_wdata <= par_src3;
				i_req   <= 1'b1;
				i_we    <= 1'b1;
				st_q      <= S_PAR_WR;
			end

			S_PAR_WR: if (i_ack) begin
				i_req <= 1'b0;
				i_we  <= 1'b0;
				if (def_pend) man[def_reg] <= def_val;
				def_pend  <= 1'b0;
				if (par_mpy) begin
					// `mpyf3stf`'s product lands in S_FP, which retires. The
					// pointer above is an AR, the product an R register, so
					// the order between them is free.
					par_mpy <= 1'b0;
					st_q    <= S_FP;
				end else begin
					retire = 1'b1;
					st_q      <= S_FETCH;
				end
			end

			S_WR: if (i_ack) begin
				i_req <= 1'b0;
				// A write into the I/O file updates the shadow, and a write to
				// one of the four DMA registers re-evaluates the enable
				// (`io_write` calls `update_dma` for exactly those four).
				is_io   = (i_addr[23:8] == IO_BASE[23:8]);
				is_dmar = is_io && (i_addr[7:0] == DMA_GLOBAL_CTL ||
									i_addr[7:0] == DMA_SOURCE     ||
									i_addr[7:0] == DMA_DEST       ||
									i_addr[7:0] == DMA_COUNT);
				// The shadow's post-write value, computed here rather than read
				// next cycle: `update_dma` runs inside `io_write`, so the burst
				// starts in this same instruction.
				nctl = (is_io && i_addr[7:0] == DMA_GLOBAL_CTL)
						 ? i_wdata : dma_ctl;
				nsrc = (is_io && i_addr[7:0] == DMA_SOURCE)
						 ? i_wdata : dma_src;
				ncnt = (is_io && i_addr[7:0] == DMA_COUNT)
						 ? i_wdata : dma_cnt;
				if (is_io) begin
					unique case (i_addr[7:0])
						DMA_GLOBAL_CTL:     dma_ctl   <= i_wdata;
						DMA_SOURCE:         dma_src   <= i_wdata;
						DMA_DEST:           dma_dst   <= i_wdata;
						DMA_COUNT:          dma_cnt   <= i_wdata;
						SPORT_GLOBAL_CTL:   sport_ctl <= i_wdata;
						SPORT_RX_CTL:       sport_rx  <= i_wdata;
						SPORT_TIMER_PERIOD: sport_per <= i_wdata;
						TIMER0_CTL:         tmr_ctl[0] <= i_wdata;
						TIMER0_PERIOD:      tmr_per[0] <= i_wdata;
						TIMER1_CTL:         tmr_ctl[1] <= i_wdata;
						TIMER1_PERIOD:      tmr_per[1] <= i_wdata;
						default: ;
					endcase
				end

				// `io_write` calls `update_timer(which)` for each timer's CTL,
				// COUNTER and PERIOD. Only CTL and PERIOD feed the computation,
				// but a COUNTER write re-arms too, so it is watched for although
				// nothing here stores it.
				for (int tw = 0; tw < 2; tw = tw + 1) begin
					tmr_hit = is_io &&
					  (i_addr[7:0] == (TIMER0_CTL     + 8'h10 * tw[7:0]) ||
					   i_addr[7:0] == (TIMER0_COUNTER + 8'h10 * tw[7:0]) ||
					   i_addr[7:0] == (TIMER0_PERIOD  + 8'h10 * tw[7:0]));
					if (tmr_hit) begin
						ntctl = (i_addr[7:0] == (TIMER0_CTL + 8'h10 * tw[7:0]))
								  ? i_wdata : tmr_ctl[tw];
						ntper = (i_addr[7:0] == (TIMER0_PERIOD + 8'h10 * tw[7:0]))
								  ? i_wdata : tmr_per[tw];
						// `if (ctl & 0xC0) == 0xC0 and period:`
						if (ntctl[7:6] == 2'b11 && ntper != 32'd0) begin
							tmr_cd[tw]   <= {ntper, 1'b0};   // cycles + 2*period
							tmr_pend[tw] <= 1'b1;
							tmr_skip[tw] <= 1'b1;
						end else begin
							tmr_pend[tw] <= 1'b0;
						end
					end
				end

				if (is_dmar && (nctl[1:0] == 2'b11) && (ncnt != 32'd0)
					&& !dma_on) begin
					// `enabled and not self.dma_on`: the burst runs now,
					// inside this instruction, which does not retire until
					// it is done.
					dma_on   <= 1'b1;
					dma_first <= 1'b1;
					dma_left <= ncnt;
					dma_addr <= nsrc + {31'd0, nctl[4]};
					i_addr <= nsrc[23:0];
					i_req  <= 1'b1;
					i_we   <= 1'b0;
					st_q     <= S_DMA;

					// `self.dma_due = self.cycles + per * n`, as a countdown.
					// `dma_skip` exempts the arming step's own end of step:
					// the model's `cycles` is already past this step's
					// increment when it computes the deadline, so this
					// step's poll must not be charged. One step either way
					// moves the interrupt onto a different instruction.
					per_n    = sp_per_eff * ncnt[15:0];
					dma_cd   <= (ncnt[31:16] != 16'd0 || per_n[41:32] != 10'd0)
								  ? 32'hFFFF_FFFF : per_n[31:0];
					dma_pend <= 1'b1;
					dma_skip <= 1'b1;
				end else begin
					if (is_dmar && !((nctl[1:0] == 2'b11) && (ncnt != 32'd0)))
					begin
						// `elif not enabled and self.dma_on: dma_due = None`
						dma_on   <= 1'b0;
						if (dma_on) dma_pend <= 1'b0;
					end
					st_q   <= S_FETCH;
					retire = 1'b1;
				end
			end

			// The burst: `for _ in range(n): dac.append(rmem(addr));
			// addr += inc`. One word per ack, contiguous, nothing else on the
			// bus: the golden stream has its reads with no fetch between them.
			S_DMA: if (i_ack) begin
				// The word goes to the serial port's FIFO as the model appends
				// it to `dac`: all of them now, played later.
				dac_stb   <= 1'b1;
				dac_word  <= i_rdata[15:0];
				dac_first <= dma_first;
				dma_first <= 1'b0;
				if (dma_left == 32'd1) begin
					i_req <= 1'b0;
					// MAME leaves DMA_COUNT and DMA_SOURCE alone until
					// `poll_dma` fires, `serial_per_word * count` cycles
					// later, and raises DINT0 there, so the walked-to
					// address is latched here and spent there. The model's
					// loop masks `dma_final` to 24 bits at every step.
					dma_fin <= dma_addr[23:0];
					st_q   <= S_FETCH;
					retire = 1'b1;
				end else begin
					dma_left <= dma_left - 32'd1;
					dma_addr <= dma_addr + {31'd0, dma_inc};
					i_addr <= dma_addr[23:0];
					i_req  <= 1'b1;
				end
			end

			S_RET: if (i_ack) begin
				i_req <= 1'b0;
				pc      <= i_rdata[23:0];
				man[SP] <= man[SP] - 32'd1;
				if (retic) man[ST] <= man[ST] | (32'd1 << 13);   // GIE
				retic   <= 1'b0;
				retire   = 1'b1;
				st_q    <= S_FETCH;
			end

			// End of step: the model's poll_timers(); poll_dma(); check_irqs()
			// plus the cycles the step earned (see the header). The work is
			// in the block after the retire block, so that a retiring cycle
			// can do it too (the fused end of step).
			S_EOS: ;

			// `trap()`: SP pre-incremented, the PC pushed, GIE cleared, and
			// the PC to 0x809FC0 + num. The PC pushed is the one the step
			// ended on (for a delayed branch, the target), so this reads `pc`
			// a cycle after S_EOS rather than computing it there.
			S_TRAP: begin
				man[SP]   <= man[SP] + 32'd1;
				i_addr  <= man[SP][23:0] + 24'd1;
				i_wdata <= {8'd0, pc};
				i_req   <= 1'b1;
				i_we    <= 1'b1;
				man[ST]   <= man[ST] & ~(32'd1 << GIE_BIT);
				pc        <= VECTOR_BASE + {19'd0, trap_num};
				st_q      <= S_TRAPW;
			end

			// The retire is held back until the push has gone out, so the
			// state line the bench compares carries the trap: the model
			// writes its line at the top of the next step, after check_irqs.
			S_TRAPW: if (i_ack) begin
				i_req   <= 1'b0;
				i_we    <= 1'b0;
				insn_done <= 1'b1;
				st_q      <= S_FETCH;
			end

			S_HALT: begin
				i_req <= 1'b0;
				st_q    <= S_HALT;
			end

			// One cycle after reset, BOOT_START only: `boot_load`'s PC and SP
			// (the rest are the reset's zeros). No fetch and no retire: the
			// golden state file's first line is this state, and the bench
			// compares against it at the first retire.
			S_BOOT: begin
				pc      <= boot_pc;
				man[SP] <= 32'h0000_00FF;
				st_q    <= S_FETCH;
			end
			endcase

			// Retire, once per delayed-branch group: MAME's `execute_delayed`
			// runs the three slots inside the branch's own step(), so the
			// model logs one state line for the branch and its slots
			// together.
			if (retire) begin
				grp_end = 1'b0;
				if (dly_cnt != 2'd0) begin
					dly_cnt <= dly_cnt - 2'd1;
					if (dly_cnt == 2'd1) begin
						// the third slot: the group ends here
						dly_pend  <= 1'b0;
						if (dly_take) pc <= dly_pc;
						grp_end = 1'b1;
						// `execute_delayed` ends with `self.delayed = False`,
						// which also ends an rpts's (`rpts_dly`)
						rpts_dly  <= 1'b0;
					end
				end else if (!dly_pend && !dly_start) begin
					grp_end = 1'b1;
				end
				insn_pc <= start_pc;

				// Each step ends in S_EOS, which spends the cycles and runs the
				// polls, except the repeat-block wrap, which the model returns
				// from before any of them. This comes after the case, so it
				// overrides the `st_q <= S_FETCH` the retiring state wrote.
				if (rpt_step) begin
					insn_done <= grp_end;
					// S_RPTW took the next word itself
					st_q      <= rptw_took ? S_EXEC : S_FETCH;
				end else if (dly_start) begin
					// The delayed branch itself does not poll. In the model
					// its step fetches (`cycles += 1`), then
					// `execute_delayed` runs the three slots inside it, each
					// with its own polls, and only then does the branch's
					// step poll. So the group's first poll is slot 1's, at
					// c+2, and there is none at c+1: the branch's cycle is
					// carried, as a trap's four are, and slot 1's S_EOS
					// spends it.
					cyc_pend <= cyc_pend + 4'd1;
					st_q     <= S_FETCH;
				end else begin
					// Fused end of step (IC_EN). The polls read ST (GIE), IE,
					// IF, `cyc_pend`, the timers and the DMA, which a retiring
					// instruction may write on this very edge. Most do not,
					// and those end their step here: the polls run in the
					// retiring cycle and the next word is taken in it too.
					// Fused only when this cycle
					//   - retires from S_EXEC, or from S_RD, S_WR, S_FP,
					//     S_PAR_WR, S_3RD2 or S_3RD2X with its last access
					//     (`i_addr`) to cageram, IRAM or the sound bank (for
					//     a float with no access of its own, `i_addr` is an
					//     earlier instruction's, which can only refuse);
					//   - added no model cycles (`cyc_wr`);
					//   - names none of ST, IE, IF, IOF, RS, RE, RC as
					//     destination (S_FP tests `fp_dst`, as `fix` may name
					//     any register; S_PAR_WR's and S_3RD2X's destinations
					//     are fixed by their encoding);
					//   - is not inside a delayed group.
					mem_ok   = (i_addr[23:16] == 8'd0) || acc_iram
							   || (i_addr[23:22] == 2'b11);        // the sound bank (read-only)
					fuse_st  = (st_q == S_EXEC)
							   || ((st_q == S_RD     || st_q == S_WR
									|| st_q == S_FP     || st_q == S_PAR_WR
									|| st_q == S_3RD2   || st_q == S_3RD2X)
								   && mem_ok);
					fuse_dst = (st_q == S_FP)     ? !(fp_dst >= 6'd21 && fp_dst <= 6'd27) :
							   (st_q == S_PAR_WR
								|| st_q == S_3RD2X) ? 1'b1
													: !(f_dst >= 5'd21 && f_dst <= 5'd27);
					fuse_ok  = IC_EN && fuse_st && fuse_dst && !cyc_wr
							   && (dly_cnt == 2'd0) && !dly_pend;
					if (fuse_ok) begin
						fuse_now = 1'b1;
					end else begin
						eos_done  <= grp_end;
						st_q      <= S_EOS;
					end
				end
			end

			// End of step: S_EOS's work, for S_EOS itself or a fused retire.
			// Its NBAs come after the retire block's, so its `st_q` wins.
			if (st_q == S_EOS || fuse_now) begin
				done_v  = (st_q == S_EOS) ? eos_done  : grp_end;
				retic_v = (st_q == S_EOS) ? eos_retic : 1'b0;
				cyc_add   = 32'd1 + {28'd0, cyc_pend};
				dma_fire  = 1'b0;
				mcyc_stb  <= 1'b1;
				mcyc_add  <= cyc_add[4:0];

				// poll_timers(): TINT0 is line 8 and TINT1 line 9, and the IF
				// bit is the line number. No memory access, so this is all of it.
				for (int k = 0; k < 2; k = k + 1) begin
					if (tmr_pend[k]) begin
						if (tmr_skip[k]) begin
							tmr_skip[k] <= 1'b0;
						end else if (tmr_cd[k] <= {1'b0, cyc_add}) begin
							// `timer_next = None; update_timer(which)`,
							// which re-arms from the current cycle, so the
							// reload does not skip this step.
							if (tmr_ctl[k][7:6] == 2'b11 && tmr_per[k] != 32'd0)
								tmr_cd[k] <= {tmr_per[k], 1'b0};
							else
								tmr_pend[k] <= 1'b0;
							// `set_irq_line(8 + which, True)`; the trailing
							// re-assert is `IF |= irq_state & 0x0F`: the
							// board's four bits, not the one just raised.
							if_next = if_next | (32'd1 << (8 + k));
							if_next = if_next | {28'd0, irq_in};
						end else begin
							tmr_cd[k] <= tmr_cd[k] - {1'b0, cyc_add};
						end
					end
				end

				// poll_dma()
				if (dma_pend) begin
					if (dma_skip)                dma_skip <= 1'b0;
					else if (dma_cd <= cyc_add)  dma_fire  = 1'b1;
					else                         dma_cd   <= dma_cd - cyc_add;
				end
				if (dma_fire) begin
					dma_pend <= 1'b0;
					dma_on   <= 1'b0;
					dma_cnt  <= 32'd0;                  // io[DMA_COUNT] = 0
					dma_src  <= {8'd0, dma_fin};        // io[DMA_SOURCE]
					// `set_irq_line(10, True)`: TMS320C3X_DINT0.
					if_next       = if_next | (32'd1 << 10);
					if_next       = if_next | {28'd0, irq_in};
				end

				// check_irqs(): `valid = IF & IE & 0x0FFF`, lowest bit wins, and
				// `which = i + 1`, so the vector is 0x809FC0 + i + 1 and the bit
				// cleared is i.
				vpre    = man[IF] & man[IE] & 32'h0000_0FFF;
				vpost   = if_next & man[IE] & 32'h0000_0FFF;
				vsel    = (retic_v && vpre != 32'd0) ? vpre : vpost;
				which_i = 0;
				for (int k = 11; k >= 0; k = k - 1)
					if (vsel[k]) which_i = k[3:0];
				// `if not self.delayed`: a delay slot recognises an interrupt
				// but does not take one (`done_v` is high exactly for the steps
				// the model runs with `delayed` false), and neither does any
				// step between an rpts and the end of the next delayed branch
				// (`rpts_dly`).
				take_irq = (vsel != 32'd0) && man[ST][GIE_BIT] && done_v
						   && !rpts_dly;

				if (take_irq) begin
					if_next  = if_next & ~(32'd1 << which_i);   // unsigned amount
					// level sensitive: taking one clears its IF bit and the
					// still-asserted line puts it back. Only IRQ0-3.
					if_next  = if_next | {28'd0, irq_in};
					trap_num <= {1'b0, which_i} + 5'd1;
					// `trap()`'s 4 cycles land after this step's poll_dma,
					// so they are carried to the next step, not spent now.
					cyc_pend <= 4'd4;
					st_q     <= S_TRAP;
				end else begin
					cyc_pend  <= 4'd0;
					insn_done <= done_v;
					if (ic_take_eos) begin
						// S_FETCH's cache hit, taken here (the end-of-step take)
						start_pc <= pc;
						ir       <= ic_word;
						pc       <= pc + 24'd1;
						ic_used  <= 1'b1;
						st_q     <= S_EXEC;
					end else if (IC_EN && ic_rpt_wrap) begin
						// a repeat wrap: S_FETCH's first test, and all
						// S_FETCH does before it is `start_pc <= pc`
						start_pc <= pc;
						st_q     <= S_RPTW;
					end else begin
						st_q     <= S_FETCH;
					end
				end

				man[IF]   <= if_next;
				eos_retic <= 1'b0;
			end
		end
	end

endmodule

`default_nettype wire
