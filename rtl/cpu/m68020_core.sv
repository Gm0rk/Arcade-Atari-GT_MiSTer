// m68020_core.sv -- MC68EC020 CPU core (correctness-first, multicycle)
//
// Built to run in lockstep with cpu68020.py, the Python model checked against
// a MAME execution trace of the game:
//   - One memory transaction at a time, at the size and in the order the
//     model performs them (fetch16 = one 16-bit read at PC; fetch32/abs.L/
//     imm.L = one 32-bit read; operand accesses at operand size). The
//     lockstep bench checks every access, not just architectural state.
//   - No prefetch or pipelining. The cache holding register and instruction
//     cache (FETCH_CHR, ICACHE_ENTRIES) change only the fetch stream.
//   - Anything not implemented latches `unimplemented` with the opcode and
//     PC and halts rather than guessing.
//
// Simplifications (matching the model):
//   - Only autovectored interrupts are taken; a divide by zero halts.
//   - NEGX ignores incoming X; ASR keeps the current sign bit per step.
//   - Not implemented: memory-indirect EA modes, memory-form shifts, BCD
//     ops, TAS, CHK, MOVEP, ADDX/SUBX -(Ay),-(Ax).
//   - SR T/M bits not modeled (never set by this game).
//
// Bus protocol: req/ack. bus_req holds until bus_ack; rdata is captured on
// the ack cycle. bus_size is in bytes: 1, 2 or 4.

module m68020_core #(
	// Instruction-stream cache holding register (M68020UM 1.6). 0 = one external
	// access per instruction word, the bus behaviour lockstep vectors generated
	// without `--fetch-path` expect.
	parameter bit FETCH_CHR = 1'b1,

	// Instruction cache entries, one long word each, direct-mapped behind the
	// CHR (M68020UM 4.1). 0 = CHR only.
	//
	// Size is a cost choice, not an accuracy one. The real part has 64, but the
	// size is only observable through CACR's CE bit with a CAAR index, and the
	// game writes CACR once and never reads it. The fetch policy follows the
	// manual; the size follows the fitter.
	parameter int ICACHE_ENTRIES = 64
) (
	input  logic        clk,
	input  logic        rst_n,

	// Interrupt priority request from the board (level-triggered, like the real
	// IPL2-0 pins), taken between instructions when ipl_in > the SR mask.
	// Autovectored only, as this board uses: VBR + (24+level)*4.
	input  logic [2:0]  ipl_in,

	output logic [31:0] bus_addr,
	output logic [2:0]  bus_size,
	output logic        bus_we,
	output logic [31:0] bus_wdata,
	output logic        bus_req,
	input  logic        bus_ack,
	input  logic [31:0] bus_rdata,

	// The access is an instruction-stream fetch (opcode or extension word), not
	// an operand access. Data accesses are never cached (M68020UM 4.1), so
	// anything caching on this port must qualify on it. It also lets the
	// lockstep bench check the data stream exactly while the fetch stream
	// differs.
	output logic        bus_ifetch,
	// One-cycle pulse when an instruction fetch was served without a bus cycle
	// (CHR or cache). `bus_ifetch && bus_req && bus_ack` counts the misses, so
	// the pair gives a hit rate (the debug overlay's IFET).
	output logic        bus_ifetch_hit,

	output logic        insn_done, // pulse: instruction or IRQ entry retired
	output logic [31:0] insn_pc,   // start PC of the current instruction
	output logic [15:0] sr_out,
	output logic        unimplemented, // sticky: halted on an unimplemented op
	output logic [15:0] unimpl_opcode,
	output logic [31:0] unimpl_pc,
	// One-cycle pulse when the RESET instruction executes; the game uses it to
	// restart the CAGE sound board.
	output logic        reset_out
);

	// helpers
	function automatic [31:0] sext8(input [7:0] v);   sext8  = {{24{v[7]}},  v}; endfunction
	function automatic [31:0] sext16(input [15:0] v); sext16 = {{16{v[15]}}, v}; endfunction
	function automatic [31:0] szmask(input [2:0] nb);
		case (nb)
			3'd1: szmask = 32'h0000_00ff;
			3'd2: szmask = 32'h0000_ffff;
			default: szmask = 32'hffff_ffff;
		endcase
	endfunction
	function automatic [31:0] sext_sz(input [31:0] v, input [2:0] nb);
		case (nb)
			3'd1: sext_sz = sext8(v[7:0]);
			3'd2: sext_sz = sext16(v[15:0]);
			default: sext_sz = v;
		endcase
	endfunction
	function automatic logic msb_of(input [31:0] v, input [2:0] nb);
		case (nb)
			3'd1: msb_of = v[7];
			3'd2: msb_of = v[15];
			default: msb_of = v[31];
		endcase
	endfunction
	function automatic [1:0] flg_sz_of(input [2:0] nb);
		case (nb)
			3'd1: flg_sz_of = 2'd0;
			3'd2: flg_sz_of = 2'd1;
			default: flg_sz_of = 2'd2;
		endcase
	endfunction
	function automatic [2:0] size_from_bits(input [1:0] b);
		case (b)
			2'd0: size_from_bits = 3'd1;
			2'd1: size_from_bits = 3'd2;
			default: size_from_bits = 3'd4;
		endcase
	endfunction

	// architectural state
	logic [31:0] pc;
	logic [15:0] ir;
	logic        s_bit;
	logic [2:0]  ipl;
	logic fN, fZ, fV, fC, fX;
	logic [31:0] cacr, vbr, sfc, dfc, caar, msp_r, isp_r;

	assign sr_out = {2'b00, s_bit, 2'b00, ipl, 3'b000, fX, fN, fZ, fV, fC};

	logic [3:0]  rf_rd_a_num, rf_rd_b_num;
	logic [31:0] rf_rd_a_data, rf_rd_b_data;
	logic        rf_wr_en;
	logic [3:0]  rf_wr_num;
	logic [1:0]  rf_wr_size;
	logic [31:0] rf_wr_data;
	logic [31:0] usp_out, ssp_out;
	logic        usp_wr, ssp_wr;
	logic [31:0] sp_wr_data;
	logic        illegal_areg_byte_write;

	m68020_regs u_regs (
		.clk(clk), .rst_n(rst_n), .supervisor(s_bit),
		.rd_a_num(rf_rd_a_num), .rd_a_data(rf_rd_a_data),
		.rd_b_num(rf_rd_b_num), .rd_b_data(rf_rd_b_data),
		.wr_en(rf_wr_en), .wr_num(rf_wr_num), .wr_size(rf_wr_size), .wr_data(rf_wr_data),
		.usp_out(usp_out), .ssp_out(ssp_out),
		.usp_wr(usp_wr), .ssp_wr(ssp_wr), .sp_wr_data(sp_wr_data),
		.illegal_areg_byte_write(illegal_areg_byte_write)
	);

	logic [31:0] flg_a, flg_b, flg_res;
	logic [1:0]  flg_sz, flg_op;
	logic flo_n, flo_z, flo_v, flo_c, flo_x, flo_xwe;
	logic [32:0] addx_full;                 // ADDX/SUBX working values
	logic [31:0] addx_a, addx_b;
	logic        addx_cy, addx_ov;
	m68020_flags u_flags (
		.a(flg_a), .b(flg_b), .result(flg_res), .size(flg_sz), .op(flg_op),
		.n_flag(flo_n), .z_flag(flo_z), .v_flag(flo_v), .c_flag(flo_c),
		.x_flag(flo_x), .x_we(flo_xwe)
	);
	localparam [1:0] FOP_ADD = 2'd0, FOP_SUB = 2'd1, FOP_CMP = 2'd2;

	function automatic logic test_cc(input [3:0] cc_i);
		case (cc_i)
			4'h0: test_cc = 1'b1;
			4'h1: test_cc = 1'b0;
			4'h2: test_cc = !fC && !fZ;
			4'h3: test_cc = fC || fZ;
			4'h4: test_cc = !fC;
			4'h5: test_cc = fC;
			4'h6: test_cc = !fZ;
			4'h7: test_cc = fZ;
			4'h8: test_cc = !fV;
			4'h9: test_cc = fV;
			4'hA: test_cc = !fN;
			4'hB: test_cc = fN;
			4'hC: test_cc = fN == fV;
			4'hD: test_cc = fN != fV;
			4'hE: test_cc = !fZ && (fN == fV);
			default: test_cc = fZ || (fN != fV);
		endcase
	endfunction

	// decode (combinational from registered ir, consumed in S_DISPATCH)
	typedef enum logic [5:0] {
		G_UNIMPL, G_MOVE, G_MOVEQ, G_IMMALU, G_CLRGRP, G_TST, G_LEA, G_PEA,
		G_SWAP, G_EXT, G_MOVEC, G_MOVE_USP, G_NOP, G_RESET, G_JSR, G_JMP, G_RTS,
		G_ADDQSUBQ, G_SCC, G_DBCC, G_BCC, G_BINOP, G_MULW, G_SHIFT,
		G_EXG, G_MOVEM, G_TO_CCR, G_TO_SR, G_FROM_SR, G_CCR_SR_IMM,
		G_CMPM, G_LINK, G_UNLK, G_MULDIVL, G_BITOP, G_RTE, G_RTR,
		G_DIVW
	} group_t;

	// binop sub-op encoding
	localparam [2:0] BOP_OR = 3'd0, BOP_AND = 3'd1, BOP_EOR = 3'd2,
					 BOP_ADDSUB = 3'd3, BOP_CMP = 3'd4, BOP_A = 3'd5, BOP_CMPA = 3'd6,
					 BOP_ADDX = 3'd7;   // ADDX/SUBX

	group_t      dec_grp;
	logic [2:0]  dec_size;
	logic [2:0]  dec_binop;
	logic        dec_is_add;

	always_comb begin
		logic [3:0] line;
		logic [1:0] szb;
		line = ir[15:12];
		szb  = ir[7:6];

		dec_grp    = G_UNIMPL;
		dec_size   = 3'd2;
		dec_binop  = BOP_OR;
		dec_is_add = 1'b0;

		case (line)
			4'h1, 4'h2, 4'h3: begin
				dec_grp  = G_MOVE;
				dec_size = (line == 4'h1) ? 3'd1 : (line == 4'h3) ? 3'd2 : 3'd4;
			end
			4'h0: begin
				if (ir == 16'h003C || ir == 16'h007C || ir == 16'h023C ||
					ir == 16'h027C || ir == 16'h0A3C || ir == 16'h0A7C) begin
					dec_grp  = G_CCR_SR_IMM;
					dec_size = ir[6] ? 3'd2 : 3'd1;
				end else if (ir[8] && ir[11:9] != 3'd7 && ir[5:3] != 3'd1) begin
					// dynamic bit op BTST/BCHG/BCLR/BSET Dn,<ea>
					// (mode 001 would be MOVEP -- unimplemented, falls to G_UNIMPL)
					dec_grp  = G_BITOP;
					dec_size = (ir[5:3] == 3'd0) ? 3'd4 : 3'd1;
				end else if ((ir & 16'hFF00) == 16'h0800) begin
					// static bit op #imm,<ea>
					dec_grp  = G_BITOP;
					dec_size = (ir[5:3] == 3'd0) ? 3'd4 : 3'd1;
				end else if (szb != 2'd3 && !ir[8] &&
							 (ir[11:9] == 3'd0 || ir[11:9] == 3'd1 || ir[11:9] == 3'd2 ||
							  ir[11:9] == 3'd3 || ir[11:9] == 3'd5 || ir[11:9] == 3'd6)) begin
					dec_grp  = G_IMMALU;
					dec_size = size_from_bits(szb);
				end
			end
			4'h4: begin
				// RESET asserts RESET# to reset external peripherals; the CPU itself
				// is unaffected. The game uses it to reset the CAGE sound board.
				if (ir == 16'h4E70) dec_grp = G_RESET;
				else if (ir == 16'h4E71) dec_grp = G_NOP;
				else if (ir == 16'h4E75) dec_grp = G_RTS;
				else if (ir == 16'h4E73) dec_grp = G_RTE;
				else if (ir == 16'h4E77) dec_grp = G_RTR;
				else if (ir == 16'h4E7A || ir == 16'h4E7B) dec_grp = G_MOVEC;
				else if ((ir & 16'hFFF0) == 16'h4E60) dec_grp = G_MOVE_USP;
				else if ((ir & 16'hFFF8) == 16'h4E50) dec_grp = G_LINK;
				else if ((ir & 16'hFFF8) == 16'h4E58) dec_grp = G_UNLK;
				else if ((ir & 16'hFFC0) == 16'h4E80) dec_grp = G_JSR;
				else if ((ir & 16'hFFC0) == 16'h4EC0) dec_grp = G_JMP;
				// LEA only takes control addressing modes; the guard keeps it from
				// swallowing EXTB.L (0x49C0|reg, mode 000) before the EXT decode.
				else if ((ir & 16'hF1C0) == 16'h41C0 &&
						 ir[5:3] != 3'd0 && ir[5:3] != 3'd1 &&
						 ir[5:3] != 3'd3 && ir[5:3] != 3'd4) dec_grp = G_LEA;
				else if ((ir & 16'hFFF8) == 16'h4840) dec_grp = G_SWAP;
				// PEA takes control addressing modes only; without the guard the
				// mask also matches BKPT #n (0x4848-484F, mode 001) and other
				// encodings it must not.
				else if ((ir & 16'hFFC0) == 16'h4840 &&
						 (ir[5:3] == 3'd2 || ir[5:3] == 3'd5 || ir[5:3] == 3'd6 ||
						  (ir[5:3] == 3'd7 && ir[2:0] <= 3'd3))) dec_grp = G_PEA;
				else if ((ir & 16'hFE38) == 16'h4800 &&
						 (ir[8:6] == 3'd2 || ir[8:6] == 3'd3 || ir[8:6] == 3'd7))
					// EXT.W / EXT.L / EXTB.L (EXTB.L is 0x49C0|reg, bit 8 set).
					// SWAP (0x4840, would alias with opmode 1) is decoded
					// earlier and never reaches this test.
					dec_grp = G_EXT;
				else if ((ir & 16'hFB80) == 16'h4880) begin
					dec_grp  = G_MOVEM;
					dec_size = ir[6] ? 3'd4 : 3'd2;
				end
				else if ((ir & 16'hFF00) == 16'h4200 && szb != 2'd3) begin
					dec_grp = G_CLRGRP; dec_binop = 3'd1;  // CLR
					dec_size = size_from_bits(szb);
				end
				else if ((ir & 16'hFF00) == 16'h4400 && szb != 2'd3) begin
					dec_grp = G_CLRGRP; dec_binop = 3'd2;  // NEG
					dec_size = size_from_bits(szb);
				end
				else if ((ir & 16'hFF00) == 16'h4600 && szb != 2'd3) begin
					dec_grp = G_CLRGRP; dec_binop = 3'd3;  // NOT
					dec_size = size_from_bits(szb);
				end
				else if ((ir & 16'hFF00) == 16'h4000 && szb != 2'd3) begin
					dec_grp = G_CLRGRP; dec_binop = 3'd0;  // NEGX (approx)
					dec_size = size_from_bits(szb);
				end
				else if ((ir & 16'hFF00) == 16'h4A00 && szb != 2'd3) begin
					dec_grp = G_TST;
					dec_size = size_from_bits(szb);
				end
				else if ((ir & 16'hFF00) == 16'h4C00) begin
					dec_grp = G_MULDIVL; dec_size = 3'd4;
				end
				else if ((ir & 16'hFFC0) == 16'h44C0) dec_grp = G_TO_CCR;
				else if ((ir & 16'hFFC0) == 16'h46C0) dec_grp = G_TO_SR;
				else if ((ir & 16'hFFC0) == 16'h40C0) dec_grp = G_FROM_SR;
			end
			4'h5: begin
				if (szb == 2'd3) begin
					// Scc's EA must be data alterable: the guard keeps the
					// TRAPcc forms (mode 7, reg 2/3/4) from decoding as Scc.
					// Mode 001 is DBcc and is tested first.
					if (ir[5:3] == 3'd1) dec_grp = G_DBCC;
					else if (ir[5:3] != 3'd7 ? 1'b1 : (ir[2:0] <= 3'd1))
											 dec_grp = G_SCC;
					dec_size = 3'd1;
				end else begin
					dec_grp    = G_ADDQSUBQ;
					dec_is_add = !ir[8];
					dec_size   = size_from_bits(szb);
				end
			end
			4'h6: dec_grp = G_BCC;
			4'h7: if (!ir[8]) dec_grp = G_MOVEQ;
			4'h8: begin
				if (ir[8] && ir[7:6] != 2'b11 && ir[5:4] == 2'b00) begin
					// SBCD (ir[7:6]=00), PACK (01), UNPK (10): not implemented.
					// Halt and record the opcode rather than executing them as
					// OR. The ir[7:6] != 11 guard keeps DIVU.W/DIVS.W, which
					// also have ir[5:4]==00 in their register form.
					dec_grp = G_UNIMPL;
				end else if (ir[8:6] != 3'd3 && ir[8:6] != 3'd7) begin
					dec_grp = G_BINOP; dec_binop = BOP_OR;
					dec_size = size_from_bits(szb);
				end else begin
					// DIVU.W (opmode 011) / DIVS.W (opmode 111)
					dec_grp = G_DIVW; dec_size = 3'd2;
				end
			end
			4'h9, 4'hD: begin
				dec_grp    = G_BINOP;
				dec_is_add = (line == 4'hD);
				if (ir[8:6] == 3'd3 || ir[8:6] == 3'd7) begin
					dec_binop = BOP_A;
					dec_size  = (ir[8:6] == 3'd3) ? 3'd2 : 3'd4;
				end else if (ir[8] && ir[5:4] == 2'b00) begin
					// ADDX (line D) / SUBX (line 9). ADDA/SUBA above already
					// consumed ir[7:6]==3, so ir[8] set with ir[5:4]==0 is
					// exactly the ADDX encoding. ir[3] picks -(Ay),-(Ax), which
					// is not implemented and halts in the execute block.
					dec_binop = BOP_ADDX;
					dec_size  = size_from_bits(szb);
				end else begin
					dec_binop = BOP_ADDSUB;
					dec_size  = size_from_bits(szb);
				end
			end
			4'hB: begin
				if (ir[8:6] == 3'd3 || ir[8:6] == 3'd7) begin
					dec_grp = G_BINOP; dec_binop = BOP_CMPA;
					dec_size = (ir[8:6] == 3'd3) ? 3'd2 : 3'd4;
				end else if (ir[8]) begin
					if (ir[5:3] == 3'd1) begin
						dec_grp = G_CMPM;
						dec_size = size_from_bits(szb);
					end else begin
						dec_grp = G_BINOP; dec_binop = BOP_EOR;
						dec_size = size_from_bits(szb);
					end
				end else begin
					dec_grp = G_BINOP; dec_binop = BOP_CMP;
					dec_size = size_from_bits(szb);
				end
			end
			4'hC: begin
				if (ir[8:6] == 3'd3 || ir[8:6] == 3'd7) begin
					dec_grp = G_MULW; dec_size = 3'd2;
				end else if ((ir & 16'h0130) == 16'h0100 && ir[7:6] == 2'b00) begin
					// ABCD shares bits 8, 5 and 4 with EXG and differs only
					// in ir[7:6] (00 vs 01/10). Not implemented: halt and
					// record the opcode.
					dec_grp = G_UNIMPL;
				end else if ((ir & 16'h0130) == 16'h0100) begin
					dec_grp = G_EXG;
				end else begin
					dec_grp = G_BINOP; dec_binop = BOP_AND;
					dec_size = size_from_bits(szb);
				end
			end
			4'hE: begin
				// register shift/rotate forms, including ROXL/ROXR (ir[4:3]==2);
				// the memory forms (ir[7:6]==3) are not decoded.
				if (ir[7:6] != 2'd3) begin
					dec_grp = G_SHIFT;
					dec_size = size_from_bits(szb);
				end
			end
			default: ;
		endcase
	end

	// FSM
	typedef enum logic [6:0] {
		S_RESET_SSP, S_RESET_PC, S_RESET_GO,
		S_MEM,
		S_FETCH, S_DECODE, S_DISPATCH,
		S_IMM_GOT,
		S_EA_START, S_EA_EXT_GOT, S_EA_IDX, S_EA_BD_GOT,
		S_SRC_READ, S_SRC_MEM_GOT, S_SRC_DISPATCH,
		S_MOVE_DST, S_MOVE_WRITE,
		S_RMW_CHK, S_RMW_READ, S_RMW_MEM_GOT, S_RMW_EXEC,
		S_FLAGS_SAMPLE, S_WRITE_RESULT,
		S_CLR_EXEC,
		S_LEA_EXEC, S_JMP_GO, S_PEA_PUSH,
		S_MOVEC_GOT, S_MOVEC_EXEC,
		S_JSR_PUSH, S_RTS_RD, S_RTS_GOT,
		S_DBCC_GOT,
		S_BCC_EXEC, S_BSR_PUSH,
		S_BINOP_EXEC,
		S_MULW_EXEC, S_DIVW_EXEC,
		S_SHIFT_INIT, S_SHIFT_STEP, S_SHIFT_FIN,
		S_EXG1, S_EXG2, S_REGOP_EXEC,
		S_MOVEM_LIST_GOT, S_MOVEM_EA_DONE, S_MOVEM_NEXT, S_MOVEM_MEM_GOT, S_MOVEM_FIN,
		S_TOSR_EXEC, S_FROMSR_WRITE,
		S_CMPM_RD1, S_CMPM_GOT1, S_CMPM_RD2, S_CMPM_GOT2,
		S_MDL_EXT_GOT, S_MDL_EXEC, S_DIV_STEP, S_DIV_FIN, S_DIV_WRREM,
		S_BITNO_GOT, S_BIT_EXEC,
		S_LINK_PUSH, S_LINK_SETAN, S_LINK_DISP_GOT,
		S_UNLK_SETSP, S_UNLK_GOT,
		S_IRQ_PUSH_PC, S_IRQ_PUSH_SR, S_IRQ_VECTOR, S_IRQ_GO,
		S_RTE_RD_SR, S_RTE_GOT_SR, S_RTE_GOT_PC, S_RTE_APPLY,
		S_DONE, S_HALT
	} state_t;
	state_t state;

	// latched decode
	group_t      grp;
	logic [2:0]  opsize;
	logic [2:0]  binop;
	logic        is_add;
	logic        ea_is_dst;             // binop direction bit (ir[8])
	logic [2:0]  reg9;                  // ir[11:9]
	logic [3:0]  cc;                    // ir[11:8]
	logic [2:0]  earlo_mode, earlo_reg; // ir[5:3], ir[2:0]
	logic [2:0]  eahi_mode, eahi_reg;   // ir[8:6], ir[11:9] (MOVE dst)

	// memory transaction
	logic [31:0] mem_addr;
	logic [2:0]  mem_size;
	logic        mem_we;
	logic [31:0] mem_wdata;
	logic        mem_is_fetch;
	state_t      mem_ret;
	logic [31:0] mem_rdata_r;

	// A fill asks the bus for the whole long word, so `bus_addr`/`bus_size`
	// may differ from the access the sequencer asked for; tb_m68020_lockstep's
	// `+relax_fetch` allows this for the fetch stream, and the data stream
	// stays strict.
	assign chr_fill  = FETCH_CHR && fetch_1lw && !chr_hit && !ic_hit;
	assign bus_addr  = chr_fill ? {mem_addr[31:2], 2'b00} : mem_addr;
	assign bus_size  = chr_fill ? 3'd4                    : mem_size;
	assign bus_we    = mem_we;
	assign bus_wdata = mem_wdata;
	// Cache holding register (M68020UM 1.6): the 32-bit long word the last
	// fetch came from, so the next sequential instruction word needs no bus
	// cycle. UM 1.6 has it serve the pipe whether or not the instruction cache
	// is enabled, so it is not gated on CACR bit 0.
	//
	// Only fetches inside one long word take part: every 2-byte fetch, and a
	// 4-byte fetch when long-word aligned. A misaligned 4-byte fetch spans two
	// long words and goes to the bus unchanged (serving it would need a
	// two-beat S_MEM).
	//
	// Data accesses never take part (M68020UM 4.1: "Data accesses are not
	// cached"). Qualifying on `mem_is_fetch` keeps this off the I/O and
	// colour-RAM windows.
	logic        chr_valid;
	logic [29:0] chr_tag;
	logic [31:0] chr_data;

	wire fetch_1lw  = mem_is_fetch &&
					  ((mem_size == 3'd2) ||
					   (mem_size == 3'd4 && !mem_addr[1]));
	wire chr_hit    = FETCH_CHR && fetch_1lw && chr_valid &&
					  (chr_tag == mem_addr[31:2]);
	wire chr_fill;      // also requires a cache miss; see below

	// big-endian: long word L holds bytes L..L+3 in [31:24]..[7:0], so a word
	// at L is [31:16] and a word at L+2 is [15:0]
	function automatic [31:0] lw_extract(input [31:0] lw, input [2:0] nb,
										 input a1);
		lw_extract = (nb == 3'd4) ? lw
				   : (a1 ? {16'd0, lw[15:0]} : {16'd0, lw[31:16]});
	endfunction

	// Instruction cache (M68020UM 4.1): direct-mapped, one long word per entry,
	// behind the CHR. On a miss "both words of the entry will be updated", so an
	// entry is a whole long word and fills share the CHR's path. Gated on CACR
	// bit 0 (E); the CHR in front of it is not.
	//
	// The tag is the full remaining address, not the manual's A31-A8 (+FC2):
	// 8 more flops an entry, and no aliasing if code ever runs outside ROM.
	//
	// Both arrays are read combinationally (`ic_hit` in the same cycle), so
	// they are pinned to MLAB, which reads asynchronously (an M10K cannot; Quartus
	// replicates the arrays). A combinational read has no read-during-write
	// ambiguity, so `no_rw_check` waives nothing.
	localparam int IC_IDX  = (ICACHE_ENTRIES <= 1) ? 1 : $clog2(ICACHE_ENTRIES);
	localparam int IC_TAGW = 32 - 2 - IC_IDX;

	logic                  ic_valid [0:ICACHE_ENTRIES-1];
	(* ramstyle = "MLAB, no_rw_check" *) logic [IC_TAGW-1:0] ic_tag  [0:ICACHE_ENTRIES-1];
	(* ramstyle = "MLAB, no_rw_check" *) logic [31:0]        ic_data [0:ICACHE_ENTRIES-1];

	wire [IC_IDX-1:0] ic_i   = mem_addr[2+IC_IDX-1:2];
	wire [IC_TAGW-1:0] ic_t  = mem_addr[31:2+IC_IDX];
	wire ic_en   = (ICACHE_ENTRIES > 0) && cacr[0];          // E
	wire ic_hit  = FETCH_CHR && fetch_1lw && !chr_hit && ic_en &&
				   ic_valid[ic_i] && (ic_tag[ic_i] == ic_t);

	// A write over a held long word drops the entry, as for the CHR: the real
	// part leaves this to software clearing CACR, and code runs from ROM, so it
	// is only a safeguard.
	wire ic_stale = mem_we && ic_en && ic_valid[ic_i] && (ic_tag[ic_i] == ic_t);

	assign bus_req   = (state == S_MEM) && !chr_hit && !ic_hit;
	assign bus_ifetch = mem_is_fetch;
	assign bus_ifetch_hit = (state == S_MEM) && (chr_hit || ic_hit);

	// EA engine
	logic [2:0]  ea_mode, ea_reg;
	logic        ea_which;      // 0 = src slot, 1 = dst slot
	state_t      ea_ret;
	logic [31:0] ea_base;
	logic [15:0] ea_ext;

	logic [1:0]  eaS_kind, eaD_kind;   // 0 reg_d, 1 reg_a, 2 mem, 3 imm
	logic [2:0]  eaS_reg,  eaD_reg;
	logic [31:0] eaS_addr, eaD_addr;
	logic [31:0] eaS_imm;

	// working values
	logic [31:0] val_src, val_dst, result;
	logic [31:0] imm_val;
	logic [31:0] tmp;
	logic        bcc_imm8;
	logic        flags_then_wb;   // FLAGS_SAMPLE -> WRITE_RESULT vs DONE
	logic [1:0]  wb_sel;          // 0 = eaD, 1 = D[reg9], 2 = A[reg9] long
	logic [5:0]  shift_cnt;
	logic [31:0] shift_val;
	logic        shift_c;
	// The 64/32 restoring divider needs 97 bits of state: R[32:0] + Q[63:0].
	// A packed 64-bit {rem,quot} only works for 32 iterations, so the remainder
	// is a separate register.
	logic [63:0] div_rem_q;      // dividend out, quotient in; also the 64-bit product
	logic [32:0] div_rem;        // partial remainder
	logic [31:0] div_divisor;
	logic [6:0]  div_cnt;
	logic        div_neg_q, div_neg_r;
	logic        div_word;       // 1 = word-form finish ({r16,q16} pack)
	logic [15:0] mdl_ext;
	logic [15:0] irq_old_sr;     // SR value captured at exception entry
	logic [2:0]  irq_level_r;
	logic        rte_is_rtr;     // RTR restores CCR only, not the whole SR
	logic [15:0] rte_sr_r;       // popped SR held until after the PC pop
	logic [15:0] movem_list;
	logic [4:0]  movem_idx;
	logic [31:0] movem_addr;

	assign insn_pc = insn_pc_r;
	logic [31:0] insn_pc_r;

	// MOVEM register number for the current bit index (predecrement stores
	// map bit i to register 15-i, everything else maps bit i to D0..A7)
	logic [3:0] movem_regnum;
	always_comb begin
		if (grp == G_MOVEM && !ir[10] && earlo_mode == 3'd4)
			movem_regnum = 4'd15 - movem_idx[3:0];
		else
			movem_regnum = movem_idx[3:0];
	end

	// EXG register numbers
	logic [3:0] exg_x_num, exg_y_num;
	always_comb begin
		case (ir[7:3])
			5'b01000: begin exg_x_num = {1'b0, reg9}; exg_y_num = {1'b0, earlo_reg}; end
			5'b01001: begin exg_x_num = {1'b1, reg9}; exg_y_num = {1'b1, earlo_reg}; end
			default:  begin exg_x_num = {1'b0, reg9}; exg_y_num = {1'b1, earlo_reg}; end
		endcase
	end

	// read port muxing
	always_comb begin
		rf_rd_a_num = {1'b0, reg9};
		rf_rd_b_num = {1'b1, ea_reg};
		case (state)
			S_EA_START:   rf_rd_b_num = {1'b1, ea_reg};
			S_EA_IDX, S_EA_BD_GOT:
						  rf_rd_a_num = {ea_ext[15], ea_ext[14:12]};
			S_SRC_READ:   rf_rd_a_num = {(eaS_kind == 2'd1), eaS_reg};
			S_RMW_CHK, S_RMW_READ:
						  rf_rd_a_num = {(eaD_kind == 2'd1), eaD_reg};
			S_BINOP_EXEC: begin
						  rf_rd_a_num = {1'b0, reg9};
						  rf_rd_b_num = {1'b1, reg9};
			end
			S_MULW_EXEC:  rf_rd_a_num = {1'b0, reg9};
			S_DIVW_EXEC:  rf_rd_a_num = {1'b0, reg9};
			S_MDL_EXEC: begin
						  // M68000PRM: bits 14-12 = DL/DQ, bits 2-0 = DH/DR.
						  rf_rd_a_num = {1'b0, mdl_ext[14:12]};  // DL / DQ
						  rf_rd_b_num = {1'b0, mdl_ext[2:0]};    // DH / DR
			end
			S_DISPATCH:   rf_rd_a_num = {1'b0, ir[11:9]};
			S_DBCC_GOT:   rf_rd_a_num = {1'b0, earlo_reg};
			S_SHIFT_INIT: begin
						  rf_rd_a_num = {1'b0, reg9};
						  rf_rd_b_num = {1'b0, earlo_reg};
			end
			S_PEA_PUSH, S_JSR_PUSH, S_BSR_PUSH, S_RTS_RD,
			S_IRQ_PUSH_PC, S_IRQ_PUSH_SR, S_RTE_RD_SR, S_RTE_GOT_PC:
						  rf_rd_b_num = 4'd15;
			S_LINK_PUSH: begin
						  rf_rd_a_num = {1'b1, earlo_reg};
						  rf_rd_b_num = 4'd15;
			end
			S_LINK_SETAN, S_LINK_DISP_GOT, S_MOVEM_FIN:
						  rf_rd_b_num = 4'd15;
			S_UNLK_SETSP: rf_rd_a_num = {1'b1, earlo_reg};
			S_UNLK_GOT:   ;
			S_CMPM_RD1:   rf_rd_b_num = {1'b1, earlo_reg};
			S_CMPM_RD2:   rf_rd_b_num = {1'b1, reg9};
			S_MOVEM_LIST_GOT: rf_rd_b_num = {1'b1, earlo_reg};
			S_MOVEM_NEXT: rf_rd_a_num = movem_regnum;
			S_MOVEC_EXEC: rf_rd_a_num = ea_ext[15:12];
			S_REGOP_EXEC: begin
						  if (grp == G_MOVE_USP) rf_rd_a_num = {1'b1, earlo_reg};
						  else                   rf_rd_a_num = {1'b0, earlo_reg};
			end
			S_EXG1, S_EXG2: begin
						  rf_rd_a_num = exg_x_num;
						  rf_rd_b_num = exg_y_num;
			end
			S_LEA_EXEC:   ;
			default: ;
		endcase
	end

	// main machine
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_RESET_SSP;
			pc <= 32'd0;
			s_bit <= 1'b1;
			ipl <= 3'd7;
			{fN, fZ, fV, fC, fX} <= '0;
			cacr <= 0; vbr <= 0; sfc <= 0; dfc <= 0; caar <= 0; msp_r <= 0; isp_r <= 0;
			insn_done <= 1'b0;
			unimplemented <= 1'b0;
			reset_out     <= 1'b0;
			rf_wr_en <= 1'b0;
			usp_wr <= 1'b0; ssp_wr <= 1'b0;
			mem_we <= 1'b0;
			mem_is_fetch <= 1'b0;
			chr_valid <= 1'b0; chr_tag <= 30'd0; chr_data <= 32'd0;
			for (int k = 0; k < ICACHE_ENTRIES; k++) ic_valid[k] <= 1'b0;
		end else begin
			insn_done <= 1'b0;
			reset_out <= 1'b0;   // one-cycle pulse
			rf_wr_en <= 1'b0;
			usp_wr <= 1'b0; ssp_wr <= 1'b0;

			case (state)
				// reset
				S_RESET_SSP: begin
					mem_addr <= 32'd0; mem_size <= 3'd4; mem_we <= 1'b0;
					mem_is_fetch <= 1'b0; mem_ret <= S_RESET_PC;
					state <= S_MEM;
				end
				S_RESET_PC: begin
					ssp_wr <= 1'b1; sp_wr_data <= mem_rdata_r;
					mem_addr <= 32'd4; mem_size <= 3'd4; mem_we <= 1'b0;
					mem_is_fetch <= 1'b0; mem_ret <= S_RESET_GO;
					state <= S_MEM;
				end
				S_RESET_GO: begin
					pc <= mem_rdata_r;
					state <= S_FETCH;
				end

				// generic memory transaction
				S_MEM: begin
					if (chr_hit) begin
						// served internally -- `bus_req` never asserted
						mem_rdata_r <= lw_extract(chr_data, mem_size,
												  mem_addr[1]);
						pc <= pc + {29'd0, mem_size};   // always a fetch
						mem_we <= 1'b0;
						mem_is_fetch <= 1'b0;
						state <= mem_ret;
					end else if (ic_hit) begin
						// cache hit. Also loads the CHR, as the real part does
						// (UM 1.6), so the next sequential word is free.
						mem_rdata_r <= lw_extract(ic_data[ic_i], mem_size,
												  mem_addr[1]);
						chr_valid <= 1'b1;
						chr_tag   <= mem_addr[31:2];
						chr_data  <= ic_data[ic_i];
						pc <= pc + {29'd0, mem_size};
						mem_we <= 1'b0;
						mem_is_fetch <= 1'b0;
						state <= mem_ret;
					end else if (bus_ack) begin
						if (chr_fill) begin
							chr_valid <= 1'b1;
							chr_tag   <= mem_addr[31:2];
							chr_data  <= bus_rdata;
							mem_rdata_r <= lw_extract(bus_rdata, mem_size,
													  mem_addr[1]);
							// fill the cache too, unless CACR bit 1 (F, freeze)
							// is set
							if (ic_en && !cacr[1]) begin
								ic_valid[ic_i] <= 1'b1;
								ic_tag[ic_i]   <= ic_t;
								ic_data[ic_i]  <= bus_rdata;
							end
						end else begin
							mem_rdata_r <= bus_rdata & szmask(mem_size);
							// Stricter than the real part, which leaves coherency
							// to software clearing CACR: a write over the held long
							// word drops it. Code runs from ROM, so this only
							// guards against executing out of RAM.
							if (mem_we && chr_valid &&
								chr_tag == mem_addr[31:2])
								chr_valid <= 1'b0;
							if (ic_stale) ic_valid[ic_i] <= 1'b0;
						end
						if (mem_is_fetch) pc <= pc + {29'd0, mem_size};
						mem_we <= 1'b0;
						mem_is_fetch <= 1'b0;
						state <= mem_ret;
					end
				end

				// fetch / decode / dispatch
				S_FETCH: begin
					if (ipl_in != 3'd0 && ipl_in > ipl) begin
						// autovectored interrupt entry, as the model's
						// _take_interrupt: capture SR, enter supervisor (the
						// regfile banks A7), push PC (long) then SR (word),
						// raise the mask, vector through VBR + (24+level)*4.
						insn_pc_r   <= pc;    // golden records the interrupted PC
						irq_old_sr  <= sr_out;
						irq_level_r <= ipl_in;
						s_bit <= 1'b1;
						state <= S_IRQ_PUSH_PC;
					end else begin
						insn_pc_r <= pc;
						mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
						mem_is_fetch <= 1'b1; mem_ret <= S_DECODE;
						state <= S_MEM;
					end
				end
				S_DECODE: begin
					ir <= mem_rdata_r[15:0];
					state <= S_DISPATCH;
				end
				S_DISPATCH: begin
					grp <= dec_grp; opsize <= dec_size;
					binop <= dec_binop; is_add <= dec_is_add;
					ea_is_dst <= ir[8];
					reg9 <= ir[11:9]; cc <= ir[11:8];
					earlo_mode <= ir[5:3]; earlo_reg <= ir[2:0];
					eahi_mode <= ir[8:6]; eahi_reg <= ir[11:9];

					case (dec_grp)
						G_MOVE: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b0; ea_ret <= S_SRC_READ;
							state <= S_EA_START;
						end
						G_MOVEQ: begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, ir[11:9]};
							rf_wr_size <= 2'd2; rf_wr_data <= sext8(ir[7:0]);
							fN <= ir[7]; fZ <= (ir[7:0] == 8'd0);
							fV <= 1'b0; fC <= 1'b0;
							state <= S_DONE;
						end
						G_IMMALU, G_CCR_SR_IMM: begin
							mem_addr <= pc;
							mem_size <= (dec_size == 3'd4) ? 3'd4 : 3'd2;
							mem_we <= 1'b0; mem_is_fetch <= 1'b1;
							mem_ret <= S_IMM_GOT;
							state <= S_MEM;
						end
						G_CLRGRP: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1;
							ea_ret <= (dec_binop == 3'd1) ? S_CLR_EXEC : S_RMW_READ;
							state <= S_EA_START;
						end
						G_TST: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_RMW_READ;
							state <= S_EA_START;
						end
						G_LEA: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_LEA_EXEC;
							state <= S_EA_START;
						end
						G_JMP: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_JMP_GO;
							state <= S_EA_START;
						end
						G_PEA: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_PEA_PUSH;
							state <= S_EA_START;
						end
						G_JSR: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_JSR_PUSH;
							state <= S_EA_START;
						end
						G_SWAP, G_EXT, G_MOVE_USP: state <= S_REGOP_EXEC;
						G_EXG: state <= S_EXG1;
						G_MOVEC: begin
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_MOVEC_GOT;
							state <= S_MEM;
						end
						G_NOP: state <= S_DONE;
						// pulse RESET# to the peripherals; no register, flag
						// or PC effect
						G_RESET: begin
							reset_out <= 1'b1;
							state     <= S_DONE;
						end
						G_RTS: state <= S_RTS_RD;
						G_RTE: begin rte_is_rtr <= 1'b0; state <= S_RTE_RD_SR; end
						G_RTR: begin rte_is_rtr <= 1'b1; state <= S_RTE_RD_SR; end
						G_ADDQSUBQ: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_RMW_CHK;
							state <= S_EA_START;
						end
						G_SCC: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_WRITE_RESULT;
							result <= test_cc(ir[11:8]) ? 32'hff : 32'h00;
							wb_sel <= 2'd0;
							state <= S_EA_START;
						end
						G_DBCC: begin
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_DBCC_GOT;
							state <= S_MEM;
						end
						G_BCC: begin
							if (ir[7:0] == 8'h00) begin
								bcc_imm8 <= 1'b0;
								mem_addr <= pc; mem_size <= 3'd2;
								mem_we <= 1'b0; mem_is_fetch <= 1'b1;
								mem_ret <= S_BCC_EXEC;
								state <= S_MEM;
							end else if (ir[7:0] == 8'hFF) begin
								bcc_imm8 <= 1'b0;
								mem_addr <= pc; mem_size <= 3'd4;
								mem_we <= 1'b0; mem_is_fetch <= 1'b1;
								mem_ret <= S_BCC_EXEC;
								state <= S_MEM;
							end else begin
								bcc_imm8 <= 1'b1;
								tmp <= sext8(ir[7:0]);
								state <= S_BCC_EXEC;
							end
						end
						G_BINOP, G_MULW, G_DIVW: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b0; ea_ret <= S_SRC_READ;
							state <= S_EA_START;
						end
						G_SHIFT: state <= S_SHIFT_INIT;
						G_MOVEM: begin
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_MOVEM_LIST_GOT;
							state <= S_MEM;
						end
						G_TO_CCR, G_TO_SR: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b0; ea_ret <= S_SRC_READ;
							opsize <= 3'd2;
							state <= S_EA_START;
						end
						G_FROM_SR: begin
							ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
							ea_which <= 1'b1; ea_ret <= S_FROMSR_WRITE;
							opsize <= 3'd2;
							state <= S_EA_START;
						end
						G_MULDIVL: begin
							// stream order per M68000PRM: ext word first, then
							// the EA extension words
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_MDL_EXT_GOT;
							state <= S_MEM;
						end
						G_BITOP: begin
							if (ir[8]) begin
								// dynamic: bit number from D[ir[11:9]], read now
								tmp <= rf_rd_a_data;
								ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
								ea_which <= 1'b1; ea_ret <= S_RMW_READ;
								state <= S_EA_START;
							end else begin
								// static: bit number is an immediate word,
								// fetched before the EA (matches the model)
								mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
								mem_is_fetch <= 1'b1; mem_ret <= S_BITNO_GOT;
								state <= S_MEM;
							end
						end
						G_CMPM: state <= S_CMPM_RD1;
						G_LINK: state <= S_LINK_PUSH;
						G_UNLK: state <= S_UNLK_SETSP;
						default: begin
							unimplemented <= 1'b1;
							unimpl_opcode <= ir;
							unimpl_pc <= insn_pc_r;
							state <= S_HALT;
						end
					endcase
				end

				// immediate value fetched
				S_IMM_GOT: begin
					imm_val <= (opsize == 3'd1) ? {24'd0, mem_rdata_r[7:0]} : mem_rdata_r;
					if (grp == G_CCR_SR_IMM) begin
						val_src <= (opsize == 3'd1) ? {24'd0, mem_rdata_r[7:0]} : mem_rdata_r;
						state <= S_TOSR_EXEC;
					end else begin
						ea_mode <= earlo_mode; ea_reg <= earlo_reg;
						ea_which <= 1'b1; ea_ret <= S_RMW_READ;
						state <= S_EA_START;
					end
				end

				S_BITNO_GOT: begin
					tmp <= {16'd0, mem_rdata_r[15:0]};
					ea_mode <= earlo_mode; ea_reg <= earlo_reg;
					ea_which <= 1'b1; ea_ret <= S_RMW_READ;
					state <= S_EA_START;
				end

				// EA engine (mirrors cpu68020.decode_ea, including fetch order and
				// address-register update timing)
				S_EA_START: begin
					case (ea_mode)
						3'd0: begin
							if (!ea_which) begin eaS_kind <= 2'd0; eaS_reg <= ea_reg; end
							else           begin eaD_kind <= 2'd0; eaD_reg <= ea_reg; end
							state <= ea_ret;
						end
						3'd1: begin
							if (!ea_which) begin eaS_kind <= 2'd1; eaS_reg <= ea_reg; end
							else           begin eaD_kind <= 2'd1; eaD_reg <= ea_reg; end
							state <= ea_ret;
						end
						3'd2: begin
							if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= rf_rd_b_data; end
							else           begin eaD_kind <= 2'd2; eaD_addr <= rf_rd_b_data; end
							state <= ea_ret;
						end
						3'd3: begin
							if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= rf_rd_b_data; end
							else           begin eaD_kind <= 2'd2; eaD_addr <= rf_rd_b_data; end
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, ea_reg};
							rf_wr_size <= 2'd2;
							rf_wr_data <= rf_rd_b_data + {29'd0, opsize};
							state <= ea_ret;
						end
						3'd4: begin
							logic [31:0] na;
							na = rf_rd_b_data -
								 ((ea_reg == 3'd7 && opsize == 3'd1) ? 32'd2 : {29'd0, opsize});
							if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= na; end
							else           begin eaD_kind <= 2'd2; eaD_addr <= na; end
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, ea_reg};
							rf_wr_size <= 2'd2; rf_wr_data <= na;
							state <= ea_ret;
						end
						3'd5, 3'd6: begin
							ea_base <= rf_rd_b_data;
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_EA_EXT_GOT;
							state <= S_MEM;
						end
						default: begin // mode 7
							case (ea_reg)
								3'd0, 3'd2, 3'd3: begin
									ea_base <= pc;   // used by 7/2 and 7/3 only
									mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
									mem_is_fetch <= 1'b1; mem_ret <= S_EA_EXT_GOT;
									state <= S_MEM;
								end
								3'd1: begin
									mem_addr <= pc; mem_size <= 3'd4; mem_we <= 1'b0;
									mem_is_fetch <= 1'b1; mem_ret <= S_EA_EXT_GOT;
									state <= S_MEM;
								end
								3'd4: begin
									mem_addr <= pc;
									mem_size <= (opsize == 3'd4) ? 3'd4 : 3'd2;
									mem_we <= 1'b0; mem_is_fetch <= 1'b1;
									mem_ret <= S_EA_EXT_GOT;
									state <= S_MEM;
								end
								default: begin
									unimplemented <= 1'b1;
									unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
									state <= S_HALT;
								end
							endcase
						end
					endcase
				end
				S_EA_EXT_GOT: begin
					case (ea_mode)
						3'd5: begin
							if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= ea_base + sext16(mem_rdata_r[15:0]); end
							else           begin eaD_kind <= 2'd2; eaD_addr <= ea_base + sext16(mem_rdata_r[15:0]); end
							state <= ea_ret;
						end
						3'd6: begin
							ea_ext <= mem_rdata_r[15:0];
							state <= S_EA_IDX;
						end
						default: begin // mode 7
							case (ea_reg)
								3'd0: begin
									if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= sext16(mem_rdata_r[15:0]); end
									else           begin eaD_kind <= 2'd2; eaD_addr <= sext16(mem_rdata_r[15:0]); end
									state <= ea_ret;
								end
								3'd1: begin
									if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= mem_rdata_r; end
									else           begin eaD_kind <= 2'd2; eaD_addr <= mem_rdata_r; end
									state <= ea_ret;
								end
								3'd2: begin
									if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= ea_base + sext16(mem_rdata_r[15:0]); end
									else           begin eaD_kind <= 2'd2; eaD_addr <= ea_base + sext16(mem_rdata_r[15:0]); end
									state <= ea_ret;
								end
								3'd3: begin
									ea_ext <= mem_rdata_r[15:0];
									state <= S_EA_IDX;
								end
								default: begin // immediate (src only)
									eaS_kind <= 2'd3;
									eaS_imm <= (opsize == 3'd1) ? {24'd0, mem_rdata_r[7:0]} : mem_rdata_r;
									state <= ea_ret;
								end
							endcase
						end
					endcase
				end
				S_EA_IDX: begin
					if (!ea_ext[8]) begin
						// brief extension word
						logic [31:0] xv;
						xv = ea_ext[11] ? rf_rd_a_data : sext16(rf_rd_a_data[15:0]);
						if (!ea_which) begin
							eaS_kind <= 2'd2;
							eaS_addr <= ea_base + sext8(ea_ext[7:0]) + (xv << ea_ext[10:9]);
						end else begin
							eaD_kind <= 2'd2;
							eaD_addr <= ea_base + sext8(ea_ext[7:0]) + (xv << ea_ext[10:9]);
						end
						state <= ea_ret;
					end else begin
						// full extension word, I/IS == 0 only (matches the model)
						if (ea_ext[2:0] != 3'd0) begin
							unimplemented <= 1'b1;
							unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
							state <= S_HALT;
						end else if (ea_ext[5:4] == 2'd2) begin
							mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_EA_BD_GOT;
							state <= S_MEM;
						end else if (ea_ext[5:4] == 2'd3) begin
							mem_addr <= pc; mem_size <= 3'd4; mem_we <= 1'b0;
							mem_is_fetch <= 1'b1; mem_ret <= S_EA_BD_GOT;
							state <= S_MEM;
						end else begin
							// bd = 0
							logic [31:0] xv, be;
							be = ea_ext[7] ? 32'd0 : ea_base;
							if (ea_ext[6]) xv = 32'd0;
							else begin
								xv = ea_ext[11] ? rf_rd_a_data : sext16(rf_rd_a_data[15:0]);
								xv = xv << ea_ext[10:9];
							end
							if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= be + xv; end
							else           begin eaD_kind <= 2'd2; eaD_addr <= be + xv; end
							state <= ea_ret;
						end
					end
				end
				S_EA_BD_GOT: begin
					logic [31:0] bd, xv, be;
					bd = (mem_size == 3'd2) ? sext16(mem_rdata_r[15:0]) : mem_rdata_r;
					be = ea_ext[7] ? 32'd0 : ea_base;
					if (ea_ext[6]) xv = 32'd0;
					else begin
						xv = ea_ext[11] ? rf_rd_a_data : sext16(rf_rd_a_data[15:0]);
						xv = xv << ea_ext[10:9];
					end
					if (!ea_which) begin eaS_kind <= 2'd2; eaS_addr <= be + bd + xv; end
					else           begin eaD_kind <= 2'd2; eaD_addr <= be + bd + xv; end
					state <= ea_ret;
				end

				// source operand
				S_SRC_READ: begin
					case (eaS_kind)
						2'd0, 2'd1: begin
							val_src <= rf_rd_a_data & szmask(opsize);
							state <= S_SRC_DISPATCH;
						end
						2'd3: begin
							val_src <= eaS_imm;
							state <= S_SRC_DISPATCH;
						end
						default: begin
							mem_addr <= eaS_addr; mem_size <= opsize;
							mem_we <= 1'b0; mem_is_fetch <= 1'b0;
							mem_ret <= S_SRC_MEM_GOT;
							state <= S_MEM;
						end
					endcase
				end
				S_SRC_MEM_GOT: begin
					val_src <= mem_rdata_r;
					state <= S_SRC_DISPATCH;
				end
				S_SRC_DISPATCH: begin
					case (grp)
						G_MULDIVL: state <= S_MDL_EXEC;   // ext word fetched at dispatch
						G_MOVE:            state <= S_MOVE_DST;
						G_TO_CCR, G_TO_SR: state <= S_TOSR_EXEC;
						G_MULW:            state <= S_MULW_EXEC;
						G_DIVW:            state <= S_DIVW_EXEC;
						default:           state <= S_BINOP_EXEC;
					endcase
				end

				// MOVE
				S_MOVE_DST: begin
					if (eahi_mode == 3'd1) begin
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, eahi_reg};
						rf_wr_size <= 2'd2;
						rf_wr_data <= sext_sz(val_src, opsize);
						state <= S_DONE;
					end else begin
						ea_mode <= eahi_mode; ea_reg <= eahi_reg;
						ea_which <= 1'b1; ea_ret <= S_MOVE_WRITE;
						state <= S_EA_START;
					end
				end
				S_MOVE_WRITE: begin
					fN <= msb_of(val_src, opsize);
					fZ <= ((val_src & szmask(opsize)) == 32'd0);
					fV <= 1'b0; fC <= 1'b0;
					if (eaD_kind == 2'd0) begin
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, eaD_reg};
						rf_wr_size <= flg_sz_of(opsize); rf_wr_data <= val_src;
						state <= S_DONE;
					end else begin
						mem_addr <= eaD_addr; mem_size <= opsize;
						mem_we <= 1'b1; mem_wdata <= val_src & szmask(opsize);
						mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
						state <= S_MEM;
					end
				end

				// RMW path (IMMALU / NEG / NOT / NEGX / TST / ADDQ non-A)
				S_RMW_CHK: begin
					// ADDQ/SUBQ to an address register: full-width, no flags.
					// Sum via a local (see BOP_A note).
					logic [31:0] addq_r;
					if (eaD_kind == 2'd1) begin
						addq_r = is_add ? (rf_rd_a_data + qdata())
										: (rf_rd_a_data - qdata());
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, eaD_reg};
						rf_wr_size <= 2'd2;
						rf_wr_data <= addq_r;
						state <= S_DONE;
					end else begin
						state <= S_RMW_READ;
					end
				end
				S_RMW_READ: begin
					case (eaD_kind)
						2'd0, 2'd1: begin
							val_dst <= rf_rd_a_data & szmask(opsize);
							state <= S_RMW_EXEC;
						end
						default: begin
							mem_addr <= eaD_addr; mem_size <= opsize;
							mem_we <= 1'b0; mem_is_fetch <= 1'b0;
							mem_ret <= S_RMW_MEM_GOT;
							state <= S_MEM;
						end
					endcase
				end
				S_RMW_MEM_GOT: begin
					val_dst <= mem_rdata_r;
					state <= S_RMW_EXEC;
				end
				S_RMW_EXEC: begin
					logic [31:0] r;
					wb_sel <= 2'd0;
					case (grp)
						G_BITOP: state <= S_BIT_EXEC;
						G_TST: begin
							fN <= msb_of(val_dst, opsize);
							fZ <= ((val_dst & szmask(opsize)) == 32'd0);
							fV <= 1'b0; fC <= 1'b0;
							state <= S_DONE;
						end
						G_CLRGRP: begin
							// binop: 2 = NEG, 0 = NEGX (approx), 3 = NOT
							if (binop == 3'd3) begin
								r = (~val_dst) & szmask(opsize);
								result <= r;
								fN <= msb_of(r, opsize); fZ <= (r == 32'd0);
								fV <= 1'b0; fC <= 1'b0;
								state <= S_WRITE_RESULT;
							end else begin
								r = (32'd0 - val_dst) & szmask(opsize);
								result <= r;
								flg_a <= 32'd0; flg_b <= val_dst; flg_res <= r;
								flg_sz <= flg_sz_of(opsize); flg_op <= FOP_SUB;
								flags_then_wb <= 1'b1;
								state <= S_FLAGS_SAMPLE;
							end
						end
						G_ADDQSUBQ: begin
							r = is_add ? ((val_dst + qdata()) & szmask(opsize))
									   : ((val_dst - qdata()) & szmask(opsize));
							result <= r;
							flg_a <= val_dst; flg_b <= qdata(); flg_res <= r;
							flg_sz <= flg_sz_of(opsize);
							flg_op <= is_add ? FOP_ADD : FOP_SUB;
							flags_then_wb <= 1'b1;
							state <= S_FLAGS_SAMPLE;
						end
						default: begin // G_IMMALU by reg9
							case (reg9)
								3'd0: begin // ORI
									r = (val_dst | imm_val) & szmask(opsize);
									result <= r;
									fN <= msb_of(r, opsize); fZ <= (r == 32'd0);
									fV <= 1'b0; fC <= 1'b0;
									state <= S_WRITE_RESULT;
								end
								3'd1: begin // ANDI
									r = (val_dst & imm_val) & szmask(opsize);
									result <= r;
									fN <= msb_of(r, opsize); fZ <= (r == 32'd0);
									fV <= 1'b0; fC <= 1'b0;
									state <= S_WRITE_RESULT;
								end
								3'd5: begin // EORI
									r = (val_dst ^ imm_val) & szmask(opsize);
									result <= r;
									fN <= msb_of(r, opsize); fZ <= (r == 32'd0);
									fV <= 1'b0; fC <= 1'b0;
									state <= S_WRITE_RESULT;
								end
								3'd2: begin // SUBI
									r = (val_dst - imm_val) & szmask(opsize);
									result <= r;
									flg_a <= val_dst; flg_b <= imm_val; flg_res <= r;
									flg_sz <= flg_sz_of(opsize); flg_op <= FOP_SUB;
									flags_then_wb <= 1'b1;
									state <= S_FLAGS_SAMPLE;
								end
								3'd3: begin // ADDI
									r = (val_dst + imm_val) & szmask(opsize);
									result <= r;
									flg_a <= val_dst; flg_b <= imm_val; flg_res <= r;
									flg_sz <= flg_sz_of(opsize); flg_op <= FOP_ADD;
									flags_then_wb <= 1'b1;
									state <= S_FLAGS_SAMPLE;
								end
								default: begin // CMPI
									r = (val_dst - imm_val) & szmask(opsize);
									flg_a <= val_dst; flg_b <= imm_val; flg_res <= r;
									flg_sz <= flg_sz_of(opsize); flg_op <= FOP_CMP;
									flags_then_wb <= 1'b0;
									state <= S_FLAGS_SAMPLE;
								end
							endcase
						end
					endcase
				end

				S_BIT_EXEC: begin
					logic [4:0] bn;
					logic [31:0] bm, r;
					bn = (opsize == 3'd4) ? tmp[4:0] : {2'd0, tmp[2:0]};
					bm = 32'd1 << bn;
					fZ <= ((val_dst & bm) == 32'd0);
					case (ir[7:6])
						2'd1: r = val_dst ^ bm;   // BCHG
						2'd2: r = val_dst & ~bm;  // BCLR
						2'd3: r = val_dst | bm;   // BSET
						default: r = val_dst;     // BTST
					endcase
					result <= r;
					wb_sel <= 2'd0;
					state <= (ir[7:6] == 2'd0) ? S_DONE : S_WRITE_RESULT;
				end

				// MULU.L/MULS.L/DIVU.L/DIVS.L
				S_MDL_EXT_GOT: begin
					mdl_ext <= mem_rdata_r[15:0];
					ea_mode <= ir[5:3]; ea_reg <= ir[2:0];
					ea_which <= 1'b0; ea_ret <= S_SRC_READ;
					state <= S_EA_START;
				end
				S_MDL_EXEC: begin
					// rf port A = dl_or_dq (D register), port B = dh_or_dr
					if (!ir[6]) begin
						// multiply: 32x32 -> 64
						logic [63:0] prod;
						if (mdl_ext[11])
							prod = $unsigned($signed({{32{rf_rd_a_data[31]}}, rf_rd_a_data})
											 * $signed({{32{val_src[31]}}, val_src}));
						else
							prod = {32'd0, rf_rd_a_data} * {32'd0, val_src};
						div_rem_q <= prod;   // reuse as {high, low}
						if (mdl_ext[10]) begin
							// 64-bit result: write DL first, DH in S_DIV_FIN
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, mdl_ext[14:12]};
							rf_wr_size <= 2'd2; rf_wr_data <= prod[31:0];
							fV <= 1'b0;
							fN <= prod[31]; fZ <= (prod[31:0] == 32'd0); fC <= 1'b0;
							state <= S_DIV_FIN;   // shared: writes DH
						end else begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, mdl_ext[14:12]};
							rf_wr_size <= 2'd2; rf_wr_data <= prod[31:0];
							if (mdl_ext[11])
								fV <= !((prod[63:31] == 33'd0) || (prod[63:31] == {33{1'b1}}));
							else
								fV <= (prod[63:32] != 32'd0);
							fN <= prod[31]; fZ <= (prod[31:0] == 32'd0); fC <= 1'b0;
							state <= S_DONE;
						end
					end else begin
						// divide: set up an iterative 64/32 restoring divider
						logic [63:0] dvd;
						logic [31:0] dvs;
						logic negq, negr;
						if (val_src == 32'd0) begin
							// divide by zero would trap; exceptions aren't
							// implemented, so halt visibly instead of guessing
							unimplemented <= 1'b1;
							unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
							state <= S_HALT;
						end else begin
							if (mdl_ext[10])
								dvd = {rf_rd_b_data, rf_rd_a_data};   // {DH, DL}
							else if (mdl_ext[11])
								dvd = {{32{rf_rd_a_data[31]}}, rf_rd_a_data};
							else
								dvd = {32'd0, rf_rd_a_data};
							dvs = val_src;
							negq = 1'b0; negr = 1'b0;
							if (mdl_ext[11]) begin
								negr = dvd[63];
								negq = dvd[63] ^ dvs[31];
								if (dvd[63]) dvd = (~dvd) + 64'd1;
								if (dvs[31]) dvs = (~dvs) + 32'd1;
							end
							div_rem_q <= dvd;    // quotient shifts in at bit 0
							div_rem <= 33'd0;
							div_divisor <= dvs;
							div_word <= 1'b0;
							div_neg_q <= negq; div_neg_r <= negr;
							div_cnt <= 7'd64;
							state <= S_DIV_STEP;
						end
					end
				end
				S_DIVW_EXEC: begin
					// DIVU.W / DIVS.W: 32-bit dividend in D[reg9], 16-bit
					// divisor from <ea>; quotient -> Dn[15:0], remainder ->
					// Dn[31:16]. Overflow (quotient beyond 16 bits) sets V
					// and leaves Dn and the other flags unchanged (as the
					// model). Reuses the iterative divider, with div_word
					// steering the finish.
					logic [63:0] dvdw;
					logic [31:0] dvsw;
					logic negqw, negrw;
					if (val_src[15:0] == 16'd0) begin
						// zero divide would trap (vector 5); only IRQs are
						// implemented, so halt visibly as the DIVL path does
						unimplemented <= 1'b1;
						unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
						state <= S_HALT;
					end else begin
						if (ir[8]) begin
							dvdw = {{32{rf_rd_a_data[31]}}, rf_rd_a_data};
							dvsw = {{16{val_src[15]}}, val_src[15:0]};
						end else begin
							dvdw = {32'd0, rf_rd_a_data};
							dvsw = {16'd0, val_src[15:0]};
						end
						negqw = 1'b0; negrw = 1'b0;
						if (ir[8]) begin
							negrw = dvdw[63];
							negqw = dvdw[63] ^ dvsw[31];
							if (dvdw[63]) dvdw = (~dvdw) + 64'd1;
							if (dvsw[31]) dvsw = (~dvsw) + 32'd1;
						end
						div_rem_q <= dvdw; div_rem <= 33'd0;
						div_divisor <= dvsw;
						div_neg_q <= negqw; div_neg_r <= negrw;
						div_word <= 1'b1;
						div_cnt <= 7'd64;
						state <= S_DIV_STEP;
					end
				end
				S_DIV_STEP: begin
					// restoring long division, one bit per cycle: shift
					// {rem,quot} left; if rem >= divisor, subtract and set q0.
					logic [32:0] rsh;
					rsh = {div_rem[31:0], div_rem_q[63]};
					if (rsh >= {1'b0, div_divisor}) begin
						div_rem   <= rsh - {1'b0, div_divisor};
						div_rem_q <= {div_rem_q[62:0], 1'b1};
					end else begin
						div_rem   <= rsh;
						div_rem_q <= {div_rem_q[62:0], 1'b0};
					end
					div_cnt <= div_cnt - 7'd1;
					if (div_cnt == 7'd1) state <= S_DIV_FIN;
				end
				S_DIV_FIN: begin
					if (!ir[6]) begin
						// multiply 64-bit finish: DH write, skipped when Dh==Dl
						// (undefined on real HW; the low half wins, as in the
						// model)
						if (mdl_ext[2:0] != mdl_ext[14:12]) begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, mdl_ext[2:0]};
							rf_wr_size <= 2'd2; rf_wr_data <= div_rem_q[63:32];
						end
						state <= S_DONE;
					end else if (div_word) begin
						// DIVx.W finish: pack {r16, q16} into D[reg9]
						logic [31:0] qw, rw;
						qw = div_neg_q ? ((~div_rem_q[31:0]) + 32'd1) : div_rem_q[31:0];
						rw = div_neg_r ? ((~div_rem[31:0]) + 32'd1) : div_rem[31:0];
						div_word <= 1'b0;
						if (ir[8] ? (($signed(qw) > 32'sd32767) ||
									 ($signed(qw) < -32'sd32768))
								  : (qw > 32'h0000FFFF)) begin
							fV <= 1'b1;    // Dn and N/Z/C untouched
							state <= S_DONE;
						end else begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, reg9};
							rf_wr_size <= 2'd2;
							rf_wr_data <= {rw[15:0], qw[15:0]};
							fN <= qw[15]; fZ <= (qw[15:0] == 16'd0);
							fV <= 1'b0; fC <= 1'b0;
							state <= S_DONE;
						end
					end else begin
						// divide finish: apply signs, write DL=quotient this
						// cycle and DH=remainder in S_DIV_WRREM (one regfile
						// write per cycle)
						logic [31:0] q, r;
						q = div_neg_q ? ((~div_rem_q[31:0]) + 32'd1) : div_rem_q[31:0];
						r = div_neg_r ? ((~div_rem[31:0]) + 32'd1) : div_rem[31:0];
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, mdl_ext[14:12]};
						rf_wr_size <= 2'd2; rf_wr_data <= q;
						tmp <= r;
						fN <= q[31]; fZ <= (q == 32'd0);
						fV <= 1'b0; fC <= 1'b0;
						// Dr==Dq discards the remainder (M68000PRM)
						state <= (mdl_ext[2:0] != mdl_ext[14:12])
								 ? S_DIV_WRREM : S_DONE;
					end
				end
				S_DIV_WRREM: begin
					// remainder write (only reached when Dr != Dq)
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, mdl_ext[2:0]};
					rf_wr_size <= 2'd2; rf_wr_data <= tmp;
					state <= S_DONE;
				end

				// shared flag-sample + writeback
				S_FLAGS_SAMPLE: begin
					fN <= flo_n; fZ <= flo_z; fV <= flo_v; fC <= flo_c;
					if (flo_xwe) fX <= flo_x;
					state <= flags_then_wb ? S_WRITE_RESULT : S_DONE;
				end
				S_WRITE_RESULT: begin
					case (wb_sel)
						2'd3: begin
							// write to the source-slot EA: binops whose <ea>
							// operand is the destination (EOR always; OR/AND/
							// ADD/SUB with the direction bit set) decode that EA
							// into the src slot. eaD would still hold the
							// previous instruction's destination.
							if (eaS_kind == 2'd0 || eaS_kind == 2'd1) begin
								rf_wr_en <= 1'b1;
								rf_wr_num <= {(eaS_kind == 2'd1), eaS_reg};
								rf_wr_size <= flg_sz_of(opsize); rf_wr_data <= result;
								state <= S_DONE;
							end else begin
								mem_addr <= eaS_addr; mem_size <= opsize;
								mem_we <= 1'b1; mem_wdata <= result & szmask(opsize);
								mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
								state <= S_MEM;
							end
						end
						2'd1: begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, reg9};
							rf_wr_size <= flg_sz_of(opsize); rf_wr_data <= result;
							state <= S_DONE;
						end
						2'd2: begin
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, reg9};
							rf_wr_size <= 2'd2; rf_wr_data <= result;
							state <= S_DONE;
						end
						default: begin
							if (eaD_kind == 2'd0 || eaD_kind == 2'd1) begin
								rf_wr_en <= 1'b1;
								rf_wr_num <= {(eaD_kind == 2'd1), eaD_reg};
								rf_wr_size <= flg_sz_of(opsize); rf_wr_data <= result;
								state <= S_DONE;
							end else begin
								mem_addr <= eaD_addr; mem_size <= opsize;
								mem_we <= 1'b1; mem_wdata <= result & szmask(opsize);
								mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
								state <= S_MEM;
							end
						end
					endcase
				end

				// CLR
				S_CLR_EXEC: begin
					fN <= 1'b0; fZ <= 1'b1; fV <= 1'b0; fC <= 1'b0;
					result <= 32'd0;
					wb_sel <= 2'd0;
					state <= S_WRITE_RESULT;
				end

				// LEA / JMP / PEA
				S_LEA_EXEC: begin
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, reg9};
					rf_wr_size <= 2'd2; rf_wr_data <= eaD_addr;
					state <= S_DONE;
				end
				S_JMP_GO: begin
					pc <= eaD_addr;
					state <= S_DONE;
				end
				S_PEA_PUSH: begin
					mem_addr <= rf_rd_b_data - 32'd4; mem_size <= 3'd4;
					mem_we <= 1'b1; mem_wdata <= eaD_addr;
					mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd4;
					state <= S_MEM;
				end

				// MOVEC
				S_MOVEC_GOT: begin
					ea_ext <= mem_rdata_r[15:0];
					state <= S_MOVEC_EXEC;
				end
				S_MOVEC_EXEC: begin
					logic [11:0] creg;
					creg = ea_ext[11:0];
					if (ir[0]) begin
						// general -> control
						case (creg)
							12'h000: sfc   <= rf_rd_a_data;
							12'h001: dfc   <= rf_rd_a_data;
							12'h002: begin
								cacr <= rf_rd_a_data;
								// CACR bit 3 (C) clears the instruction cache.
								// UM 4.3.1 scopes C to cache entries; the CHR
								// is dropped too, since writing C means the
								// instruction stream is no longer trusted.
								if (rf_rd_a_data[3]) begin              // C
									chr_valid <= 1'b0;
									for (int k = 0; k < ICACHE_ENTRIES; k++)
										ic_valid[k] <= 1'b0;
								end
								// CE clears the single entry CAAR's index
								// field selects (UM 4.3.1).
								if (rf_rd_a_data[2] && ICACHE_ENTRIES > 0)
									ic_valid[caar[2+IC_IDX-1:2]] <= 1'b0;
							end
							12'h800: begin usp_wr <= 1'b1; sp_wr_data <= rf_rd_a_data; end
							12'h801: vbr   <= rf_rd_a_data;
							12'h802: caar  <= rf_rd_a_data;
							12'h803: msp_r <= rf_rd_a_data;
							12'h804: isp_r <= rf_rd_a_data;
							default: begin
								unimplemented <= 1'b1;
								unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
							end
						endcase
						state <= unimplemented ? S_HALT : S_DONE;
					end else begin
						// control -> general
						logic [31:0] v;
						case (creg)
							12'h000: v = sfc;
							12'h001: v = dfc;
							12'h002: v = cacr;
							12'h800: v = usp_out;
							12'h801: v = vbr;
							12'h802: v = caar;
							12'h803: v = msp_r;
							default: v = isp_r;
						endcase
						rf_wr_en <= 1'b1; rf_wr_num <= ea_ext[15:12];
						rf_wr_size <= 2'd2; rf_wr_data <= v;
						state <= S_DONE;
					end
				end

				// JSR / RTS / BSR
				S_JSR_PUSH: begin
					mem_addr <= rf_rd_b_data - 32'd4; mem_size <= 3'd4;
					mem_we <= 1'b1; mem_wdata <= pc;
					mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd4;
					pc <= eaD_addr;
					state <= S_MEM;
				end
				S_RTS_RD: begin
					mem_addr <= rf_rd_b_data; mem_size <= 3'd4;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_RTS_GOT;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data + 32'd4;
					state <= S_MEM;
				end
				S_RTS_GOT: begin
					pc <= mem_rdata_r;
					state <= S_DONE;
				end

				// DBcc
				S_DBCC_GOT: begin
					if (!test_cc(cc)) begin
						logic [15:0] newcnt;
						newcnt = rf_rd_a_data[15:0] - 16'd1;
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
						rf_wr_size <= 2'd1; rf_wr_data <= {16'd0, newcnt};
						if (newcnt != 16'hFFFF)
							pc <= insn_pc_r + 32'd2 + sext16(mem_rdata_r[15:0]);
					end
					state <= S_DONE;
				end

				// Bcc / BRA / BSR
				S_BCC_EXEC: begin
					logic [31:0] disp, target;
					disp = bcc_imm8 ? tmp
						 : (mem_size == 3'd2) ? sext16(mem_rdata_r[15:0]) : mem_rdata_r;
					target = insn_pc_r + 32'd2 + disp;
					if (cc == 4'd1) begin
						tmp <= target;
						state <= S_BSR_PUSH;
					end else if (cc == 4'd0 || test_cc(cc)) begin
						pc <= target;
						state <= S_DONE;
					end else begin
						state <= S_DONE;
					end
				end
				S_BSR_PUSH: begin
					mem_addr <= rf_rd_b_data - 32'd4; mem_size <= 3'd4;
					mem_we <= 1'b1; mem_wdata <= pc;
					mem_is_fetch <= 1'b0; mem_ret <= S_DONE;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd4;
					pc <= tmp;
					state <= S_MEM;
				end

				// BINOP (OR/AND/EOR/ADD/SUB/CMP/ADDA/SUBA/CMPA)
				S_BINOP_EXEC: begin
					logic [31:0] dval, r;
					dval = rf_rd_a_data & szmask(opsize);
					case (binop)
						BOP_OR, BOP_AND, BOP_EOR: begin
							if (binop == BOP_OR)       r = (dval | val_src) & szmask(opsize);
							else if (binop == BOP_AND) r = (dval & val_src) & szmask(opsize);
							else                        r = (dval ^ val_src) & szmask(opsize);
							result <= r;
							fN <= msb_of(r, opsize); fZ <= (r == 32'd0);
							fV <= 1'b0; fC <= 1'b0;
							wb_sel <= (binop == BOP_EOR) ? 2'd3 : (ea_is_dst ? 2'd3 : 2'd1);
							state <= S_WRITE_RESULT;
						end
						BOP_ADDSUB: begin
							if (is_add) begin
								r = (dval + val_src) & szmask(opsize);
								flg_a <= dval; flg_b <= val_src; flg_op <= FOP_ADD;
							end else if (ea_is_dst) begin
								r = (val_src - dval) & szmask(opsize);
								flg_a <= val_src; flg_b <= dval; flg_op <= FOP_SUB;
							end else begin
								r = (dval - val_src) & szmask(opsize);
								flg_a <= dval; flg_b <= val_src; flg_op <= FOP_SUB;
							end
							result <= r; flg_res <= r;
							flg_sz <= flg_sz_of(opsize);
							flags_then_wb <= 1'b1;
							wb_sel <= ea_is_dst ? 2'd3 : 2'd1;
							state <= S_FLAGS_SAMPLE;
						end
						BOP_ADDX: begin
							// ADDX/SUBX Dy,Dx: Dx <- Dx (+|-) Dy (+|-) X.
							// Flags are set inline, not through m68020_flags:
							// X enters the sum, and Z is cleared on a non-zero
							// result and otherwise unchanged (the multi-precision
							// convention).
							if (ir[3]) begin
								// -(Ay),-(Ax) form: not implemented, halt
								// visibly rather than mis-execute.
								unimplemented <= 1'b1;
								unimpl_opcode <= ir; unimpl_pc <= insn_pc_r;
								state <= S_HALT;
							end else begin
								addx_a = dval    & szmask(opsize);
								addx_b = val_src & szmask(opsize);
								if (is_add)
									addx_full = {1'b0, addx_a} + {1'b0, addx_b}
											  + {32'd0, fX};
								else
									addx_full = {1'b0, addx_a} - {1'b0, addx_b}
											  - {32'd0, fX};
								r = addx_full[31:0] & szmask(opsize);
								addx_cy = addx_full[{3'd0, opsize} << 3];
								if (is_add)
									addx_ov = (msb_of(addx_a, opsize) == msb_of(addx_b, opsize))
										   && (msb_of(r, opsize) != msb_of(addx_a, opsize));
								else
									addx_ov = (msb_of(addx_a, opsize) != msb_of(addx_b, opsize))
										   && (msb_of(r, opsize) != msb_of(addx_a, opsize));
								result <= r;
								fN <= msb_of(r, opsize);
								fZ <= fZ && (r == 32'd0);   // cleared, never set
								fV <= addx_ov;
								fC <= addx_cy;
								fX <= addx_cy;
								wb_sel <= 2'd1;             // destination is Dx, not the EA
								state  <= S_WRITE_RESULT;
							end
						end
						BOP_CMP: begin
							r = (dval - val_src) & szmask(opsize);
							flg_a <= dval; flg_b <= val_src; flg_res <= r;
							flg_sz <= flg_sz_of(opsize); flg_op <= FOP_CMP;
							flags_then_wb <= 1'b0;
							state <= S_FLAGS_SAMPLE;
						end
						BOP_A: begin
							// ADDA/SUBA: full 32-bit address-register result
							// from a sign-extended sized operand. The sum goes
							// through a local: Verilator 5.020 miscompiles the
							// inline `result <= cond ? (x+f(y)) : (x-f(y))`.
							// House style: function-bearing NBA right-hand
							// sides are hoisted into locals.
							logic [31:0] bopa_sum;
							bopa_sum = is_add
								? (rf_rd_b_data + sext_sz(val_src, opsize))
								: (rf_rd_b_data - sext_sz(val_src, opsize));
							result <= bopa_sum;
							wb_sel <= 2'd2;
							state <= S_WRITE_RESULT;
						end
						default: begin // BOP_CMPA
							logic [31:0] sv;
							sv = sext_sz(val_src, opsize);
							r = rf_rd_b_data - sv;
							flg_a <= rf_rd_b_data; flg_b <= sv; flg_res <= r;
							flg_sz <= 2'd2; flg_op <= FOP_CMP;
							flags_then_wb <= 1'b0;
							state <= S_FLAGS_SAMPLE;
						end
					endcase
				end

				// MULU.W / MULS.W
				S_MULW_EXEC: begin
					logic [31:0] r;
					if (!ir[8])
						r = rf_rd_a_data[15:0] * val_src[15:0];
					else
						r = $unsigned($signed({{16{rf_rd_a_data[15]}}, rf_rd_a_data[15:0]})
									  * $signed({{16{val_src[15]}}, val_src[15:0]}));
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, reg9};
					rf_wr_size <= 2'd2; rf_wr_data <= r;
					fN <= r[31]; fZ <= (r == 32'd0);
					fV <= 1'b0; fC <= 1'b0;
					state <= S_DONE;
				end

				// shifts / rotates (register form, iterative like the model)
				S_SHIFT_INIT: begin
					shift_cnt <= ir[5] ? rf_rd_a_data[5:0]
							   : {3'd0, (reg9 == 3'd0) ? 3'd0 : reg9} | (reg9 == 3'd0 ? 6'd8 : 6'd0);
					shift_val <= rf_rd_b_data & szmask(opsize);
					shift_c <= fC;
					if ((ir[5] ? rf_rd_a_data[5:0]
					   : ({3'd0, (reg9 == 3'd0) ? 3'd0 : reg9} | (reg9 == 3'd0 ? 6'd8 : 6'd0))) == 6'd0) begin
						// count 0: C=0 for shifts and ROL/ROR, but C=X for
						// ROX (the extend bit is copied to carry).
						shift_c <= (ir[4:3] == 2'd2) ? fX : 1'b0;
						state <= S_SHIFT_FIN;
					end else begin
						state <= S_SHIFT_STEP;
					end
				end
				S_SHIFT_STEP: begin
					logic cbit;
					logic [31:0] nv;
					logic msb;
					msb = msb_of(shift_val, opsize);
					if (ir[8]) begin // left
						cbit = msb;
						if (ir[4:3] == 2'd3)
							nv = ((shift_val << 1) | {31'd0, msb}) & szmask(opsize); // ROL
						else if (ir[4:3] == 2'd2)
							nv = ((shift_val << 1) | {31'd0, fX}) & szmask(opsize);  // ROXL: old X in
						else
							nv = (shift_val << 1) & szmask(opsize);                   // ASL/LSL
					end else begin                                                    // right
						cbit = shift_val[0];
						if (ir[4:3] == 2'd0)
							nv = (shift_val >> 1) |
								 (shift_val & (32'd1 << ({29'd0, opsize} * 8 - 1)));  // ASR (model approx)
						else if (ir[4:3] == 2'd1)
							nv = shift_val >> 1;                                       // LSR
						else if (ir[4:3] == 2'd2)
							nv = (shift_val >> 1) |
								 ({31'd0, fX} << ({29'd0, opsize} * 8 - 1));           // ROXR: old X in
						else
							nv = (shift_val >> 1) |
								 ({31'd0, shift_val[0]} << ({29'd0, opsize} * 8 - 1)); // ROR
					end
					shift_val <= nv;
					shift_c <= cbit;
					if (ir[4:3] != 2'd3) fX <= cbit;   // all but ROL/ROR update X per step
					if (shift_cnt == 6'd1) state <= S_SHIFT_FIN;
					shift_cnt <= shift_cnt - 6'd1;
				end
				S_SHIFT_FIN: begin
					fC <= shift_c;
					fN <= msb_of(shift_val, opsize);
					fZ <= ((shift_val & szmask(opsize)) == 32'd0);
					fV <= 1'b0;
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
					rf_wr_size <= flg_sz_of(opsize); rf_wr_data <= shift_val;
					state <= S_DONE;
				end

				// EXG / SWAP / EXT / MOVE USP
				S_EXG1: begin
					tmp <= rf_rd_a_data;
					rf_wr_en <= 1'b1; rf_wr_num <= exg_x_num;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data;
					state <= S_EXG2;
				end
				S_EXG2: begin
					rf_wr_en <= 1'b1; rf_wr_num <= exg_y_num;
					rf_wr_size <= 2'd2; rf_wr_data <= tmp;
					state <= S_DONE;
				end
				S_REGOP_EXEC: begin
					case (grp)
						G_SWAP: begin
							logic [31:0] sv;
							sv = {rf_rd_a_data[15:0], rf_rd_a_data[31:16]};
							rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
							rf_wr_size <= 2'd2; rf_wr_data <= sv;
							fN <= sv[31]; fZ <= (sv == 32'd0);
							fV <= 1'b0; fC <= 1'b0;
							state <= S_DONE;
						end
						G_EXT: begin
							logic [31:0] ev;
							case (ir[8:6])
								3'd2: begin // EXT.W
									ev = sext8(rf_rd_a_data[7:0]) & 32'hffff;
									rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
									rf_wr_size <= 2'd1; rf_wr_data <= ev;
									fN <= ev[15]; fZ <= (ev[15:0] == 16'd0);
								end
								3'd3: begin // EXT.L
									ev = sext16(rf_rd_a_data[15:0]);
									rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
									rf_wr_size <= 2'd2; rf_wr_data <= ev;
									fN <= ev[31]; fZ <= (ev == 32'd0);
								end
								default: begin // EXTB.L
									ev = sext8(rf_rd_a_data[7:0]);
									rf_wr_en <= 1'b1; rf_wr_num <= {1'b0, earlo_reg};
									rf_wr_size <= 2'd2; rf_wr_data <= ev;
									fN <= ev[31]; fZ <= (ev == 32'd0);
								end
							endcase
							fV <= 1'b0; fC <= 1'b0;
							state <= S_DONE;
						end
						default: begin // G_MOVE_USP
							if (ir[3]) begin
								rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, earlo_reg};
								rf_wr_size <= 2'd2; rf_wr_data <= usp_out;
							end else begin
								usp_wr <= 1'b1; sp_wr_data <= rf_rd_a_data;
							end
							state <= S_DONE;
						end
					endcase
				end

				// MOVEM
				S_MOVEM_LIST_GOT: begin
					movem_list <= mem_rdata_r[15:0];
					movem_idx <= 5'd0;
					if (earlo_mode == 3'd3 || earlo_mode == 3'd4) begin
						movem_addr <= rf_rd_b_data;
						state <= S_MOVEM_NEXT;
					end else begin
						ea_mode <= earlo_mode; ea_reg <= earlo_reg;
						ea_which <= 1'b1; ea_ret <= S_MOVEM_EA_DONE;
						state <= S_EA_START;
					end
				end
				S_MOVEM_EA_DONE: begin
					movem_addr <= eaD_addr;
					state <= S_MOVEM_NEXT;
				end
				S_MOVEM_NEXT: begin
					if (movem_idx == 5'd16) begin
						state <= S_MOVEM_FIN;
					end else if (!movem_list[movem_idx[3:0]]) begin
						movem_idx <= movem_idx + 5'd1;
					end else if (!ir[10] && earlo_mode == 3'd4) begin
						// register -> memory, predecrement
						mem_addr <= movem_addr - {29'd0, opsize};
						mem_size <= opsize; mem_we <= 1'b1;
						mem_wdata <= rf_rd_a_data & szmask(opsize);
						mem_is_fetch <= 1'b0; mem_ret <= S_MOVEM_NEXT;
						movem_addr <= movem_addr - {29'd0, opsize};
						movem_idx <= movem_idx + 5'd1;
						state <= S_MEM;
					end else if (ir[10]) begin
						// memory -> register
						mem_addr <= movem_addr; mem_size <= opsize;
						mem_we <= 1'b0; mem_is_fetch <= 1'b0;
						mem_ret <= S_MOVEM_MEM_GOT;
						state <= S_MEM;
					end else begin
						// register -> memory, non-predec
						mem_addr <= movem_addr; mem_size <= opsize;
						mem_we <= 1'b1;
						mem_wdata <= rf_rd_a_data & szmask(opsize);
						mem_is_fetch <= 1'b0; mem_ret <= S_MOVEM_NEXT;
						movem_addr <= movem_addr + {29'd0, opsize};
						movem_idx <= movem_idx + 5'd1;
						state <= S_MEM;
					end
				end
				S_MOVEM_MEM_GOT: begin
					// load value via a local (house style, see BOP_A note)
					logic [31:0] movem_v;
					movem_v = (opsize == 3'd2) ? sext16(mem_rdata_r[15:0])
											   : mem_rdata_r;
					rf_wr_en <= 1'b1;
					rf_wr_num <= movem_regnum;
					rf_wr_size <= 2'd2;
					rf_wr_data <= movem_v;
					movem_addr <= movem_addr + {29'd0, opsize};
					movem_idx <= movem_idx + 5'd1;
					state <= S_MOVEM_NEXT;
				end
				S_MOVEM_FIN: begin
					if (earlo_mode == 3'd3 || (!ir[10] && earlo_mode == 3'd4)) begin
						rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, earlo_reg};
						rf_wr_size <= 2'd2; rf_wr_data <= movem_addr;
					end
					state <= S_DONE;
				end

				// CCR / SR moves
				S_TOSR_EXEC: begin
					if (grp == G_TO_CCR) begin
						{fX, fN, fZ, fV, fC} <= val_src[4:0];
					end else if (grp == G_TO_SR) begin
						{fX, fN, fZ, fV, fC} <= val_src[4:0];
						s_bit <= val_src[13];
						ipl <= val_src[10:8];
					end else begin // G_CCR_SR_IMM
						logic [15:0] cur, nv;
						cur = sr_out;
						case (ir)
							16'h003C: nv = {cur[15:8], (cur[7:0] | val_src[7:0])};
							16'h023C: nv = {cur[15:8], (cur[7:0] & val_src[7:0])};
							16'h0A3C: nv = {cur[15:8], (cur[7:0] ^ val_src[7:0])};
							16'h007C: nv = cur | val_src[15:0];
							16'h027C: nv = cur & val_src[15:0];
							default:  nv = cur ^ val_src[15:0];
						endcase
						{fX, fN, fZ, fV, fC} <= nv[4:0];
						if (ir[6]) begin // SR forms
							s_bit <= nv[13];
							ipl <= nv[10:8];
						end
					end
					state <= S_DONE;
				end
				S_FROMSR_WRITE: begin
					result <= {16'd0, sr_out};
					wb_sel <= 2'd0;
					state <= S_WRITE_RESULT;
				end

				// CMPM (Ay)+,(Ax)+
				S_CMPM_RD1: begin
					mem_addr <= rf_rd_b_data; mem_size <= opsize;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_CMPM_GOT1;
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, earlo_reg};
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data + {29'd0, opsize};
					state <= S_MEM;
				end
				S_CMPM_GOT1: begin
					val_src <= mem_rdata_r;
					state <= S_CMPM_RD2;
				end
				S_CMPM_RD2: begin
					mem_addr <= rf_rd_b_data; mem_size <= opsize;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_CMPM_GOT2;
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, reg9};
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data + {29'd0, opsize};
					state <= S_MEM;
				end
				S_CMPM_GOT2: begin
					logic [31:0] r;
					r = (mem_rdata_r - val_src) & szmask(opsize);
					flg_a <= mem_rdata_r; flg_b <= val_src; flg_res <= r;
					flg_sz <= flg_sz_of(opsize); flg_op <= FOP_CMP;
					flags_then_wb <= 1'b0;
					state <= S_FLAGS_SAMPLE;
				end

				// LINK / UNLK
				S_LINK_PUSH: begin
					mem_addr <= rf_rd_b_data - 32'd4; mem_size <= 3'd4;
					mem_we <= 1'b1; mem_wdata <= rf_rd_a_data;
					mem_is_fetch <= 1'b0; mem_ret <= S_LINK_SETAN;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd4;
					state <= S_MEM;
				end
				S_LINK_SETAN: begin
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, earlo_reg};
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data;   // new A7
					mem_addr <= pc; mem_size <= 3'd2; mem_we <= 1'b0;
					mem_is_fetch <= 1'b1; mem_ret <= S_LINK_DISP_GOT;
					state <= S_MEM;
				end
				S_LINK_DISP_GOT: begin
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2;
					rf_wr_data <= rf_rd_b_data + sext16(mem_rdata_r[15:0]);
					state <= S_DONE;
				end
				S_UNLK_SETSP: begin
					// UNLK An: SP = An; An = (SP)+; final SP = An + 4.
					// The pop address and the final SP both come straight
					// from the port-A read of An: a regfile write lands a
					// cycle later, so SP must not be read back in the next
					// state. Rule: never read a register through a port in
					// the state immediately after writing it.
					mem_addr <= rf_rd_a_data; mem_size <= 3'd4;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_UNLK_GOT;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_a_data + 32'd4;
					state <= S_MEM;
				end
				S_UNLK_GOT: begin
					rf_wr_en <= 1'b1; rf_wr_num <= {1'b1, earlo_reg};
					rf_wr_size <= 2'd2; rf_wr_data <= mem_rdata_r;
					state <= S_DONE;
				end

				// autovectored interrupt entry
				// (port B is muxed to A7 in these states, see the read-port mux)
				S_IRQ_PUSH_PC: begin
					mem_addr <= rf_rd_b_data - 32'd4; mem_size <= 3'd4;
					mem_we <= 1'b1; mem_wdata <= pc;
					mem_is_fetch <= 1'b0; mem_ret <= S_IRQ_PUSH_SR;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd4;
					state <= S_MEM;
				end
				S_IRQ_PUSH_SR: begin
					mem_addr <= rf_rd_b_data - 32'd2; mem_size <= 3'd2;
					mem_we <= 1'b1; mem_wdata <= {16'd0, irq_old_sr};
					mem_is_fetch <= 1'b0; mem_ret <= S_IRQ_VECTOR;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data - 32'd2;
					ipl <= irq_level_r;
					state <= S_MEM;
				end
				S_IRQ_VECTOR: begin
					mem_addr <= vbr + {25'd0, (5'd24 + {2'd0, irq_level_r}), 2'b00};
					mem_size <= 3'd4; mem_we <= 1'b0;
					mem_is_fetch <= 1'b0; mem_ret <= S_IRQ_GO;
					state <= S_MEM;
				end
				S_IRQ_GO: begin
					pc <= mem_rdata_r;
					insn_done <= 1'b1;   // the model counts the entry as one step
					state <= S_FETCH;
				end

				// RTE / RTR
				S_RTE_RD_SR: begin
					mem_addr <= rf_rd_b_data; mem_size <= 3'd2;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_RTE_GOT_SR;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data + 32'd2;
					state <= S_MEM;
				end
				S_RTE_GOT_SR: begin
					// Only latch the popped SR here. Applying it now would
					// switch the A7 bank before the PC pop, so a return to
					// user mode would pop PC from the user stack. The model
					// likewise pops SR and PC from the supervisor stack and
					// calls set_sr last.
					rte_sr_r <= mem_rdata_r[15:0];
					state <= S_RTE_GOT_PC;
				end
				S_RTE_GOT_PC: begin
					mem_addr <= rf_rd_b_data; mem_size <= 3'd4;
					mem_we <= 1'b0; mem_is_fetch <= 1'b0; mem_ret <= S_RTE_APPLY;
					rf_wr_en <= 1'b1; rf_wr_num <= 4'd15;
					rf_wr_size <= 2'd2; rf_wr_data <= rf_rd_b_data + 32'd4;
					state <= S_MEM;
				end
				S_RTE_APPLY: begin
					pc <= mem_rdata_r;
					{fX, fN, fZ, fV, fC} <= rte_sr_r[4:0];
					if (!rte_is_rtr) begin
						s_bit <= rte_sr_r[13];   // bank switch here, after both pops
						ipl <= rte_sr_r[10:8];
					end
					insn_done <= 1'b1;
					state <= S_FETCH;
				end

				// retire
				S_DONE: begin
					insn_done <= 1'b1;
					state <= S_FETCH;
				end
				S_HALT: state <= S_HALT;

				default: state <= S_HALT;
			endcase
		end
	end

	// ADDQ/SUBQ quick data: 0 encodes 8
	function automatic [31:0] qdata();
		qdata = (reg9 == 3'd0) ? 32'd8 : {29'd0, reg9};
	endfunction

endmodule
