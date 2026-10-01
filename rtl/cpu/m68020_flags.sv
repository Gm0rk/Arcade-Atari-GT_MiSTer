// m68020_flags.sv -- 68020 condition-code generation
//
// Combinational N/Z/V/C/X for the arithmetic and logical op classes at
// byte/word/long size, kept separate so it can be tested exhaustively.
// Semantics follow the Python model (cpu68020.py), checked in lockstep
// against a MAME execution trace:
//
//   ADD:  C = carry out of the size's MSB
//         V = (~(a ^ b) & (a ^ result)) & signbit
//         X = C
//   SUB:  C = borrow, i.e. (b > a) unsigned
//         V = ((a ^ b) & (a ^ result)) & signbit
//         X = C
//   CMP:  as SUB, but X is not updated (x_we low)
//   LOGIC (AND/OR/EOR/MOVE/MOVEQ/NOT/TST):
//         V = 0, C = 0, X not updated
//   All:  N = MSB of result, Z = result == 0 (at the operating size)
//
// The caller applies X only when x_we is high; every other flag is always
// valid for the selected op.

module m68020_flags (
	input  logic [31:0] a,          // destination / minuend
	input  logic [31:0] b,          // source / subtrahend
	input  logic [31:0] result,     // computed result (masked or not)
	input  logic [1:0]  size,       // 0 = byte, 1 = word, 2 = long
	input  logic [1:0]  op,         // OP_* below

	output logic n_flag,
	output logic z_flag,
	output logic v_flag,
	output logic c_flag,
	output logic x_flag,
	output logic x_we               // high when X should be written
);

	localparam logic [1:0] OP_ADD   = 2'd0;
	localparam logic [1:0] OP_SUB   = 2'd1;
	localparam logic [1:0] OP_CMP   = 2'd2;
	localparam logic [1:0] OP_LOGIC = 2'd3;

	// size masking
	logic [31:0] mask_v;
	logic [31:0] signbit;
	always_comb begin
		unique case (size)
			2'd0:    begin mask_v = 32'h0000_00ff; signbit = 32'h0000_0080; end
			2'd1:    begin mask_v = 32'h0000_ffff; signbit = 32'h0000_8000; end
			default: begin mask_v = 32'hffff_ffff; signbit = 32'h8000_0000; end
		endcase
	end

	wire [31:0] am = a      & mask_v;
	wire [31:0] bm = b      & mask_v;
	wire [31:0] rm = result & mask_v;

	// ADD carry: raw sum exceeds the size's mask. 33 bits so the long case
	// doesn't wrap.
	wire [32:0] raw_sum = {1'b0, am} + {1'b0, bm};
	wire add_carry = (raw_sum > {1'b0, mask_v});
	// SUB/CMP: borrow iff subtrahend is larger, unsigned, at size.
	wire sub_borrow = (bm > am);

	// overflow
	wire add_v = |((~(am ^ bm) & (am ^ rm)) & signbit);
	wire sub_v = |(( (am ^ bm) & (am ^ rm)) & signbit);

	always_comb begin
		n_flag = |(rm & signbit);
		z_flag = (rm == 32'd0);

		unique case (op)
			OP_ADD: begin
				c_flag = add_carry;
				v_flag = add_v;
				x_flag = add_carry;
				x_we   = 1'b1;
			end
			OP_SUB: begin
				c_flag = sub_borrow;
				v_flag = sub_v;
				x_flag = sub_borrow;
				x_we   = 1'b1;
			end
			OP_CMP: begin
				c_flag = sub_borrow;
				v_flag = sub_v;
				x_flag = sub_borrow;   // value irrelevant; x_we gates it
				x_we   = 1'b0;
			end
			default: begin // OP_LOGIC
				c_flag = 1'b0;
				v_flag = 1'b0;
				x_flag = 1'b0;
				x_we   = 1'b0;
			end
		endcase
	end

endmodule
