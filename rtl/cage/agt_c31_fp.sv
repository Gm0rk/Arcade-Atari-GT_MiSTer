// agt_c31_fp.sv -- the TMS320C31's extended-precision float ALU.
//
// `addf`, `subf`, `mpyf`, `negf` and the int/float conversions from
// tools/cage31.py, which is MAME's 320c3x_ops.ipp transcribed. Purely
// combinational: operands and ST in, result and ST out. A module rather than
// functions in agt_c31.sv so `tb_c31_float` can drive it directly and aim
// at the branches the sound program alone would not reach (underflow,
// exponent overflow, cancellation to zero, 32-place alignment shifts).
//
// Format (1.0.31): an 8-bit exponent and a 32-bit two's-complement mantissa
// with the implicit bit folded into the sign bit:
//     value = ((sign_extend32(stored) ^ 0x80000000) / 2**31) * 2**exp
// so +1.0 is {man 0x00000000, exp 0} and -1.0 is {man 0x80000000, exp -1},
// because the negative mantissa range is [-2,-1). The unpacked form is
// therefore 34 bits signed (`fp_ext`), and the cast to the wider type comes
// before the XOR, never after, as in MAME's `(int64_t)mantissa() ^ 0x80000000`.
//
// A zero result underflows: `_fp_pack` and `mpyf` force `exp = -128` for a
// zero mantissa and then test `exp <= -128`, setting UF and LUF. MAME's
// `subf` has a guard meant to stop that, but it is inert (the branch above
// has already forced the mantissa to 0x80000000). Copied with the
// behaviour, not the intent.
`default_nettype none

module agt_c31_fp (
	input  wire  [2:0]  op,
	input  wire  [7:0]  e1,
	input  wire  [31:0] m1,
	input  wire  [7:0]  e2,
	input  wire  [31:0] m2,
	input  wire  [31:0] st_i,

	output logic [7:0]  ro_e,
	output logic [31:0] ro_m,
	output logic [31:0] st_o
);

	// `negf` is also reached from subf's 32-place alignment branch. The two
	// conversions live here rather than in the core because they carry their own
	// normalise (the same clz/clo the pack uses), and so `tb_c31_float` can aim
	// at their edges.
	localparam [2:0] FP_ADD = 3'd0, FP_SUB = 3'd1, FP_MPY  = 3'd2,
					 FP_NEG = 3'd3, FP_I2F = 3'd4, FP_F2I  = 3'd5,
					 FP_F2IQ = 3'd6;   // float2int with setflags false

	// ST bits, as in agt_c31.sv
	localparam int C_BIT = 0, V_BIT = 1, Z_BIT = 2, N_BIT = 3,
				   UF_BIT = 4, LV_BIT = 5, LUF_BIT = 6;
	localparam logic [31:0] M_NZVUF = (32'd1 << N_BIT) | (32'd1 << Z_BIT) |
									  (32'd1 << V_BIT) | (32'd1 << UF_BIT);

	// helpers
	// `s32(stored) ^ 0x80000000`, in the width the result actually needs.
	function automatic signed [33:0] fp_ext(input [31:0] s);
		begin
			fp_ext = $signed({{2{s[31]}}, s}) ^ 34'h0_8000_0000;
		end
	endfunction

	// `32 if v == 0 else 32 - v.bit_length()`
	function automatic [5:0] fp_clz(input [31:0] v);
		integer i;
		begin
			fp_clz = 6'd32;
			for (i = 0; i < 32; i = i + 1)
				if (v[i]) fp_clz = 6'd31 - i[5:0];
		end
	endfunction

	function automatic [5:0] fp_clo(input [31:0] v);
		begin
			fp_clo = fp_clz(~v);
		end
	endfunction

	// `_fp_pack`: normalise, clamp, store.
	// Returns {v, u, exp[7:0], stored_man[31:0]}. The two flag bits say which
	// of V|LV and UF|LUF to OR in; nothing else about ST is decided here.
	function automatic [41:0] fp_pack(input signed [35:0] m_in,
									  input signed [9:0]  e_in);
		logic signed [35:0] m;
		logic signed [9:0]  e;
		logic [5:0]         cnt;
		logic               fv, fu;
		begin
			m = m_in; e = e_in; fv = 1'b0; fu = 1'b0;
			if (m == 36'sd0 || e == -10'sd128) begin
				e = -10'sd128;
				m = 36'sh0_80000000;
			end else if (m >= 36'sh1_00000000 || m < -36'sh1_00000000) begin
				m = m >>> 1;
				e = e + 10'sd1;
			end else if (m >= -36'sh0_80000000 && m < 36'sh0_80000000) begin
				// already inside 2^31: shift up by the leading run. One shot,
				// not a loop: clz/clo give the whole count.
				cnt = (m > 0) ? fp_clz(m[31:0]) : fp_clo(m[31:0]);
				if (m != 36'sd0) begin
					m = m <<< cnt;
					e = e - $signed({4'd0, cnt});
				end
			end
			if (e <= -10'sd128) begin
				m = 36'sh0_80000000; e = -10'sd128; fu = 1'b1;
			end else if (e > 10'sd127) begin
				m = (m < 0) ? 36'sd0 : 36'sh0_FFFFFFFF;
				e = 10'sd127; fv = 1'b1;
			end
			fp_pack = {fv, fu, e[7:0], m[31:0] ^ 32'h8000_0000};
		end
	endfunction

	// an early return: the result is one of the operands, unchanged
	function automatic [41:0] fp_copy(input [7:0] e, input [31:0] m);
		begin
			fp_copy = {2'b00, e, m};
		end
	endfunction

	// `negf`
	function automatic [41:0] fp_negf(input [7:0] e, input [31:0] s);
		begin
			if (e == 8'h80)              fp_negf = {2'b00, 8'h80, 32'd0};
			else if (s[30:0] != 31'd0)   fp_negf = {2'b00, e, (~s) + 32'd1};
			else                         fp_negf = {2'b00,
													(s == 32'd0) ? e - 8'd1
																 : e + 8'd1,
													s ^ 32'h8000_0000};
		end
	endfunction

	// addf / subf differ in three places only: addf tests s1 for zero and subf
	// does not, subf negates the mantissa it aligns, and subf's 32-place branch
	// on the s2 side returns -s2 rather than s2.
	function automatic [41:0] fp_addsub(input               sub,
										input [7:0]  ea, input [31:0] ma,
										input [7:0]  eb, input [31:0] mb);
		logic signed [33:0] x, y;
		logic signed [9:0]  xa, xb, e, cnt;
		begin
			xa = $signed({{2{ea[7]}}, ea});
			xb = $signed({{2{eb[7]}}, eb});
			x  = fp_ext(ma);
			y  = fp_ext(mb);
			if (!sub && xa == -10'sd128) begin
				fp_addsub = fp_copy(eb, mb);
			end else if (xb == -10'sd128) begin
				fp_addsub = fp_copy(ea, ma);
			end else if (xa > xb) begin
				e   = xa;
				cnt = xa - xb;
				if (cnt >= 10'sd32) fp_addsub = fp_copy(ea, ma);
				else begin
					y = y >>> cnt[5:0];
					fp_addsub = fp_pack(sub ? ({{2{x[33]}}, x} - {{2{y[33]}}, y})
											: ({{2{x[33]}}, x} + {{2{y[33]}}, y}),
										e);
				end
			end else begin
				e   = xb;
				cnt = xb - xa;
				if (cnt >= 10'sd32)
					fp_addsub = sub ? fp_negf(eb, mb) : fp_copy(eb, mb);
				else begin
					x = x >>> cnt[5:0];
					fp_addsub = fp_pack(sub ? ({{2{x[33]}}, x} - {{2{y[33]}}, y})
											: ({{2{x[33]}}, x} + {{2{y[33]}}, y}),
										e);
				end
			end
		end
	endfunction

	// mpyf: 1.0.31 -> 1.1.23 for the multiply, signed, with the >>8 arithmetic
	// and the ^0x800000 restoring the implicit bit for both signs. The product is
	// 1.2.46 and comes back to 1.2.31 by >>15, so mpyf is only good to about
	// 2^-23, a property of the chip, not of this transcription.
	//
	// It does not share fp_pack: the product is always at least 2^31 in
	// magnitude, so there is no left-normalise, and the overflow branch can need
	// two right shifts where fp_pack's needs one.
	function automatic [41:0] fp_mul(input [7:0]  ea, input [31:0] ma,
									 input [7:0]  eb, input [31:0] mb);
		logic signed [24:0] p, q;
		logic signed [49:0] prod, sh;
		logic signed [35:0] m;
		logic signed [9:0]  e;
		logic               fv, fu;
		begin
			fv = 1'b0; fu = 1'b0;
			if (ea == 8'h80 || eb == 8'h80) begin
				fp_mul = {2'b00, 8'h80, 32'd0};
			end else begin
				p    = $signed({ma[31], ma[31:8]}) ^ 25'h0800000;
				q    = $signed({mb[31], mb[31:8]}) ^ 25'h0800000;
				prod = p * q;
				// 1.2.46 -> 1.2.31. The 50-bit product narrows to 36 and
				// the bits dropped are sign extension: |product| <= 2**48,
				// so |m| <= 2**33.
				//
				// Keep the two steps. Written as
				//     m = (prod >>> 15) & 36'hF_FFFF_FFFF;
				// the AND's unsigned operand makes the whole expression
				// unsigned, so `>>>` becomes a logical shift. The shift gets
				// its own signed assignment; the part-select is the
				// truncation.
				sh   = prod >>> 15;
				m    = sh[35:0];
				e    = $signed({{2{ea[7]}}, ea}) + $signed({{2{eb[7]}}, eb});
				if (m == 36'sd0) begin
					e = -10'sd128;
					m = 36'sh0_80000000;
				end else if (m >= 36'sh1_00000000) begin
					m = m >>> 1;
					e = e + 10'sd1;
					if (m >= 36'sh1_00000000) begin
						m = m >>> 1;
						e = e + 10'sd1;
					end
				end else if (m < -36'sh1_00000000) begin
					m = m >>> 1;
					e = e + 10'sd1;
				end
				if (e <= -10'sd128) begin
					m = 36'sh0_80000000; e = -10'sd128; fu = 1'b1;
				end else if (e > 10'sd127) begin
					m = (m < 0) ? 36'sd0 : 36'sh0_FFFFFFFF;
					e = 10'sd127; fv = 1'b1;
				end
				fp_mul = {fv, fu, e[7:0], m[31:0] ^ 32'h8000_0000};
			end
		end
	endfunction

	// int2float (`FLOAT`): the integer is already in the destination's mantissa
	// when this runs (`r_man[dreg7] = src_int(...)` then `int2float(dreg7)`), so
	// there is one operand and no exponent in. The two special cases are not
	// symmetric: 0 becomes {0, -128} but 0xFFFFFFFF (-1) becomes {0x80000000,
	// -1}, because -1.0's mantissa range is [-2,-1) and the normalise below
	// would give it the wrong exponent.
	function automatic [41:0] fp_i2f(input [31:0] v);
		logic [5:0]         cnt;
		logic [31:0]        m;
		logic signed [9:0]  e;
		begin
			if (v == 32'd0) begin
				fp_i2f = {2'b00, 8'h80, 32'd0};
			end else if (v == 32'hFFFF_FFFF) begin
				fp_i2f = {2'b00, 8'hFF, 32'h8000_0000};
			end else begin
				cnt = v[31] ? fp_clo(v) : fp_clz(v);
				m   = v << cnt;
				e   = 10'sd31 - $signed({4'd0, cnt});
				fp_i2f = {2'b00, e[7:0], m ^ 32'h8000_0000};
			end
		end
	endfunction

	// float2int (`FIX`): `shift = 31 - exp`, so a big exponent means there is
	// nothing to shift and the result saturates with V set, while a negative
	// exponent means the value is below 1 and only its sign survives. The XOR in
	// the middle branch is the implicit bit, put back where the shift left it.
	// The result is an integer, so the caller takes the mantissa only and the
	// N/Z rule is `or_nz`, not `or_nzf`.
	function automatic [41:0] fp_f2i(input [7:0] e, input [31:0] m);
		logic signed [9:0]  sh;
		logic signed [31:0] sm;
		logic [31:0]        res;
		logic               fv;
		begin
			sh = 10'sd31 - $signed({{2{e[7]}}, e});
			fv = 1'b0;
			if (sh <= 10'sd0) begin
				res = m[31] ? 32'h8000_0000 : 32'h7FFF_FFFF;
				fv  = 1'b1;
			end else if (sh > 10'sd31) begin
				res = {32{m[31]}};              // `(int32_t)man >> 31`
			end else begin
				// The shift is arithmetic and needs an assignment of
				// its own. In
				//     ($signed(m) >>> sh) ^ (32'd1 << ...)
				// the XOR's unsigned operand makes the whole expression
				// unsigned (a shift's left operand is context-determined),
				// so `>>>` would become a logical shift and every negative
				// value would convert wrong.
				sm  = $signed(m) >>> sh[4:0];
				res = sm ^ (32'd1 << (5'd31 - sh[4:0]));
			end
			fp_f2i = {fv, 1'b0, e, res};
		end
	endfunction

	// The one place the flag rule is written. Every float op clears N/Z/V/UF
	// first (LV and LUF are sticky), the pack ORs in V|LV or UF|LUF, and
	// `or_nzf` takes N from the mantissa's sign bit and Z from `exp == -128`,
	// which is the representation of zero, so a mantissa test would be wrong.
	logic [41:0] r;
	always_comb begin
		unique case (op)
			FP_MPY:           r = fp_mul(e1, m1, e2, m2);
			FP_SUB:           r = fp_addsub(1'b1, e1, m1, e2, m2);
			FP_NEG:           r = fp_negf(e1, m1);
			FP_I2F:           r = fp_i2f(m1);
			FP_F2I, FP_F2IQ:  r = fp_f2i(e1, m1);
			default:          r = fp_addsub(1'b0, e1, m1, e2, m2);
		endcase

		ro_e = r[39:32];
		ro_m = r[31:0];

		// `float2int(r, setflags=False)` does not touch ST at all, not even the
		// clear. `fix` passes `dreg31 < 8`, so an integer destination outside
		// R0-R7 converts silently.
		if (op == FP_F2IQ) begin
			st_o = st_i;
		end else begin
			st_o          = st_i & ~M_NZVUF;
			if (r[41]) st_o = st_o | (32'd1 << V_BIT)  | (32'd1 << LV_BIT);
			if (r[40]) st_o = st_o | (32'd1 << UF_BIT) | (32'd1 << LUF_BIT);
			st_o[N_BIT]   = r[31];
			// `or_nz` for the integer result, `or_nzf` for a float one: Z from
			// the value in the first case and from `exp == -128` in the
			// second.
			st_o[Z_BIT]   = (op == FP_F2I) ? (r[31:0] == 32'd0)
										   : (r[39:32] == 8'h80);
		end
	end

endmodule

`default_nettype wire
