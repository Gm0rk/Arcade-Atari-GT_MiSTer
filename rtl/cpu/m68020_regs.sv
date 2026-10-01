// m68020_regs.sv -- 68020 register file (D0-D7, A0-A7, USP/SSP)
//
//   1. Data register writes are size-masked: a byte write to D0 changes only
//      D0[7:0], a word write only D0[15:0].
//   2. Address register writes are always 32 bits; a word write is sign
//      extended first. The 68020 cannot encode a byte write to an address
//      register, so one raises illegal_areg_byte_write.
//   3. A7 is banked: register 15 is SSP or USP by the S bit (`supervisor`).
//      Both are also exposed for MOVE USP,An / MOVE An,USP.
//
// Only USP and a single SSP are implemented, i.e. the SR's M bit is treated
// as always 0 (its reset state; the Python model assumes the same). MSP
// would go here if ever needed.
//
// Register numbering follows the instruction encoding: 0-7 = D0-D7,
// 8-15 = A0-A7.

module m68020_regs (
	input  logic        clk,
	input  logic        rst_n,

	input  logic        supervisor,     // S bit: selects which A7 is visible

	// two asynchronous read ports
	input  logic [3:0]  rd_a_num,
	output logic [31:0] rd_a_data,
	input  logic [3:0]  rd_b_num,
	output logic [31:0] rd_b_data,

	// write port
	input  logic        wr_en,
	input  logic [3:0]  wr_num,
	input  logic [1:0]  wr_size,        // 0 = byte, 1 = word, 2 = long
	input  logic [31:0] wr_data,

	// explicit USP/SSP access (MOVE USP,An / MOVE An,USP; exception stacking)
	output logic [31:0] usp_out,
	output logic [31:0] ssp_out,
	input  logic        usp_wr,
	input  logic        ssp_wr,
	input  logic [31:0] sp_wr_data,

	// byte-size write to an address register (not encodable: a decoder bug)
	output logic        illegal_areg_byte_write
);

	logic [31:0] d [0:7];
	logic [31:0] a [0:6];   // A0-A6; A7 is banked below
	logic [31:0] usp;
	logic [31:0] ssp;

	assign usp_out = usp;
	assign ssp_out = ssp;

	wire [31:0] a7 = supervisor ? ssp : usp;

	// Reads are always_comb, not `assign x = f(num)`: a continuous assign
	// calling a function only re-evaluates when `num` changes, so it would
	// return a stale value after a write to the same register number.
	always_comb begin
		if (!rd_a_num[3])              rd_a_data = d[rd_a_num[2:0]];
		else if (rd_a_num[2:0] != 3'd7) rd_a_data = a[rd_a_num[2:0]];
		else                           rd_a_data = a7;
	end

	always_comb begin
		if (!rd_b_num[3])              rd_b_data = d[rd_b_num[2:0]];
		else if (rd_b_num[2:0] != 3'd7) rd_b_data = a[rd_b_num[2:0]];
		else                           rd_b_data = a7;
	end

	// Address registers: word writes sign-extend to 32 bits; long writes pass
	// through. Data registers: size-masked merge with the existing value.
	wire is_areg = wr_num[3];

	wire [31:0] areg_wdata = (wr_size == 2'd1)
							  ? {{16{wr_data[15]}}, wr_data[15:0]}
							  : wr_data;

	wire [31:0] dreg_old = d[wr_num[2:0]];
	logic [31:0] dreg_wdata;
	always_comb begin
		unique case (wr_size)
			2'd0:    dreg_wdata = {dreg_old[31:8],  wr_data[7:0]};
			2'd1:    dreg_wdata = {dreg_old[31:16], wr_data[15:0]};
			default: dreg_wdata = wr_data;
		endcase
	end

	assign illegal_areg_byte_write = wr_en && is_areg && (wr_size == 2'd0);

	always_ff @(posedge clk or negedge rst_n) begin
		// loop variable declared in-block: a module-level `integer i` makes
		// Quartus infer a latch for it
		if (!rst_n) begin
			for (int i = 0; i < 8; i = i + 1) d[i] <= 32'd0;
			for (int i = 0; i < 7; i = i + 1) a[i] <= 32'd0;
			usp <= 32'd0;
			ssp <= 32'd0;
		end else begin
			if (wr_en) begin
				if (!is_areg) begin
					d[wr_num[2:0]] <= dreg_wdata;
				end else if (wr_num[2:0] != 3'd7) begin
					a[wr_num[2:0]] <= areg_wdata;
				end else begin
					if (supervisor) ssp <= areg_wdata;
					else            usp <= areg_wdata;
				end
			end
			// explicit stack-pointer writes (exception processing, MOVE An,USP) take
			// priority; they never coincide with a conflicting wr_en to register 15
			if (usp_wr) usp <= sp_wr_data;
			if (ssp_wr) ssp <= sp_wr_data;
		end
	end

endmodule
