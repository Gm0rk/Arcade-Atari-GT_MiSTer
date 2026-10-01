// agt_colorram.sv -- Atari GT color RAM and palette pens.
//
// Models the colorram region (CPU 0xD80000-0xDFFFFF) as MAME's
// colorram_w/colorram_r do (atarigt_v.cpp). Ranges are word indices,
// word_index = (byte_addr & 0x7ffff) >> 1:
//   [0x10000,0x14000)  TRAM (T-Mek only; not implemented)
//   [0x20000,0x28000)  pen[word_index & 0x7fff].red/green = data[15:8]/[7:0]
//   [0x30000,0x38000)  pen[word_index & 0x7fff].blue = data[7:0]
//   anything else      plain storage
//
// Implemented: CRAM at word_index 0-0x3FFF (both banks selected by
// (color_latch&0x08)<<10; Primal Rage masks cra to 0xfff), color_latch at
// 0x18000, and the 32768 x 24-bit pens (MRAM). TRAM and the rest of the
// space are not implemented; Primal Rage's blend path never reads them.

module agt_colorram (
	input  logic        clk,
	input  logic        rst_n,

	// CPU write port: byte address relative to 0xD80000
	input  logic [18:0] wr_byte_addr,
	input  logic [15:0] wr_data,
	input  logic        wr_en,

	// CRAM read port
	input  logic [13:0] cram_addr,     // word_index, bank offset added by caller
	output logic [15:0] cram_data,

	// color_latch (word_index 0x18000)
	output logic [15:0] color_latch,

	// Pen read ports: R, G and B may each come from a different pen index
	// (see agt_colormix_pf_primrage.sv).
	input  logic [14:0] pen_addr_r,
	input  logic [14:0] pen_addr_g,
	input  logic [14:0] pen_addr_b,
	output logic [23:0] pen_data_r,
	output logic [23:0] pen_data_g,
	output logic [23:0] pen_data_b
);

	// CRAM covers word_index 0x0000-0x3FFF directly: the two banks
	// (0-4095 and 8192-12287) are not contiguous, so the raw index is used.
	//
	// An M10K has at most two ports and synchronous reads. Each read port
	// therefore gets its own single-read copy (three pen copies, one CRAM),
	// written identically, and reads are registered: data is valid the cycle
	// after the address. Pinned to M10K so the fitter cannot demote them to
	// registers.
	(* ramstyle = "M10K" *) logic [15:0] cram_mem  [0:16383];
	logic [23:0] pens_mem_r [0:32767];
	logic [23:0] pens_mem_g [0:32767];
	logic [23:0] pens_mem_b [0:32767];

	// Zero-init via $readmemh (Quartus caps initial-block loops at 5000
	// iterations; $readmemh becomes a BRAM init file).
	initial $readmemh("mem/colorram_cram_zero.hex", cram_mem);
	initial $readmemh("mem/colorram_pens_zero.hex", pens_mem_r);
	initial $readmemh("mem/colorram_pens_zero.hex", pens_mem_g);
	initial $readmemh("mem/colorram_pens_zero.hex", pens_mem_b);

	wire [18:0] wr_word_index = wr_byte_addr[18:1];  // (addr & 0x7ffff) >> 1

	always_ff @(posedge clk) begin
		if (wr_en) begin
			// CRAM: word_index 0x0000-0x3FFF
			if (wr_word_index < 19'h4000) begin
				cram_mem[wr_word_index[13:0]] <= wr_data;
			end
			// color_latch: word_index 0x18000
			if (wr_word_index == 19'h18000) begin
				color_latch <= wr_data;
			end
			// MRAM red+green: word_index 0x20000-0x27FFF (blue is written separately
			// in the 0x30000 range). All pen copies are updated identically.
			if (wr_word_index >= 19'h20000 && wr_word_index < 19'h28000) begin
				pens_mem_r[wr_word_index[14:0]][23:16] <= wr_data[15:8];
				pens_mem_r[wr_word_index[14:0]][15:8]  <= wr_data[7:0];
				pens_mem_g[wr_word_index[14:0]][23:16] <= wr_data[15:8];
				pens_mem_g[wr_word_index[14:0]][15:8]  <= wr_data[7:0];
				pens_mem_b[wr_word_index[14:0]][23:16] <= wr_data[15:8];
				pens_mem_b[wr_word_index[14:0]][15:8]  <= wr_data[7:0];
			end
			// MRAM blue: word_index 0x30000-0x37FFF
			if (wr_word_index >= 19'h30000 && wr_word_index < 19'h38000) begin
				pens_mem_r[wr_word_index[14:0]][7:0] <= wr_data[7:0];
				pens_mem_g[wr_word_index[14:0]][7:0] <= wr_data[7:0];
				pens_mem_b[wr_word_index[14:0]][7:0] <= wr_data[7:0];
			end
		end
	end

	// Registered reads, one RAM copy per port. The consuming FSM holds each
	// address stable across its lookup state, so the latency is absorbed.
	always_ff @(posedge clk) begin
		cram_data  <= cram_mem[cram_addr[13:0]];
		pen_data_r <= pens_mem_r[pen_addr_r];
		pen_data_g <= pens_mem_g[pen_addr_g];
		pen_data_b <= pens_mem_b[pen_addr_b];
	end

endmodule
