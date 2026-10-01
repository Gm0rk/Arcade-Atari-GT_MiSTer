`timescale 1ns/1ps
// agt_mo_checksum.sv -- the MO command register and the checksum answer
//
// From MAME:
//   atarigt.cpp  mo_command_w @ 0xd7a200:
//       if (ACCESSING_BITS_0_15)
//           command = ((data & 0xffff) == 2) ? COMMAND_CHECKSUM
//                                            : COMMAND_DRAW;
//   atarirle.cpp control_write, on the MOGO rising edge:
//       COMMAND_DRAW     -> sort_and_render()
//       COMMAND_CHECKSUM -> compute_checksum()
//   atarirle.cpp compute_checksum():
//       reqsums = ram.read(0) + 1, capped at 256;
//       for (i = 0; i < reqsums; i++) ram.write(i, m_checksums[i]);
//   atarirle.cpp device start:
//       checksum[chunk] = u16 sum of the 0x10000 big-endian words of each
//       0x20000-byte chunk of the object ROM (256 chunks over 32 MB).
//
// This module:
//   * accumulates the 256 chunk checksums as the RLE region downloads, from
//     the same dl_rle_wr byte stream the SDRAM writer consumes;
//   * shadows objlist halfword 0 (the request count) and the 0xd7a200
//     command word off the snoop bus, so it needs no memmap read port;
//   * gates MOGO: DRAW commands pass through as mogo_draw (the renderer's
//     start), CHECKSUM commands start the answer sequencer instead;
//   * answers by writing sums[0..reqsums-1] into objlist halfwords
//     0..reqsums-1 through a req/ack write port the memmap services in idle
//     cycles, keeping one writer on the shared arrays. The write also travels
//     the snoop bus, so the renderer-side objlist copy mirrors it.
module agt_mo_checksum (
	input  logic        clk,
	input  logic        rst_n,

	// download tap: the RLE byte stream, linear addresses
	// dl_rst_n is the accumulator's own reset: rst_n (cpu_core_rst_n) is low for
	// the whole ROM download, the window this module accumulates in. Both resets
	// are in the clk domain: a reset-domain split, not a clock crossing.
	input  logic        dl_rst_n,
	input  logic        dl_rle_wr,
	input  logic [24:0] dl_rle_addr,
	input  logic [7:0]  dl_rle_data,

	// snoop bus: every shared-RAM write, one 32-bit word per beat
	input  logic        snoop_we,
	input  logic [13:0] snoop_addr,
	input  logic [3:0]  snoop_be,
	input  logic [31:0] snoop_wd,

	// MOGO in, DRAW-gated MOGO out
	input  logic        mogo_pulse,
	output logic        mogo_draw,

	// checksum write-back port into the memmap's shared RAM
	output logic        chkw_req,
	output logic [10:0] chkw_half,      // halfword index inside the window
	output logic [15:0] chkw_data,
	input  logic        chkw_ack,

	output logic        chk_busy        // answering (for the debug overlay)
);

	// snoop word addresses, from the shared window base 0xd70000:
	//   objlist halfword 0 lives in word (0xd78000-0xd70000)/4 = 0x2000
	//   the command word    lives in word (0xd7a200-0xd70000)/4 = 0x2880
	localparam logic [13:0] SN_OBJ0 = 14'h2000;
	localparam logic [13:0] SN_CMD  = 14'h2880;

	// Chunk checksums, filled during download. 256 x 16 simple dual port: one
	// writer (the commit below), one reader (the answer sequencer). The ramstyle
	// keeps Quartus from silently demoting it to registers. No initial values:
	// an MLAB cannot be initialised on Cyclone V, and the download tap writes
	// every chunk before the CPU is released. The sequencer reads only after
	// that, so a write and a read never meet (no_rw_check).
	(* ramstyle = "MLAB, no_rw_check" *) logic [15:0] sums [0:255];

	logic [7:0]  hold_hi;               // byte at the even address
	logic [15:0] run_sum;               // current chunk's running sum

	wire        rle_odd     = dl_rle_addr[0];
	wire        rle_last    = &dl_rle_addr[16:1];   // last word of the chunk
	wire [7:0]  rle_chunk   = dl_rle_addr[24:17];
	wire [15:0] rle_word    = {hold_hi, dl_rle_data};   // ROM_REGION16_BE

	always_ff @(posedge clk or negedge dl_rst_n) begin
		if (!dl_rst_n) begin
			hold_hi <= 8'd0;
			run_sum <= 16'd0;
		end else if (dl_rle_wr) begin
			if (!rle_odd) hold_hi <= dl_rle_data;
			else begin
				if (rle_last) begin
					sums[rle_chunk] <= run_sum + rle_word;
					run_sum         <= 16'd0;
				end else begin
					run_sum <= run_sum + rle_word;
				end
			end
		end
	end

	// snoop shadows. count: the high half of objlist word 0 is halfword 0
	// (big-endian, the same mapping the objlist snoop copy uses).
	logic [15:0] req_count;
	logic        cmd_checksum;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			req_count    <= 16'd0;
			cmd_checksum <= 1'b0;       // COMMAND_DRAW: MAME's power-up value
		end else begin
			if (snoop_we && snoop_addr == SN_OBJ0) begin
				if (snoop_be[3]) req_count[15:8] <= snoop_wd[31:24];
				if (snoop_be[2]) req_count[7:0]  <= snoop_wd[23:16];
			end
			// mo_command_w: only when the low 16 bits are accessed
			if (snoop_we && snoop_addr == SN_CMD
				&& (snoop_be[1] || snoop_be[0]))
				cmd_checksum <= (snoop_wd[15:0] == 16'd2);
		end
	end

	// MOGO gate: DRAW passes through combinationally, in the same cycle;
	// CHECKSUM starts the sequencer instead.
	assign mogo_draw = mogo_pulse && !cmd_checksum;

	// answer sequencer: reqsums = ram[0] + 1, capped at 256 (compute_checksum)
	typedef enum logic [1:0] { CK_IDLE, CK_RD, CK_REQ } ck_t;
	ck_t         ckst;
	logic [8:0]  idx;                   // 0..255
	logic [8:0]  limit;                 // 1..256
	logic [15:0] sums_q;

	assign chk_busy = (ckst != CK_IDLE);

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ckst      <= CK_IDLE;
			idx       <= 9'd0;
			limit     <= 9'd0;
			sums_q    <= 16'd0;
			chkw_req  <= 1'b0;
			chkw_half <= 11'd0;
			chkw_data <= 16'd0;
		end else begin
			case (ckst)
				CK_IDLE: if (mogo_pulse && cmd_checksum) begin
					idx   <= 9'd0;
					limit <= (req_count >= 16'd255) ? 9'd256
													: 9'(req_count + 16'd1);
					ckst  <= CK_RD;
				end
				CK_RD: begin
					sums_q <= sums[idx[7:0]];
					ckst   <= CK_REQ;
				end
				CK_REQ: begin
					chkw_req  <= 1'b1;
					chkw_half <= {3'd0, idx[7:0]};
					chkw_data <= sums_q;
					if (chkw_req && chkw_ack) begin
						chkw_req <= 1'b0;
						if (idx + 9'd1 == limit) ckst <= CK_IDLE;
						else begin
							idx  <= idx + 9'd1;
							ckst <= CK_RD;
						end
					end
				end
				default: ckst <= CK_IDLE;
			endcase
		end
	end

endmodule
