// agt_cpu_meter.sv -- game tick rate and main-RAM stall meter for the overlay.
//
// The ROM's vblank_irq_handler does its fixed work at interrupt level, then,
// if the previous tick has finished, runs (primrage_ghidra_notes.md,
// Finding 8):
//     0x10334  move.b #$80,$FFFF8708   <- tick starts (busy flag set)
//     0x1034C  jsr    game_main_loop_tick
//     0x10352  clr.b  $FFFF8708        <- tick ends   (busy flag cleared)
// These are the only writes to $FFFF8708 (0x10326 is a tst.b). Main RAM is in
// SDRAM, so they go through the main-RAM port, and this module only watches
// the CPU's side of that port. Benched by tb/tb_cpu_meter.sv.
//
// All outputs are latched so the overlay reads a stable value. ticks_last = 60
// means full speed. The vblank interrupt starts at line 240, so
// (tick_line - 240) mod 262 is how long the interrupt-level work took.
// ram_stall_last uses CPUI's units (CPUI's ROM stall excludes this port);
// stall/acc is the average cycles per access. A frame runs from one vblank
// rising edge to the next.
//
// The match is narrow on purpose. A byte write to offset 0 of the long at
// 0xFFFF8708 arrives as ram_addr = 0x78708 and ram_be = 4'b1000 (lane 3 is
// offset 0), with the byte in ram_wdata[31:24] (agt_main_memmap's
// be8_of/wval64_of). A word or long write covering the byte, a write in
// another lane, or a read of the flag is not a tick edge.

module agt_cpu_meter #(
	parameter logic [18:0] FLAG_ADDR  = 19'h78708,  // $FFFF8708 in the main-RAM window
	parameter int          WINDOW_VBL = 60          // vblanks per ticks_last window
)(
	input  wire         clk,
	input  wire         rst_n,

	input  wire         vblank,          // level; rising edge = frame boundary
	input  wire  [8:0]  vcount,

	// CPU side of the main-RAM port (bc_ram_* at the top)
	input  wire         ram_req,         // held until ack
	input  wire         ram_ack,
	input  wire         ram_we,
	input  wire  [18:0] ram_addr,
	input  wire  [3:0]  ram_be,
	input  wire  [31:0] ram_wdata,

	output logic [15:0] ticks_last,     // ticks ended in the last WINDOW_VBL vblanks
	output logic [15:0] tick_line,      // vcount the latest tick started on
	output logic [15:0] ram_stall_last, // req && !ack cycles last frame, >> 8
	output logic [15:0] ram_acc_last    // port transactions last frame, >> 4
);

	// one event per transaction: the request's rising edge
	logic req_d;
	wire  req_rise = ram_req && !req_d;
	wire  flag_byte_wr = req_rise && ram_we && (ram_addr == FLAG_ADDR) &&
						 (ram_be == 4'b1000);
	wire  tick_start = flag_byte_wr && (ram_wdata[31:24] == 8'h80);
	wire  tick_end   = flag_byte_wr && (ram_wdata[31:24] == 8'h00);

	logic       vbl_d;
	wire        vbl_rise = vblank && !vbl_d;

	logic [15:0] ticks_win;              // ticks ended in the current window
	logic [7:0]  vbl_in_win;             // vblanks seen in the current window
	logic [23:0] stall_frame;
	logic [19:0] acc_frame;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			req_d <= 1'b0; vbl_d <= 1'b0;
			ticks_win <= 16'd0; vbl_in_win <= 8'd0;
			stall_frame <= 24'd0; acc_frame <= 20'd0;
			ticks_last <= 16'd0; tick_line <= 16'd0;
			ram_stall_last <= 16'd0; ram_acc_last <= 16'd0;
		end else begin
			req_d <= ram_req;
			vbl_d <= vblank;

			if (tick_start) tick_line <= {7'd0, vcount};

			// per-frame counters: latch and restart on the vblank edge
			if (vbl_rise) begin
				ram_stall_last <= stall_frame[23:8];
				ram_acc_last   <= acc_frame[19:4];
				stall_frame    <= (ram_req && !ram_ack) ? 24'd1 : 24'd0;
				acc_frame      <= req_rise ? 20'd1 : 20'd0;
			end else begin
				if (ram_req && !ram_ack && stall_frame != 24'hFFFFFF)
					stall_frame <= stall_frame + 24'd1;
				if (req_rise && acc_frame != 20'hFFFFF)
					acc_frame <= acc_frame + 20'd1;
			end

			// per-window tick count; a tick ending as the window closes counts
			// in the new window
			if (vbl_rise && vbl_in_win == WINDOW_VBL[7:0] - 8'd1) begin
				ticks_last <= ticks_win;
				ticks_win  <= tick_end ? 16'd1 : 16'd0;
				vbl_in_win <= 8'd0;
			end else begin
				if (vbl_rise) vbl_in_win <= vbl_in_win + 8'd1;
				if (tick_end && ticks_win != 16'hFFFF) ticks_win <= ticks_win + 16'd1;
			end
		end
	end

endmodule
