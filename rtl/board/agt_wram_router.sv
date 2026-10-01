// agt_wram_router.sv -- serves the game's work RAM from on-chip M10K instead
// of SDRAM.
//
// Main-RAM accesses to SDRAM cost 13-18 cycles each, and every stack push
// and game variable is one. The game keeps all of them in the top 32 KB of
// the main-RAM window:
//   * all main-RAM variables labelled in the Ghidra disassembly lie in
//     0xFFFF8008..0xFFFFF2E8; none below 0xFFFF8000
//   * the boot code word-clears "workram ffff8000..top", and keeps its
//     warm-boot magic at $FFFF8026
//   * reset SSP is 0 and nothing loads SP with an absolute value, so the
//     stack grows down from 0xFFFFFFFC
// So 0xFFFF8000..0xFFFFFFFF (main-RAM port byte offsets 0x78000..0x7FFFF,
// cpu_addr[18:15] == 4'hF) is served here with one cycle of wait. The rest of
// the window still goes to SDRAM: slow but correct.
//
// A module rather than inline so the routing can be benched
// (tb/tb_wram_router.sv). ENABLE = 0 is a pure pass-through to SDRAM.
//
// Protocol (agt_main_memmap's main-RAM port): cpu_req is a level held until
// cpu_ack; the memmap drops it on the edge after it sees ack and keeps it low
// for at least one cycle before the next request (M_RAM2's second beat).
// This side answers with a one-cycle ack, data valid with it, then waits for
// req to fall before accepting again, so a request is never answered twice.
//
// Clear: ram_clr (Arcade-Atari-GT.sv) zeroes the SDRAM window on every reset
// release, holding the CPU until done. The boot code keys warm vs cold boot on
// $FFFF8026 == 0xC0EDBABE, so RAM that survived a reset would turn a cold
// boot into a warm one. The BRAM is cleared on the same condition (clr_run)
// and the top level also holds the CPU on clr_done. 8,192 cycles, far inside
// the SDRAM clear's ~1.3 M.
//
// Storage: four byte-wide arrays, one per lane, each 8192 x 8 = 8 M10K, 32 in
// all (the per-lane style agt_demo_memories uses).

module agt_wram_router #(
	parameter bit         ENABLE = 1'b1,
	parameter logic [3:0] WINDOW = 4'hF     // cpu_addr[18:15] of the on-chip window
)(
	input  wire         clk,
	input  wire         rst_n,

	// CPU side: agt_board_core's main-RAM port
	input  wire         cpu_req,
	input  wire         cpu_we,
	input  wire  [18:0] cpu_addr,           // long-aligned byte offset in the 512 KB window
	input  wire  [3:0]  cpu_be,             // lane 3 = MSB = byte offset 0
	input  wire  [31:0] cpu_wdata,
	output wire         cpu_ack,
	output wire  [31:0] cpu_rdata,

	// SDRAM side: toward the ram mux in Arcade-Atari-GT.sv
	output wire         sdr_req,
	input  wire         sdr_ack,
	input  wire  [31:0] sdr_rdata,

	// clear, in the same reset domain and on the same condition as ram_clr
	input  wire         clr_run,
	output logic        clr_done
);

generate if (!ENABLE) begin : g_off
	assign sdr_req   = cpu_req;
	assign cpu_ack   = sdr_ack;
	assign cpu_rdata = sdr_rdata;
	always_ff @(posedge clk or negedge rst_n)
		if (!rst_n) clr_done <= 1'b0; else clr_done <= 1'b1;
end else begin : g_on

	wire sel = (cpu_addr[18:15] == WINDOW);

	// Storage. Splitting lanes to reach the M10K x5 mode's parity bits saves
	// nothing: Quartus builds a 5-bit array from five x1 blocks, the same as 8
	// bits. The parity bits are reachable only above 32 bits wide (x40).
	(* ramstyle = "M10K" *) logic [7:0] wram_b3 [0:8191];   // bits 31:24, byte offset 0
	(* ramstyle = "M10K" *) logic [7:0] wram_b2 [0:8191];
	(* ramstyle = "M10K" *) logic [7:0] wram_b1 [0:8191];
	(* ramstyle = "M10K" *) logic [7:0] wram_b0 [0:8191];   // bits 7:0,  byte offset 3

	// the one port: the clear owns it until clr_done
	logic [12:0] clr_addr;
	logic        acc_go;                    // this cycle's CPU access, if any
	logic [12:0] p_addr;
	logic        p_we;
	logic [3:0]  p_be;
	logic [31:0] p_wdata;
	always_comb begin
		if (!clr_done) begin
			p_addr = clr_addr; p_we = clr_run; p_be = 4'b1111; p_wdata = 32'd0;
		end else begin
			p_addr = cpu_addr[14:2]; p_we = acc_go && cpu_we;
			p_be = cpu_be; p_wdata = cpu_wdata;
		end
	end

	logic [31:0] q;
	always_ff @(posedge clk) begin
		if (p_we && p_be[3]) wram_b3[p_addr] <= p_wdata[31:24];
		if (p_we && p_be[2]) wram_b2[p_addr] <= p_wdata[23:16];
		if (p_we && p_be[1]) wram_b1[p_addr] <= p_wdata[15:8];
		if (p_we && p_be[0]) wram_b0[p_addr] <= p_wdata[7:0];
		q <= {wram_b3[p_addr], wram_b2[p_addr], wram_b1[p_addr], wram_b0[p_addr]};
	end

	// the responder: one access per request, ack one cycle later
	logic w_ack, w_hold;                    // w_hold: answered, waiting for req to fall
	assign acc_go = clr_done && cpu_req && sel && !w_ack && !w_hold;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			w_ack <= 1'b0; w_hold <= 1'b0;
			clr_addr <= 13'd0; clr_done <= 1'b0;
		end else begin
			// clear: one long per cycle while clr_run, then done until reset
			if (!clr_done && clr_run) begin
				if (clr_addr == 13'h1FFF) clr_done <= 1'b1;
				clr_addr <= clr_addr + 13'd1;
			end
			w_ack <= acc_go;                // q (registered with the access) is valid with it
			if (acc_go)        w_hold <= 1'b1;
			else if (!cpu_req) w_hold <= 1'b0;
		end
	end

	assign sdr_req   = cpu_req && !sel;
	assign cpu_ack   = sel ? w_ack : sdr_ack;
	assign cpu_rdata = sel ? q     : sdr_rdata;
end endgenerate

endmodule
