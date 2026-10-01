// agt_board_core.sv -- Atari GT main board: 68EC020, memory map and interrupt
// generation behind plain memory ports.
//
// tb_freerun instantiates this module and boots the Primal Rage ROM through
// its self-test to the game's main loop, so the packaging is
// regression-covered.
//
// Clocks: cpu_clk runs the core, memmap and all memory ports. Only vblank_in
// (from agt_video's timing) is on pix_clk; agt_irq_gen crosses it to cpu_clk
// (toggle + 2FF).
//
// Memory ports are cpu_clk req/ack; an ack may take one or more cycles.
//   rom_*   main program ROM, byte address, 32-bit data; 2MB, in SDRAM (too
//           big for BRAM)
//   ram_*   main RAM, byte address, 32-bit data, byte enables; 512KB
//   cram_*  colorram window, byte address, 16-bit data; shared with the video
//           side's palette storage
//   cage_*  CAGE audio comm (agt_cage_top)
//
// The memmap owns interrupt pending/ack/ipl (acks at 0xE0A000/0xE0C000, per
// atarigt.cpp). This module adds the sources via agt_irq_gen: IRQ4 = vblank
// rising edge (1 per frame), IRQ6 = flat 250 Hz timer (4.17 per frame), and
// the live vblank_level the game polls through sport2 bit 7.
//
// The observability outputs are for benches and the debug overlay; leave them
// unconnected when unused.
`default_nettype none

module agt_board_core (
	// cpu_clk domain
	input  wire         cpu_clk,
	input  wire         cpu_rst_n,
	// Holds only the 68020 in reset while the rest of the board runs, so the
	// EEPROM restore can be replayed into the memmap before the game's first
	// fetch. Tie to 1'b0 where there is nothing to wait for.
	input  wire         cpu_hold,

	// pix_clk domain (from the video block)
	input  wire         pix_clk,
	input  wire         pix_rst_n,
	input  wire         vblank_in,

	// main program ROM
	output wire  [20:0] rom_addr,
	output wire         rom_req,
	input  wire         rom_ack,
	input  wire  [31:0] rom_rdata,

	// main RAM
	output wire  [18:0] ram_addr,
	output wire  [3:0]  ram_be,
	output wire         ram_we,
	output wire  [31:0] ram_wdata,
	output wire         ram_req,
	input  wire         ram_ack,
	input  wire  [31:0] ram_rdata,

	// colorram
	output wire  [18:0] cram_addr,
	output wire         cram_we,
	output wire  [15:0] cram_wdata,
	output wire         cram_req,
	input  wire         cram_ack,
	input  wire  [15:0] cram_rdata,

	// CAGE comm
	output wire         cage_req,
	output wire         cage_we,
	output wire  [3:0]  cage_be,
	output wire  [31:0] cage_wdata,
	input  wire         cage_ack,
	input  wire  [31:0] cage_rdata,
	input  wire         cage_irq,

	// inputs and board latches
	input  wire  [31:0] p1_p2_port,
	input  wire  [15:0] coin_in,
	input  wire         service_n,      // SERVICE / self-test, active low
	// Interrupt visibility for the debug overlay: tells a game waiting on
	// interrupts that never arrive from one that is simply drawing nothing.
	output wire         dbg_video_int_set,
	output wire         dbg_scanline_int_set,
	output wire         dbg_vid_ack,
	output wire         dbg_scan_ack,
	output wire         snoop_we,
	output wire  [13:0] snoop_addr,
	output wire  [3:0]  snoop_be,
	output wire  [31:0] snoop_wd,
	output wire  [31:0] led_value,
	output wire  [31:0] latch_value,
	output wire         latch_wr,
	output wire  [15:0] prot_hit_count,

	// checksum write-back pass-through
	input  wire         chkw_req,
	input  wire  [10:0] chkw_half,
	input  wire  [15:0] chkw_data,
	output wire         chkw_ack,

	// NVRAM save/restore pass-through
	input  wire         nvw_req,
	input  wire  [10:0] nvw_addr,
	input  wire  [7:0]  nvw_data,
	output wire         nvw_ack,
	input  wire         nvr_req,
	input  wire  [10:0] nvr_addr,
	output wire  [7:0]  nvr_data,
	output wire         nvr_valid,
	output wire         eeprom_wr_evt,
	output wire         eeprom_rd_evt,

	// observability (benches, debug overlay)
	// Instruction-fetch strobes, for the IFET overlay slot to count per video
	// frame (the core has no notion of a frame). `bus_ifetch && rom_req &&
	// rom_ack` is a fetch that took a bus cycle; `bus_ifetch_hit` one that did
	// not.
	output wire         bus_ifetch,
	output wire         bus_ifetch_hit,
	output wire         insn_done,
	output wire  [31:0] insn_pc,
	output wire  [15:0] sr_out,
	output wire         unimplemented,
	output wire  [15:0] unimpl_opcode,
	output wire  [31:0] unimpl_pc,
	output wire         cpu_reset_out   // RESET instruction executed
);

	// CPU <-> memmap bus
	wire [31:0] bus_addr, bus_wdata, bus_rdata;
	wire [2:0]  bus_size;
	wire        bus_we, bus_req, bus_ack;
	wire [2:0]  ipl;

	// interrupt sources (pix_clk in, cpu_clk out)
	wire video_int_set, scanline_int_set, vblank_level;
	assign dbg_video_int_set    = video_int_set;
	assign dbg_scanline_int_set = scanline_int_set;

	agt_irq_gen u_irq (
		.pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .vblank_in(vblank_in),
		.cpu_clk(cpu_clk), .cpu_rst_n(cpu_rst_n),
		.video_int_set(video_int_set),
		.scanline_int_set(scanline_int_set),
		.vblank_level(vblank_level)
	);

	m68020_core u_cpu (
		.clk(cpu_clk), .rst_n(cpu_rst_n & ~cpu_hold), .ipl_in(ipl),
		.bus_addr(bus_addr), .bus_size(bus_size), .bus_we(bus_we),
		.bus_wdata(bus_wdata), .bus_req(bus_req), .bus_ack(bus_ack),
		.bus_rdata(bus_rdata),
		.bus_ifetch(bus_ifetch), .bus_ifetch_hit(bus_ifetch_hit),
		.insn_done(insn_done), .insn_pc(insn_pc), .sr_out(sr_out),
		.reset_out(cpu_reset_out),
		.unimplemented(unimplemented), .unimpl_opcode(unimpl_opcode),
		.unimpl_pc(unimpl_pc)
	);

	agt_main_memmap u_memmap (
		.clk(cpu_clk), .rst_n(cpu_rst_n),
		.cpu_addr(bus_addr), .cpu_size(bus_size), .cpu_we(bus_we),
		.cpu_wdata(bus_wdata), .cpu_req(bus_req), .cpu_ack(bus_ack),
		.cpu_rdata(bus_rdata),
		.rom_addr(rom_addr), .rom_req(rom_req), .rom_ack(rom_ack),
		.rom_rdata(rom_rdata),
		.ram_addr(ram_addr), .ram_be(ram_be), .ram_we(ram_we),
		.ram_wdata(ram_wdata), .ram_req(ram_req), .ram_ack(ram_ack),
		.ram_rdata(ram_rdata),
		.cram_addr(cram_addr), .cram_we(cram_we), .cram_wdata(cram_wdata),
		.cram_req(cram_req), .cram_ack(cram_ack), .cram_rdata(cram_rdata),
		.cage_req(cage_req), .cage_we(cage_we), .cage_be(cage_be),
		.cage_wdata(cage_wdata), .cage_ack(cage_ack), .cage_rdata(cage_rdata),
		.video_int_set(video_int_set), .scanline_int_set(scanline_int_set),
		.cage_irq(cage_irq), .vblank_level(vblank_level), .ipl_out(ipl),
		.p1_p2_port(p1_p2_port), .coin_in(coin_in),
		.service_n(service_n),
		.dbg_vid_ack(dbg_vid_ack), .dbg_scan_ack(dbg_scan_ack),
		.snoop_we(snoop_we), .snoop_addr(snoop_addr),
		.snoop_be(snoop_be), .snoop_wd(snoop_wd),
		.led_value(led_value),
		.latch_value(latch_value), .latch_wr(latch_wr),
		.prot_hit_count(prot_hit_count),
		.chkw_req(chkw_req), .chkw_half(chkw_half),
		.chkw_data(chkw_data), .chkw_ack(chkw_ack),
		.nvw_req(nvw_req), .nvw_addr(nvw_addr), .nvw_data(nvw_data),
		.nvw_ack(nvw_ack),
		.nvr_req(nvr_req), .nvr_addr(nvr_addr), .nvr_data(nvr_data),
		.nvr_valid(nvr_valid), .eeprom_wr_evt(eeprom_wr_evt),
		.eeprom_rd_evt(eeprom_rd_evt)
	);

endmodule
`default_nettype wire
