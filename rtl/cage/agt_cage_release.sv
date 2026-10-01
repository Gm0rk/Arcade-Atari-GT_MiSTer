// agt_cage_release.sv -- when the DSP may run, and how it stops.
//
// Release (clk_sys, `run`):
//   run = !cage_reset             the game has released it (control[1:0] != 0;
//                                 the latch's /SNDRES resets the comm, which
//                                 zeroes control)
//       && !dl_active             no download is rewriting cageram or IRAM
//                                 (the comm keeps its control word through a
//                                 download; the warm reset comes after it)
//       && !need_kick             a download ends a run: the next release needs
//                                 `cage_reset` seen high since (the warm reset
//                                 after every download gives that), or a DSP
//                                 released before a re-download would restart
//                                 on its own, then again at the warm reset
//       && sdr_ready              the controller
//       && ld_done && !ld_bad     the whole boot table parsed
//       && rl_done                ramload finished: every word is in SDRAM,
//                                 not in its FIFO, and read back (CGRM)
//       && !ix_wait               no IRAM word still crossing
//       && pc_ok                  the entry point has crossed
//       && (run || quiet)         once held, released again only after clk_dsp
//                                 has confirmed the hold, so a control 0-then-3
//                                 kick shorter than the synchroniser's window
//                                 is never lost
//   A set with no boot EPROM never gets `rl_done`, so its DSP is never
//   released.
//
// Start state: as `cage31.py`'s `boot_load`, pc = entry and SP = 0xFF
// (`agt_c31`'s S_BOOT). The parser's `entry` crosses once per parse through
// agt_cdc_hs and is held in clk_dsp, stable for the whole hold.
//
// Hold (clk_dsp, `core_rst_n`): the core's reset is asserted and released on
// a clk_dsp edge; an asynchronous assert would drop `mem_req`/`mem_we` at an
// arbitrary moment while `agt_cage_bus` (not reset) samples them. The bus and
// the cache are not reset by a hold: cageram survives the DSP's reset on the
// board, and the newest copy of a variable may be a dirty cache line. The
// started access completes, and the core is released again only when
// `mem_busy` is low, so no late ack meets the restarted core's first fetch.
//
// Invalidate (clk_dsp, `mem_rst_n`): a download rewrites cageram, so
// `mem_rst_n` resets agt_cage_bus and agt_cage_dcache (clearing `valid`)
// while a download is on, but only once the core is held and the bus has
// drained, so no access is cut in half.
//
// Quiet (clk_sys, `quiet`, gates ramload's `sdr_ready`): when a download
// begins with the DSP running, the cache may still be writing a dirty line
// back through the bridge, and those words could land after ZERO has cleared
// the first lines, where the program keeps two variables (0x00002, 0x00005).
// `quiet` is high when the core is held and the bus idle, confirmed from
// clk_dsp, so ramload (which checks `sdr_ready` only when it starts) cannot
// start while anything of the old run is in flight. A value crossed back is
// stale for a round trip (at most 4 clk_dsp edges and 2 clk_sys edges, about
// 156 ns or 9 clk_sys cycles), so `quiet` also needs `run` low for QUIET_WAIT
// clk_sys cycles (16: 279 ns).
//
// Resets: `rst_sys_n`/`rst_dsp_n` are the plumbing reset (dl_rst_n,
// synchronised into clk_dsp): power-on and PLL lock only, like every CAGE
// crossing. Never the warm reset, which reaches the DSP as `cage_reset`.
//
// SDC: the three synchronisers' first stages are named `cagerst_sync_*` so
// `set_false_path -to [get_registers {*cagerst_sync*}]` covers them; keep the
// names. The entry's crossing is agt_cdc_hs's.
`default_nettype none

module agt_cage_release #(
	parameter int QUIET_WAIT = 16
) (
	// clk_sys
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	input  wire         cage_reset,     // agt_cage_top.cage_reset
	input  wire         dl_active,      // agt_rom_download.rom_loading
	input  wire         sdr_ready,
	input  wire         ld_done,        // agt_cage_boot.done
	input  wire         ld_bad,         // agt_cage_boot.bad
	input  wire  [23:0] ld_entry,       // agt_cage_boot.entry
	input  wire         rl_done,        // agt_cage_ramload.done
	input  wire         ix_wait,        // agt_cage_iramxfer.wait_o
	output logic        run,            // the release
	output wire         quiet,          // held and drained: ramload may start
	output logic [15:0] n_releases,     // rising edges of `run` since power-on
	output wire  [31:0] witness,

	// clk_dsp
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	input  wire         mem_busy,       // d_req | dc busy | br busy | s_req | b_req
	output logic        core_rst_n,     // -> agt_c31.rst_n
	output logic        mem_rst_n,      // -> agt_cage_bus, agt_cage_dcache
	output wire  [23:0] boot_pc         // -> agt_c31.boot_pc
);

	// clk_sys
	// The entry point, once per parse. `pc_sent` falls with every download,
	// so a stale entry from the previous parse is never the one released on.
	logic        pc_sent;
	wire         pc_busy;
	wire         pc_send = ld_done && !ld_bad && !dl_active && !pc_sent && !pc_busy;
	// `pc_busy` rises on the edge that sets `pc_sent` and falls once the
	// destination has the word, so this is "sent and delivered".
	wire         pc_ok   = pc_sent && !pc_busy;

	wire boot_ok = sdr_ready && ld_done && !ld_bad && rl_done && !ix_wait && pc_ok;

	localparam logic [4:0] QW = QUIET_WAIT; // must fit in 5 bits
	logic [4:0]  low_cnt;                   // clk_sys cycles since `run` fell
	logic [1:0]  cagerst_sync_quiet;        // clk_dsp's "held and idle", here
	logic        run_q;
	logic        need_kick;             // a download ended the last run
	logic        core_go, held_idle;    // clk_dsp, below

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			pc_sent            <= 1'b0;
			need_kick          <= 1'b0;
			run                <= 1'b0;
			run_q              <= 1'b0;
			low_cnt            <= 5'd0;
			cagerst_sync_quiet <= 2'b00;
			n_releases         <= 16'd0;
		end else begin
			if (dl_active)    pc_sent <= 1'b0;
			else if (pc_send) pc_sent <= 1'b1;

			if (dl_active)       need_kick <= 1'b1;
			else if (cage_reset) need_kick <= 1'b0;

			run   <= boot_ok && !dl_active && !need_kick && !cage_reset
					 && (run || quiet);
			run_q <= run;
			if (run && !run_q && n_releases != 16'hFFFF)
				n_releases <= n_releases + 16'd1;

			if (run)                          low_cnt <= 5'd0;
			else if (low_cnt != QW)           low_cnt <= low_cnt + 5'd1;

			cagerst_sync_quiet <= {cagerst_sync_quiet[0], held_idle};
		end
	end

	assign quiet = !run && (low_cnt == QW) && cagerst_sync_quiet[1];

	agt_cdc_hs #(.W(24)) u_pc (
		.s_clk(clk_sys), .s_rst_n(rst_sys_n),
		.s_send(pc_send), .s_data(ld_entry), .s_busy(pc_busy),
		.d_clk(clk_dsp), .d_rst_n(rst_dsp_n),
		.d_pulse(), .d_data(boot_pc)
	);

	// Overlay `RLSE`. A running DSP is `8EFC nnnn`; nnnn counts releases. Each
	// nibble is four conditions, 1 = satisfied, so a missing one is a missing bit:
	//   [31:28] run, quiet, 0, 0
	//   [27:24] !cage_reset (the game released it), !dl_active, !need_kick, 0
	//   [23:20] sdr_ready, ld_done, !ld_bad, rl_done
	//   [19:16] !ix_wait, pc_ok, 0, 0
	assign witness = {run, quiet, 2'b00,
					  !cage_reset, !dl_active, !need_kick, 1'b0,
					  sdr_ready, ld_done, !ld_bad, rl_done,
					  !ix_wait, pc_ok, 2'b00,
					  n_releases};

	// clk_dsp
	logic [1:0] cagerst_sync_run, cagerst_sync_dl;
	wire        run_d = cagerst_sync_run[1];
	wire        dl_d  = cagerst_sync_dl[1];

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			cagerst_sync_run <= 2'b00;
			cagerst_sync_dl  <= 2'b00;
			core_go          <= 1'b0;
			mem_rst_n        <= 1'b0;
			held_idle        <= 1'b0;
		end else begin
			cagerst_sync_run <= {cagerst_sync_run[0], run};
			cagerst_sync_dl  <= {cagerst_sync_dl[0], dl_active};

			// Released only onto an idle bus; held on the edge `run` falls.
			core_go <= run_d && mem_rst_n && (core_go || !mem_busy);

			// Invalidate while a download is on, once nothing is in flight.
			if (dl_d && !core_go && !mem_busy) mem_rst_n <= 1'b0;
			else if (!dl_d)                    mem_rst_n <= 1'b1;

			held_idle <= !core_go && !mem_busy;
		end
	end

	assign core_rst_n = core_go;

endmodule

`default_nettype wire
