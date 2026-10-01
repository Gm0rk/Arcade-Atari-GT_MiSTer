// agt_cage.sv -- the CAGE sound board as the game instantiates it.
//
// One module, so the bench (`tb_cage`, through these ports only) tests what
// gets compiled.
//
//   clk_sys                                           clk_dsp
//   download bytes -> agt_cage_boot                   agt_c31 (BOOT_START)
//                      |        \                       |
//              agt_cage_ramload  agt_cage_iramxfer ==> boot port
//                      | A                              agt_cage_bus
//   SDRAM <- agt_cage_portmux <== agt_cage_membridge <== agt_cage_dcache
//                      | B = a second portmux:
//                      |   the cache's bridge, then agt_cage_sbank <== s_*
//   68020 -> agt_cage_top <== agt_cage_mbx_cdc ==> the bus's mailbox
//            (or agt_cage_stub, below)
//   agt_cage_release: when the core runs, how it stops, when ramload starts
//   audio_l/r <== agt_cage_dac <== the core's serial-port words
//   agt_cage_icache beside agt_c31: fetches that hit skip the port
//
// 1. Who answers the 68020: the real comm (`agt_cage_top`) when the last
//    download delivered a boot table that parsed, else `agt_cage_stub`. A set
//    with no boot EPROM can never release its DSP, and a comm with no DSP
//    behind it would leave every command pending for ever; the stub consumes
//    each after 64 cycles. The choice follows the data, not the set, and is
//    made once as the download ends: `saw_boot` (the boot region's first
//    byte arrived in this download) and the parser's `done && !bad`. `done`
//    alone would not do: it stays high from the previous download when the
//    new one has no boot region. The 68020 is held for the whole download,
//    so the switch never lands in the middle of one of its accesses.
//
// 2. The bus's boot-ROM port (`b_*`, 0x400000) is acked the cycle after the
//    request with zero data; tied off, the bus would wait for ever. The
//    sound-bank port (`s_*`, 0xC00000-0xFFFFFF) is `agt_cage_sbank`: the sound
//    data in SDRAM at `SND_BASE`, byte-reversed into the DSP's little-endian
//    words.
//
// 3. `agt_c31` halts (S_HALT, `unimplemented`) on any handler it does not
//    implement, and a halted core never reads the mailbox again. So
//    `dsp_halted` (the flag, synchronised) turns on the stub's behaviour for
//    commands only: a pending command is consumed 64 clk_sys cycles after it
//    is posted, as `agt_cage_stub` does. `w_rlse` bit 29 reports the halt
//    (running `8EFC`, halted `AEFC`); the game's reset kick (control 0, then
//    non-zero) clears it with the core's reset. `w_chlt` keeps what it halted
//    on across those kicks, for the overlay's CHLT.
//
// 4. Sound: `agt_c31` hands over every word its DMA feeds the serial port and
//    the model cycles each step earns. `agt_cage_dac` plays the words in those
//    cycles (the DSP's own time, as its timers run), mixes the four channels
//    to stereo as `atarigt_stereo` does, and crosses the pair to clk_sys.
//
// 5. `agt_cage_icache` holds 1,024 words of code. The core looks up its next
//    fetch a cycle ahead (two consecutive words per lookup) and takes the word
//    in S_FETCH or S_EOS on a hit; otherwise it fetches through the port and
//    the cache keeps the word returned. A download flushes it (`mem_rst_n`); a
//    kick does not, the code being the same after it.
//
// 6. `agt_cage_bus` passes cageram accesses straight to `agt_cage_dcache` and
//    its ack straight back; a read hit costs the core two cycles. The bus
//    holds no copy of the request, so `d_req` falls the moment the core is
//    held while a fill may still be in flight: `mem_busy` takes the cache's
//    own `busy`.
//
// 7. `w_dstl`: where the DSP's cycles go, four shares of the last 2^DSTL_W
//    clk_dsp cycles, each x256:
//      [31:24]  the data cache's bridge busy (`m_req`): line fills and
//               write-backs, the data misses
//      [23:16]  the core held by the governor (`dbg_held`), (8)
//      [15:8]   a port fetch waiting (`c_req && c_ifetch && !c_ack`): the
//               instruction cache's misses (their line fills also count in
//               [31:24])
//      [7:0]    the core in S_FP (`dbg_fp`): the float ALU, the mixing's
//               signature; the program's wait loop has none
//    Each count is DSTL_W bits against a 2^DSTL_W-cycle window, so its top
//    eight bits are the share and nothing saturates. One agt_cdc_hs a window.
//
// 8. Real-time governor: everything audible is in model time (4), so the sound
//    is as fast as the core earns model cycles. `agt_cage_gov` keeps a credit
//    (+1428 a clk_dsp cycle, -3125 a model cycle: the chip's ratio at 630/17)
//    and `hold`s the core in S_EXEC while it is negative, so model time never
//    runs ahead of real time by more than GOV_CAP (+48). GOV_CAP is also what
//    a slow stretch may bank: the program mixes a burst (slower than the chip)
//    then polls for the DMA (faster), and the poll must be able to make up
//    the deficit or every screen slows. 65,536 is more than a whole burst
//    (128 words x 384 = 49,152 model cycles). A slower stretch is never held;
//    a faster one runs ahead until the bank is spent, then at the chip's rate.
//
// 9. `agt_cage_membridge #(.LINE(1))` crosses a whole cache line once each
//    way, posts the write-back, and makes one port access per line
//    (`p_line`: agt_sdram reads or writes the four words as one burst on one
//    row). Its `busy` joins `mem_busy`, so the release never counts the bus
//    idle while a posted line is still going to SDRAM. BRIDGE_LINE 0 builds
//    the word-at-a-time bridge, for benches that compare the two.
//
// Resets
//   dl_rst_n     power-on and PLL lock only: the parser, ramload, every
//                crossing, the port mux, the release, the select.
//   cage_rst_n   the 68020 side: sys_rst_n && the latch's /SNDRES bit.
//                Resets whichever comm is answering.
//   agt_cage_release resets the core on every hold, and the bus and the
//                cache only when a download rewrites cageram.
//
// SDC: every crossing inside is an agt_cdc_hs (`*cdchs_*`) or a
// `cagerst_sync*` synchroniser. Keep those names: the SDC's false-path cuts
// match them. `cagerst_sync_chlt` is a capture, not a synchroniser (it samples
// a bus that has held still, on the synchronised halt's edge) and carries the
// name so the same cut covers it.
`default_nettype none

module agt_cage #(
	parameter logic [25:0] BASE     = 26'h2B20000,  // SDR_BASE_CAGERAM
	parameter logic [25:0] SND_BASE = 26'h2720000,  // SDR_BASE_CAGE
	// DSTL's window, 2^DSTL_W clk_dsp cycles (25: 0.905 s at 630/17).
	// `tb_cage` sets 20 so a window closes inside its first phase.
	parameter int          DSTL_W   = 25,
	// Real-time governor (8). GOV_EN 0 builds it with `en` low: never holds.
	// GOV_CAP: the model cycles a slow stretch may bank for the wait loop after
	// it to spend.
	parameter bit          GOV_EN   = 1'b1,
	parameter int          GOV_CAP  = 65536,
	// The cache's bridge crosses a line at a time (9); 0: a word at a time.
	parameter bit          BRIDGE_LINE = 1'b1
) (
	input  wire         clk_sys,
	input  wire         clk_dsp,
	input  wire         dl_rst_n,       // clk_sys: power-on / PLL only
	input  wire         cage_rst_n,     // clk_sys: the 68020 side

	// the download (clk_sys)
	input  wire         dl_active,      // agt_rom_download.rom_loading
	input  wire         dl_first,       // the boot region's first byte
	input  wire         dl_valid,       // a byte of the boot region
	input  wire  [7:0]  dl_byte,
	output wire         dl_wait,        // -> ioctl_wait
	input  wire         sdr_ready,

	// agt_sdram's cage port (clk_sys): a word, or the cache's line
	output wire  [25:0] p_addr,
	output wire         p_we,
	output wire         p_line,
	output wire  [127:0] p_wdata,
	output wire         p_req,
	input  wire         p_ack,
	input  wire  [127:0] p_rdata,

	// the 68020 at 0xC00000: agt_cage_stub's contract (clk_sys)
	input  wire         req,
	input  wire         we,
	input  wire  [3:0]  be,
	input  wire  [31:0] wdata,
	output wire         ack,
	output wire  [31:0] rdata,
	output wire         irq,
	output wire  [15:0] dbg_last_cmd,
	output wire  [15:0] dbg_control,
	output wire  [15:0] dbg_ctrl_writes,
	output wire  [15:0] dbg_main_reads,

	// audio: signed 16-bit, clk_sys
	input  wire         vblank,         // clk_sys: the witness's window
	output wire  [15:0] audio_l,
	output wire  [15:0] audio_r,

	// status (clk_sys)
	output logic        dsp_present,    // 1 = the real comm answers the 68020
	output wire  [31:0] w_cgrm,         // agt_cage_ramload's witness
	output wire  [31:0] w_rlse,         // agt_cage_release's witness, halt in bit 29
	output wire  [31:0] w_chlt,         // last halt: ir[31:16], pc[15:0] (3)
	output wire  [31:0] w_sbnk,         // agt_cage_sbank's witness
	output wire  [31:0] w_dacw,         // agt_cage_dac's witness
	output wire  [31:0] w_dstl          // where the DSP's cycles go (7)
);

	// the plumbing reset in clk_dsp
	wire rst_dsp_n;
	agt_cage_rstsync u_rs_dsp (.clk(clk_dsp), .arst_n(dl_rst_n), .rst_n(rst_dsp_n));

	// the boot table, parsed out of the download
	wire         ld_we, ld_done, ld_bad, i_we;
	wire  [17:0] ld_addr;
	wire  [31:0] ld_data, i_data;
	wire  [10:0] i_addr;
	wire  [23:0] ld_entry;

	agt_cage_boot u_boot (
		.clk(clk_sys), .rst_n(dl_rst_n),
		.dl_first(dl_first), .dl_valid(dl_valid), .dl_byte(dl_byte),
		.ram_we(ld_we), .ram_addr(ld_addr), .ram_data(ld_data),
		.iram_we(i_we), .iram_addr(i_addr), .iram_data(i_data),
		.done(ld_done), .bad(ld_bad),
		.boot_width(), .boot_bctrl(), .entry(ld_entry), .words_loaded(),
		.blocks_loaded(), .wr_hi(), .oor_words()
	);

	// cageram into SDRAM, started only once the DSP side is quiet
	wire         rl_wait, rl_done, quiet;
	wire  [25:0] a_addr;  wire a_we, a_req, a_ack;  wire [31:0] a_wdata;  wire [127:0] a_rdata;

	agt_cage_ramload #(.BASE(BASE), .WORDS(65536), .FIFO_DEPTH(8)) u_ramload (
		.clk(clk_sys), .rst_n(dl_rst_n),
		.sdr_ready(sdr_ready && quiet), .dl_active(dl_active),
		.ld_we(ld_we), .ld_addr(ld_addr), .ld_data(ld_data), .ld_done(ld_done),
		.wait_o(rl_wait),
		.p_addr(a_addr), .p_we(a_we), .p_wdata(a_wdata), .p_req(a_req),
		.p_ack(a_ack), .p_rdata(a_rdata[31:0]),
		.phase(), .done(rl_done), .sum(),
		.n_loaded(), .n_dropped(), .witness(w_cgrm)
	);

	// the 12 IRAM words into the core's boot port
	wire         ix_wait;
	wire         boot_we;  wire [10:0] boot_addr;  wire [31:0] boot_data;
	agt_cage_iramxfer u_iram (
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n),
		.i_we(i_we), .i_addr(i_addr), .i_data(i_data),
		.wait_o(ix_wait), .n_dropped(),
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.boot_we(boot_we), .boot_addr(boot_addr), .boot_data(boot_data)
	);

	assign dl_wait = rl_wait || ix_wait;

	// The SDRAM port: ramload (A, priority) and the DSP side (B). B is itself a
	// portmux of the cache's bridge (its A) and the sound bank (its B). Both hold
	// `req` until `ack` and drop it there, and so does a portmux's own `p_req`,
	// so a mux is a legal master of a mux. The core makes one access at a time,
	// so the two never contend: the line bridge's posted write-back is always
	// followed by the fill the core is waiting for, which queues behind it.
	// ramload and the sound bank are word masters: `line` low, data in [31:0].
	wire  [25:0] b_paddr;  wire b_pwe, b_pline, b_preq, b_pack;  wire [127:0] b_pwdata, b_prdata;
	wire  [25:0] c_paddr;  wire c_pwe, c_pline, c_preq, c_pack;  wire [127:0] c_pwdata, c_prdata;
	wire  [25:0] n_paddr;  wire n_pwe, n_preq, n_pack;  wire [31:0] n_pwdata;  wire [127:0] n_prdata;

	agt_cage_portmux u_mux_dsp (
		.clk(clk_sys), .rst_n(dl_rst_n),
		.a_addr(c_paddr), .a_we(c_pwe), .a_line(c_pline), .a_wdata(c_pwdata), .a_req(c_preq), .a_ack(c_pack), .a_rdata(c_prdata),
		.b_addr(n_paddr), .b_we(n_pwe), .b_line(1'b0), .b_wdata({96'd0, n_pwdata}), .b_req(n_preq), .b_ack(n_pack), .b_rdata(n_prdata),
		.p_addr(b_paddr), .p_we(b_pwe), .p_line(b_pline), .p_wdata(b_pwdata), .p_req(b_preq), .p_ack(b_pack), .p_rdata(b_prdata),
		.n_a(), .n_b()
	);

	agt_cage_portmux u_mux (
		.clk(clk_sys), .rst_n(dl_rst_n),
		.a_addr(a_addr), .a_we(a_we), .a_line(1'b0), .a_wdata({96'd0, a_wdata}), .a_req(a_req), .a_ack(a_ack), .a_rdata(a_rdata),
		.b_addr(b_paddr), .b_we(b_pwe), .b_line(b_pline), .b_wdata(b_pwdata), .b_req(b_preq), .b_ack(b_pack), .b_rdata(b_prdata),
		.p_addr(p_addr), .p_we(p_we), .p_line(p_line), .p_wdata(p_wdata), .p_req(p_req), .p_ack(p_ack), .p_rdata(p_rdata),
		.n_a(), .n_b()
	);

	// who answers the 68020 (1)
	logic dl_q, saw_boot;
	always_ff @(posedge clk_sys or negedge dl_rst_n) begin
		if (!dl_rst_n) begin
			dl_q        <= 1'b0;
			saw_boot    <= 1'b0;
			dsp_present <= 1'b0;
		end else begin
			dl_q <= dl_active;
			if (dl_active && !dl_q)  saw_boot <= 1'b0;       // a new download
			if (dl_valid && dl_first) saw_boot <= 1'b1;      // ... with a boot region
			if (!dl_active && dl_q)                          // it has ended
				dsp_present <= saw_boot && ld_done && !ld_bad;
		end
	end

	wire         top_ack, top_irq, top_irq0_level, cmd_post, cage_reset;
	wire         cpu_to_cage_ready, cage_to_cpu_ready;
	wire  [31:0] top_rdata;
	wire  [15:0] from_main;
	wire  [15:0] top_last_cmd, top_control, top_ctrl_writes, top_main_reads;
	wire         s_cmd_read, s_resp_we;
	wire  [15:0] s_resp_data;

	// (3): a halted core's commands are consumed as the stub does
	wire         c_unimpl;                          // agt_c31, clk_dsp, below
	wire  [31:0] c_dbg_ir;                          // agt_c31, clk_dsp
	wire  [23:0] c_dbg_pc;
	wire         c_dbg_fp;                          // agt_c31, clk_dsp
	logic [1:0]  cagerst_sync_halt;
	wire         dsp_halted = cagerst_sync_halt[1];
	logic [6:0]  halt_cnt;
	logic        halt_consume;
	always_ff @(posedge clk_sys or negedge dl_rst_n) begin
		if (!dl_rst_n) begin
			cagerst_sync_halt <= 2'b00;
			halt_cnt          <= 7'd0;
			halt_consume      <= 1'b0;
		end else begin
			cagerst_sync_halt <= {cagerst_sync_halt[0], c_unimpl};
			halt_consume      <= 1'b0;
			if (dsp_halted && cpu_to_cage_ready && !halt_consume) begin
				if (halt_cnt == 7'd64) begin               // CONSUME_DELAY
					halt_consume <= 1'b1;
					halt_cnt     <= 7'd0;
				end else
					halt_cnt <= halt_cnt + 7'd1;
			end else
				halt_cnt <= 7'd0;
		end
	end

	// What it halted on, for the overlay's CHLT: the last halt since power-on.
	//   upper 16  the instruction word's top half, ir[31:16]: opcode and
	//             addressing mode, which name the handler
	//   lower 16  the PC it started at, [15:0]. The code is 1000-3496;
	//             IRAM's vector branches read 9FC1-9FCB (0x809FCx)
	// A top half that differs from the boot image's word at that PC is a fetch
	// that went wrong, not a handler the core lacks.
	//
	// Crossing: `dbg_ir`/`dbg_pc` stop changing the cycle `unimplemented` rises,
	// and are sampled only on the edge its two-flop copy rises, two clk_sys
	// cycles later, on a bus that has held still since. The capture register is
	// named `cagerst_sync_*` so the SDC's `-to *cagerst_sync*` cut covers it.
	// Reset by `dl_rst_n` only, so a halt stays readable after the game's kick
	// releases the DSP again.
	logic [31:0] cagerst_sync_chlt;
	logic        halt_seen;
	always_ff @(posedge clk_sys or negedge dl_rst_n) begin
		if (!dl_rst_n) begin
			cagerst_sync_chlt <= 32'd0;
			halt_seen         <= 1'b0;
		end else begin
			halt_seen <= dsp_halted;
			if (dsp_halted && !halt_seen)
				cagerst_sync_chlt <= {c_dbg_ir[31:16], c_dbg_pc[15:0]};
		end
	end
	assign w_chlt = cagerst_sync_chlt;

	agt_cage_top u_top (
		.clk(clk_sys), .rst_n(cage_rst_n),
		.req(req && dsp_present), .we(we), .be(be), .wdata(wdata),
		.ack(top_ack), .rdata(top_rdata), .irq(top_irq),
		.dsp_irq0_level(top_irq0_level), .cmd_post(cmd_post),
		.from_main(from_main),
		.cpu_to_cage_ready(cpu_to_cage_ready),
		.cage_to_cpu_ready(cage_to_cpu_ready),
		.dsp_cmd_read(s_cmd_read || halt_consume), .dsp_resp_we(s_resp_we),
		.dsp_resp_data(s_resp_data),
		.cage_reset(cage_reset),
		.audio_l(), .audio_r(), .dsp_present(),      // agt_cage_dac drives the audio
		.dbg_last_cmd(top_last_cmd), .dbg_control(top_control),
		.dbg_ctrl_writes(top_ctrl_writes), .dbg_main_reads(top_main_reads)
	);

	wire         stub_ack, stub_irq;
	wire  [31:0] stub_rdata;
	wire  [15:0] stub_last_cmd, stub_control, stub_ctrl_writes, stub_main_reads;

	agt_cage_stub u_stub (
		.clk(clk_sys), .rst_n(cage_rst_n),
		.req(req && !dsp_present), .we(we), .be(be), .wdata(wdata),
		.ack(stub_ack), .rdata(stub_rdata), .irq(stub_irq),
		.dbg_last_cmd(stub_last_cmd), .dbg_control(stub_control),
		.dbg_ctrl_writes(stub_ctrl_writes), .dbg_main_reads(stub_main_reads)
	);

	assign ack             = dsp_present ? top_ack         : stub_ack;
	assign rdata           = dsp_present ? top_rdata       : stub_rdata;
	assign irq             = dsp_present ? top_irq         : stub_irq;
	assign dbg_last_cmd    = dsp_present ? top_last_cmd    : stub_last_cmd;
	assign dbg_control     = dsp_present ? top_control     : stub_control;
	assign dbg_ctrl_writes = dsp_present ? top_ctrl_writes : stub_ctrl_writes;
	assign dbg_main_reads  = dsp_present ? top_main_reads  : stub_main_reads;

	// the crossings
	wire  [15:0] mb_from_main, mb_resp_data;
	wire         mb_cmd_read, mb_resp_we, mb_wait, dsp_irq0;

	agt_cage_mbx_cdc u_mbx (
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n),
		.s_cmd_post(cmd_post), .s_from_main(from_main), .s_ready(cpu_to_cage_ready),
		.s_cmd_read(s_cmd_read), .s_resp_we(s_resp_we), .s_resp_data(s_resp_data),
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.d_from_main(mb_from_main), .d_irq0(dsp_irq0),
		.d_cmd_read(mb_cmd_read), .d_resp_we(mb_resp_we), .d_resp_data(mb_resp_data),
		.d_wait(mb_wait),
		.n_posts(), .n_clears(), .n_stale()
	);

	wire  [15:0] m_addr;  wire m_req, m_we, m_ack;  wire [31:0] m_wdata, m_rdata;

	wire         br_busy;                  // a posted line still out

	agt_cage_membridge #(.BASE(BASE), .LINE(BRIDGE_LINE)) u_bridge (
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.m_addr(m_addr), .m_req(m_req), .m_we(m_we), .m_wdata(m_wdata),
		.m_rdata(m_rdata), .m_ack(m_ack), .busy(br_busy),
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n),
		.p_addr(c_paddr), .p_we(c_pwe), .p_line(c_pline), .p_wdata(c_pwdata), .p_req(c_preq),
		.p_ack(c_pack), .p_rdata(c_prdata),
		.n_rd(), .n_wr(), .n_rdl(), .n_wrl(), .n_ovr()
	);

	// the release
	wire         core_rst_n, mem_rst_n, mem_busy;
	wire  [23:0] boot_pc;
	wire  [31:0] rel_witness;

	agt_cage_release u_release (
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n),
		.cage_reset(cage_reset), .dl_active(dl_active), .sdr_ready(sdr_ready),
		.ld_done(ld_done), .ld_bad(ld_bad), .ld_entry(ld_entry),
		.rl_done(rl_done), .ix_wait(ix_wait),
		.run(), .quiet(quiet), .n_releases(), .witness(rel_witness),
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.mem_busy(mem_busy),
		.core_rst_n(core_rst_n), .mem_rst_n(mem_rst_n), .boot_pc(boot_pc)
	);
	// agt_cage_release leaves bit 29 at 0; it carries the halt (3).
	assign w_rlse = {rel_witness[31:30], dsp_halted, rel_witness[28:0]};

	// clk_dsp: the core, its bus, its cache
	wire  [23:0] c_addr;
	wire         c_req, c_we, c_ifetch, c_ack;
	wire  [31:0] c_wdata, c_rdata;

	// IOF's INXF1 is `cage_to_cpu_ready` (the reply not yet taken), crossed into
	// clk_dsp by a 2-flop synchroniser named `cagerst_sync*` so the SDC's cut
	// covers it. INXF0 is `dsp_irq0` itself: in `cage.cpp` IOF bit 3 and IRQ0 are
	// set by the same post and cleared by the same read, and `agt_cage_mbx_cdc`
	// clears its copy in the cycle after the DSP's read. The program never reads
	// IOF; both are for the register file's sake.
	logic [1:0] cagerst_sync_xf1;
	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) cagerst_sync_xf1 <= 2'b00;
		else            cagerst_sync_xf1 <= {cagerst_sync_xf1[0], cage_to_cpu_ready};
	end

	// the core's serial-port feed and model clock, for u_dac below
	wire         c_dac_stb, c_dac_first, c_mcyc_stb;
	wire  [15:0] c_dac_word;
	wire  [4:0]  c_mcyc_add;
	wire  [24:0] c_dac_per;
	// the instruction cache's lookup, for u_icache below
	wire  [23:0] c_ic_la;
	wire         c_ic_hit0, c_ic_hit1, c_ic_used;       // two words per lookup
	wire  [31:0] c_ic_word0, c_ic_word1;

	// the governor's hold, and the cycles the core waited on it
	wire         c_gov_hold, c_dbg_held;

	agt_c31 #(.BOOT_START(1'b1), .IC_EN(1'b1), .HOLD_EN(1'b1)) u_c31 (
		.clk(clk_dsp), .rst_n(core_rst_n),
		.mem_addr(c_addr), .mem_req(c_req), .mem_we(c_we),
		.mem_wdata(c_wdata), .mem_rdata(c_rdata), .mem_ack(c_ack),
		.mem_ifetch(c_ifetch),
		.irq_in({3'b000, dsp_irq0}),
		.xf_in({cagerst_sync_xf1[1], dsp_irq0}),
		.boot_we(boot_we), .boot_addr(boot_addr), .boot_data(boot_data),
		.boot_pc(boot_pc),
		.insn_done(), .insn_pc(), .unimplemented(c_unimpl),
		.dbg_ir(c_dbg_ir), .dbg_pc(c_dbg_pc),
		.dbg_fp(c_dbg_fp),
		.hold(c_gov_hold), .dbg_held(c_dbg_held),
		.dac_stb(c_dac_stb), .dac_word(c_dac_word),
		.dac_first(c_dac_first), .mcyc_stb(c_mcyc_stb),
		.mcyc_add(c_mcyc_add), .dac_per(c_dac_per),
		.ic_la(c_ic_la), .ic_hit0(c_ic_hit0), .ic_word0(c_ic_word0),
		.ic_hit1(c_ic_hit1), .ic_word1(c_ic_word1), .ic_used(c_ic_used)
	);

	// Real-time governor (8): `hold` while the model is ahead of the chip's time.
	// On `rst_dsp_n`: a core held in reset earns nothing, so the credit waits at
	// its cap and the release runs at once.
	agt_cage_gov #(.CAP(GOV_CAP)) u_gov (
		.clk(clk_dsp), .rst_n(rst_dsp_n), .en(GOV_EN),
		.mcyc_stb(c_mcyc_stb), .mcyc_add(c_mcyc_add),
		.hold(c_gov_hold), .n_hold()
	);

	// Instruction cache: filled from the fetches the port answers, flushed while
	// a download rewrites cageram (`mem_rst_n`, as the data cache).
	agt_cage_icache u_icache (
		.clk(clk_dsp), .rst_n(rst_dsp_n), .flush(!mem_rst_n),
		.la(c_ic_la), .hit0(c_ic_hit0), .word0(c_ic_word0),
		.hit1(c_ic_hit1), .word1(c_ic_word1),
		.p_req(c_req), .p_ack(c_ack), .p_we(c_we), .p_ifetch(c_ifetch),
		.p_addr(c_addr), .p_rdata(c_rdata),
		.n_fill(), .n_inval(), .flushing()
	);

	// Serial port and DACs: the core's words, played in the model's cycles.
	// `flush` while the core is held: a kick drops what was queued and restarts
	// the channel count.
	agt_cage_dac u_dac (
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n), .flush(!core_rst_n),
		.dac_stb(c_dac_stb), .dac_word(c_dac_word), .dac_first(c_dac_first),
		.mcyc_stb(c_mcyc_stb), .mcyc_add(c_mcyc_add), .dac_per(c_dac_per),
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n), .vblank(vblank),
		.audio_l(audio_l), .audio_r(audio_r), .witness(w_dacw),
		.n_push(), .n_pop(), .n_ovr(), .n_words()
	);

	wire  [15:0] d_addr;  wire d_req, d_we, d_ack;  wire [31:0] d_wdata, d_rdata;
	wire  [21:0] s_addr;  wire s_req;
	wire  [31:0] s_rdata; wire s_ack;
	wire  [18:0] b_addr;  wire b_req;

	agt_cage_bus u_bus (
		.clk(clk_dsp), .rst_n(mem_rst_n),
		.c_addr(c_addr), .c_req(c_req), .c_we(c_we),
		.c_wdata(c_wdata), .c_ifetch(c_ifetch),
		.c_rdata(c_rdata), .c_ack(c_ack),
		.d_addr(d_addr), .d_req(d_req), .d_we(d_we), .d_wdata(d_wdata),
		.d_rdata(d_rdata), .d_ack(d_ack),
		.mb_from_main(mb_from_main), .mb_cmd_read(mb_cmd_read),
		.mb_resp_we(mb_resp_we), .mb_resp_data(mb_resp_data), .mb_wait(mb_wait),
		// Boot ROM (2): acked the cycle after, with zero. `b_req` is the bus's own
		// register and it drops it on the edge it sees the ack, so this is one
		// access each, never a loop.
		.s_addr(s_addr), .s_req(s_req), .s_rdata(s_rdata), .s_ack(s_ack),
		.b_addr(b_addr), .b_req(b_req), .b_rdata(8'd0), .b_ack(b_req),
		.n_fetch(), .n_ram_r(), .n_ram_w(), .n_mail_r(), .n_mail_w(),
		.n_nopw(), .n_sound_r(), .n_boot_r(), .n_oob()
	);
	// `d_req` comes straight from the core, so it falls the moment the core is
	// held; the cache's `busy` covers an access still in flight, and the
	// bridge's a posted line write still going to SDRAM (9).
	wire dc_busy;
	assign mem_busy = d_req || dc_busy || br_busy || s_req || b_req;

	// the sound bank
	agt_cage_sbank #(.BASE(SND_BASE)) u_sbank (
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.s_addr(s_addr), .s_req(s_req), .s_rdata(s_rdata), .s_ack(s_ack),
		.clk_sys(clk_sys), .rst_sys_n(dl_rst_n),
		.p_addr(n_paddr), .p_we(n_pwe), .p_wdata(n_pwdata), .p_req(n_preq),
		.p_ack(n_pack), .p_rdata(n_prdata[31:0]),
		.witness(w_sbnk), .n_words()
	);

	agt_cage_dcache u_dcache (
		.clk(clk_dsp), .rst_n(mem_rst_n),
		.c_addr(d_addr), .c_req(d_req), .c_we(d_we), .c_wdata(d_wdata),
		.c_rdata(d_rdata), .c_ack(d_ack),
		.m_addr(m_addr), .m_req(m_req), .m_we(m_we), .m_wdata(m_wdata),
		.m_rdata(m_rdata), .m_ack(m_ack),
		.n_hit(), .n_miss(), .n_wb(), .busy(dc_busy)
	);

	// DSTL (7): a free-running 2^DSTL_W-cycle window. In its last cycle the four
	// counts (which then hold cycles 0 .. 2^DSTL_W - 2 of it) are latched and
	// sent, and that cycle's own conditions start the next window's.
	logic [DSTL_W-1:0] dstl_win, dstl_mis, dstl_gov, dstl_ifw, dstl_fp;
	logic        dstl_send;
	logic [31:0] dstl_word;
	wire         dstl_c_mis = m_req;
	wire         dstl_c_gov = c_dbg_held;
	wire         dstl_c_ifw = c_req && c_ifetch && !c_ack;
	wire         dstl_c_fp  = c_dbg_fp;
	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			dstl_win  <= '0;
			dstl_mis  <= '0;
			dstl_gov  <= '0;
			dstl_ifw  <= '0;
			dstl_fp   <= '0;
			dstl_send <= 1'b0;
			dstl_word <= 32'd0;
		end else begin
			dstl_send <= 1'b0;
			dstl_win  <= dstl_win + 1'b1;
			if (&dstl_win) begin
				dstl_word <= {dstl_mis[DSTL_W-1 -: 8], dstl_gov[DSTL_W-1 -: 8],
							  dstl_ifw[DSTL_W-1 -: 8], dstl_fp[DSTL_W-1 -: 8]};
				dstl_send <= 1'b1;
				dstl_mis  <= {{(DSTL_W-1){1'b0}}, dstl_c_mis};
				dstl_gov  <= {{(DSTL_W-1){1'b0}}, dstl_c_gov};
				dstl_ifw  <= {{(DSTL_W-1){1'b0}}, dstl_c_ifw};
				dstl_fp   <= {{(DSTL_W-1){1'b0}}, dstl_c_fp};
			end else begin
				dstl_mis  <= dstl_mis + {{(DSTL_W-1){1'b0}}, dstl_c_mis};
				dstl_gov  <= dstl_gov + {{(DSTL_W-1){1'b0}}, dstl_c_gov};
				dstl_ifw  <= dstl_ifw + {{(DSTL_W-1){1'b0}}, dstl_c_ifw};
				dstl_fp   <= dstl_fp  + {{(DSTL_W-1){1'b0}}, dstl_c_fp};
			end
		end
	end

	// `dstl_word` is latched on the edge that raises `dstl_send` and then holds
	// for 2^DSTL_W cycles, far longer than the crossing's round trip, so the
	// crossing copies a still register and is never busy at a send. The core
	// held or halted counts nothing: a whole window of that is `0000 0000`,
	// delivered all the same.
	wire         dstl_busy;
	wire  [31:0] dstl_q;
	agt_cdc_hs #(.W(32)) u_dstl (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(dstl_send), .s_data(dstl_word), .s_busy(dstl_busy),
		.d_clk(clk_sys), .d_rst_n(dl_rst_n),
		.d_pulse(), .d_data(dstl_q)
	);
	assign w_dstl = dstl_q;

endmodule

`default_nettype wire
