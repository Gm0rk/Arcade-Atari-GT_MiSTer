//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

module emu
(
	`include "sys/emu_ports.vh"
);

// Default values for ports not used in this core

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// SDRAM pins are driven by agt_sdram below. SDRAM_CLK comes from its own PLL
// tap at -2.5 ns (rtl/pll/pll_0002.v): an inverted clk_sys left only ~1-3 ns
// of read-capture setup on real silicon.
assign SDRAM_CLK = clk_sdram;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

// AUDIO_S/L/R are the CAGE board's DACs, assigned after u_cage (declared
// before use). No mix: the board is stereo.
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"

// Two builds from this one source. Arcade-Atari-GT.qpf is the release: no
// Debug page in the OSD and nothing drawn over the picture.
// Arcade-Atari-GT-Debug.qpf defines AGT_DEBUG (its .qsf's one extra line) and
// adds the page's settings (osd_*, after hps_io), the overlay (the build
// stamp and the counters in the top-left corner) and agt_video's SignalTap
// taps. Its rbf is Arcade-Atari-GT-Debug.rbf, loaded by the .mra files in
// mra/_Atari GT Debug/. tools/check_debug_build.py checks the pair.
`ifdef AGT_DEBUG
localparam bit DEBUG_BUILD = 1'b1;
`else
localparam bit DEBUG_BUILD = 1'b0;
`endif

localparam CONF_STR = {
	"AtariGT;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"-;",
`ifdef AGT_DEBUG
	// The Debug page, debug build only.
	"P1,Debug;",
	"P1-;",
	"P1O[9:8],SDRAM Read Delay,0,1,2,3;",
	"P1O[11:10],SDRAM Write Hold,1,2,0;",
	"P1-;",
	// Sprite Render Off (status[17]) holds the MO render pass off; the MOGO
	// counters still advance.
	"P1O[17],Sprite Render,On,Off;",
	// Sprite Fetch Yield: cycles the tile/char ports must have been quiet
	// before a sprite fetch may take the SDRAM (agt_tile_arb rp_quiet;
	// Off = 0).
	"P1O[19:18],Sprite Fetch Yield,Off,4,8,12;",
	// Load Sprite ROM Off skips the 32 MB sprite (RLE) region at download;
	// the CAGE sound data loads either way. The download runs once, when the
	// MRA is loaded, so a change takes effect only after reloading the MRA.
	"P1O[16],Load Sprite ROM,On,Off;",
	// Debug Blocks: the dbg_bits status blocks, top-left. Debug Text: the
	// dbg_words hex overlay.
	"P1O[13],Debug Blocks,Off,On;",
	"P1O[14],Debug Text,On,Off;",
	// MO Erase: Span erases the scanline span since the last partial update,
	// as MAME does. Full frame ignores the watermark and clears the whole
	// buffer once per frame at vblank: not MAME-accurate, but known-good.
	"P1O[21],MO Erase,Span,Full frame;",
	// Obj List: Snapshot (default) matches MAME, which reads the object list
	// at one instant (the MOGO edge). Live reads it across the render pass,
	// during which the game can rewrite it.
	"P1O[22],Obj List,Snapshot,Live;",
	// Line 0 Fetch: when the video pipeline reads line 0's scroll entry (the
	// sky's scroll) for the coming frame. Late = line 259, after the game's
	// vblank handler has rewritten it; Vblank start = line 240, before it,
	// which draws a horizon bar in the jungle.
	"P1O[23],Line 0 Fetch,Late,Vblank start;",
	// Video Priority: when the line renderer's fetches go first at the SDRAM,
	// ahead of an aged CAGE request and the CPU. Always (default): every
	// fetch. Late: once the line in progress has run agt_video's LATE_AT
	// cycles. Off: never (D-646's order). Always+Sprites (D-648): Always, and
	// a sprite fetch goes first too while a line fetch waits behind it.
	"P1O[25:24],Video Priority,Always,Late,Off,Always+Sprites;",
	// Sprites Wait for Text (D-649): On (default) keeps sprite fetches off the
	// SDRAM tile port until the alpha (text) pass has not asked for a
	// character for TEXT_HOLD cycles, so a line of text is fetched without a
	// sprite access in each gap. Off: D-648's order.
	"P1O[26],Sprites Wait for Text,On,Off;",
	"-;",
`endif
	"O[12],Service Mode,Off,On;",
	"-;",
	// Keep both reset entries (the R[0]-only form did not reset this core on
	// hardware), and keep them ahead of the J line, as the MiSTer templates
	// and Midway-V-Unit do: after it, R[0] reset but left the OSD open, which
	// suggests a one-entry index skew in menu.cpp. menu.cpp issues the same
	// two user_io_status_set calls for T and R, so the core cannot tell them
	// apart.
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	// Buttons in the cabinet's order, row by row (four attack buttons per
	// player in a 2x2 block): Quick High, Fierce High / Quick Low, Fierce Low.
	// Which switch bit each is depends on the revision (agt_input_map.sv, from
	// the game's CONTROLS TEST). The .mra's <buttons> replace these names on
	// hardware, and tools/gen_mra.py copies them from this line, so this is
	// the one place they are written.
	"J1,Quick High,Fierce High,Quick Low,Fierce Low,Start,Coin;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [31:0] joystick_0, joystick_1;
wire  [10:0] ps2_key;

// Declared ahead of hps_io, which uses it (ioctl_wait); driven by agt_sdram.
wire sdr_dl_busy;
// agt_cage_ramload's download backpressure, declared here for the same reason.
wire cg_ld_wait;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({status[5]}),

	// MRA ROM download stream -> agt_rom_download -> SDRAM
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),         // [15:0] in sys/hps_io.sv
	// Download backpressure. agt_sdram holds one download byte and drops any
	// strobe that arrives while it is still pending, so its dl_busy must stall
	// the HPS. The NVRAM restore shares this stream and its service write takes
	// a few cycles (nv_wait). agt_cage_ramload's FIFO holds 8 boot words and
	// raises cg_ld_wait at 6; tb_cage_ramload +mutant=1 shows the words lost
	// without it.
	.ioctl_wait(sdr_dl_busy | nv_wait | cg_ld_wait),

	// NVRAM save
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_rd(ioctl_rd),
	.ioctl_din(ioctl_din),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.ps2_key(ps2_key)
);

wire        ioctl_download, ioctl_wr;
wire        ioctl_upload, ioctl_rd;
wire        ioctl_upload_req, nv_wait, nv_dirty;
wire [7:0]  ioctl_din, ioctl_upload_index;
wire [26:0] ioctl_addr;
wire [7:0]  ioctl_dout;
// hps_io declares ioctl_index as [15:0]; keep the full width (agt_nvram reads
// all 16 bits).
wire [15:0] ioctl_index;

// The Debug page's settings. The release build has no page and fixes each at
// its menu default (the first option, status 0), so a config the debug build
// saved changes nothing in it. tools/check_debug_build.py checks each constant
// against its debug-build expression at status 0.
`ifdef AGT_DEBUG
wire [1:0] osd_rd_delay      = status[9:8];    // SDRAM Read Delay, 0-3
wire [1:0] osd_wr_hold       = status[11:10];  // SDRAM Write Hold: 0 = 1, 1 = 2, 2 = 0
wire       osd_dbg_blocks    = status[13];     // Debug Blocks: 1 = On
wire       osd_dbg_text      = ~status[14];    // Debug Text: status 0 = On
wire       osd_rle_skip      = status[16];     // Load Sprite ROM: 1 = Off
wire       osd_mo_off        = status[17];     // Sprite Render: 1 = Off
wire [1:0] osd_fetch_yield   = status[19:18];  // Sprite Fetch Yield: Off/4/8/12
wire       osd_mo_erase_full = status[21];     // MO Erase: 1 = Full frame
wire       osd_obj_live      = status[22];     // Obj List: 1 = Live
wire       osd_line0_vblank  = status[23];     // Line 0 Fetch: 1 = Vblank start
wire [1:0] osd_vid_prio      = status[25:24];  // Video Priority: Always/Late/Off/Always+Sprites
wire       osd_text_hold     = ~status[26];    // Sprites Wait for Text: status 0 = On
`else
wire [1:0] osd_rd_delay      = 2'd0;
wire [1:0] osd_wr_hold       = 2'd0;
wire       osd_dbg_blocks    = 1'b0;
wire       osd_dbg_text      = 1'b1;           // On, as the menu; drives nothing (DBG_OVERLAY 0)
wire       osd_rle_skip      = 1'b0;
wire       osd_mo_off        = 1'b0;
wire [1:0] osd_fetch_yield   = 2'd0;
wire       osd_mo_erase_full = 1'b0;
wire       osd_obj_live      = 1'b0;
wire       osd_line0_vblank  = 1'b0;
wire [1:0] osd_vid_prio      = 2'd0;
wire       osd_text_hold     = 1'b1;           // On, as the menu
`endif

// Clocks: one PLL (rtl/pll.v)

wire clk_sys;   // 57.27272 MHz: render pipeline (8x pixel clock)
wire clk_sdram; // 57.27272 MHz, -2.5 ns: SDRAM chip clock
wire clk_pix;   // 7.15909 MHz: Atari GT pixel clock (14.31818/2)
// The core resets (reset, dl_rst_n) also hold while the PLL is unlocked, so
// agt_sdram never starts its init sequence (PRECHARGE ALL, refresh, LOAD MODE
// REGISTER) on an unstable clock.
wire pll_locked;
// CAGE DSP clock: 630/17 = 37.058823 MHz (the board's oscillator is
// 33.8688 MHz). The 630 MHz VCO is fixed: clk_pix is the board's 14.318181/2
// pixel clock and clk_sys exactly 8x it. The DSP keeps its own time in model
// cycles, so this clock sets only the core's throughput. DCLK on the overlay
// measures it: 618,443 clk_dsp ticks a frame.
wire clk_dsp;
pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_pix),
	.outclk_2(clk_sdram),
	.outclk_3(clk_dsp),                // CAGE DSP, 630/17 = 37.0588 MHz
	.locked(pll_locked)
);

// Two-flop synchronizer: pll_locked is asynchronous to clk_sys and every
// consumer is an asynchronous reset, so its release must be synchronous or
// two resets could release on opposite sides of one clock edge.
reg [1:0] pll_lk_sync;
always @(posedge clk_sys) pll_lk_sync <= {pll_lk_sync[0], pll_locked};
wire pll_lk = pll_lk_sync[1];

// Stretched reset. RESET, buttons[1] or a rising edge of status[0] loads a
// 65,536-cycle hold (~1.1 ms of clk_sys), long enough for every reset
// consumer (SDRAM init, the download gate, the CPU release). The HPS sets
// and then clears status[0], and the width of that pulse is not guaranteed.
// The level terms stay, so a held reset still works.
reg         status0_d;
reg  [16:0] rst_hold;
always @(posedge clk_sys) begin
	status0_d <= status[0];
	if (RESET | buttons[1] | (status[0] & ~status0_d))
		rst_hold <= 17'h1_0000;
	else if (rst_hold != 17'd0)
		rst_hold <= rst_hold - 17'd1;
end

wire reset = ~pll_lk | RESET | status[0] | buttons[1] | (rst_hold != 17'd0);

// Video pipeline clocking: clk_sys (57.27 MHz) drives the render pipeline and
// clk_pix (7.159 MHz, the Atari GT pixel rate) the timing and scanout;
// agt_video bridges the two with a 4-phase CDC handshake.

wire sys_rst_n = ~reset;
// The download reset domain: ~RESET and PLL lock only, never status[0] or
// buttons[1] (why: at the download witnesses further down). Declared here,
// ahead of its first port connection: a port-connected name used before its
// declaration is an implicit net.
wire dl_rst_n = ~RESET & pll_lk;

wire video_rst_n_sysclk;

// Video reset into the clk_pix domain: asynchronous assert, synchronous
// release. video_rst_n_sysclk is made in clk_sys but gates clk_pix logic in
// agt_video.
reg [1:0] vrst_sync_pix;
always @(posedge clk_pix or negedge video_rst_n_sysclk) begin
	if (!video_rst_n_sysclk) vrst_sync_pix <= 2'b00;
	else vrst_sync_pix <= {vrst_sync_pix[0], 1'b1};
end
wire video_rst_n = vrst_sync_pix[1];

wire [12:0] pfram_addr;
wire        pfram_rd;
wire [15:0] pfram_data;
wire        pfram_data_valid;
wire [21:0] tile_rom_addr;
wire        tile_rom_rd;
// Tile source: always SDRAM (tile_src_demo = 0 ties off the demo BRAM).
// Only the selected source gets the rd strobe, but the responses are
// OR-combined rather than gated by the selection, so an in-flight fetch
// always completes: the fetcher has no timeout, and a dropped response would
// hang the line renderer.
wire        tile_src_demo = 1'b0;
wire [7:0]  demo_tile_data,  sdr_tile_data;
wire [31:0] sdr_tile_data32;
wire [31:0] tile_rom_data32 = sdr_tile_data32;
wire        tile_rom_rd3;
// 96-bit burst return, joined to sdr_tile_data96 further down. With
// USE_BURST3 the decoder's WIDE_FETCH path takes L/M/H from it; left
// undriven, every tile renders a constant pixel index.
wire [95:0] tile_rom_data96;
wire        demo_tile_valid, sdr_tile_valid;
wire [7:0]  tile_rom_data       = demo_tile_valid ? demo_tile_data : sdr_tile_data;
wire        tile_rom_data_valid = demo_tile_valid | sdr_tile_valid;
wire [10:0] alpharam_addr;
wire        alpharam_rd;
wire [15:0] alpharam_data;
wire        alpharam_data_valid;
wire [13:0] cram_addr;
wire [15:0] cram_data;
wire [15:0] color_latch;
wire [14:0] pen_addr_r, pen_addr_g, pen_addr_b;
wire [23:0] pen_data_r, pen_data_g, pen_data_b;

// CPU colorram write-through: one strobe per accepted cram write (the ack
// cycle), in clk_sys like the video render side.
wire cpu_cr_strobe = bc_cram_req & bc_cram_we & bc_cram_ack;

// Bring-up status bits, drawn by agt_video as colour blocks in the top-left
// corner (bit list at dbg_bits). They tell a stalled CPU from one held in
// reset, which look the same on screen.
reg dbg_dl_seen, dbg_insn_seen, dbg_latch_seen, dbg_cram_seen;
reg [24:0] dbg_hb_div;
reg dbg_heartbeat;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		dbg_dl_seen <= 1'b0; dbg_insn_seen <= 1'b0;
		dbg_latch_seen <= 1'b0; dbg_cram_seen <= 1'b0;
		dbg_hb_div <= 26'd0; dbg_heartbeat <= 1'b0;
	end else begin
		if (ioctl_download) dbg_dl_seen   <= 1'b1;
		if (bc_insn_done)   dbg_insn_seen <= 1'b1;
		if (bc_latch_wr)    dbg_latch_seen<= 1'b1;
		if (cpu_cr_strobe)  dbg_cram_seen <= 1'b1;
		// The heartbeat advances only while instructions retire, so a frozen
		// CPU shows a static block. It toggles every 20,000,000 instructions:
		// about 8.7 s at ~2.29 M instructions/s, a 17.4 s period, not 1 Hz.
		if (bc_insn_done) dbg_hb_div <= dbg_hb_div + 25'd1;
		if (dbg_hb_div == 25'd20000000) begin
			dbg_hb_div <= 26'd0;
			dbg_heartbeat <= ~dbg_heartbeat;
		end
	end
end

// SDRAM write+read loopback self-test (agt_sdram_loopback,
// tb/tb_sdram_loopback.sv). Runs once after sdr_ready, before the CPU is
// released: writes known words to scratch addresses through the cram port
// (an even and an odd SDRAM word) and the ram port (a 32-bit word), reads
// them back and compares. It involves no download, so next to the ROM-vector
// check (bit 0) it separates a corrupting download path from bad raw
// read/write timing. lb_done gates the ROM readback, and so the CPU release.
wire        lb_done, lb_even_ok, lb_odd_ok;   // driven by u_lb below
wire [15:0] lb_even_rd, lb_odd_rd;
wire [31:0] lb_r32_rd;
wire        lb_r32_ok, lb_r32_we, lb_r32_req;
wire [31:0] lb_r32_wdata;
wire [18:0] lb_addr;
wire        lb_we, lb_req;
wire [15:0] lb_wdata;

agt_sdram_loopback u_lb (
	.clk(clk_sys), .rst_n(sys_rst_n), .sdr_ready(sdr_ready),
	.lb_addr(lb_addr), .lb_we(lb_we), .lb_req(lb_req), .lb_wdata(lb_wdata),
	.cram_ack(bc_cram_ack), .cram_rdata(bc_cram_rdata),
	.lb_r32_we(lb_r32_we), .lb_r32_req(lb_r32_req),
	.lb_r32_wdata(lb_r32_wdata),
	.ram_ack(bc_ram_ack), .ram_rdata(bc_ram_rdata),
	.lb_done(lb_done), .lb_even_ok(lb_even_ok), .lb_odd_ok(lb_odd_ok),
	.lb_even_rd(lb_even_rd), .lb_odd_rd(lb_odd_rd),
	.lb_r32_rd(lb_r32_rd), .lb_r32_ok(lb_r32_ok)
);

// Work-RAM clear on reset. The Atari GT has no reset button: its only CPU
// resets are power-on, after which 0xF80000-0xFFFFFF holds garbage, and the
// watchdog, after which RAM keeps the game's state and the game reports
// WATCHDOG RESET. A core reset with RAM intact looks like the watchdog, so it
// must emulate a power cycle instead: the 512 KB work RAM is zeroed before
// the CPU is released.
// Declared here for the ram mux below; the FSM that drives them sits further
// down, after rbk_done.
reg  [18:0] ram_clr_addr;
reg  [1:0]  ram_clr_st;
reg         ram_clr_req, ram_clr_done;
reg  [31:0] ram_clr_cycles;
// The clear owns the port from loopback done until it finishes. bc_ram_req is
// idle throughout: cpu_core_rst_n, which includes ram_clr_done, holds the CPU.
wire        ram_clr_owns = lb_done && !ram_clr_done;

// The game's work RAM is on-chip: agt_wram_router (after the clear FSM below)
// serves 0xFFFF8000..0xFFFFFFFF (main-RAM port offsets 0x78000..0x7FFFF,
// every variable and the stack) from M10K with one wait cycle instead of
// 13-18 through SDRAM. Only CPU requests outside that window reach the mux
// below (wram_sdr_req). The loopback and the clear still own the SDRAM port
// and its ack (bc_ram_ack); the CPU and the meter see the routed ack
// (cpu_ram_ack). WRAM_ONCHIP = 0 makes the router a pass-through.
localparam bit WRAM_ONCHIP = 1'b1;
wire        wram_sdr_req;           // the CPU's request, when outside the window
wire        cpu_ram_ack;            // to agt_board_core and the meter
wire [31:0] cpu_ram_rdata;
wire        wram_clr_done;          // joins cpu_core_rst_n, like ram_clr_done

// ram port mux: loopback, then the clear, then the board core.
wire [18:0] ram_mux_addr  = !lb_done ? 19'h7F100 : ram_clr_owns ? ram_clr_addr : bc_ram_addr;
wire [3:0]  ram_mux_be    = !lb_done ? 4'b1111   : ram_clr_owns ? 4'b1111      : bc_ram_be;
wire        ram_mux_we    = !lb_done ? lb_r32_we : ram_clr_owns ? 1'b1         : bc_ram_we;
wire [31:0] ram_mux_wdata = !lb_done ? lb_r32_wdata : ram_clr_owns ? 32'd0     : bc_ram_wdata;
wire        ram_mux_req   = !lb_done ? lb_r32_req   : ram_clr_owns ? ram_clr_req : wram_sdr_req;

wire [18:0] cram_mux_addr  = lb_done ? bc_cram_addr  : lb_addr;
wire        cram_mux_we    = lb_done ? bc_cram_we    : lb_we;
wire [15:0] cram_mux_wdata = lb_done ? bc_cram_wdata : lb_wdata;
wire        cram_mux_req   = lb_done ? bc_cram_req   : lb_req;

// Post-download ROM readback. The loopbacks never touch the ROM region: in
// agt_sdram the ram port lands at word base 0x100000 and cram at 0x140000,
// while ROM (words 0x000000-0x0FFFFF) is written only by the download port
// and read only by the rom port. After the download, and before the CPU is
// released, this reads ROM byte 4 (the reset PC, 0x00000664) back through the
// rom port: the same word the CPU's second fetch reads.
//   bit15 rbk_ok       readback == 0x00000664
//   bit14 rbk_nonzero  readback has any bit set
//   15 R, 14 R: all zeros, never written; 15 R, 14 G: written but wrong
reg  [3:0]  rbk_st;   // states 0..10: readback, ROM-region loopback, scan
reg         rbk_done, rbk_ok, rbk_nonzero;
// The raw readback word; its high half is on the block overlay (dbg_bits).
reg  [31:0] rbk_data;
// Scan: after byte 4, sweep ROM words 0..0x1FC for the halfword 0x0664 and
// record its halfword index. In the image it occurs once in the first 512
// halfwords, at index 3 (byte 6), so the index found names any shift:
//   0003 in place, 0002 one halfword early (SDRAM word N holds source word
//   N+1), 0004 one halfword late, FFFF not found
reg  [15:0] rbk_found_hw;
reg  [2:0]  rlb_cnt;
reg  [20:0] rbk_addr;
reg         rbk_req;

// ROM-region loopback: writes A5 5A 12 34 through the download port and reads
// it back through the rom port, the one write/read pair the other loopbacks
// never cover. Non-destructive: the word at RLB_ADDR (near the top of the
// region) is read first and restored afterwards.
//   rlb_rd == A55A1234: both paths work
//   rlb_rd == 00000000: the pair is broken
localparam [20:0] RLB_ADDR = 21'h1FFFF0;
reg  [31:0] rlb_rd, rlb_orig;
reg         rlb_wr;
reg  [20:0] rlb_addr;
reg  [7:0]  rlb_data;
wire        rlb_owns = dl_rom_loaded && !rbk_done;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rbk_st <= 4'd0; rbk_done <= 1'b0;
		rbk_ok <= 1'b0; rbk_nonzero <= 1'b0; rbk_data <= 32'd0;
		rbk_found_hw <= 16'hFFFF;
		rbk_addr <= 21'd0; rbk_req <= 1'b0;
		rlb_rd <= 32'd0; rlb_orig <= 32'd0;
		rlb_wr <= 1'b0; rlb_addr <= 21'd0; rlb_data <= 8'd0;
	end else if (!rbk_done && lb_done && sdr_ready && dl_rom_loaded) begin
		case (rbk_st)
			4'd0: begin rbk_addr <= 21'h000004; rbk_req <= 1'b1; rbk_st <= 4'd1; end
			4'd1: if (bc_rom_ack) begin
					  rbk_req     <= 1'b0;
					  rbk_data    <= bc_rom_rdata;
					  rbk_ok      <= (bc_rom_rdata == 32'h0000_0664);
					  rbk_nonzero <= |bc_rom_rdata;
					  rbk_st      <= 3'd2;
				  end
			// 2: wait for the ack to drop so one ack cannot be seen twice,
			// then run the ROM-region loopback (6..10) and the scan (3..5)
			4'd2: if (!bc_rom_ack) begin
					  rbk_addr <= RLB_ADDR; rbk_req <= 1'b1; rbk_st <= 4'd6;
				  end
			// 6: save the original word at RLB_ADDR
			4'd6: if (bc_rom_ack) begin
					  rbk_req <= 1'b0; rlb_orig <= bc_rom_rdata;
					  rlb_cnt <= 3'd0; rbk_st <= 4'd7;
				  end
			// 7: write A5 5A 12 34 through the DL port, one byte per
			//    one-cycle strobe, honouring dl_busy. agt_sdram latches on
			//    (dl_wr && !dl_pend), so a longer strobe writes a byte twice.
			4'd7: if (!rlb_wr && !sdr_dl_busy) begin
					  rlb_wr   <= 1'b1;
					  rlb_addr <= RLB_ADDR + {18'd0, rlb_cnt};
					  rlb_data <= (rlb_cnt == 3'd0) ? 8'hA5 :
								  (rlb_cnt == 3'd1) ? 8'h5A :
								  (rlb_cnt == 3'd2) ? 8'h12 : 8'h34;
				  end else if (rlb_wr) begin
					  rlb_wr  <= 1'b0;
					  rlb_cnt <= rlb_cnt + 3'd1;
					  if (rlb_cnt == 3'd3) rbk_st <= 4'd8;
				  end
			// 8: read it back through the ROM port
			4'd8: begin
					  rlb_wr <= 1'b0;
					  if (!sdr_dl_busy) begin
						  rbk_addr <= RLB_ADDR; rbk_req <= 1'b1; rbk_st <= 4'd9;
					  end
				  end
			4'd9: if (bc_rom_ack) begin
					  rbk_req <= 1'b0; rlb_rd <= bc_rom_rdata;
					  rlb_cnt <= 3'd0; rbk_st <= 4'd10;
				  end
			// 10: put the original bytes back, so a good image is unharmed
			4'd10: if (!rlb_wr && !sdr_dl_busy) begin
					  rlb_wr   <= 1'b1;
					  rlb_addr <= RLB_ADDR + {18'd0, rlb_cnt};
					  rlb_data <= (rlb_cnt == 3'd0) ? rlb_orig[31:24] :
								  (rlb_cnt == 3'd1) ? rlb_orig[23:16] :
								  (rlb_cnt == 3'd2) ? rlb_orig[15:8]  : rlb_orig[7:0];
				  end else if (rlb_wr) begin
					  // rlb_wr must be clear before leaving: DL outranks
					  // rom_req in the arbiter, so a stuck strobe starves
					  // the scan's read, rbk_done never sets, and
					  // cpu_core_rst_n holds the CPU in reset.
					  rlb_wr  <= 1'b0;
					  rlb_cnt <= rlb_cnt + 3'd1;
					  if (rlb_cnt == 3'd3) begin
						  rbk_addr <= 21'd0; rbk_req <= 1'b1; rbk_st <= 4'd3;
					  end
				  end
			4'd3: if (bc_rom_ack) begin
					  rbk_req <= 1'b0;
					  if (bc_rom_rdata[31:16] == 16'h0664) begin
						  rbk_found_hw <= {6'd0, rbk_addr[10:2], 1'b0};
						  rbk_st <= 4'd5;
					  end else if (bc_rom_rdata[15:0] == 16'h0664) begin
						  rbk_found_hw <= {6'd0, rbk_addr[10:2], 1'b1};
						  rbk_st <= 4'd5;
					  end else
						  rbk_st <= 4'd4;
				  end
			4'd4: if (!bc_rom_ack) begin
					  if (rbk_addr >= 21'h0003FC) rbk_st <= 4'd5;   // 512 halfwords
					  else begin rbk_addr <= rbk_addr + 21'd4; rbk_req <= 1'b1;
								 rbk_st <= 4'd3; end
				  end
			4'd5: if (!bc_rom_ack) rbk_done <= 1'b1;
			default: ;
		endcase
	end
end

// rom port mux: the readback owns it until rbk_done, then the board core.
// Safe because cpu_core_rst_n below also waits for rbk_done, so bc_rom_req
// cannot be asserted while the readback holds the port.
wire [20:0] rom_mux_addr = rbk_done ? bc_rom_addr : rbk_addr;
wire        rom_mux_req  = rbk_done ? bc_rom_req  : rbk_req;

// SDRAM read-path self-check. The CPU's first two ROM fetches after reset are
// the reset vectors at 0 and 4, known constants of the image (SP 0x00000000,
// PC 0x00000664). Comparing the first two rom_ack payloads with them tests
// the real read path on the board (traces, capture window, CL2 alignment).
reg [1:0] rdchk_n;
reg       rdchk_ok;
// Raw data of the first two ROM fetches, beside the verdict.
reg [31:0] rdchk_d0, rdchk_d1;
// Low address byte of each counted access (expect 0x00, 0x04). a1 = 0x00
// means one access was counted twice; a1 = 0x04 with a wrong d1 means the
// data is wrong.
reg [7:0] rdchk_a0, rdchk_a1;
// Count rising edges of (req && ack): on hardware bc_rom_ack can stay high
// for more than one cycle per transaction, and a level-sensitive term would
// count one access twice. The sim SDRAM model drops ack after one cycle, so
// the benches cannot show this.
reg ack_seen_d1;
wire rom_ack_rise = bc_rom_req && bc_rom_ack && !ack_seen_d1;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rdchk_n <= 2'd0; rdchk_ok <= 1'b0;
		rdchk_d0 <= 32'd0; rdchk_d1 <= 32'd0;
		rdchk_a0 <= 8'd0; rdchk_a1 <= 8'd0;
		ack_seen_d1 <= 1'b0;
	end else begin
		ack_seen_d1 <= bc_rom_req && bc_rom_ack;
		if (rom_ack_rise && rdchk_n != 2'd2) begin
			rdchk_n <= rdchk_n + 2'd1;
			if (rdchk_n == 2'd0) begin
				rdchk_ok <= (bc_rom_rdata == 32'h0000_0000);
				rdchk_d0 <= bc_rom_rdata;
				rdchk_a0 <= bc_rom_addr[7:0];
			end
			else
				begin
				rdchk_ok <= rdchk_ok && (bc_rom_rdata == 32'h0000_0664);
				rdchk_d1 <= bc_rom_rdata;
				rdchk_a1 <= bc_rom_addr[7:0];
				end
		end
	end
end

// SDRAM byte map (26-bit byte addresses, 64 MB):
//   0x0000000 maincpu    2 MB     0x0200000 RAM      512 KB
//   0x0280000 cram     512 KB     0x0300000 tiles      4 MB
//   0x0700000 chars    128 KB     0x0720000 RLE       32 MB
//   0x2720000 CAGE sound 4 MB     0x2B20000 CAGE RAM 256 KB
// The download wires below are declared ahead of the download witnesses that
// use them: a net referenced before its declaration becomes an implicit 1-bit
// net, and Quartus then rejects the explicit declaration.
localparam [25:0] SDR_BASE_TILES = 26'h0300000;
localparam [25:0] SDR_BASE_CHARS = 26'h0700000;   // after the 4 MB tile region
// RLE sprite data: the full 32 MB region mapped 1:1, all-zero stretches
// included, so the renderer needs no address fold. 0x0720000..0x271FFFF.
localparam [25:0] SDR_BASE_RLE   = 26'h0720000;
// CAGE sound data, 4 MB: 0x2720000..0x2B1FFFF. Byte 0 is DSP word 0xD00000,
// so the DSP address of byte b is 0xD00000 + b/4.
localparam [25:0] SDR_BASE_CAGE  = 26'h2720000;
// CAGE RAM: the C31's 64K-word program/data RAM, 256 KB, 0x2B20000..0x2B5FFFF
// (the top of the map, 43.38 MB of 64). Written only through agt_sdram's
// cage_* port, by agt_cage_ramload at download time and by the DSP.
localparam [25:0] SDR_BASE_CAGERAM = 26'h2B20000;

wire        dl_tiles_wr, dl_chars_wr, dl_rle_wr;
wire [21:0] dl_tiles_addr;
wire [16:0] dl_chars_addr;
wire [24:0] dl_rle_addr;
wire        dl_cage_wr;                   // CAGE sound data
wire [22:0] dl_cage_addr;

// One writer at a time: the regions are disjoint in the ioctl stream, so only
// one of these strobes can be high in any cycle.
// rle_load_en follows the Debug page's Load Sprite ROM (On, the first option,
// loads the sprite region; the release build always loads it). The download
// runs once, when the MRA is loaded, so a change needs the MRA reloaded.
wire        rle_load_en = ~osd_rle_skip;
wire        dl_any_wr   = dl_maincpu_wr | dl_tiles_wr | dl_chars_wr
						| (dl_rle_wr && rle_load_en)
						| dl_cage_wr;
// The sound data is not behind rle_load_en: that switch only drops the 32 MB
// sprite region, and the CAGE checksum must read the same in both positions.
// The tiles region is stored plane-adjacent, so the renderer's three per-row
// plane reads are three columns of one open SDRAM row instead of three row
// misses 1 MB apart. This remap must stay bit-identical to agt_tile_phys() in
// agt_tile_sdram.sv (REMAP=1 on the tiles instance): the two are the write
// and read halves of one permutation, and a mismatch scrambles every tile.
// Chars and RLE are not remapped.
function automatic logic [21:0] agt_tile_phys_dl(input logic [21:0] b);
	logic [17:0] wordidx;   // b[19:2], plane-local; plane b[21:20] added below
	logic [21:0] w12;
	begin
		wordidx = b[19:2];
		w12 = {4'd0, wordidx} << 4;
		agt_tile_phys_dl = w12 + {18'd0, b[21:20], 2'd0} + {20'd0, b[1:0]};
	end
endfunction

wire [25:0] dl_any_addr = dl_tiles_wr ? (SDR_BASE_TILES + {4'd0, agt_tile_phys_dl(dl_tiles_addr)})
						: dl_chars_wr ? (SDR_BASE_CHARS + {9'd0, dl_chars_addr})
						: dl_rle_wr   ? (SDR_BASE_RLE + dl_rle_addr)
						: dl_cage_wr  ? (SDR_BASE_CAGE + {3'd0, dl_cage_addr})
									  : {5'd0, dl_maincpu_addr};

wire        dl_maincpu_wr;
wire [20:0] dl_maincpu_addr;
wire [7:0]  dl_rom_data;

// CAGE sound-data witness: a 32-bit sum of every byte of the region as it
// downloads, to compare with the same sum over the set's stream as gen_mra.py
// assembles it. In dl_rst_n, not cpu_core_rst_n: cpu_core_rst_n is low for
// the whole ROM download, the one window this must count in.
logic [31:0] cage_sum_q;
always_ff @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) cage_sum_q <= 32'd0;
	else if (dl_cage_wr) cage_sum_q <= cage_sum_q + {24'd0, dl_rom_data};
end

// Controller inputs. The map is in rtl/board/agt_input_map.sv, instanced
// below next to agt_rom_download. Bits follow mame_src/atarigt.cpp's P1_P2
// and COIN ports, all IP_ACTIVE_LOW: idle is high and a press pulls the bit
// low.
// The map depends on the Primal Rage revision: the parent set's four attack
// buttons are on bits 25, 26, 27 and 1 (P2: 9, 10, 11 and 3), with Start on
// 24 / 8; the two older sets' are on 24..27 (P2: 8..11), and Quick High is
// also Start. The .mra says which (rom index 1, byte 1 -> dl_panel_id). The
// module header has the table, taken from the game's own CONTROLS TEST;
// tb/tb_input_map.sv checks the RTL against vectors that
// tools/check_inputs.py derives from the driver.
wire [31:0] p1_p2_port;
wire [15:0] coin_in;

// MO control bits, from the 0xE08000 latch. atarigt.cpp does
// m_rle->control_write((data >> 27) & 7) and atarirle.h has MOGO = 1,
// ERASE = 2, FRAME = 4, so MOGO is latch bit 27, not bit 0. A render starts
// on MOGO's rising edge (atarirle.cpp:117).
// MOGO witness = {MOGO edges in vblank, all MOGO edges}; equal counts mean
// every render is triggered in vblank. The MO VRAM is single-buffered, so a
// pass triggered mid-frame writes the buffer the mixer is reading. Only
// rising edges count: other latch writes (ERASE, CAGE control) are not
// renders.
wire       mogo_bit = bc_latch[27];
// CONTROL_FRAME, bit 2 of the field: latch bit 29 (on the board, 68.DISA;
// see mo_ctrl below).
wire       frame_bit = bc_latch[29];
// CONTROL_ERASE: latch bit 28. atarirle.cpp erases when the OLD control bits
// had ERASE set, independently of MOGO, and clears only the scanline span
// since the last partial update, on the frame the old bits selected:
//
//   if ((oldbits & CONTROL_ERASE) != 0)
//       m_vram[0][(oldbits & CONTROL_FRAME) >> 2].fill(0, cliprect);
wire       erase_bit = bc_latch[28];
// MAME's control_write opens with an early return:
//     int const oldbits = m_control_bits;
//     if (oldbits == data) return;
// so a latch write that does not change the MO control bits does nothing: no
// erase, no render and no m_partial_scanline update. The latch also carries
// LEDs and coin counters, so mo_ctrl_wr fires only on a change; otherwise the
// erase watermark outruns the erases and sprites smear.
// mo_ctrl is the three bits MAME's control byte holds: MOGO(1), ERASE(2),
// FRAME(4).
// On the board (Primal Rage Operator's Manual, Figure 5-1 sheet 9) the MO
// control latch is the LS273 at 17A, clocked by 68.LATCH and cleared by
// 68.RES:
//     XD13 -> 68.DISA        XD12 -> ERASE        XD11 -> /MOGO
//     XD8  -> VCR1           XD3  -> CC.L         XD0  -> CC.R
// MAME's latch_w comment lists the same map, so bits 29:27 are XD13:XD11.
// MAME's CONTROL_FRAME is 68.DISA on the board: there is no frame-select bit
// on this latch. The change gate still includes it; disa_toggles and
// moerase_toggles count the two groups separately.
wire [2:0] mo_ctrl = bc_latch[29:27];
wire       disa_bit = bc_latch[29];        // 68.DISA per the schematic
reg        disa_bit_d;
reg [15:0] disa_toggles, moerase_toggles;
reg  [2:0] mo_ctrl_d;
wire       mo_ctrl_changed = bc_latch_wr && (mo_ctrl != mo_ctrl_d);
reg        erase_bit_d;
reg        erase_pulse;
reg        mogo_bit_d;
reg [15:0] mogo_total, mogo_in_vblank;
reg [15:0] erase_count;    // changes made with the old ERASE bit set
reg [15:0] ctrl_chg_count; // control writes that changed the bits
// NVRAM restore witness: restore bytes that arrived (nvrw_bytes) and CPU
// EEPROM reads taken before the restore finished (nvrw_early_rd). In
// dl_rst_n, which only RESET clears: the restore arrives once, at core load,
// and a witness must outlive the warm resets that follow.
reg [15:0] nvrw_bytes, nvrw_early_rd;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		nvrw_bytes <= 16'd0; nvrw_early_rd <= 16'd0;
	end else begin
		if (nvw_ack && nvrw_bytes != 16'hFFFF)
			nvrw_bytes <= nvrw_bytes + 16'd1;
		// a CPU EEPROM read taken while the restore has not finished
		if (eeprom_rd_evt && nvrw_bytes < 16'd2048 && nvrw_early_rd != 16'hFFFF)
			nvrw_early_rd <= nvrw_early_rd + 16'd1;
	end
end
// The second erase site: MAME's vblank_callback, on the rising edge of
// vblank. It erases to the bottom of the frame only if CONTROL_ERASE is set,
// but resets m_partial_scanline on every vblank rising edge. So two pulses,
// same cycle: vblank_rise_pulse carries the unconditional reset to
// agt_mo_vram's erase_to_bottom, vblank_erase_pulse the conditional sweep to
// start_erase.
reg  vblank_d, vblank_rise_pulse, vblank_erase_pulse;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		vblank_d <= 1'b0; vblank_rise_pulse <= 1'b0; vblank_erase_pulse <= 1'b0;
	end else begin
		vblank_d <= vid_in_vblank;
		vblank_rise_pulse  <= vid_in_vblank && !vblank_d;
		vblank_erase_pulse <= vid_in_vblank && !vblank_d && erase_bit;
	end
end
// The erase request must be in the same cycle as mo_ctrl_wr. agt_mo_vram
// takes the span from the watermark as it stands when start_erase is sampled
// and then advances the watermark to the current line (MAME's order); a
// request one cycle later would see top = vcount + 1 > bottom, an empty span.
// erase_bit_d is the old bit (updated only on a change), i.e. MAME's
// oldbits & CONTROL_ERASE. erase_pulse is a registered copy for the counter.
wire erase_req_now = mo_ctrl_changed && erase_bit_d;
// Which buffer the sweep clears. control_write erases
// m_vram[..][(oldbits & CONTROL_FRAME) >> 2]; latch_value already holds the
// new word while latch_wr is high (agt_main_memmap registers both on the same
// edge), so the old bit is mo_ctrl_d[2], updated at the end of this cycle.
// vblank_callback erases the current one. Both requests are single cycles; if
// they coincide the vblank site wins, since its span is the larger and the
// control write's span is empty after it anyway.
wire mo_erase_frame_sel = vblank_rise_pulse ? frame_bit : mo_ctrl_d[2];
reg        mogo_pulse;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		mogo_bit_d <= 1'b0; mogo_total <= 16'd0; mogo_in_vblank <= 16'd0;
		mo_ctrl_d <= 3'd0; disa_bit_d <= 1'b0;
		disa_toggles <= 16'd0; moerase_toggles <= 16'd0;
		erase_bit_d <= 1'b0; erase_pulse <= 1'b0; erase_count <= 16'd0;
		ctrl_chg_count <= 16'd0;
		mogo_pulse <= 1'b0;
	end else begin
		erase_pulse <= 1'b0;
		// Only a change of the control bits is a control_write (MAME's early
		// return); bc_latch_wr alone is not.
		if (mo_ctrl_changed) begin
			if (ctrl_chg_count != 16'hFFFF) ctrl_chg_count <= ctrl_chg_count + 16'd1;
			mogo_bit_d  <= mogo_bit;
			// MAME erases on the old bits, the value in force before this
			// write, independent of MOGO.
			erase_bit_d <= erase_bit;
			if (erase_bit_d) begin
				erase_pulse <= 1'b1;
				if (erase_count != 16'hFFFF) erase_count <= erase_count + 16'd1;
			end
		end
		if (bc_latch_wr) begin
			mo_ctrl_d  <= mo_ctrl;
			disa_bit_d <= disa_bit;
			// 68.DISA is not an MO signal on the board; MOGO and ERASE
			// are. Count the two groups apart.
			if (disa_bit != disa_bit_d && disa_toggles != 16'hFFFF)
				disa_toggles <= disa_toggles + 16'd1;
			if (mo_ctrl[1:0] != mo_ctrl_d[1:0] && moerase_toggles != 16'hFFFF)
				moerase_toggles <= moerase_toggles + 16'd1;
		end
		if (bc_latch_wr) mogo_bit_d <= mogo_bit;
		mogo_pulse <= 1'b0;
		// Sprite Render Off (osd_mo_off) suppresses only the render pass; the
		// counters still advance, so MOGO still shows the game asking.
		if (bc_latch_wr && mogo_bit && !mogo_bit_d) begin
			mogo_pulse <= ~osd_mo_off;
			mogo_total <= mogo_total + 16'd1;
			if (v_vblank) mogo_in_vblank <= mogo_in_vblank + 16'd1;
		end
	end
end

// Sprite pixel witness: SPXW = {object-list words read, MO pixels written},
// both saturating. MOGO counts renders requested; this counts what the
// renderer reads and produces.
reg [15:0] spx_pixels, spx_objreads;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		spx_pixels <= 16'd0; spx_objreads <= 16'd0;
	end else begin
		// Saturating: a pinned FFFF is visibly pinned, while a wrapped value
		// looks plausible.
		if (vid_rnd_wr_valid && spx_pixels   != 16'hFFFF)
			spx_pixels   <= spx_pixels   + 16'd1;
		if (obj_rd           && spx_objreads != 16'hFFFF)
			spx_objreads <= spx_objreads + 16'd1;
	end
end

// Scroll jitter witness. The alpharam is shared between the scroll fetcher and
// the alpha pass, switched on al_busy, and alpharam_data returns a cycle after
// its address: if al_busy rises while a scroll read is in flight, the scroll
// word returned belongs to the alpha pass. Simulation runs the passes in the
// same order every line, so only hardware can show it. The scroll is sampled
// at one fixed scanline every frame.
wire [9:0] vid_xscroll;
wire [8:0] vid_yscroll;
wire       vid_ss_done;
wire [8:0] vid_ss_line;
wire [8:0] vid_vcount;

localparam [8:0] SCRL_PROBE_LINE = 9'd100;   // mid-screen, always rendered

// Count only jumps: a real scroll pans a few pixels per frame, while a
// corrupted read lands arbitrarily far away. 32 pixels is above any
// plausible per-frame pan (the screen is 240 lines) and far below the ~170
// average distance of a random 9-bit value.
localparam int SCRL_JUMP = 32;

reg  [8:0]  scrl_y, scrl_y_prev;
reg  [15:0] scrl_jump_cnt, scrl_max_delta;
reg         scrl_seen;
wire [8:0]  scrl_y_now = vid_yscroll;
wire [9:0]  scrl_delta = (scrl_y_now >= scrl_y_prev)
					   ? ({1'b0, scrl_y_now} - {1'b0, scrl_y_prev})
					   : ({1'b0, scrl_y_prev} - {1'b0, scrl_y_now});

always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		scrl_y <= 9'd0; scrl_y_prev <= 9'd0;
		scrl_jump_cnt <= 16'd0; scrl_max_delta <= 16'd0;
		scrl_seen <= 1'b0;
	end else if (vid_ss_done && vid_ss_line == SCRL_PROBE_LINE) begin
		scrl_y      <= scrl_y_now;
		scrl_y_prev <= scrl_y;
		scrl_seen   <= 1'b1;
		if (scrl_seen) begin
			if (scrl_delta > SCRL_JUMP[9:0])
				scrl_jump_cnt <= scrl_jump_cnt + 16'd1;
			// the largest delta seen: a smooth pan stays small, a corrupted
			// read shows a number no animation would produce
			if ({6'd0, scrl_delta} > scrl_max_delta)
				scrl_max_delta <= {6'd0, scrl_delta};
		end
	end
end
wire [15:0] scrl_val = {7'd0, scrl_y};

// Render duration witness. A pass that runs into active display competes
// with the tile and char fetches for the SDRAM tile port, where sprites have
// the lowest priority. Each pass is timed from mogo_pulse until
// vid_render_busy has risen and fallen again, so a pass deferred behind an
// erase sweep counts in full.
//   RDUR = {last completed pass >> 8, longest pass since reset >> 8}
//   rdur_late = passes that finished outside vblank
//   MOST = {vcount at MOGO, vcount at pass end}
// The MO VRAM is single-buffered and scanned out while it is drawn: rows
// drawn after the raster has passed them cannot show in that frame.
reg [15:0] mo_start_line, mo_end_line;
reg [15:0] rdur_late, rdur_max, rdur_last;
reg [23:0] rdur_cnt;
reg        rdur_busy, rdur_seen;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rdur_late <= 16'd0; rdur_max <= 16'd0; rdur_last <= 16'd0;
		mo_start_line <= 16'd0; mo_end_line <= 16'd0;
		rdur_cnt  <= 24'd0; rdur_busy <= 1'b0; rdur_seen <= 1'b0;
	end else begin
		if (mogo_pulse) begin rdur_busy <= 1'b1; rdur_seen <= 1'b0; rdur_cnt <= 24'd0;
							  mo_start_line <= {7'd0, vid_vcount}; end
		else if (rdur_busy) begin
			if (rdur_cnt != 24'hFFFFFF) rdur_cnt <= rdur_cnt + 24'd1;
			if (vid_render_busy) rdur_seen <= 1'b1;
			if (rdur_seen && !vid_render_busy) begin
				rdur_busy <= 1'b0;
				rdur_last <= rdur_cnt[23:8];
				mo_end_line <= {7'd0, vid_vcount};
				if (rdur_cnt[23:8] > rdur_max) rdur_max <= rdur_cnt[23:8];
				// finished while the raster was in active display
				if (!vid_in_vblank && rdur_late != 16'hFFFF)
					rdur_late <= rdur_late + 16'd1;
			end
		end
	end
end

// Blit phase split of the last completed pass (the same shape as RDUR):
// cycles agt_rle_blit spent in its MEASURE traversal (meas), emitting pixels
// (emit) and waiting on ROM data (wait), each >> 8.
//   MWAI = {wait >> 8, emit >> 8}
// meas_last should stay 0 while the prescan cache hits (MAME prescans once,
// in device_start); it is still counted, just not shown.
reg [15:0] meas_last, emit_last, wait_last;
reg [23:0] meas_cnt, emit_cnt, wait_cnt;
reg        meas_busy, meas_seen;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		meas_last <= 16'd0; emit_last <= 16'd0; wait_last <= 16'd0;
		meas_cnt  <= 24'd0; emit_cnt  <= 24'd0; wait_cnt  <= 24'd0;
		meas_busy <= 1'b0;  meas_seen <= 1'b0;
	end else begin
		if (mogo_pulse) begin
			meas_busy <= 1'b1; meas_seen <= 1'b0;
			meas_cnt  <= 24'd0; emit_cnt <= 24'd0; wait_cnt <= 24'd0;
		end else if (meas_busy) begin
			if (dbg_blit_meas && meas_cnt != 24'hFFFFFF) meas_cnt <= meas_cnt + 24'd1;
			if (dbg_blit_emit && emit_cnt != 24'hFFFFFF) emit_cnt <= emit_cnt + 24'd1;
			if (dbg_blit_wait && wait_cnt != 24'hFFFFFF) wait_cnt <= wait_cnt + 24'd1;
			if (vid_render_busy) meas_seen <= 1'b1;
			if (meas_seen && !vid_render_busy) begin
				meas_busy <= 1'b0;
				meas_last <= meas_cnt[23:8];
				emit_last <= emit_cnt[23:8];
				wait_last <= wait_cnt[23:8];
			end
		end
	end
end

// CPU throughput per frame:
//   CPUI = {instructions retired last frame >> 4, ROM-port stall cycles >> 8}
// The 68020 core runs at 57.27 MHz against the real part's 25 MHz to offset
// its higher cycles per instruction. A real 25 MHz 68EC020 at ~6 cycles per
// instruction retires roughly 70,000 instructions per 59.92 Hz frame; >> 4
// keeps that in 16 bits (4,375) with 4 more bits of resolution than >> 8.
reg [15:0] insn_last, cstall_last;
// clk_dsp census: a free-running Gray-coded counter in clk_dsp, synchronized
// into clk_sys and differenced at the vblank edge. Gray, because a binary
// count sampled asynchronously can tear across a carry and read a value it
// never held, which would look like a wrong PLL divisor.
localparam int DCW = 20;                    // ~618k ticks/frame needs 20 bits

reg  [1:0]     dsp_rst_sync;
always @(posedge clk_dsp or negedge sys_rst_n)
	if (!sys_rst_n) dsp_rst_sync <= 2'b00;
	else            dsp_rst_sync <= {dsp_rst_sync[0], 1'b1};
wire dsp_rst_n = dsp_rst_sync[1];

reg [DCW-1:0] dsp_bin, dsp_gray;
always @(posedge clk_dsp or negedge dsp_rst_n)
	if (!dsp_rst_n) begin dsp_bin <= 0; dsp_gray <= 0; end
	else begin
		dsp_bin  <= dsp_bin + 1'b1;
		dsp_gray <= (dsp_bin + 1'b1) ^ ((dsp_bin + 1'b1) >> 1);
	end

reg [DCW-1:0] dg_s1, dg_s2;
always @(posedge clk_sys) begin dg_s1 <= dsp_gray; dg_s2 <= dg_s1; end

integer dgi;
reg [DCW-1:0] dsp_sys_bin;
always @* begin
	dsp_sys_bin[DCW-1] = dg_s2[DCW-1];
	for (dgi = DCW-2; dgi >= 0; dgi = dgi - 1)
		dsp_sys_bin[dgi] = dsp_sys_bin[dgi+1] ^ dg_s2[dgi];
end

reg [DCW-1:0] dsp_prev;
reg [19:0]    sys_frame;
reg [15:0]    dclk_dsp_last, dclk_sys_last;
reg           dclk_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		dsp_prev <= 0; sys_frame <= 0; dclk_vbl_d <= 1'b0;
		dclk_dsp_last <= 16'd0; dclk_sys_last <= 16'd0;
	end else begin
		dclk_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !dclk_vbl_d) begin
			// clk_dsp ticks since the last vblank, with clk_sys ticks as the
			// control: 955,776 by construction (3648 x 262), so a wrong control
			// means the census is broken, not the PLL.
			dclk_dsp_last <= (dsp_sys_bin - dsp_prev) >> 4;
			dclk_sys_last <= sys_frame[19:4];
			dsp_prev  <= dsp_sys_bin;
			sys_frame <= 20'd0;
		end else if (sys_frame != 20'hFFFFF) begin
			sys_frame <= sys_frame + 20'd1;
		end
	end
end

// Instruction-fetch census per frame, the same shape as CPUI: fetches served
// without a bus cycle (bc_ifetch_hit) and fetches that took one
// (bc_ifetch && bc_rom_req && bc_rom_ack), each >> 4 (roughly 40,000 fetch
// calls a frame). measure_icache.py predicts 52.1% served without a bus
// cycle for game code with the 16-entry cache.
wire bc_ifetch, bc_ifetch_hit;
reg [19:0] ifhit_frame, ifmiss_frame;
reg [15:0] ifhit_last, ifmiss_last;
reg        ifet_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		ifhit_last <= 16'd0; ifmiss_last <= 16'd0;
		ifhit_frame <= 20'd0; ifmiss_frame <= 20'd0; ifet_vbl_d <= 1'b0;
	end else begin
		ifet_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !ifet_vbl_d) begin
			ifhit_last   <= ifhit_frame[19:4];
			ifmiss_last  <= ifmiss_frame[19:4];
			ifhit_frame  <= 20'd0;
			ifmiss_frame <= 20'd0;
		end else begin
			if (bc_ifetch_hit && ifhit_frame != 20'hFFFFF)
				ifhit_frame <= ifhit_frame + 20'd1;
			if (bc_ifetch && bc_rom_req && bc_rom_ack &&
				ifmiss_frame != 20'hFFFFF)
				ifmiss_frame <= ifmiss_frame + 20'd1;
		end
	end
end

reg [19:0] insn_frame;
reg [23:0] cstall_frame;
reg        cpui_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		insn_last <= 16'd0; cstall_last <= 16'd0;
		insn_frame <= 20'd0; cstall_frame <= 24'd0; cpui_vbl_d <= 1'b0;
	end else begin
		cpui_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !cpui_vbl_d) begin
			insn_last    <= {insn_frame[19:4]};
			cstall_last  <= cstall_frame[23:8];
			insn_frame   <= 20'd0;
			cstall_frame <= 24'd0;
		end else begin
			if (bc_insn_done && insn_frame != 20'hFFFFF)
				insn_frame <= insn_frame + 20'd1;
			// CPU asked the bus for something and did not get it this cycle
			if ((bc_rom_req && !bc_rom_ack) && cstall_frame != 24'hFFFFFF)
				cstall_frame <= cstall_frame + 24'd1;
		end
	end
end

// Game ticks and main-RAM wait (agt_cpu_meter, bench tb/tb_cpu_meter.sv).
// The ROM marks each game tick with two writes to its busy flag: 0x80 to
// $FFFF8708 as vblank_irq_handler starts game_main_loop_tick, 0x00 when it
// returns (primrage_ghidra_notes.md, Finding 8). Both arrive on the main-RAM
// port, which CPUI's stall count does not cover.
//   GTCK = {ticks completed in the last 60 vblanks,
//           vcount at which the last tick started}
//   RAMS = {main-RAM port wait cycles last frame >> 8,
//           main-RAM accesses last frame >> 4}
wire [15:0] gtck_ticks, gtck_line, rams_stall, rams_acc;
agt_cpu_meter #(.FLAG_ADDR(19'h78708), .WINDOW_VBL(60)) u_cpu_meter (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.vblank(vid_in_vblank), .vcount(vid_vcount),
	.ram_req(bc_ram_req), .ram_ack(cpu_ram_ack), .ram_we(bc_ram_we),   // routed ack
	.ram_addr(bc_ram_addr), .ram_be(bc_ram_be), .ram_wdata(bc_ram_wdata),
	.ticks_last(gtck_ticks), .tick_line(gtck_line),
	.ram_stall_last(rams_stall), .ram_acc_last(rams_acc)
);

// Control-write trace. Per frame (vblank edge to vblank edge) the first four
// MO control changes are captured as
//     {index[15:12], newbits[11:9] = FRAME,ERASE,MOGO, vcount[8:0]}
// and latched at the frame edge into CTR0 = {ev0, ev1}, CTR1 = {ev2, ev3}.
// index 0..3 is the event's order; an unused slot reads F000. With more than
// four changes, ev3's index field holds the frame's total (F = 15+).
// vcount >= 0F0 is vblank.
reg [15:0] ctr_ev [0:3];
reg [15:0] ctr_ev_q [0:3];
reg [3:0]  ctr_n;
reg        ctr_vbl_d;
integer    ctr_i;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		for (ctr_i = 0; ctr_i < 4; ctr_i = ctr_i + 1) begin
			ctr_ev[ctr_i] <= 16'hF000; ctr_ev_q[ctr_i] <= 16'hF000;
		end
		ctr_n <= 4'd0; ctr_vbl_d <= 1'b0;
	end else begin
		ctr_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !ctr_vbl_d) begin
			// latch the frame just ended, restart
			for (ctr_i = 0; ctr_i < 4; ctr_i = ctr_i + 1) begin
				ctr_ev_q[ctr_i] <= ctr_ev[ctr_i];
				ctr_ev[ctr_i]   <= 16'hF000;
			end
			// total count rides in ev3's index field when it overflowed
			if (ctr_n > 4'd4) ctr_ev_q[3] <= {ctr_n, ctr_ev[3][11:0]};
			ctr_n <= 4'd0;
		end else if (mo_ctrl_changed) begin
			if (ctr_n < 4'd4)
				ctr_ev[ctr_n[1:0]] <= {ctr_n, mo_ctrl, vid_vcount};
			if (ctr_n != 4'hF) ctr_n <= ctr_n + 4'd1;
		end
	end
end

// Object-list writes vs the render. MAME reads the list atomically at the
// MOGO edge (sort_and_render); our pass can start up to 80,640 cycles later
// (behind the erase sweep) and reads the list for most of a frame, so a list
// the game rewrites after MOGO renders half-rewritten. Per frame, latched at
// the vblank edge:
//   OLWR = {objlist words written between MOGO and the render start,
//           objlist words written while the render runs}
reg [15:0] olw_defer, olw_render, olw_defer_q, olw_render_q;
reg        olw_window;      // MOGO seen, render not yet started
reg        olw_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		olw_defer <= 16'd0; olw_render <= 16'd0;
		olw_defer_q <= 16'd0; olw_render_q <= 16'd0;
		olw_window <= 1'b0; olw_vbl_d <= 1'b0;
	end else begin
		olw_vbl_d <= vid_in_vblank;
		if (mogo_pulse) olw_window <= 1'b1;
		else if (vid_render_busy) olw_window <= 1'b0;
		if (vid_in_vblank && !olw_vbl_d) begin
			olw_defer_q <= olw_defer; olw_render_q <= olw_render;
			olw_defer <= 16'd0; olw_render <= 16'd0;
		end else if (objw_any) begin
			if (vid_render_busy) begin
				if (olw_render != 16'hFFFF) olw_render <= olw_render + 16'd1;
			end else if (olw_window || mogo_pulse) begin
				if (olw_defer != 16'hFFFF) olw_defer <= olw_defer + 16'd1;
			end
		end
	end
end

// MO pixel value witness: MOPX = {non-zero mo_pixel count, last non-zero
// value}. The mixer (agt_colormix_primrage) decides colour from mo_pixel
// alone:
//     mopri   = mo_pixel[15:12]
//     mo_wins = (mo_pixel[5:0] != 0) && (mo_pixel[11] || mgep || pf==0)
//     pen     = mo_wins ? {1'b1, mo_pixel[10:0]} : ...
reg [15:0] mopx_count;
reg [15:0] mopx_last;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		mopx_count <= 16'd0; mopx_last <= 16'd0;
	end else if (vid_mo_sel != 16'd0) begin
		mopx_last <= vid_mo_sel;
		if (mopx_count != 16'hFFFF) mopx_count <= mopx_count + 16'd1;
	end
end

// RLE sprite data witness: RLED = {words the renderer read, non-zero words}.
// The download writes the region through agt_sdram's dl port and the
// renderer reads it through the three-way tile-port arbiter, so these are
// separate paths. Reads that are all zero mean the data is not where the
// renderer looks: wrong base, wrong address shift, or the region never
// reached SDRAM.
reg [15:0] rled_reads, rled_nonzero;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rled_reads <= 16'd0; rled_nonzero <= 16'd0;
	end else begin
		if (rle_rom_data_valid && rled_reads != 16'hFFFF)
			rled_reads <= rled_reads + 16'd1;
		// Watch rle_word32, the word the renderer receives (cache or SDRAM);
		// the raw SDRAM return (rle_sdr_data32) would miss every cache hit.
		if (rle_rom_data_valid && (rle_word32 != 32'd0)
							   && rled_nonzero != 16'hFFFF)
			rled_nonzero <= rled_nonzero + 16'd1;
	end
end

// Object-list write witness: OBJW = {writes into 0xd78000-0xd78fff, writes
// of a non-zero low byte to an entry's order word (offset 6)}, the order
// being the field the scan looks for. Window: 0xd78000 is snoop word 0x2000,
// matching agt_demo_memories' sn_ob.
wire sn_ob_top   = (gv_snoop_addr >= 14'h2000) && (gv_snoop_addr < 14'h2400);
wire objw_any    = gv_snoop_we && sn_ob_top;
// Entry word 6 is the order: in a 32-bit snoop word, entry words 6/7 sit at
// snoop_addr[0] == 1, word 6 in the high half (byte enables [3:2]).
wire objw_order  = objw_any && gv_snoop_addr[0]
							&& (gv_snoop_be[2])
							&& (gv_snoop_wd[23:16] != 8'd0);

reg [15:0] objw_total, objw_orders;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		objw_total <= 16'd0; objw_orders <= 16'd0;
	end else begin
		// saturating: a wrapped count could pass for a small genuine one
		if (objw_any   && objw_total  != 16'hFFFF) objw_total  <= objw_total  + 16'd1;
		if (objw_order && objw_orders != 16'hFFFF) objw_orders <= objw_orders + 16'd1;
	end
end

// Snoop byte-enable witness for the alpha window: snbe_alpha_wr counts the
// game's writes into it, snbe_partial those that enable only one byte of a
// halfword. The window condition is copied from agt_demo_memories' sn_al, so
// the counter and the tilemap copy agree on what "alpha" is.
wire sn_al_top = (gv_snoop_addr >= 14'h1800) && (gv_snoop_addr < 14'h1C00);
wire sn_al_wr  = gv_snoop_we && sn_al_top;
wire sn_al_hi_ok = gv_snoop_be[3] && gv_snoop_be[2];
wire sn_al_lo_ok = gv_snoop_be[1] && gv_snoop_be[0];
// partial: at least one halfword of the write has only one byte enabled
wire sn_al_partial = sn_al_wr &&
					 ((gv_snoop_be[3] ^ gv_snoop_be[2]) ||
					  (gv_snoop_be[1] ^ gv_snoop_be[0]));

reg [15:0] snbe_alpha_wr, snbe_partial;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		snbe_alpha_wr <= 16'd0; snbe_partial <= 16'd0;
	end else begin
		if (sn_al_wr && snbe_alpha_wr != 16'hFFFF)
			snbe_alpha_wr <= snbe_alpha_wr + 16'd1;
		if (sn_al_partial && snbe_partial != 16'hFFFF)
			snbe_partial  <= snbe_partial  + 16'd1;
	end
end

// Scroll-word witness: the same two counts for the alpha window's scroll and
// bank registers only. MAME atarigt_v.cpp scanline_update reads them with
//     offset = ((scanline & ~7) << 3) + 48;   then 16 basemem_read(offset++)
// so in each 64-word alpha row words 48..63 are scroll/bank and 0..47 text:
// alpha offset bits [5:4] == 2'b11. A snoop word carries a pair of alpha
// words (gv_snoop_addr[9:0]; even = byte enables [3:2], odd = [1:0], as in
// agt_demo_memories), so the alpha offset is {gv_snoop_addr[9:0], half} and
// offset[5:4] is gv_snoop_addr[4:3]. scr_wr = 0 while snbe_alpha_wr is large
// would mean this mask is wrong.
wire sn_al_scroll = sn_al_wr && (gv_snoop_addr[4:3] == 2'b11);

reg [15:0] scr_wr, scr_partial;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		scr_wr <= 16'd0; scr_partial <= 16'd0;
	end else begin
		if (sn_al_scroll && scr_wr != 16'hFFFF)
			scr_wr      <= scr_wr      + 16'd1;
		if (sn_al_scroll && sn_al_partial && scr_partial != 16'hFFFF)
			scr_partial <= scr_partial + 16'd1;
	end
end

// Line-0 scroll race (agt_scroll_race_meter): per frame, the game's vblank
// writes to line 0's scroll entry at or after the line the video fetches it
// on (late_q: that frame used the stale value) and before it (early_q, in
// time). The fetch line follows the Debug page's Line 0 Fetch
// (osd_line0_vblank): Late is LINE0_FETCH_LINE, Vblank start is line 240.
localparam int LINE0_FETCH_LINE = 259;   // agt_video's line-0 fetch line
wire [8:0]  line0_fetch_at = osd_line0_vblank ? 9'd240 : 9'(LINE0_FETCH_LINE);
wire [15:0] pflw_late_q, pflw_early_q;
agt_scroll_race_meter u_pflw (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.snoop_we(gv_snoop_we), .snoop_addr(gv_snoop_addr),
	.vblank(vid_in_vblank), .vcount(vid_vcount), .fetch_line(line0_fetch_at),
	.late_q(pflw_late_q), .early_q(pflw_early_q)
);

// Download witnesses (dl_rst_n domain): bytes written to the tile and char
// regions, the tile bytes that were non-zero, and the sprite RLE bytes
// (counted only while Load Sprite ROM is on).
reg [23:0] tiles_wr_count, chars_wr_count;
reg [23:0] tiles_nz_count;   // non-zero tile bytes
reg [31:0] rle_wr_count;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		tiles_wr_count <= 24'd0; chars_wr_count <= 24'd0;
		tiles_nz_count <= 24'd0;
		rle_wr_count <= 32'd0;
	end else begin
		if (dl_tiles_wr) begin
			tiles_wr_count <= tiles_wr_count + 24'd1;
			if (|dl_rom_data && tiles_nz_count != 24'hFFFFFF)
				tiles_nz_count <= tiles_nz_count + 24'd1;
		end
		if (dl_chars_wr) chars_wr_count <= chars_wr_count + 24'd1;
		if (dl_rle_wr && rle_load_en) rle_wr_count <= rle_wr_count + 32'd1;
	end
end

// CAGE access witness: cage_acc_count counts completed accesses (bc_cage_req
// with cage_ack), cage_last_ctrl keeps the last one's cage_rdata[15:0].
reg [15:0] cage_acc_count;
reg [15:0] cage_last_ctrl;
// cage_req_edges counts rising edges of bc_cage_req, the transactions the
// memmap issued, to compare with those that completed.
reg        cage_req_d;
reg [15:0] cage_req_edges;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		cage_acc_count <= 16'd0; cage_last_ctrl <= 16'd0;
		cage_req_d <= 1'b0; cage_req_edges <= 16'd0;
	end else begin
		cage_req_d <= bc_cage_req;
		if (bc_cage_req && !cage_req_d && cage_req_edges != 16'hFFFF)
			cage_req_edges <= cage_req_edges + 16'd1;
		if (bc_cage_req && cage_ack) begin
			cage_acc_count <= cage_acc_count + 16'd1;
			cage_last_ctrl <= cage_rdata[15:0];
		end
	end
end

// Colour-RAM read witness. One read port serves all three layers: alpha
// reads words 0x000-0x0FF, playfield 0x000-0xFFF, motion objects
// 0x1000-0x17FF. cram_data arrives one cycle after cram_addr (registered
// read), so the address is delayed to pair them.
//   cram_hi_last_nz  the last non-zero word read from 0x1000-0x17FF
//   cram_hi_tot      cycles the paired address sat in that range (cycles,
//                    not lookups: the mixer holds cram_addr); saturates
reg [13:0] cram_addr_d;
reg [15:0] cram_hi_last_nz, cram_hi_tot;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		cram_addr_d <= 14'd0; cram_hi_last_nz <= 16'd0; cram_hi_tot <= 16'd0;
	end else begin
		cram_addr_d <= cram_addr;
		if (cram_addr_d >= 14'h1000 && cram_addr_d < 14'h1800) begin
			if (cram_hi_tot != 16'hFFFF) cram_hi_tot <= cram_hi_tot + 16'd1;
			if (|cram_data) cram_hi_last_nz <= cram_data;
		end
	end
end

reg [31:0] insn_count;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) insn_count <= 32'd0;
	else if (bc_insn_done) insn_count <= insn_count + 32'd1;
end

reg [15:0] vack_count, sack_count;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		vack_count <= 16'd0; sack_count <= 16'd0;
	end else begin
		if (bc_vid_ack)  vack_count <= vack_count + 16'd1;
		if (bc_scan_ack) sack_count <= sack_count + 16'd1;
	end
end

reg [15:0] vint_count, sint_count;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		vint_count <= 16'd0; sint_count <= 16'd0;
	end else begin
		if (bc_video_int_set)    vint_count <= vint_count + 16'd1;
		if (bc_scanline_int_set) sint_count <= sint_count + 16'd1;
	end
end

reg [15:0] cram_last_addr, cram_last_data;
// Colour-RAM write witness, sampled on the rising edge of the write request,
// where address and data are valid (by the ack the bus may have moved on).
// bc_cram_addr is a byte address in the 0x80000 window (agt_main_memmap), so
// the MO palette, words 0x1000-0x17FF, is bytes 0x2000-0x2FFF.
//   cram_last_addr/data  the last write, any range
//   cram_all_writes      all writes; cram_mo_writes those into the MO palette
//   cram_mo_addr/data    the last MO-palette write
//   cram_mo_nz           MO-palette writes with non-zero data
wire        cram_wr_req   = bc_cram_req & bc_cram_we;
reg         cram_wr_req_d;
wire        cram_wr_start = cram_wr_req & ~cram_wr_req_d;   // rising edge
reg  [15:0] cram_mo_writes, cram_all_writes;
reg  [15:0] cram_mo_addr, cram_mo_data, cram_mo_nz;
// cram_mo_lo: writes to words 0x1000-0x11FF, the MO palette's low quarter;
// it can never exceed cram_mo_writes.
reg  [15:0] cram_mo_lo;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		cram_last_addr <= 16'd0; cram_last_data <= 16'd0;
		cram_mo_writes <= 16'd0; cram_all_writes <= 16'd0;
		cram_mo_addr <= 16'd0; cram_mo_data <= 16'd0; cram_mo_nz <= 16'd0;
		cram_mo_lo <= 16'd0;
		cram_wr_req_d  <= 1'b0;
	end else begin
		cram_wr_req_d <= cram_wr_req;
		if (cram_wr_start) begin
			cram_last_addr <= bc_cram_addr[15:0];
			cram_last_data <= bc_cram_wdata;
			if (cram_all_writes != 16'hFFFF)
				cram_all_writes <= cram_all_writes + 16'd1;
			// MO palette: words 0x1000-0x17FF = bytes 0x2000-0x2FFF
			if (bc_cram_addr[18:0] >= 19'h2000 && bc_cram_addr[18:0] <= 19'h2FFF) begin
				if (cram_mo_writes != 16'hFFFF)
					cram_mo_writes <= cram_mo_writes + 16'd1;
				cram_mo_addr <= bc_cram_addr[15:0];
				cram_mo_data <= bc_cram_wdata;
				if (|bc_cram_wdata && cram_mo_nz != 16'hFFFF)
					cram_mo_nz <= cram_mo_nz + 16'd1;
				// bytes 0x2000-0x23FF = words 0x1000-0x11FF
				if (bc_cram_addr[18:0] <= 19'h23FF && cram_mo_lo != 16'hFFFF)
					cram_mo_lo <= cram_mo_lo + 16'd1;
			end
		end
	end
end

// Tile fetch from SDRAM: agt_tile_sdram turns the renderer's BRAM-style ROM
// handshake into tile port requests and adds the region base.
wire [25:0] sdr_tile_addr;
wire        sdr_tile_req;
wire        sdr_tile_ack;
wire [7:0]  sdr_tile_port_data;
wire [31:0] sdr_tile_port_data32;
wire [95:0] sdr_tile_port_data96;
wire [95:0] sdr_tile_data96;
assign      tile_rom_data96 = sdr_tile_data96;
wire        tp_tile_burst3;
wire        sdr_tile_burst3;
wire [95:0] sdr_tile_burst_data96;

agt_tile_sdram #(.BASE(26'h0300000), .REMAP(1'b1)) u_tile_sdram (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.rom_addr(tile_rom_addr), .rom_rd(tile_rom_rd && !tile_src_demo),
	.rom_rd3(tile_rom_rd3 && !tile_src_demo),
	.rom_data(sdr_tile_data), .rom_data_valid(sdr_tile_valid),
	.rom_data32(sdr_tile_data32), .rom_data96(sdr_tile_data96),
	.tile_addr(tp_tile_addr), .tile_req(tp_tile_req),
	.tile_burst3(tp_tile_burst3), .tile_data96(sdr_tile_port_data96),
	.tile_ack(tp_tile_ack), .tile_data(sdr_tile_port_data), .tile_data32(sdr_tile_port_data32)
);

// Chars from SDRAM: the same adapter at the char region's base (u_char_sdram
// below), so the alpha layer draws the game's characters.
wire [16:0] charrom_addr;
wire        charrom_rd;
// sprite pixel witness (agt_video's rnd_wr_valid)
wire        vid_rnd_wr_valid;
// object-list debug taps
wire [15:0] dbg_obj_w0, dbg_obj_w4;
wire        dbg_obj_valid;
wire [7:0]  dbg_obj_rejects;
// dbg_obj_full is agt_rle_objlist's dbg_full_scans: passes that reached slot
// 255 (completed), not a list-full flag. The name stays for port
// compatibility.
wire [15:0] dbg_obj_starts, dbg_obj_full;
wire [15:0] dbg_obj_zscale, dbg_obj_offscr, dbg_scale_or, dbg_scale_and;
wire [15:0] dbg_mo_cram, dbg_mo_latch;  wire [23:0] dbg_mo_rgb;
wire [15:0] dbg_mo_hits;
wire [15:0] dbg_mo_nz;
wire [15:0] dbg_drop_alt, dbg_drop_erase;
wire [15:0] dbg_drop_oobx, dbg_drop_ooby;
wire signed [15:0] dbg_wr_xmin, dbg_wr_xmax;
wire [15:0] dbg_wild_code, dbg_wild_scale, dbg_wild_width;
wire signed [15:0] dbg_wild_draw_x;
wire [15:0] dbg_wild_hdr, dbg_wild_mwidth;
wire [15:0] dbg_blk_cra, dbg_blk_cram, dbg_blk_latch;
wire [15:0] dbg_start_lost, dbg_pass_done;
// Probe pixel, native coordinates. The colour mixer's dbg_prb_* taps capture
// the MO and playfield pixels there, the cram address it chose and cram[that].
localparam logic [8:0] PROBE_X = 9'd57;
localparam logic [8:0] PROBE_Y = 9'd55;
wire [15:0] dbg_prb_mo, dbg_prb_pf, dbg_prb_cra, dbg_prb_cram;
wire [15:0] dbg_big_code, dbg_big_width;
wire [15:0] dbg_big_w0, dbg_big_w1;
wire [15:0] dbg_big_x, dbg_big_y, dbg_sml_x, dbg_sml_y;
wire [15:0] dbg_pc_hits, dbg_pc_misses;
wire [15:0] dbg_hflip_cnt, dbg_obj_cnt;
wire [15:0] dbg_hf_live, dbg_hf_first, dbg_hf_w0;

// CAGE download path. The sound board's boot parser and RAM loader live in
// agt_cage (rtl/cage/agt_cage.sv), instantiated beside agt_rom_download
// below. These are the wires it shares with the rest of the top level: the
// boot region's stream from the decoder, the download flag, the SDRAM cage
// port and the overlay witnesses.
wire        dl_cb_wr;
wire [18:0] dl_cb_addr;
wire        dl_rom_loading;             // agt_rom_download.rom_loading, below
// cg_ld_wait (ORed into ioctl_wait) is declared ahead of hps_io. It is
// agt_cage's dl_wait: ramload's FIFO and the IRAM transfer's skid.
wire [25:0]  cg_p_addr;
wire         cg_p_we, cg_p_line, cg_p_req, cg_p_ack;
wire [127:0] cg_p_wdata, cg_p_rdata;
wire [31:0] cg_ld_witness;              // agt_cage_ramload's witness, no slot
wire [31:0] cg_rlse;                    // RLSE, slot 11
wire [31:0] cg_chlt;                    // CHLT, slot 4
wire [31:0] cg_sbnk;                    // agt_cage_sbank's witness, no slot
wire [31:0] cg_dacw;                    // DACW, slot 8
wire [31:0] cg_dstl;                    // DSTL, slot 5
wire [15:0] cg_audio_l, cg_audio_r;     // the CAGE DACs, signed
wire        cg_dsp_present;             // 1 = the real DSP answers the 68020
wire [15:0] dbg_ob_hf_set, dbg_ob_wr;

// Object-list write witness, per frame: snooped writes to entry word 0
// (ob_w0_pulse) and those with hflip (word 0 bit 15) set (ob_hf_pulse),
// latched at the vblank edge into obwr_*_last.
wire        ob_w0_pulse, ob_hf_pulse;
reg  [19:0] obwr_hf_frame, obwr_w0_frame;
reg  [15:0] obwr_hf_last, obwr_w0_last;
reg  [15:0] obhw_hf, obhw_w0;      // max in any single frame since reset
reg         obwr_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		obwr_hf_last <= 16'd0; obwr_w0_last <= 16'd0;
		obhw_hf <= 16'd0; obhw_w0 <= 16'd0;
		obwr_hf_frame <= 20'd0; obwr_w0_frame <= 20'd0; obwr_vbl_d <= 1'b0;
	end else begin
		obwr_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !obwr_vbl_d) begin
			obwr_hf_last  <= obwr_hf_frame[15:0];
			obwr_w0_last  <= obwr_w0_frame[15:0];
			if (obwr_hf_frame[15:0] > obhw_hf) obhw_hf <= obwr_hf_frame[15:0];
			if (obwr_w0_frame[15:0] > obhw_w0) obhw_w0 <= obwr_w0_frame[15:0];
			obwr_hf_frame <= 20'd0;
			obwr_w0_frame <= 20'd0;
		end else begin
			if (ob_hf_pulse && obwr_hf_frame != 20'hFFFFF)
				obwr_hf_frame <= obwr_hf_frame + 20'd1;
			if (ob_w0_pulse && obwr_w0_frame != 20'hFFFFF)
				obwr_w0_frame <= obwr_w0_frame + 20'd1;
		end
	end
end
// RLE ROM handshake per frame: requests the renderer issued (rleh_req) and
// valids the ROM path returned (rleh_val), latched at vblank, >> 1 so a
// heavy pass fits 16 bits. They must be equal: an extra valid would be taken
// by the MEASURE pass (one read outstanding) as its row header.
reg [16:0] rleh_req, rleh_val;
reg [15:0] rleh_req_q, rleh_val_q;
reg        rleh_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rleh_req <= 17'd0; rleh_val <= 17'd0; rleh_vbl_d <= 1'b0;
		rleh_req_q <= 16'd0; rleh_val_q <= 16'd0;
	end else begin
		rleh_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !rleh_vbl_d) begin
			rleh_req_q <= rleh_req[16:1]; rleh_val_q <= rleh_val[16:1];
			rleh_req <= 17'd0; rleh_val <= 17'd0;
		end else begin
			if (rle_rom_rd         && rleh_req != 17'h1FFFF) rleh_req <= rleh_req + 17'd1;
			if (rle_rom_data_valid && rleh_val != 17'h1FFFF) rleh_val <= rleh_val + 17'd1;
		end
	end
end
wire [15:0] dbg_mogo_deferred;           // renders held back by an erase sweep
wire [15:0] dbg_wr_behind, dbg_wr_ahead; // sprite writes behind | ahead of the beam, last frame, >>2
wire [15:0] dbg_erase_sweeps;            // sweeps that actually ran
// the cram address the last MO pixel used (from agt_video)
wire [13:0] dbg_mo_cra;
// Per-frame deltas of the mixer's free-running counts, taken at the vblank
// edge: MO pixels it selected (dbg_mo_hits) and MO pixels it saw opaque
// (dbg_mo_nz). Zero hits means the mixer never selected the MO layer.
reg  [15:0] mo_hits_prev, mo_hits_frame;
reg  [15:0] mo_nz_prev, mo_nz_frame;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		mo_hits_prev <= 16'd0; mo_hits_frame <= 16'd0;
		mo_nz_prev <= 16'd0; mo_nz_frame <= 16'd0;
	end else if (vid_in_vblank && !pght_vbl_d) begin
		mo_hits_frame <= dbg_mo_hits - mo_hits_prev;   // wrap-safe
		mo_hits_prev  <= dbg_mo_hits;
		mo_nz_frame <= dbg_mo_nz - mo_nz_prev;
		mo_nz_prev  <= dbg_mo_nz;
	end
end
wire [8:0]  dbg_obj_lastidx;
wire [15:0] dbg_obj_examined, dbg_obj_emitted;
wire [15:0] dbg_obj_examined_pf, dbg_obj_emitted_pf;   // per pass
wire [15:0] dbg_stage_ot, dbg_stage_bl;
wire        dbg_blit_wait, dbg_blit_emit;
wire        dbg_blit_meas;
wire [23:0] dbg_st_obj, dbg_st_tbl, dbg_st_blt;
wire [15:0] vid_mo_sel;   // the MO pixel as the mixer sees it
wire        vid_render_busy, vid_in_vblank;
wire [15:0] vid_line_worst, vid_scroll_worst, vid_alpha_worst, vid_render_worst;
wire        render_overrun;   // pixel_clk domain, from agt_video
wire [31:0] vid_ovr_where;    // OVRL and STAT's upper half (D-648): agt_video's
wire [15:0] vid_ovr_pass;     // dbg_ovr_where / dbg_ovr_pass

// Per-frame MO pixel count, latched at vblank: a rate (mopx_count is the run
// total).
reg  [15:0] momv_i, momv_frame;
// OR and AND of every non-zero MO pixel in the frame: which bits ever move
reg  [15:0] momv_or_i, momv_and_i, momv_or, momv_and;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		momv_i <= 16'd0; momv_frame <= 16'd0;
		momv_or_i <= 16'd0; momv_and_i <= 16'hFFFF;
		momv_or <= 16'd0;   momv_and <= 16'd0;
	end else if (vid_in_vblank && !pght_vbl_d) begin
		momv_frame <= momv_i; momv_i <= 16'd0;
		momv_or  <= momv_or_i;  momv_or_i  <= 16'd0;
		momv_and <= momv_and_i; momv_and_i <= 16'hFFFF;
	end else if (vid_mo_sel != 16'd0) begin
		if (momv_i != 16'hFFFF) momv_i <= momv_i + 16'd1;
		momv_or_i  <= momv_or_i  | vid_mo_sel;
		momv_and_i <= momv_and_i & vid_mo_sel;
	end
end

// Tile data witness, per frame, latched at vblank: tile fetches that returned
// non-zero data (tnz_nz) and all tile fetches (tnz_tot); nz <= tot.
wire        tnz_hit = sdr_tile_valid;
wire        tnz_nzd = sdr_tile_valid && (|sdr_tile_data32 || |sdr_tile_data96);
reg  [15:0] tnz_nz_i, tnz_tot_i, tnz_nz, tnz_tot;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		tnz_nz_i <= 16'd0; tnz_tot_i <= 16'd0;
		tnz_nz   <= 16'd0; tnz_tot   <= 16'd0;
	end else if (vid_in_vblank && !pght_vbl_d) begin
		tnz_nz <= tnz_nz_i; tnz_tot <= tnz_tot_i;
		tnz_nz_i <= 16'd0;  tnz_tot_i <= 16'd0;
	end else begin
		if (tnz_hit && tnz_tot_i != 16'hFFFF) tnz_tot_i <= tnz_tot_i + 16'd1;
		if (tnz_nzd && tnz_nz_i  != 16'hFFFF) tnz_nz_i  <= tnz_nz_i  + 16'd1;
	end
end

// Tile-port page-hit witness, from agt_sdram: tile accesses (sdr_tile_acc),
// those that found their row already open (sdr_tile_pghit) and those whose
// bank had some row open (sdr_tile_bankopen). Counted within a frame and
// latched at the vblank edge; a per-run count would saturate within
// milliseconds. A full-tile frame is about 129 x 240 = ~31k accesses
// (0x79xx), which fits 16 bits. hits <= accesses.
wire        sdr_tile_acc, sdr_tile_pghit, sdr_tile_bankopen;
reg  [15:0] pght_hits_acc, pght_acc_acc;   // accumulating within the frame
reg  [15:0] pght_hits,     pght_acc;       // latched: last complete frame
reg  [15:0] pgbo_i,        pgbo;           // bank had a row open, same framing
reg         pght_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		pght_hits_acc <= 16'd0; pght_acc_acc <= 16'd0;
		pght_hits <= 16'd0;     pght_acc <= 16'd0;
		pgbo_i <= 16'd0;        pgbo <= 16'd0;
		pght_vbl_d <= 1'b0;
	end else begin
		pght_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !pght_vbl_d) begin
			// frame boundary: publish, then restart
			pght_hits <= pght_hits_acc; pght_acc <= pght_acc_acc;
			pgbo      <= pgbo_i;
			pght_hits_acc <= 16'd0;     pght_acc_acc <= 16'd0; pgbo_i <= 16'd0;
		end else begin
			if (sdr_tile_acc   && pght_acc_acc  != 16'hFFFF) pght_acc_acc  <= pght_acc_acc  + 16'd1;
			if (sdr_tile_pghit && pght_hits_acc != 16'hFFFF) pght_hits_acc <= pght_hits_acc + 16'd1;
			if (sdr_tile_bankopen && pgbo_i != 16'hFFFF)     pgbo_i        <= pgbo_i        + 16'd1;
		end
	end
end

// Line-buffer overrun witness (OVRN, slot 2). agt_video's scanout swaps line
// buffers at H_LAST if the renderer has signalled the next line; if not, it
// raises render_overrun and shows the old buffer again: a repeated line, and
// the picture below it one line lower, for that frame only.
// render_overrun is a one-pixel-clock pulse (8 clk_sys cycles): synchronise,
// edge-detect, count. In the sys_rst_n domain on purpose, so an OSD reset
// starts a comparison clean.
//   ovr_lines   overrun lines
//   ovr_frames  frames with at least one overrun (<= ovr_lines)
// Both saturate.
reg  [2:0]  ovr_sync;
reg  [15:0] ovr_lines, ovr_frames;
reg         ovr_this_frame, ovr_vbl_d;
wire        ovr_edge = ovr_sync[1] && !ovr_sync[2];
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		ovr_sync <= 3'd0; ovr_lines <= 16'd0; ovr_frames <= 16'd0;
		ovr_this_frame <= 1'b0; ovr_vbl_d <= 1'b0;
	end else begin
		ovr_sync  <= {ovr_sync[1:0], render_overrun};
		ovr_vbl_d <= vid_in_vblank;
		if (ovr_edge) begin
			if (ovr_lines != 16'hFFFF) ovr_lines <= ovr_lines + 16'd1;
			ovr_this_frame <= 1'b1;
		end
		if (vid_in_vblank && !ovr_vbl_d) begin
			if (ovr_this_frame && ovr_frames != 16'hFFFF)
				ovr_frames <= ovr_frames + 16'd1;
			ovr_this_frame <= 1'b0;
		end
	end
end
wire [10:0] obj_rd_addr;
wire        obj_snap_busy;   // object-list snapshot copy in progress
wire        obj_rd, obj_rd_valid;
wire [15:0] obj_rd_data;
wire [7:0]  charrom_data;
wire [31:0] charrom_data32;
wire        charrom_data_valid;

// Sprite RLE data: the tile port's third consumer, at the RLE region's base;
// the renderer addresses the region 1:1. ADDR_W(25): the region is 32 MB and
// needs 25 bits of byte address (the default 22 would truncate). Bursts of
// three: rom_rd3 requests addr+0/+4/+8 in one controller transaction and
// rom_data96 carries the three pairs to the pair cache below (rom_rd and
// rom_data32 are unused). rom_data_valid pulses once whichever request line
// fired.
agt_tile_sdram #(.BASE(SDR_BASE_RLE), .ADDR_W(25)) u_rle_sdram (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.rom_addr(rle_fetch_addr), .rom_rd(1'b0),   // the clamped base, not the request
	.rom_data(), .rom_data_valid(rle_sdr_valid),
	.rom_data32(),
	.tile_addr(rp_tile_addr), .tile_req(rp_tile_req),
	.tile_ack(rp_tile_ack), .tile_data(sdr_tile_port_data), .tile_data32(sdr_tile_port_data32),
	.rom_rd3(rle_fetch_rd), .rom_data96(rle_sdr_data96), .tile_burst3(rp_tile_burst3), .tile_data96(rp_tile_data96)
);

agt_tile_sdram #(.BASE(SDR_BASE_CHARS)) u_char_sdram (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.rom_addr({5'd0, charrom_addr}), .rom_rd(charrom_rd),
	.rom_data(charrom_data), .rom_data_valid(charrom_data_valid),
	.rom_data32(charrom_data32),
	.tile_addr(cp_tile_addr), .tile_req(cp_tile_req),
	.tile_ack(cp_tile_ack), .tile_data(sdr_tile_port_data), .tile_data32(sdr_tile_port_data32),
	.rom_rd3(1'b0), .rom_data96(), .tile_burst3(), .tile_data96(96'd0)
);

// Chars and tiles share the tile port but never overlap: agt_video's
// sequencer runs the alpha pass (chars) to completion before the line render
// (tiles). The mux is still explicit: if the ordering ever changes, a
// priority mux degrades to one waiting, where wire-ORing two masters would
// corrupt both.
wire [25:0] tp_tile_addr, cp_tile_addr;
wire        tp_tile_req,  cp_tile_req,  rp_tile_req;
wire [25:0] rp_tile_addr;
// The RLE adapter's burst request and 96-bit return, carried through the
// tile port arbiter to the controller.
wire        rp_tile_burst3;
wire [95:0] rp_tile_data96;
// The renderer reads 16-bit words; the adapter is byte-addressed and returns
// a 32-bit word pair, of which the addressed half is taken.
wire [23:0] rle_rom_waddr;
wire        rle_rom_rd;
wire        rle_sdr_valid;
wire [31:0] rle_sdr_data32;   // unused at BURST=3: declared, not referenced
wire [95:0] rle_sdr_data96;   // rom_rd3 return, first word read in [95:64]

// The renderer reads the RLE stream sequentially in 16-bit words and the
// tile port returns 32-bit pairs, so two consecutive words share a pair:
//   waddr N   -> byte 2N   -> pair N>>1
//   waddr N+1 -> byte 2N+2 -> pair N>>1   (N even)
// The pair cache below serves the second word without another SDRAM fetch.
wire [24:0] rle_wpair   = rle_rom_waddr[23:1];      // which 32-bit word
wire [25:0] rle_rom_addr = {rle_rom_waddr, 1'b0};
// Where a sprite burst is fetched from. agt_rle_pair_cache clamps the window
// base so three pairs never cross a 2 KB SDRAM row (the burst column would
// carry out); the adapter must fetch from that base, not the requested
// address, or the clamp does nothing. Pair-aligned: the controller's burst
// column ignores the word-within-pair bit.
wire [24:0] rle_fetch_base;
wire [31:0] rle_miss_data;     // the requested pair out of a return
wire [24:0] rle_fetch_addr = {rle_fetch_base[22:0], 2'b00};

// RLE pair cache, three pairs per fetch (agt_rle_pair_cache, benched by
// tb_rle_pair_cache).
wire        rle_cache_hit;
wire [31:0] rle_cache_d;
wire        rle_hit_valid;
wire        rle_fetch_rd;
wire [24:0] rle_data_pair;     // the pair the data is (tag+offset on a hit), for the delivery witness
// agt_rle_pair_cache takes sdr_data lowest pair first (offset 0, the
// requested pair, in [31:0]); agt_sdram's rom_data96 puts the first word read
// (addr+0) in [95:64] and the last (addr+8) in [31:0]. Swizzled here rather
// than in the module, so the module keeps the convention its bench checks.
// The middle pair does not move.
wire [95:0] rle_sdr_data96_lo0 = {rle_sdr_data96[31:0], rle_sdr_data96[63:32], rle_sdr_data96[95:64]};
agt_rle_pair_cache #(.BURST(3)) u_rle_pair_cache (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.req_wpair(rle_wpair), .req_rd(rle_rom_rd),
	.fetch_rd(rle_fetch_rd), .fetch_pair_base(rle_fetch_base),
	.sdr_valid(rle_sdr_valid), .sdr_data(rle_sdr_data96_lo0),
	.hit(rle_cache_hit), .hit_valid(rle_hit_valid),
	.data(rle_cache_d), .miss_data(rle_miss_data),
	.data_pair(rle_data_pair)
);

// Delivery witness: on every valid, compare the pair the renderer asked for
// (rle_wpair) with the pair the data belongs to (rle_data_pair, from the
// cache: tag+offset on a hit, the fetched pair on an SDRAM return). Any
// difference is a mis-delivered word. rlem_bad counts them per frame
// (rlem_badhit: those that were cache hits), latched at vblank; rlea_* keep
// the first mismatch of each frame.
wire        rle_pair_bad   = rle_rom_data_valid && (rle_data_pair != rle_wpair);
reg  [16:0] rlem_bad, rlem_badhit;
reg  [15:0] rlem_bad_q, rlem_badhit_q;
reg  [15:0] rlea_want_q, rlea_got_q;
reg         rlea_seen, rlem_vbl_d;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		rlem_bad <= 17'd0; rlem_badhit <= 17'd0; rlem_vbl_d <= 1'b0;
		rlem_bad_q <= 16'd0; rlem_badhit_q <= 16'd0;
		rlea_want_q <= 16'd0; rlea_got_q <= 16'd0; rlea_seen <= 1'b0;
	end else begin
		rlem_vbl_d <= vid_in_vblank;
		if (vid_in_vblank && !rlem_vbl_d) begin
			rlem_bad_q    <= rlem_bad[15:0];
			rlem_badhit_q <= rlem_badhit[15:0];
			rlem_bad <= 17'd0; rlem_badhit <= 17'd0;
			rlea_seen <= 1'b0;
		end else if (rle_pair_bad) begin
			if (rlem_bad != 17'h1FFFF) rlem_bad <= rlem_bad + 17'd1;
			if (rle_cache_hit && rlem_badhit != 17'h1FFFF)
				rlem_badhit <= rlem_badhit + 17'd1;
			if (!rlea_seen) begin
				rlea_seen   <= 1'b1;
				rlea_want_q <= rle_wpair[15:0];       // pair the blit asked for
				rlea_got_q  <= rle_data_pair[15:0];   // pair it was handed
			end
		end
	end
end

// The word the renderer receives. On a miss the cache module picks the
// requested pair out of the arrived burst (`miss_data`): an unclamped fetch
// starts at the requested pair (offset 0), a clamped one below it, putting
// it at offset 1 or 2.
wire [31:0] rle_word32 = rle_cache_hit ? rle_cache_d : rle_miss_data;
wire [15:0] rle_rom_data = rle_rom_waddr[0] ? rle_word32[15:0]
											: rle_word32[31:16];
wire        rle_rom_data_valid = rle_sdr_valid | rle_hit_valid;
// Tile port arbiter: chars, tiles and the sprite renderer share one SDRAM
// port. Sprites get the lowest priority: MOGO fires almost always in
// vblank, so the sprite pass wants the port when the line renderer does
// not, and a pass that lands mid-frame slows down rather than shearing a
// scanline. Sprites are a per-frame pass, so this is contention across a
// frame; the per-line budget is unaffected.
wire tp_tile_ack, cp_tile_ack, rp_tile_ack;
localparam logic [4:0] TEXT_HOLD = 5'd24;   // cycles, agt_tile_arb cp_hold
wire tarb_video;       // the arbiter's request is the line renderer's
wire tarb_vwait;       // ...or a sprite fetch a line-renderer fetch waits behind
wire vid_line_late;    // agt_video: the line in progress is late

agt_tile_arb u_tile_arb (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.cp_addr(cp_tile_addr), .cp_req(cp_tile_req), .cp_ack(cp_tile_ack),
	.tp_addr(tp_tile_addr), .tp_req(tp_tile_req), .tp_ack(tp_tile_ack),
	.rp_addr(rp_tile_addr), .rp_req(rp_tile_req), .rp_ack(rp_tile_ack),
	.rp_quiet(osd_fetch_yield == 2'd0 ? 4'd0 : osd_fetch_yield == 2'd1 ? 4'd4 : osd_fetch_yield == 2'd2 ? 4'd8 : 4'd12),   // Sprite Fetch Yield: 0/4/8/12
	// Sprites Wait for Text (D-649). The alpha pass's gap between one
	// character's data and the next request is 14-23 cycles (tb_video_sdram's
	// ALPHA GAPS), so 24 covers every gap in a run of characters.
	.cp_hold(osd_text_hold ? TEXT_HOLD : 5'd0),
	.sdr_addr(sdr_tile_addr), .sdr_req(sdr_tile_req), .sdr_ack(sdr_tile_ack),
	// The burst flag rides with the granted request; chars never burst.
	.tp_burst3(tp_tile_burst3), .cp_burst3(1'b0), .rp_burst3(rp_tile_burst3),
	.sdr_burst3(sdr_tile_burst3), .sdr_data96(sdr_tile_burst_data96),
	.tp_data96(sdr_tile_port_data96), .cp_data96(), .rp_data96(rp_tile_data96),
	.sdr_video(tarb_video), .sdr_video_wait(tarb_vwait)
);
// The line renderer's fetch goes first at the SDRAM (agt_sdram tile_first)
// always, or while its line is late (agt_video line_late), or never, as the
// Debug page's Video Priority says; the release build uses Always.
// Always+Sprites (D-648) adds the sprite fetch a line fetch is queued behind:
// the arbiter cannot take the port back from it, so without this the line
// waits as long as the sprite fetch does (behind the cage and the CPU).
wire sdr_tile_first = (osd_vid_prio == 2'd3) ? (tarb_video || tarb_vwait)
					: tarb_video && ((osd_vid_prio == 2'd0) ? 1'b1
									: (osd_vid_prio == 2'd1) ? vid_line_late : 1'b0);

// Snoop witness: counts the game's writes into the snooped tilemap
// windows. Zero means the game has not written a tilemap yet; nonzero with
// no text on screen puts the fault in the video-side copy or the alpha
// render path.
reg [31:0] snoop_wr_count;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) snoop_wr_count <= 32'd0;
	else if (gv_snoop_we) snoop_wr_count <= snoop_wr_count + 32'd1;
end

// High-water PC: the furthest address retired. A single PC sample moves
// too fast to read; this says how far the boot got.
reg [31:0] pc_max;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) pc_max <= 32'd0;
	else if (bc_insn_done && bc_insn_pc > pc_max) pc_max <= bc_insn_pc;
end

// Download witnesses at the ROM write port:
//   DLWR  maincpu bytes agt_rom_download wrote   (expect 00200000)
//   DLSM  sum of those bytes, mod 2^32            (expect 060A68F6)
//   IOWR  every ioctl_wr pulse during any download
//   IOIX  the last ioctl_index seen during a download
// IOWR and IOIX are deliberately not gated by index: they count what
// arrived, not what the decoder accepted. agt_rom_download writes only
// index 0 but sets rom_loaded at the end of any download.
reg [31:0] dl_wr_count, dl_wr_sum;
// Reset domain: these witnesses reset on dl_rst_n (~RESET and the PLL
// lock), like agt_rom_download and agt_sdram, not on sys_rst_n, which also
// takes status[0] and buttons[1]. Otherwise an OSD or button reset would
// clear them while rom_loaded survives, and they would read as if the
// download never happened. dl_rst_n must never gain status[0] or
// buttons[1]: a warm reset would lose rom_loaded and hold the CPU for good.
// An unlocked PLL has no valid clock, so there is nothing to preserve.
// (dl_rst_n is declared beside sys_rst_n.)

// Dropped-strobe witness: maincpu writes presented while the controller was
// busy, which agt_sdram discards. With ioctl_wait stalling the stream this
// must read 0; if not, the stall acts too late and the port needs a FIFO.
reg [31:0] dl_drop_count;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) dl_drop_count <= 32'd0;
	else if (dl_maincpu_wr && sdr_dl_busy) dl_drop_count <= dl_drop_count + 32'd1;
end

// NVRAM download witness: what the framework sent under index 2, counted
// before any of the core's gating (nvrw_bytes counts what the core
// accepted). The pair tells "the core dropped it" from "it was never sent".
// The framework sends index 2 while status[0] holds the core in reset, so
// this lives in dl_rst_n. A clean restore is 0x800 bytes in one burst.
reg [15:0] nv_dl_bytes;
reg [7:0]  nv_dl_bursts;
reg        nv_dl_active;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		nv_dl_bytes <= 16'd0; nv_dl_bursts <= 8'd0; nv_dl_active <= 1'b0;
	end else begin
		nv_dl_active <= ioctl_download && (ioctl_index[7:0] == 8'd2);
		if (ioctl_download && (ioctl_index[7:0] == 8'd2)) begin
			if (!nv_dl_active && nv_dl_bursts != 8'hFF)
				nv_dl_bursts <= nv_dl_bursts + 8'd1;
			if (ioctl_wr && nv_dl_bytes != 16'hFFFF)
				nv_dl_bytes <= nv_dl_bytes + 16'd1;
		end
	end
end

// RAM-clear witness, in dl_rst_n so it outlives the resets it counts.
// ram_clr_passes should be warm_rst_count - 1: the boot-time release of
// sys_rst_n comes before the ROM is loaded and cannot complete a pass.
// Fewer passes means the clear is not running on every warm reset.
// ram_clr_cyc_hi is the last pass's length in cycles >> 8, about 0x1400
// (1.3 M cycles); 0000 with passes > 0 means the pass did not really run.
reg [15:0] ram_clr_passes;
reg [15:0] ram_clr_cyc_hi;
reg        ram_clr_done_d;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		ram_clr_passes <= 16'd0; ram_clr_cyc_hi <= 16'd0; ram_clr_done_d <= 1'b0;
	end else begin
		ram_clr_done_d <= ram_clr_done;
		if (ram_clr_done && !ram_clr_done_d) begin
			if (ram_clr_passes != 16'hFFFF) ram_clr_passes <= ram_clr_passes + 16'd1;
			ram_clr_cyc_hi <= ram_clr_cycles[23:8];
		end
	end
end

reg [31:0] io_wr_count;
reg [7:0]  io_last_index;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		io_wr_count   <= 32'd0;
		io_last_index <= 8'hFF;
	end else if (ioctl_download) begin
		io_last_index <= ioctl_index[7:0];
		if (ioctl_wr) io_wr_count <= io_wr_count + 32'd1;
	end
end
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		dl_wr_count <= 32'd0;
		dl_wr_sum   <= 32'd0;
	end else if (dl_maincpu_wr) begin
		dl_wr_count <= dl_wr_count + 32'd1;
		dl_wr_sum   <= dl_wr_sum + {24'd0, dl_rom_data};
	end
end

// Counts sys_rst_n releases. Power-on counts as one, so anything above 1
// means a warm reset happened and every sys_rst_n-domain reading on screen
// is post-reset.
reg [15:0] warm_rst_count;
reg        sys_rst_n_d;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		warm_rst_count <= 16'd0;
		sys_rst_n_d    <= 1'b0;
	end else begin
		sys_rst_n_d <= sys_rst_n;
		if (sys_rst_n && !sys_rst_n_d) warm_rst_count <= warm_rst_count + 16'd1;
	end
end

// status[0] pulse witness: st0_len_max is the longest status[0] high time
// in clk_sys cycles. In dl_rst_n because sys_rst_n is asserted during the
// pulse it measures. The OSD's "Reset" and "Reset and close OSD" send the
// same pulse (menu.cpp sets status[0], then clears it: a few microseconds,
// hundreds of clk_sys cycles), and every sys_rst_n consumer is an
// asynchronous reset, so the core cannot tell them apart. With
// warm_rst_count: a count step with a length of 0 came from RESET or
// buttons[1], not status[0]. Both saturate.
reg [15:0] st0_len, st0_len_max;
always @(posedge clk_sys or negedge dl_rst_n) begin
	if (!dl_rst_n) begin
		st0_len <= 16'd0; st0_len_max <= 16'd0;
	end else begin
		if (status[0]) begin
			if (st0_len != 16'hFFFF) st0_len <= st0_len + 16'd1;
		end else
			st0_len <= 16'd0;
		if (st0_len > st0_len_max) st0_len_max <= st0_len;
	end
end

// Build stamp, overlay row 0: `BLD` and nine digits, the build date
// (YYMMDD, from `BUILD_DATE`) then BUILD_TAG. A date alone cannot separate
// two builds on one day, and a tag alone cannot say when it was built.
// BUILD_TAG is three BCD digits identifying the delivery, bumped by hand:
// sys/build_id.tcl is framework code and generates only a date.
localparam logic [11:0] BUILD_TAG = 12'h649;

// `BUILD_DATE` is a string from build_id.tcl, `%y%m%d` (6 chars). Declared
// 64 bits and read from the bottom, so a longer string still yields its last
// six characters: sim/build_id.v carries an 8-character YYYYMMDD, and
// Verilog left-pads a short string, so both widths give YYMMDD.
localparam logic [63:0] BUILD_DATE_STR = `BUILD_DATE;
// ASCII '0'..'9' is 8'h30..8'h39, so each digit's low nibble is its value.
wire [35:0] dbg_build_id = {                        // 9 digits: YYMMDD, then the tag
	BUILD_DATE_STR[43:40], BUILD_DATE_STR[35:32],   // YY
	BUILD_DATE_STR[27:24], BUILD_DATE_STR[19:16],   // MM
	BUILD_DATE_STR[11:8],  BUILD_DATE_STR[3:0],     // DD
	BUILD_TAG
};

wire [383:0] dbg_words = {
	// One 32-bit word per overlay row, slot 11 (the top row) first, shown as
	// eight hex digits beside its label. The trailing `// <slot> <NAME>` of
	// each entry is parsed by tools/gen_dbg_labels.py (the labels) and
	// tools/check_docs.py (the README table): keep that form, and rerun
	// gen_dbg_labels when a slot changes. More witnesses stay computed without
	// a slot (rlem_bad_q, olw_defer_q, cage_sum_q, cg_ld_witness, cg_sbnk,
	// gtck_ticks, ...); showing one is a one-line change here.
	//
	// RLSE: agt_cage_release's witness, with bit 29 the DSP core's halt. Each
	// hex digit of the upper half is four conditions, 1 = satisfied:
	//   [31:28]  run, quiet, halted, 0 (8 running, A halted on an
	//            unimplemented handler, 4 held)
	//   [27:24]  released by the game (control[1:0] != 0), no download active,
	//            a warm reset since the last download, 0 (E = all)
	//   [23:20]  SDRAM ready, boot table parsed, not bad, ramload done (F = all)
	//   [19:16]  no IRAM word in flight, entry point crossed, 0, 0 (C = all)
	//   [15:0]   releases since power-on
	// A running DSP reads 8EFC nnnn.
	{cg_rlse[31:16], cg_rlse[15:0]},  // 11 RLSE  release: conditions | releases
	// HALT: why the 68020 core stopped. It halts on purpose on an opcode it
	// does not implement and latches it: {opcode, 15'd0, unimplemented}. The
	// flag qualifies the opcode, so read it first; the halt PC is slot 0. With
	// the flag clear and the CPU stopped, it halted at another S_HALT site.
	// There is no watchdog (0xE0E000 is decoded and dropped), so a halt is
	// permanent while the video runs on.
	{bc_unimpl_opcode, 15'd0, bc_unimpl},  // 10 HALT  halt opcode | unimplemented
	// OBJN: {objects examined, objects emitted} by the last object pass,
	// latched per pass (a cumulative count cannot answer a per-pass question).
	// emitted < examined counts objects accepted but not drawn. The cumulative
	// saturating pair is still kept; tb_rle_renderer asserts on it.
	{dbg_obj_examined_pf, dbg_obj_emitted_pf},  // 9 OBJN  examined | emitted, per pass
	// DACW: agt_cage_dac's witness, on clk_sys.
	//   upper 16  the largest |left| or |right| sent to AUDIO_L/R since reset
	//   lower 16  words the serial port played in the last 60 vblanks
	// The chip plays 44,100 words a second (11,025 on each of four channels),
	// 44,159 = 0xAC7F in 60 vblanks at 59.92 Hz, so lower / 0xAC7F is the DSP's
	// speed on the board relative to the chip, SDRAM traffic included. It
	// varies by screen with the mixing load; above 0xAC7F the sound plays fast
	// and sharp, which the real-time governor (agt_cage_gov) prevents.
	{cg_dacw[31:16], cg_dacw[15:0]},      // 8 DACW  audio peak | words per 60 vbl
	// STAT: the lower half is dbg_bits[15:0] (bit list at dbg_bits below). A
	// booted core reads EF7F or EFFF (bit 7 is the heartbeat); bit 0 clear
	// means the CPU's first ROM reads did not match the image. The upper half
	// (zero until D-648) is the range of picture lines that have overrun since
	// reset, {lowest, highest}: FF00 before the first (lowest > highest), and
	// equal halves if every overrun is on one line. With OVRN and OVRL.
	{vid_ovr_where[31:16], dbg_bits[15:0]},  // 7 STAT  overrun lines lo hi | dbg_bits[15:0]
	// OVRL (D-648; DCLK until then): the last overrun, as agt_video saw it.
	//   [31:24]  the picture line (0-239) the renderer was late with
	//   [23:16]  that line's alpha pass, 16 clk_sys cycles a step (FF: 4,080+)
	//   [15:8]   that line's render (playfield) pass, the same units
	//   [7:0]    overruns since reset while the sprite renderer was drawing
	//            (saturates at FF); against OVRN's upper half
	// A line is 3,648 cycles; ~3,550 is the budget. 0000 0000 with OVRN 0.
	// DCLK ({clk_dsp, clk_sys ticks a frame >> 4}, 96FC E957 on every build
	// since the PLL settled) stays in the source, and synthesis drops it while
	// no slot shows it; showing it again is this line.
	{vid_ovr_where[15:8], vid_ovr_pass, vid_ovr_where[7:0]},  // 6 OVRL  late line, alpha | render, sprite-pass count
	// DSTL: agt_cage's witness of where the DSP's clk_dsp cycles go. Four shares
	// of the last 2^25 clk_dsp cycles (0.91 s at 630/17), each x256 (40 = 25%):
	//   [31:24]  data cache bridge busy: line fills and write-backs
	//   [23:16]  core held by the real-time governor (ahead of the chip's time)
	//   [15:8]   a port fetch waiting: the instruction cache's misses
	//   [7:0]    core in S_FP: the float ALU, the mixing's signature
	// A fetch that misses both caches counts in [31:24] and [15:8] alike.
	{cg_dstl[31:16], cg_dstl[15:0]},      // 5 DSTL  miss | held | fetch | float, x256
	// CHLT = the CAGE core's last halt since power-on (agt_cage's w_chlt):
	//   upper 16  the halting instruction's ir[31:16]. Bits 31:21 are MAME's
	//             dispatch index (upper >> 5), which names the handler
	//   lower 16  its PC[15:0]. The program's code is 1000-3496 (the boot image
	//             runs on to 8EF5 as data); IRAM's vector branches are 9FC1-9FCB
	// 0000 0000 means no halt. It survives the game's reset kicks (only power-on
	// or a download clears it), so read it with RLSE, whose bit 29 is the halt
	// of the current run. `python tools\decode_chlt.py <CHLT>` names the handler
	// and compares the word with the boot image at that PC.
	{cg_chlt[31:16], cg_chlt[15:0]},      // 4 CHLT  last CAGE halt: ir[31:16] | PC
	// CPUI = {instructions retired last frame >> 4, cycles the CPU waited on the
	// ROM port last frame >> 8}. A real 25 MHz 68EC020 retires roughly 70,000
	// instructions a frame (~1117 at >> 4).
	{insn_last, cstall_last},             // 3 CPUI  insn/frame>>4 | rom stall>>8
	// OVRN = {line-buffer overruns, frames with at least one}, both since the
	// last reset, saturating. An overrun is a scanline the renderer had not
	// finished in time: the previous line repeats and the game picture below it
	// shifts down one line for that frame.
	{ovr_lines, ovr_frames},              // 2 OVRN  overrun lines | frames with >= 1
	// RDUR = {last render pass >> 8, longest pass since reset >> 8}, in clk_sys
	// cycles from MOGO to the end of the pass, deferral included.
	{rdur_last, rdur_max},                // 1 RDUR  last pass>>8 | max pass>>8
	bc_insn_pc                            // 0 PC
};

wire [47:0] dbg_bits = {
	// Status blocks (layout at agt_video's dbg_bits port):
	//   [47:32]  rbk_data[31:16], the upper half of the post-download readback
	//            of ROM byte 4
	//   [31:16]  rbk_found_hw, the halfword index where the scan found 0x0664
	//   [15:0]   status bits, below
	// The ram/cram loopbacks (bits 8-11) never touch the ROM region: agt_sdram
	// puts ram at word base 0x100000 and cram at 0x140000, while ROM is at
	// 0x000000 and is written only by the download port.
	rbk_data[31:16],
	rbk_found_hw,
	rbk_ok,             // 15: post-download ROM readback == 0x00000664
	rbk_nonzero,        // 14: that readback is nonzero
	rdchk_n,            // 13-12: CPU ROM fetches checked (stops at 2)
	lb_r32_ok,          // 11: 32-bit (two-beat) readback correct
	lb_done,            // 10: loopback ran
	lb_odd_ok,          // 9
	lb_even_ok,         // 8

	dbg_heartbeat,      // 7
	dbg_cram_seen,      // 6
	dbg_latch_seen,     // 5
	dbg_insn_seen,      // 4
	cpu_core_rst_n,     // 3
	sdr_ready,          // 2
	dl_rom_loaded,      // 1
	rdchk_ok            // 0: first two ROM fetches match the image
};

// OBJ_SNAPSHOT = 0 removes the object-list snapshot arrays (MLAB) and their
// copy engine. The Debug page's Obj List switch (osd_obj_live, debug build)
// selects Snapshot or Live without a recompile; the release build uses the
// snapshot.
localparam bit OBJ_SNAPSHOT = 1'b1;
agt_demo_memories #(.OBJ_SNAPSHOT(OBJ_SNAPSHOT)) u_demo_mem
(
	.clk_sys(clk_sys),
	.rst_n(sys_rst_n),
	.video_rst_n(video_rst_n_sysclk),
	.pfram_addr(pfram_addr), .pfram_rd(pfram_rd),
	.pfram_data(pfram_data), .pfram_data_valid(pfram_data_valid),
	.tile_rom_addr(tile_rom_addr), .tile_rom_rd(tile_rom_rd && tile_src_demo),
	.tile_rom_data(demo_tile_data), .tile_rom_data_valid(demo_tile_valid),
	.alpharam_addr(alpharam_addr), .alpharam_rd(alpharam_rd),
	.alpharam_data(alpharam_data), .alpharam_data_valid(alpharam_data_valid),
	.cram_addr(cram_addr), .cram_data(cram_data), .color_latch(color_latch),
	.pen_addr_r(pen_addr_r), .pen_addr_g(pen_addr_g), .pen_addr_b(pen_addr_b),
	.pen_data_r(pen_data_r), .pen_data_g(pen_data_g), .pen_data_b(pen_data_b),
	.pal_fixture_en(1'b1),          // the demo palette, simulation only
	// Colour RAM writes fire on the rising edge of the CPU's cram write request
	// (cram_wr_start), where address and data are valid. At the ack
	// (cpu_cr_strobe) they are not reliably present any more.
	.cpu_cr_wr(cram_wr_start),
	.cpu_cr_addr(bc_cram_addr), .cpu_cr_data(bc_cram_wdata),
	.snoop_we(gv_snoop_we), .snoop_addr(gv_snoop_addr),
	.obj_rd_addr(obj_rd_addr), .obj_rd(obj_rd),
	.obj_rd_data(obj_rd_data), .obj_rd_valid(obj_rd_valid),
	// Snapshot the list on the pulse that starts the render: mogo_draw, the
	// DRAW MOGO edge itself (line 240, before the game tick starts at ~251), not
	// the deferred render start, which can be 22 lines later.
	.obj_snap_start(mogo_draw), .obj_snap_busy(obj_snap_busy),
	.obj_use_live(osd_obj_live),   // 0 = Snapshot (default), 1 = Live
	.dbg_ob_hf_set(dbg_ob_hf_set), .dbg_ob_wr(dbg_ob_wr),
	.dbg_ob_w0_pulse(ob_w0_pulse), .dbg_ob_hf_pulse(ob_hf_pulse),
	.snoop_be(gv_snoop_be), .snoop_wd(gv_snoop_wd),
	.tile_rom_rd3(1'b0), .tile_rom_data96()
);

wire [23:0] rgb;
wire        v_hsync, v_vsync, v_hblank, v_vblank, v_active;

// MO_VRAM_TARGETS = 1 drops the TMO buffer (T-MEK's layer; Primal Rage draws
// only MO) to save block RAM. vid_mo_alt_wr counts writes aimed at it: if it
// ever goes non-zero the sprite layer is missing something and this needs 2.
//
// MO_VRAM_DOUBLE = 1 double-buffers MO as MAME does: the game's FRAME bit
// (latch bit 29) picks the render target once per frame, so a pass that takes
// most of a frame draws into the back buffer instead of racing the raster.
// 0 is the single-buffer fallback, with no other change.
//
// The second buffer is ~160 M10K and fits only while the arrays pinned to
// M10K stay pinned (tools/pin_rams.py --check; six are in sys/, which a
// framework update overwrites). Short of block RAM, synthesis demotes
// unpinned arrays to registers and the fit fails on LABs, not memory;
// tools/check_ram_summary.py shows a demotion.
agt_video #(.MO_VRAM_TARGETS(1), .MO_VRAM_DOUBLE(1),
			.DBG_NSLOTS(12), .BUILD_DIGITS(9),           // 9-digit build stamp
			.LINE0_FETCH_LINE(LINE0_FETCH_LINE),
			.DBG_OVERLAY(DEBUG_BUILD),                   // the overlays and the
			.DBG_TAPS(DEBUG_BUILD)) u_video              // SignalTap taps: debug build only
(
	.line0_fetch_vblank(osd_line0_vblank),                // Line 0 Fetch: 0 = Late (line 259)
	.mo_alt_wr_count(vid_mo_alt_wr),
	.pixel_clk(clk_pix),
	.clk_sys(clk_sys),
	.rst_n(video_rst_n),
	.pfram_addr(pfram_addr), .pfram_rd(pfram_rd),
	.pfram_data(pfram_data), .pfram_data_valid(pfram_data_valid),
	.tile_rom_addr(tile_rom_addr), .tile_rom_rd(tile_rom_rd),
	.tile_rom_rd3(tile_rom_rd3), .tile_rom_data96(tile_rom_data96),
	.tile_rom_data(tile_rom_data), .tile_rom_data32(tile_rom_data32),
	.tile_rom_data_valid(tile_rom_data_valid),
	.charrom_addr(charrom_addr), .charrom_rd(charrom_rd),
	.charrom_data(charrom_data), .charrom_data32(charrom_data32),
	.charrom_data_valid(charrom_data_valid),
	.alpharam_addr(alpharam_addr), .alpharam_rd(alpharam_rd),
	.alpharam_data(alpharam_data), .alpharam_data_valid(alpharam_data_valid),
	.cram_addr(cram_addr), .cram_data(cram_data), .color_latch(color_latch),
	.pen_addr_r(pen_addr_r), .pen_addr_g(pen_addr_g), .pen_addr_b(pen_addr_b),
	.pen_data_r(pen_data_r), .pen_data_g(pen_data_g), .pen_data_b(pen_data_b),
	.rgb(rgb),
	.dbg_bits(dbg_bits),
	.dbg_words(dbg_words),
	// polarity follows the OSD labels: blocks default off, text default on
	.dbg_blocks_en(osd_dbg_blocks), .dbg_text_en(osd_dbg_text),
	.game_owns_screen(game_owns_screen),
	// DRAW-gated: checksum MOGOs must not start a render (MAME calls
	// sort_and_render() only for COMMAND_DRAW)
	.mogo_pulse(mogo_draw), .rnd_wr_valid_dbg(vid_rnd_wr_valid),
	.obj_rd_addr(obj_rd_addr), .obj_rd(obj_rd),
	.obj_rd_data(obj_rd_data), .obj_rd_valid(obj_rd_valid),
	.dbg_obj_w0(dbg_obj_w0), .dbg_obj_w4(dbg_obj_w4),
	.dbg_obj_valid(dbg_obj_valid), .dbg_obj_rejects(dbg_obj_rejects),
	.dbg_obj_starts(dbg_obj_starts), .dbg_obj_full(dbg_obj_full),
	.dbg_mo_cram(dbg_mo_cram), .dbg_mo_latch(dbg_mo_latch), .dbg_mo_rgb(dbg_mo_rgb),
	.dbg_mo_hits(dbg_mo_hits), .dbg_mo_cra(dbg_mo_cra),
	.dbg_mo_nz(dbg_mo_nz),
	.dbg_drop_alt(dbg_drop_alt), .dbg_drop_erase(dbg_drop_erase),
	.dbg_drop_oobx(dbg_drop_oobx), .dbg_drop_ooby(dbg_drop_ooby),
	.dbg_wr_xmin(dbg_wr_xmin), .dbg_wr_xmax(dbg_wr_xmax),
	.dbg_wild_code(dbg_wild_code), .dbg_wild_scale(dbg_wild_scale),
	.dbg_wild_draw_x(dbg_wild_draw_x), .dbg_wild_width(dbg_wild_width),
	.dbg_wild_hdr(dbg_wild_hdr), .dbg_wild_mwidth(dbg_wild_mwidth),
	.dbg_blk_cra(dbg_blk_cra), .dbg_blk_cram(dbg_blk_cram), .dbg_blk_latch(dbg_blk_latch),
	.probe_x(PROBE_X), .probe_y(PROBE_Y),
	.dbg_prb_mo(dbg_prb_mo), .dbg_prb_pf(dbg_prb_pf), .dbg_prb_cra(dbg_prb_cra), .dbg_prb_cram(dbg_prb_cram),
	.dbg_start_lost(dbg_start_lost), .dbg_pass_done(dbg_pass_done),
	.dbg_pc_hits(dbg_pc_hits), .dbg_pc_misses(dbg_pc_misses),
	.dbg_big_x(dbg_big_x), .dbg_big_y(dbg_big_y),
	.dbg_sml_x(dbg_sml_x), .dbg_sml_y(dbg_sml_y),
	.dbg_big_code(dbg_big_code), .dbg_big_width(dbg_big_width),
	.dbg_big_w0(dbg_big_w0), .dbg_big_w1(dbg_big_w1),
	.dbg_hf_live(dbg_hf_live), .dbg_hf_first(dbg_hf_first), .dbg_hf_w0(dbg_hf_w0),
	.dbg_hflip_cnt(dbg_hflip_cnt), .dbg_obj_cnt(dbg_obj_cnt),
	.dbg_mogo_deferred(dbg_mogo_deferred),
	.dbg_wr_behind(dbg_wr_behind), .dbg_wr_ahead(dbg_wr_ahead),
	.dbg_erase_sweeps(dbg_erase_sweeps),
	.mo_erase_full_frame(osd_mo_erase_full),              // MO Erase: 0 = Span, 1 = Full frame
	.dbg_obj_zscale(dbg_obj_zscale), .dbg_obj_offscr(dbg_obj_offscr),
	.dbg_scale_or(dbg_scale_or), .dbg_scale_and(dbg_scale_and),
	.dbg_obj_lastidx(dbg_obj_lastidx),
	.dbg_obj_examined(dbg_obj_examined), .dbg_obj_emitted(dbg_obj_emitted),
	.dbg_obj_examined_pf(dbg_obj_examined_pf), .dbg_obj_emitted_pf(dbg_obj_emitted_pf),
	.dbg_stage_ot(dbg_stage_ot), .dbg_stage_bl(dbg_stage_bl),
	.dbg_blit_meas(dbg_blit_meas),
	.dbg_blit_wait(dbg_blit_wait), .dbg_blit_emit(dbg_blit_emit),
	.dbg_st_obj(dbg_st_obj), .dbg_st_tbl(dbg_st_tbl), .dbg_st_blt(dbg_st_blt),
	.dbg_mo_sel(vid_mo_sel),
	.dbg_render_busy(vid_render_busy), .dbg_in_vblank(vid_in_vblank),
	.dbg_line_worst(vid_line_worst), .dbg_scroll_worst(vid_scroll_worst),
	.dbg_alpha_worst(vid_alpha_worst), .dbg_render_worst(vid_render_worst),
	.dbg_ovr_where(vid_ovr_where), .dbg_ovr_pass(vid_ovr_pass),
	.mo_frame_select(frame_bit),
	.mo_erase_frame(mo_erase_frame_sel),
	.mo_list_snap_busy(obj_snap_busy),
	.mo_erase_pulse(erase_req_now | vblank_erase_pulse),  // same cycle as mo_ctrl_wr
	.mo_erase_to_bottom(vblank_rise_pulse),               // every vblank; resets the watermark
	.mo_ctrl_wr(mo_ctrl_changed),                         // only when the bits change
	.rle_rom_waddr(rle_rom_waddr), .rle_rom_rd(rle_rom_rd),
	.rle_rom_data(rle_rom_data), .rle_rom_data_valid(rle_rom_data_valid),
	.dbg_xscroll(vid_xscroll), .dbg_yscroll(vid_yscroll),
	.dbg_ss_done(vid_ss_done), .dbg_ss_line(vid_ss_line),
	.dbg_vcount(vid_vcount),
	.dbg_build_id(dbg_build_id),
	.hsync(v_hsync), .vsync(v_vsync),
	.hblank(v_hblank), .vblank(v_vblank),
	.pixel_active(v_active),
	.render_overrun(render_overrun),
	.line_late(vid_line_late)
);

assign CLK_VIDEO = clk_pix;
assign CE_PIXEL = 1'b1;

assign VGA_DE = v_active;
assign VGA_HS = v_hsync;
assign VGA_VS = v_vsync;
// Blank the game picture until the game owns the screen. The demo scene is
// preloaded into the video memories, so without this a test pattern would
// show through the whole boot (the RAM march test alone runs for seconds).
// Gating the output needs no new power-on state, where compiling the demo
// data out would need one for pfram, the colorram init table and their
// consumers.
//
// game_owns_screen latches on the game's first colorram write, early in boot
// before anything is drawn, and holds until reset. It blanks for longer than
// intended only if the game never writes colorram, i.e. the core is not
// booting, which STAT already reports. A running game is never blanked.
reg game_owns_screen;
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n)          game_owns_screen <= 1'b0;
	else if (cpu_cr_strobe)  game_owns_screen <= 1'b1;
end

// The blanking is inside agt_video, on the game picture only: gating the
// final output here would also blank the debug overlays, which are what is
// needed while the screen is otherwise blank.
assign VGA_R  = v_active ? rgb[23:16] : 8'd0;
assign VGA_G  = v_active ? rgb[15:8]  : 8'd0;
assign VGA_B  = v_active ? rgb[7:0]   : 8'd0;

// Atari GT main board: 68EC020, memory map and interrupt generation
// (agt_board_core). tb_freerun boots the real Primal Rage ROM through it,
// past the self-test to the game's main loop, with vblank and 250 Hz
// interrupts at the hardware's rates. Interrupts come from the video block's
// vblank (pixel domain; the CDC is inside agt_irq_gen).
//
// ROM and main RAM live in SDRAM (agt_sdram), loaded through hps_io ioctl
// and agt_rom_download (the .mra maincpu region); tb_freerun_sdram runs the
// real image through this path against the golden model. The CPU is held in
// reset until the ROM is loaded and the SDRAM is ready.
//
// Colour RAM: the raw 512 KB window is backed by SDRAM at 0x280000 for
// readback (the self-test reads it), and every cram write is also decoded
// into the video side's agt_colorram (cram/color_latch/pens) through
// agt_demo_memories' write-through port, on the same clock. The memmap taps
// protection at accept time, before dispatch.
//
// Sound: u_cage (agt_cage, instantiated below beside agt_rom_download, whose
// stream and SDRAM ready it takes) is the CAGE board. It keeps
// agt_cage_stub's 68020 contract and overlay counters, and the stub itself
// still answers when a download brought no DSP boot table (primrage20,
// T-MEK). Its wires to the board core and the overlay:
wire        cage_ack, cage_irq;
wire [15:0] cg_last_cmd, cg_control, cg_ctrl_wr, cg_main_rd;   // overlay counters
wire [31:0] cage_rdata;

// Clocking: cpu_clk = clk_sys = 57.27272 MHz. The real 68EC020 runs at
// 25 MHz (MAME: 50_MHz_XTAL/2), so the soft core is deliberately clocked at
// 229% of it: it takes far more cycles per instruction than the real chip
// (which has a pipeline and a 256-byte instruction cache). Game logic is
// frame-paced (it waits on vblank and the frame counter), so a faster CPU
// buys back throughput rather than running the game fast. CPUI shows
// instructions retired per frame, against ~70,000 for a real 25 MHz part.
//
// The video clocks are exact: clk_pix = 7.159090 MHz = 14.318181 MHz / 2
// (MAME's set_raw), and 456 x 262 gives 59.922748 Hz against MAME's
// 59.922743 Hz, the difference being rounding in the PLL's declared value.

wire [20:0] bc_rom_addr;
wire        bc_rom_req;
wire [18:0] bc_ram_addr;
wire [3:0]  bc_ram_be;
wire        bc_ram_we, bc_ram_req;
wire [31:0] bc_ram_wdata;
wire [18:0] bc_cram_addr;
wire        bc_cram_we, bc_cram_req;
wire [15:0] bc_cram_wdata;
wire        bc_cage_req, bc_cage_we;
wire [3:0]  bc_cage_be;
wire [31:0] bc_cage_wdata;
wire [31:0] bc_led, bc_latch;
wire        bc_latch_wr;
wire        bc_insn_done;
wire        bc_video_int_set, bc_scanline_int_set;
wire        bc_vid_ack, bc_scan_ack;
wire [31:0] bc_insn_pc;      // overlay slot 0: where the CPU is
// Why the CPU halted: m68020_core halts visibly on an opcode it does not
// implement, rather than guess, and latches the opcode and PC (overlay slot
// 10, HALT).
wire        bc_unimpl;
// The CAGE board's reset is latch bit 21 at 0xE08000: /XRESET in atarigt.cpp,
// whose latch_w calls m_cage->reset_w(BIT(~data,21)). reset_w asserts on 1,
// so bit 21 low holds the board in reset. It also clears the control
// register, which the stub already does on (control & 3) == 0, so holding
// rst_n low is equivalent. The latch resets to 0, so the board starts in
// reset until the game releases it, as on the hardware.
//
// MAME's CAGE does not listen to the 68020 RESET instruction, so
// cpu_reset_out (the right signal for a peripheral reset) stays unconnected.
wire        cage_rst_n = sys_rst_n && bc_latch[21];
wire [15:0] bc_unimpl_opcode;
wire [31:0] bc_unimpl_pc;     // should equal bc_insn_pc
wire [15:0] bc_prot_hits;

wire        bc_rom_ack, bc_ram_ack;
wire [31:0] bc_rom_rdata, bc_ram_rdata;
wire        dl_rom_loaded, sdr_ready;   // sdr_dl_busy declared above, at hps_io
wire        bc_cram_ack;
wire [15:0] bc_cram_rdata;
// Game and control-panel selection from the .mra (rom index 1).
//   dl_game_id  0 = T-MEK, 1 = Primal Rage (carried, not consumed)
//   dl_panel_id 0 = dedicated start button (primrage, v2.3)
//               1 = an attack button doubles as start (primrageo, primrage20)
// dl_panel_id selects the controller map (agt_input_map). An .mra without
// the bytes gets the defaults (1, 0).
wire [7:0]  dl_game_id, dl_panel_id;

// The download decoder resets on the hard reset only. sys_rst_n also takes
// the OSD Reset (status[0]) and the menu button; if those cleared
// rom_loaded, a soft reset would hold the CPU for good (the ROM survives in
// refreshed SDRAM, the flag would not). rom_loaded clears when a new
// download starts and sets again when it completes.
agt_rom_download u_dl
(
	.clk(clk_sys), .rst_n(~RESET),
	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index[7:0]),
	.maincpu_wr(dl_maincpu_wr), .maincpu_addr(dl_maincpu_addr),
	.tiles_wr(dl_tiles_wr), .tiles_addr(dl_tiles_addr),
	.chars_wr(dl_chars_wr), .chars_addr(dl_chars_addr),
	.rle_wr(dl_rle_wr), .rle_addr(dl_rle_addr),
	.cageboot_wr(dl_cb_wr), .cageboot_addr(dl_cb_addr),
	.cage_wr(dl_cage_wr), .cage_addr(dl_cage_addr),
	.proms_wr(), .proms_addr(),
	.rom_data(dl_rom_data),
	// rom_loading is high for an index-0 download only (hps_io sets the index
	// by its own command before the transfer starts), so its rise starts
	// agt_cage_ramload once per ROM load and never on an NVRAM one.
	.rom_loading(dl_rom_loading), .rom_loaded(dl_rom_loaded),
	.game_id(dl_game_id), .panel_id(dl_panel_id)
);

// Controller map. Combinational; dl_panel_id is static after the download
// (agt_rom_download resets on ~RESET only, so a soft reset keeps it).
agt_input_map u_input_map
(
	.joy0(joystick_0), .joy1(joystick_1),
	.panel_id(dl_panel_id),
	.p1_p2_port(p1_p2_port), .coin_in(coin_in)
);

// CAGE sound board. agt_cage is the whole board: boot-table parser and
// ramload, IRAM transfer, port mux, SDRAM bridge, mailbox crossing, release,
// the TMS320C31 core on clk_dsp with its bus and cache, and the 68020's comm,
// with agt_cage_stub inside for a download that brought no boot table.
// tb_cage drives this module through these same ports.
//
// Every name below must be declared above this line:
//   clk_dsp     the PLL's outclk_3, 630/17 = 37.0588 MHz
//   dl_rst_n    ~RESET & pll_lk: power-on only, like every crossing
//   cage_rst_n  sys_rst_n && the latch's sound-reset bit 21
//   dl_*        the decoder's cage-boot region stream
agt_cage #(
	.BASE(SDR_BASE_CAGERAM),
	.SND_BASE(SDR_BASE_CAGE)                          // the sound bank
) u_cage (
	.clk_sys(clk_sys), .clk_dsp(clk_dsp),
	.dl_rst_n(dl_rst_n), .cage_rst_n(cage_rst_n),
	.dl_active(dl_rom_loading),
	.dl_first(dl_cb_wr && (dl_cb_addr == 19'd0)),
	.dl_valid(dl_cb_wr), .dl_byte(dl_rom_data),
	.dl_wait(cg_ld_wait), .sdr_ready(sdr_ready),
	.p_addr(cg_p_addr), .p_we(cg_p_we), .p_line(cg_p_line), .p_wdata(cg_p_wdata),
	.p_req(cg_p_req), .p_ack(cg_p_ack), .p_rdata(cg_p_rdata),
	.req(bc_cage_req), .we(bc_cage_we), .be(bc_cage_be),
	.wdata(bc_cage_wdata), .ack(cage_ack), .rdata(cage_rdata),
	.irq(cage_irq),
	.dbg_last_cmd(cg_last_cmd), .dbg_control(cg_control),
	.dbg_ctrl_writes(cg_ctrl_wr), .dbg_main_reads(cg_main_rd),
	// the serial port's words, played in the DSP's own time
	.vblank(vid_in_vblank),
	.audio_l(cg_audio_l), .audio_r(cg_audio_r),
	.dsp_present(cg_dsp_present),
	.w_cgrm(cg_ld_witness), .w_rlse(cg_rlse),
	.w_chlt(cg_chlt),
	.w_sbnk(cg_sbnk),
	.w_dacw(cg_dacw),
	.w_dstl(cg_dstl)
);

// The CAGE board's DACs are the core's audio: agt_cage_dac's output on
// clk_sys, signed 16-bit, left = (ch1 + ch2) / 2 and right = (ch0 + ch3) / 2
// of the serial port's four channels, as MAME's atarigt_stereo. It changes at
// the DSP's own rate, so pitch and tempo follow the DSP's speed (see DACW).
assign AUDIO_S = 1'b1;
assign AUDIO_L = cg_audio_l;
assign AUDIO_R = cg_audio_r;

agt_sdram u_sdram
(
	// Reset domain: ~RESET only, like agt_rom_download. sys_rst_n also takes
	// status[0] and buttons[1], and a warm reset would re-run the controller's
	// whole INIT (precharge all, refresh, load mode register) after the ROM was
	// written. The SDRAM holds the ROM, so it must survive warm resets just as
	// rom_loaded does.
	.clk(clk_sys), .rst_n(~RESET), .ready(sdr_ready),
	.rd_delay(osd_rd_delay), .wr_shift(osd_wr_hold == 2'd0 ? 2'd1 : osd_wr_hold == 2'd1 ? 2'd2 : 2'd0),
	.rom_addr(rom_mux_addr), .rom_req(rom_mux_req),
	.rom_ack(bc_rom_ack), .rom_rdata(bc_rom_rdata),
	.ram_addr(ram_mux_addr), .ram_be(ram_mux_be), .ram_we(ram_mux_we),
	.ram_wdata(ram_mux_wdata), .ram_req(ram_mux_req),
	.ram_ack(bc_ram_ack), .ram_rdata(bc_ram_rdata),
	.cram_addr(cram_mux_addr), .cram_we(cram_mux_we),
	.cram_wdata(cram_mux_wdata), .cram_req(cram_mux_req),
	.cram_ack(bc_cram_ack), .cram_rdata(bc_cram_rdata),
	// CAGE port: agt_cage's port mux, shared by agt_cage_ramload (zero, load and
	// check, once per download) and the DSP's memory bridge and sound bank.
	.cage_addr(cg_p_addr), .cage_we(cg_p_we), .cage_line(cg_p_line), .cage_wdata(cg_p_wdata),
	.cage_req(cg_p_req), .cage_ack(cg_p_ack), .cage_rdata(cg_p_rdata),
	.tile_first(sdr_tile_first),
	.tile_addr(sdr_tile_addr), .tile_req(sdr_tile_req),
	.tile_ack(sdr_tile_ack), .tile_data(sdr_tile_port_data), .tile_data32(sdr_tile_port_data32),
	// the burst flag travels with the granted request
	.tile_burst3(sdr_tile_burst3), .tile_data96(sdr_tile_burst_data96),
	.dbg_tile_acc(sdr_tile_acc), .dbg_tile_pghit(sdr_tile_pghit),
	.dbg_tile_bankopen(sdr_tile_bankopen),
	.dl_wr(dl_any_wr),
	.dl_addr(dl_any_addr),
	// DL port mux: after dl_rom_loaded and before rbk_done the ROM-region
	// loopback owns it. The download is finished by then, so there is no
	// contention, and rlb_owns is false during the download itself.
	.dl_data(dl_rom_data), .dl_busy(sdr_dl_busy),
	.SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQ(SDRAM_DQ),
	.SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
	.SDRAM_CKE(SDRAM_CKE)
);

// Work-RAM clear. Declared at the ram mux, driven here where rbk_done
// exists. Runs once per sys_rst_n release, so every warm reset clears the
// work RAM again: 131,072 32-bit words at roughly ten cycles each, about
// 1.3 M clk_sys cycles (~23 ms).
//
// The three-state req/ack handshake is deliberate: agt_sdram takes req as a
// level held until ack and needs it to fall before the next request. Waiting
// for !bc_ram_ack in state 2 ensures that by construction, not by timing.
always @(posedge clk_sys or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		ram_clr_addr <= 19'd0; ram_clr_st <= 2'd0;
		ram_clr_req  <= 1'b0;  ram_clr_done <= 1'b0;
		ram_clr_cycles <= 32'd0;
	end else if (!ram_clr_done && lb_done && rbk_done && sdr_ready) begin
		if (ram_clr_cycles != 32'hFFFFFFFF)
			ram_clr_cycles <= ram_clr_cycles + 32'd1;
		case (ram_clr_st)
			2'd0: begin ram_clr_req <= 1'b1; ram_clr_st <= 2'd1; end
			2'd1: if (bc_ram_ack) begin ram_clr_req <= 1'b0; ram_clr_st <= 2'd2; end
			2'd2: if (!bc_ram_ack) begin
					  if (ram_clr_addr == 19'h7FFFC) ram_clr_done <= 1'b1;
					  else ram_clr_addr <= ram_clr_addr + 19'd4;
					  ram_clr_st <= 2'd0;
				  end
			default: ram_clr_st <= 2'd0;
		endcase
	end
end

// On-chip work RAM window and its clear. clr_run is the ram_clr FSM's own
// run condition, so the window is zeroed on every reset release, as the
// SDRAM window is. The boot code tells warm from cold boot by $FFFF8026 ==
// 0xC0EDBABE, which lives in this BRAM: a window that survived a reset
// would turn a cold boot into a warm one. Bench: tb/tb_wram_router.sv.
agt_wram_router #(.ENABLE(WRAM_ONCHIP), .WINDOW(4'hF)) u_wram (
	.clk(clk_sys), .rst_n(sys_rst_n),
	.cpu_req(bc_ram_req), .cpu_we(bc_ram_we), .cpu_addr(bc_ram_addr),
	.cpu_be(bc_ram_be), .cpu_wdata(bc_ram_wdata),
	.cpu_ack(cpu_ram_ack), .cpu_rdata(cpu_ram_rdata),
	.sdr_req(wram_sdr_req), .sdr_ack(bc_ram_ack), .sdr_rdata(bc_ram_rdata),
	.clr_run(lb_done && rbk_done && sdr_ready), .clr_done(wram_clr_done)
);

// The CPU is held until the ROM is in place, the controller has initialized,
// the ROM readback has finished (it owns the rom port until rbk_done, and a
// CPU fetch overlapping it would corrupt both), and both work-RAM clears are
// done (the CPU must not fetch from RAM mid-clear).
wire cpu_core_rst_n = sys_rst_n & dl_rom_loaded & sdr_ready & rbk_done & ram_clr_done
					& wram_clr_done;

// game -> video tilemap snoop (see agt_demo_memories for the windows)
wire [15:0] vid_mo_alt_wr;      // writes aimed at the dropped TMO buffer
wire        gv_snoop_we;
wire [13:0] gv_snoop_addr;
wire [3:0]  gv_snoop_be;
wire [31:0] gv_snoop_wd;

// MO command register and checksum answer. The game's MOGO is either DRAW or
// CHECKSUM (0xd7a200). mogo_draw is the DRAW-gated pulse the video sees;
// checksum MOGOs run the answer engine, which writes the download-time chunk
// sums back into the objlist window through the memmap's idle-cycle service
// port.
wire        chkw_req, chkw_ack, chk_busy, mogo_draw;
wire [10:0] chkw_half;
wire [15:0] chkw_data;

agt_mo_checksum u_mo_checksum (
	.clk(clk_sys), .rst_n(cpu_core_rst_n), .dl_rst_n(dl_rst_n),
	.dl_rle_wr(dl_rle_wr), .dl_rle_addr(dl_rle_addr),
	.dl_rle_data(dl_rom_data),
	.snoop_we(gv_snoop_we), .snoop_addr(gv_snoop_addr),
	.snoop_be(gv_snoop_be), .snoop_wd(gv_snoop_wd),
	.mogo_pulse(mogo_pulse), .mogo_draw(mogo_draw),
	.chkw_req(chkw_req), .chkw_half(chkw_half),
	.chkw_data(chkw_data), .chkw_ack(chkw_ack),
	.chk_busy(chk_busy)
);

// EEPROM persistence: the 28C16's settings, high scores and bookkeeping
// survive a power cycle. Restore arrives as a download on the .mra's nvram
// index, the save is a read stream, and both reach the EEPROM through the
// memmap's single-writer service ports rather than a second port on the
// arrays.
wire        nvw_req, nvw_ack, nvr_req, nvr_valid;
wire [10:0] nvw_addr, nvr_addr;
wire [7:0]  nvw_data, nvr_data;
wire        eeprom_wr_evt, eeprom_rd_evt;

// Two resets. The capture side is in dl_rst_n because the framework sends
// index 2 while status[0] holds the core in reset (user_io.cpp:1459 -> 1576
// -> 1708); the replay side is in the board domain and holds the CPU via
// cpu_hold until it has finished.
wire nv_restore_done;
agt_nvram u_nvram (
	.clk(clk_sys), .rst_n(cpu_core_rst_n), .dl_rst_n(dl_rst_n),
	.ioctl_download(ioctl_download), .ioctl_upload(ioctl_upload),
	.ioctl_wr(ioctl_wr), .ioctl_rd(ioctl_rd),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_din(ioctl_din), .ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index), .nv_wait(nv_wait),
	.nvw_req(nvw_req), .nvw_addr(nvw_addr), .nvw_data(nvw_data),
	.nvw_ack(nvw_ack),
	.nvr_req(nvr_req), .nvr_addr(nvr_addr), .nvr_data(nvr_data),
	.nvr_valid(nvr_valid),
	.eeprom_wr_evt(eeprom_wr_evt), .nv_dirty(nv_dirty),
	.nv_restore_done(nv_restore_done)
);

agt_board_core u_board
(
	.cpu_clk(clk_sys), .cpu_rst_n(cpu_core_rst_n),
	.bus_ifetch(bc_ifetch), .bus_ifetch_hit(bc_ifetch_hit),
	.cpu_hold(~nv_restore_done),   // CPU waits for the EEPROM replay
	.pix_clk(clk_pix), .pix_rst_n(video_rst_n), .vblank_in(v_vblank),

	.rom_addr(bc_rom_addr), .rom_req(bc_rom_req),
	.rom_ack(bc_rom_ack), .rom_rdata(bc_rom_rdata),
	.ram_addr(bc_ram_addr), .ram_be(bc_ram_be), .ram_we(bc_ram_we),
	.ram_wdata(bc_ram_wdata), .ram_req(bc_ram_req),
	.ram_ack(cpu_ram_ack), .ram_rdata(cpu_ram_rdata),       // routed: on-chip window or SDRAM
	.cram_addr(bc_cram_addr), .cram_we(bc_cram_we),
	.cram_wdata(bc_cram_wdata), .cram_req(bc_cram_req),
	.cram_ack(bc_cram_ack), .cram_rdata(bc_cram_rdata),
	.cage_req(bc_cage_req), .cage_we(bc_cage_we), .cage_be(bc_cage_be),
	.cage_wdata(bc_cage_wdata), .cage_ack(cage_ack), .cage_rdata(cage_rdata),
	.cage_irq(cage_irq),

	.p1_p2_port(p1_p2_port), .coin_in(coin_in),
	// OSD "Service Mode" -> the real SELFTEST line (active low). Held, not
	// pulsed: the game samples the level, so the menu stays reachable for as
	// long as the option is On.
	.service_n(~status[12]),
	.dbg_video_int_set(bc_video_int_set),
	.dbg_scanline_int_set(bc_scanline_int_set),
	.dbg_vid_ack(bc_vid_ack), .dbg_scan_ack(bc_scan_ack),
	.snoop_we(gv_snoop_we), .snoop_addr(gv_snoop_addr),
	.snoop_be(gv_snoop_be), .snoop_wd(gv_snoop_wd),
	.led_value(bc_led), .latch_value(bc_latch), .latch_wr(bc_latch_wr),
	.prot_hit_count(bc_prot_hits),
	.chkw_req(chkw_req), .chkw_half(chkw_half),
	.chkw_data(chkw_data), .chkw_ack(chkw_ack),
	.nvw_req(nvw_req), .nvw_addr(nvw_addr), .nvw_data(nvw_data),
	.nvw_ack(nvw_ack),
	.nvr_req(nvr_req), .nvr_addr(nvr_addr), .nvr_data(nvr_data),
	.nvr_valid(nvr_valid), .eeprom_wr_evt(eeprom_wr_evt),
	.eeprom_rd_evt(eeprom_rd_evt),

	// observability; the halt reason feeds overlay slot 10 (HALT)
	.insn_done(bc_insn_done), .insn_pc(bc_insn_pc), .sr_out(),
	.unimplemented(bc_unimpl), .unimpl_opcode(bc_unimpl_opcode),
	.unimpl_pc(bc_unimpl_pc),
	.cpu_reset_out()          // peripheral reset; unused by this board
);

reg  [26:0] act_cnt;
always @(posedge clk_sys) act_cnt <= act_cnt + 1'd1;
assign LED_USER    = act_cnt[26]  ? act_cnt[25:18]  > act_cnt[7:0]  : act_cnt[25:18]  <= act_cnt[7:0];

endmodule
