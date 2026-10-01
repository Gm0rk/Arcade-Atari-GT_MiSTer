// agt_sdram.sv -- SDR SDRAM controller for the Atari GT board core
//
// Serves the board core's ROM, main-RAM and colorram ports, the video tile
// port, the CAGE port and the MRA download write stream from the MiSTer
// SDRAM (16-bit SDR, AS4C32M16 class: 4 banks x 8192 rows x 1024 columns).
//
// Memory map (SDRAM byte addresses):
//   0x0000000 - 0x01FFFFF   maincpu program ROM (2 MB, written by download)
//   0x0200000 - 0x027FFFF   main RAM (512 KB)
//   0x0280000 - 0x02FFFFF   colorram (512 KB)
//   The tile, dl and cage ports take full byte addresses; the caller adds
//   the region base (tiles 0x300000, chars 0x600000, cageram 0x2B20000).
//
// Clock: the CPU clock, 57.27 MHz (17.46 ns): tRCD and tRP are 1 cycle,
// tRFC 4, CL2. SDRAM_CLK comes from a phase-shifted PLL output at the top
// level. Mode: CL2, burst length 2, sequential, single-location writes.
//   * Reads fetch a word pair from an even column (big-endian: beat 0 =
//     bits [31:16]) and leave the row open; a read that hits the open row
//     skips ACTIVE and tRCD.
//   * Writes always ACTIVE, then issue one WRITE per 16-bit word with
//     auto-precharge on the last, holding data and DQM across a window
//     around each command (wr_shift).
//   * A cage line (`cage_line`, four 32-bit words) is one access: four READs
//     on the open row, or one ACTIVE and eight WRITEs. A line write steps
//     aside after any 32-bit word when the video or the CPU is waiting: that
//     WRITE auto-precharges, up to two rom/tile accesses go, and the same
//     request resumes at its next word (`cl_resume`, `cage_yield`), so no
//     other port waits behind a whole line write (43 cycles at wr_shift 1).
//   * DQM byte masking does not work on this board, so no write relies on
//     it: partial RAM writes are read-modify-write and download bytes are
//     paired into full words.
//   * Refresh: 8192 rows / 64 ms = 7.8125 us per row -> a 447-cycle timer,
//     taken at S_IDLE when due, after precharging all banks if a row is open.
//
// Priority at S_IDLE: refresh > second half of an RMW > download > a late
// line's tile fetch (`tile_first`) > aged cage (CAGE_AGE) > rom/tile
// round-robin > ram > cram > cage. During a download the top level holds
// the CPU, so the download has the chip to itself.
//
// Data capture: each SDRAM_DQ pin feeds exactly one register, `dq_q`, kept
// in the pin's I/O element (FAST_INPUT_REGISTER in the .qsf). The SDC
// false-paths every port, so a fabric register sampling the pins directly
// gets an untimed, fit-dependent route. Every capture takes `dq_q` one edge
// after the pins were sampled; the `*_served` gates still close in the
// sampling state, so S_IDLE cannot re-grant a request whose ack is pending.
//
// Benches: tb_sdram (init, each port, byte masks, refresh under load,
// download then readback) and tb_freerun_sdram (the free-run with this
// controller and sim/sdram_chip_model.sv in place of BRAM, ROM loaded
// through the real download path, pc-trace compared against the golden
// model: SDRAM wait states must change nothing but wall time).
`default_nettype none

module agt_sdram #(
	// The chip needs ~200 us of stable clock before init (11,454 cycles at
	// 57.27 MHz). Shrunk in simulation.
	parameter int INIT_WAIT   = 11454,
	parameter int REFRESH_DIV = 447,
	// Cycles a cage request may wait before it goes ahead of rom and tile (see
	// `cage_urgent`); strict lowest priority starves it under load.
	parameter int CAGE_AGE    = 64
) (
	input  wire         clk,
	input  wire         rst_n,
	output logic        ready,          // init complete; ports served

	// Extra read-wait cycles between the CL2 pipeline and data capture, set at
	// runtime (OSD "SDRAM Read Delay"). Beat alignment on silicon depends on
	// clock phase, trace delays and tAC, which simulation does not model; find
	// the value on hardware with the top level's read self-check (overlay bit
	// 0). Simulation runs 0.
	input  wire  [1:0]  rd_delay,

	// Write hold width (OSD "SDRAM Write Shift"): widens the window in which
	// data and DQM are held around each write command by wr_shift cycles on
	// each side, so the chip's exact latching edge does not matter. Simulation
	// runs 0.
	input  wire  [1:0]  wr_shift,   // extra hold cycles each side of a write

	// ROM read port (board core)
	input  wire  [20:0] rom_addr,       // byte address, [1:0]=00
	input  wire         rom_req,
	output logic        rom_ack,        // 1-cycle pulse with data
	output logic [31:0] rom_rdata,

	// main RAM port (board core)
	input  wire  [18:0] ram_addr,       // byte address, [1:0]=00
	input  wire  [3:0]  ram_be,         // [3]=byte at addr (bits 31:24)
	input  wire         ram_we,
	input  wire  [31:0] ram_wdata,
	input  wire         ram_req,
	output logic        ram_ack,
	output logic [31:0] ram_rdata,

	// colorram port (board core cram): 16-bit only, the memmap turns byte
	// writes into RMW beats. Backs the raw 512 KB window for readback; the
	// video-side derived state is written through at the top level.
	input  wire  [18:0] cram_addr,      // byte address, [0]=0
	input  wire         cram_we,
	input  wire  [15:0] cram_wdata,
	input  wire         cram_req,
	output logic        cram_ack,
	output logic [15:0] cram_rdata,

	// CAGE port: everything the CAGE sound board keeps in SDRAM (cageram behind
	// agt_cage_dcache's miss side, the boot loader's writes into it, later the
	// sound bank).
	//   * Full byte address, like tile and dl: the caller adds the region base
	//     (cageram at 0x2B20000, after the sound data).
	//   * Whole 32-bit words only: no byte enables, never an RMW.
	//   * Lowest priority at S_IDLE until it has waited CAGE_AGE cycles, then
	//     ahead of rom and tile. Aging bounds the wait and costs the video at
	//     most one cage access per aged request (tb_sdram_cage).
	//   * Protocol: hold req until ack (a one-cycle pulse), then drop it for at
	//     least one cycle. `cage_served` blocks re-acceptance until req falls,
	//     as on the other ports, so a master that re-raises req right after ack
	//     without a low cycle is never served again. agt_c31 does exactly that,
	//     so it must not drive this port directly; the crossing in front of
	//     this port is the master, and it drops req.
	//   * Inputs are compared `=== 1'b1` so benches that leave the port
	//     unconnected see it idle rather than X.
	//   * `cage_line` = 1: a line, four words at a 16-byte-aligned address,
	//     in one access and one ack: four READs on the row (eight beats), or
	//     one ACTIVE and eight WRITEs. Word k is bits [32k +: 32] of
	//     cage_wdata / cage_rdata. `cage_line` = 0: one word, bits [31:0].
	input  wire  [25:0] cage_addr,      // byte address, [1:0]=00 ([3:0] a line)
	input  wire         cage_we,
	input  wire         cage_line,
	input  wire  [127:0] cage_wdata,
	input  wire         cage_req,
	output logic        cage_ack,       // 1-cycle pulse; with data on reads
	output logic [127:0] cage_rdata,

	// download byte-write port (agt_rom_download)
	input  wire         dl_wr,          // 1-cycle strobe per byte
	// Tile read port: byte-granular reads for the video tile fetcher. Like dl,
	// it takes a full SDRAM byte address and the caller adds the region base
	// (tiles at 0x300000, chars at 0x600000).
	// The tile request is the line renderer's and its line is late (the top
	// level: agt_tile_arb's sdr_video and agt_video's line_late): it goes
	// ahead of an aged cage request and of rom. Compared `=== 1'b1`, so an
	// unconnected input changes nothing.
	input  wire         tile_first,
	input  wire  [25:0] tile_addr,
	input  wire         tile_req,
	output logic        tile_ack,
	output logic [7:0]  tile_data,
	// The whole word pair of a tile read, so a decoder gets a full tile-row
	// plane in one access.
	output logic [31:0] tile_data32,
	// Three-plane burst: a tile's L/M/H planes are twelve consecutive bytes
	// (+0/+4/+8). tile_burst3 reads all three with three READs on one ACTIVATE
	// without returning to arbitration, so no other master can open another row
	// in between. tile_data96 returns {L,M,H} as three 32-bit words.
	// Single-plane reads still work.
	input  wire         tile_burst3,
	output logic [95:0] tile_data96,
	// Tile-port page-hit witness: one-cycle pulses at the S_ACT commit of each
	// tile access, dbg_tile_acc per access and dbg_tile_pghit when its row was
	// already open. The top level counts them (PGHT).
	output logic        dbg_tile_acc,
	output logic        dbg_tile_pghit,
	// dbg_tile_bankopen: the bank had some row open, whichever it was.
	// page_hit = bank open AND row match; this separates the two halves.
	output logic        dbg_tile_bankopen,

	input  wire  [25:0] dl_addr,        // full byte address; caller picks region
	input  wire  [7:0]  dl_data,
	output logic        dl_busy,        // backpressure: drop strobes when 1

	// SDRAM chip pins
	output logic [12:0] SDRAM_A,
	output logic [1:0]  SDRAM_BA,
	inout  wire  [15:0] SDRAM_DQ,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE,
	output wire         SDRAM_CKE
);

	// Per-bank open row: row_open[b] = bank b has a row active, open_row[b] =
	// which one. A page hit is that bank open on the same row.
	wire         page_hit;
	assign page_hit = row_open[acc_word[24:23]]
				   && (open_row[acc_word[24:23]] == acc_word[22:10]);
	logic [3:0]  row_open;
	logic [12:0] open_row [0:3];

	// commands: {nCS, nRAS, nCAS, nWE}
	localparam logic [3:0] CMD_NOP      = 4'b0111;
	localparam logic [3:0] CMD_ACTIVE   = 4'b0011;
	localparam logic [3:0] CMD_READ     = 4'b0101;   // with A10=1: READA
	localparam logic [3:0] CMD_WRITE    = 4'b0100;   // with A10=1: WRITEA
	localparam logic [3:0] CMD_PRECHG   = 4'b0010;   // with A10=1: all banks
	localparam logic [3:0] CMD_REFRESH  = 4'b0001;
	localparam logic [3:0] CMD_LOADMODE = 4'b0000;

	// Mode register: CL=2, burst=2, sequential, single-location writes (M9=1).
	//
	// On this board the chip and the controller do not agree which cycle is
	// beat 0 of a write burst, so a halfword write made by masking one beat of
	// a burst with DQM fails. With M9=1 each WRITE writes exactly one word at
	// the column in the address (A0 included), with no beat to align, and the
	// write path holds data and DQM stable across a window around the command
	// so the chip's exact latching edge does not matter. Reads keep BL2 (M9
	// affects writes only).
	localparam logic [12:0] MODE_WORD = 13'b000_1_00_010_0_001;

	logic [3:0] cmd;
	assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd;

	// CKE is tied high: no power-down or clock suspend. The fitter reports
	// SDRAM_CKE stuck at VCC and SDRAM_nCS stuck at GND, by design (every
	// command, NOP included, keeps the chip selected; DESELECT is never
	// issued).
	assign SDRAM_CKE = 1'b1;

	logic [15:0] dq_out;
	logic        dq_oe;
	assign SDRAM_DQ = dq_oe ? dq_out : 16'hZZZZ;

	// The pins' only sampling register (see the header): keep every capture
	// reading `dq_q`, never SDRAM_DQ. No reset and nothing else on its input,
	// so the fitter can put it in the I/O element (Arcade-Atari-GT.qsf:
	// FAST_INPUT_REGISTER ON -to SDRAM_DQ[*]).
	logic [15:0] dq_q;
	always_ff @(posedge clk) dq_q <= SDRAM_DQ;

	// Byte to word address (unused; acc_word below has the bank/row/column
	// split).
	function automatic logic [21:0] word_of(input logic [24:0] byte_addr);
		word_of = byte_addr[22:1];
	endfunction

	// Held copies of the winning port's request, for the access FSM.
	logic        acc_we, acc_is_ram, acc_is_dl, acc_is_cram;
	logic [2:0]  wr_idx;              // 16-bit word of the write (0..7)
	logic        acc_line;            // a cage line (with acc_is_cage)
	logic [127:0] acc_lwdata;         // ...its four words
	wire         acc_cline = acc_is_cage && acc_line;
	// 16-bit accesses (cram, download) are one word, so their single command
	// carries auto-precharge; 32-bit accesses precharge on word 1, a cage
	// line on word 7.
	wire         wr_last_word = (acc_is_cram | acc_is_dl) ? 1'b1
							  : acc_cline ? (wr_idx == 3'd7) : wr_idx[0];
	// A cage line write stepping aside (see the header). The decision is taken
	// with the WRITE that ends a 32-bit word (its A10), and held in wr_split
	// for that word's last state. cl_resume is the word the request continues
	// at when next granted; it clears whenever cage_req is low, so a new
	// request always starts at word 0. cage_yield: rom/tile grants still owed
	// before the cage may go again.
	logic        wr_split;
	logic [2:0]  cl_resume;
	logic [1:0]  cage_yield;
	// Word address in one AS4C32M16SB chip (64 MB). The map fits in one chip;
	// the second chip of a 128 MB module is selected by board logic this
	// controller does not drive.
	//   4 banks x 8192 rows x 1024 columns x 16 bits = 32M words
	//   acc_word[24:23] -> bank    (2)
	//   acc_word[22:10] -> row     (13)
	//   acc_word[9:0]   -> column  (10)
	// The part decodes only A[9:0] as column (A10 is auto-precharge, A11/A12
	// are don't-care in a column cycle); a column bit placed anywhere else is
	// dropped by the chip and aliases silently.
	logic [24:0] acc_word;              // even for CPU bursts; exact for dl
	logic [31:0] acc_wdata;
	logic [3:0]  acc_be;                // beat0 uses [3:2], beat1 [1:0]
	// Download writes are full 16-bit words. The high-side DQM mask does not
	// take effect on this board (a masked odd-byte write lands in both halves),
	// so the even byte is held and one full-word write, both DQM lines low, is
	// issued when its odd partner arrives; the stream is strictly sequential.
	// Three stages, because at write time the next even byte may already be in
	// the live hold:
	//   dl_hi     live hold, overwritten by every even byte
	//   dl_hi_r   snapshot taken when the pair is queued (with dl_pend)
	//   dl_hi_tx  snapshot taken when the transaction starts (with dl_byte)
	logic [7:0]  dl_hi;
	logic [7:0]  dl_hi_r;
	logic [7:0]  dl_hi_tx;
	logic [7:0]  dl_byte;               // odd (low) byte
	logic        dl_hold_v;

	// Release gates (`*_served`): req is level-held until acked (memmap
	// contract) and dropped the cycle after ack, so it is still high in the ack
	// cycle. The gate blocks re-acceptance until req falls; without it the same
	// transaction is served twice and its stale ack can complete the next
	// request with the previous address's data. dl_wr is a strobe latched into
	// a 1-deep buffer.
	//
	// Partial RAM writes are read-modify-write: DQM masking does not work on
	// this board (the chip model's +nodqm reproduces it), so a write with
	// ram_be != 4'b1111 reads the word pair, merges the enabled bytes and
	// writes all 32 bits back with both DQM lines low.
	logic        rmw_pend;               // this access still owes its write
	logic [3:0]  rmw_be;
	logic [31:0] rmw_wdata;
	logic        acc_is_tile;
	logic        tile_odd;          // which byte of the 16-bit word
	logic        rom_served, ram_served, cram_served, tile_served;
	logic        cage_served, acc_is_cage;
	logic        act_second_pass;   // S_ACT re-entered after a precharge retry
	logic        acc_burst3;        // this tile access wants all 3 planes
	logic [2:0]  b3_issued;         // READ commands issued (0..3)
	logic [2:0]  b3_beat;           // data beats captured (0..6)
	// The burst must honour rd_delay exactly as the plain read path does, or
	// every beat is captured early.
	logic [7:0]  b3_dly;            // read-data latency shifter, rd_delay-aware
	// Capture tap = rd_delay + 2, matching the plain path's `{1'b0, rd_delay}`
	// wait. Keep it three bits: an index expression is self-determined, so a
	// two-bit `rd_delay + 2'd2` wraps and captures early at Read Delay 2 and 3.
	wire  [2:0]  b3_cap_idx = {1'b0, rd_delay} + 3'd2;   // 2..5, never wraps
	wire         b3_capture = b3_dly[b3_cap_idx] && (b3_beat != 3'd6);
	// Burst column: nine bits, carry discarded, so A10 in the READ below is the
	// literal 1'b0 for every client. A wider sum shifts every field up, and a
	// burst crossing the 2 KB row then sets A10 (auto-precharge) on a READ.
	// A straddling request still wraps within the open row and reads wrong
	// data, so clients must not straddle (agt_rle_pair_cache clamps its window;
	// the tile path does likewise).
	wire  [8:0]  b3_col = acc_word[9:1] + {7'd0, b3_issued[2:1]};
	// A cage line read: the same shape with four READs (columns +0, +2, +4,
	// +6) and eight beats. A line is 16-byte aligned, so it never straddles a
	// row.
	logic [3:0]  rl_issued;         // cycles in S_RDL_CMD (READs at 0, 2, 4, 6)
	logic [3:0]  rl_beat;           // data beats captured (0..8)
	logic [7:0]  rl_dly;            // read-data latency shifter, as b3_dly
	wire         rl_capture = rl_dly[b3_cap_idx] && (rl_beat != 4'd8);
	wire  [8:0]  rl_col = acc_word[9:1] + {7'd0, rl_issued[2:1]};
	// rom/tile round-robin: when both are pending they alternate, so the line
	// renderer waits at most one rom access and the CPU at most one tile
	// access. With rom strictly first, a CPU that re-requests as soon as it is
	// served can hold the renderer off indefinitely.
	logic        last_grant_rom;
	wire         rom_want  = rom_req  && !rom_ack  && !rom_served;
	wire         tile_want = tile_req && !tile_ack && !tile_served;
	// A late line's fetch goes before an aged cage request and before rom.
	wire         tile_go_first = tile_want && (tile_first === 1'b1);
	// While a split cage line owes rom/tile their grants, the cage arms wait.
	wire         cage_yielding = (cage_yield != 2'd0) && (rom_want || tile_want);
	wire         wr_split_now = acc_cline && wr_idx[0] && (wr_idx != 3'd7)
							  && (tile_want || rom_want);
	wire         cage_want = (cage_req === 1'b1) && !cage_ack && !cage_served;
	// How long the pending cage request has waited: counts every cycle
	// cage_want holds, including other ports' accesses; saturates. With the
	// port tied off it is constant 0 and the urgent arm folds away.
	logic [7:0]  cage_age;
	wire         cage_urgent = cage_want && (cage_age >= CAGE_AGE[7:0]) && !cage_yielding;
	logic        dl_pend;
	logic [25:0] dl_addr_r;
	logic [7:0]  dl_data_r;
	assign dl_busy = dl_pend;

	// refresh timer
	logic [9:0]  ref_cnt;
	logic        ref_due;

	typedef enum logic [4:0] {
		S_INIT_WAIT, S_INIT_PRE, S_INIT_RFC1, S_INIT_RFC1W, S_INIT_RFC2,
		S_INIT_RFC2W, S_INIT_MODE, S_INIT_MODEW,
		S_IDLE, S_REF, S_REFW, S_PRE_ALL,
		S_RD_CMD_OPEN,
		S_ACT, S_RD_CMD, S_RD_W1, S_RD_W2, S_RD_WX, S_RD_D0, S_RD_D1,
		S_RD3_CMD, S_RD3_D, S_RDL_CMD, S_RDL_D,
		S_WR_PRE, S_WR_CMD, S_WR_POST, S_WR_REC1, S_WR_REC2
	} st_t;
	st_t st;

	logic [13:0] init_cnt;
	logic [2:0]  wcnt;                  // in-state wait counter

	// A capture owed to the next edge, taken from `dq_q`: set in the sampling
	// state, carried out after the case statement below.
	localparam logic [2:0] CK_ROM = 3'd0, CK_RAM = 3'd1, CK_CRAM = 3'd2,
						   CK_TILE = 3'd3, CK_CAGE = 3'd4, CK_RMW = 3'd5;
	logic        cap0_v, cap1_v;        // beat 0 / beat 1 of a plain read
	logic [2:0]  cap_kind;              // which port the read was for
	logic        cap_oddw;              // acc_word[0] of that read
	logic        cap_todd;              // tile_odd of that read
	logic        b3c_v;                 // a burst beat owed
	logic [2:0]  b3c_beat;              // ...which one (0..5)
	logic        rlc_v;                 // a cage line beat owed
	logic [2:0]  rlc_beat;              // ...which one (0..7)

	// The 16-bit word a write drives, and its DQM. A cage line's word j is
	// the high half (j even) or low half (j odd) of 32-bit word j/2.
	logic [15:0] wr_d;
	logic        wr_mh, wr_ml;
	always_comb begin
		if (acc_is_dl) begin
			wr_d = {dl_hi_tx, dl_byte}; wr_mh = 1'b0; wr_ml = 1'b0;   // full word
		end else if (acc_is_cram) begin
			wr_d = acc_wdata[15:0];     wr_mh = 1'b0; wr_ml = 1'b0;
		end else if (acc_cline) begin
			wr_d = acc_lwdata[{wr_idx[2:1], ~wr_idx[0], 4'd0} +: 16];
			wr_mh = 1'b0; wr_ml = 1'b0;
		end else if (!wr_idx[0]) begin
			wr_d = acc_wdata[31:16];    wr_mh = ~acc_be[3]; wr_ml = ~acc_be[2];
		end else begin
			wr_d = acc_wdata[15:0];     wr_mh = ~acc_be[1]; wr_ml = ~acc_be[0];
		end
	end
	// Column of the write: the pair base plus the word index (a 32-bit
	// access is even, a line 8-aligned, so the sum never carries out).
	wire  [9:0]  wr_col = acc_word[9:0] + {7'd0, wr_idx};

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			st <= S_INIT_WAIT; init_cnt <= 14'd0; wcnt <= 3'd0;
			row_open <= 4'b0000;
			cmd <= CMD_NOP;
			SDRAM_A <= 13'd0; SDRAM_BA <= 2'd0;
			SDRAM_DQML <= 1'b1; SDRAM_DQMH <= 1'b1;
			dq_oe <= 1'b0; dq_out <= 16'd0;
			ready <= 1'b0;
			rom_ack <= 1'b0; ram_ack <= 1'b0;
			rom_rdata <= 32'd0; ram_rdata <= 32'd0;
			acc_we <= 1'b0; acc_is_ram <= 1'b0; acc_is_dl <= 1'b0;
			acc_word <= 25'd0; acc_wdata <= 32'd0; acc_be <= 4'd0;
			rmw_pend <= 1'b0; rmw_be <= 4'd0; rmw_wdata <= 32'd0;
			dl_hi <= 8'd0; dl_hi_r <= 8'd0; dl_hi_tx <= 8'd0;
			dl_byte <= 8'd0; dl_hold_v <= 1'b0;
			dl_pend <= 1'b0; dl_addr_r <= 26'd0; dl_data_r <= 8'd0;
			rom_served <= 1'b0; ram_served <= 1'b0; cram_served <= 1'b0;
			last_grant_rom <= 1'b0;
			tile_served <= 1'b0; acc_is_tile <= 1'b0; tile_odd <= 1'b0;
			tile_ack <= 1'b0; tile_data <= 8'd0; tile_data32 <= 32'd0;
			tile_data96 <= 96'd0; acc_burst3 <= 1'b0;
			b3_issued <= 3'd0; b3_beat <= 3'd0; b3_dly <= 8'd0;
			dbg_tile_acc <= 1'b0; dbg_tile_pghit <= 1'b0;
			dbg_tile_bankopen <= 1'b0;
			act_second_pass <= 1'b0;
			acc_is_cram <= 1'b0; wr_idx <= 3'd0;
			acc_line <= 1'b0; acc_lwdata <= 128'd0;
			wr_split <= 1'b0; cl_resume <= 3'd0; cage_yield <= 2'd0;
			rl_issued <= 4'd0; rl_beat <= 4'd0; rl_dly <= 8'd0;
			rlc_v <= 1'b0; rlc_beat <= 3'd0;
			cram_ack <= 1'b0; cram_rdata <= 16'd0;
			cage_ack <= 1'b0; cage_rdata <= 128'd0;
			cage_served <= 1'b0; acc_is_cage <= 1'b0; cage_age <= 8'd0;
			ref_cnt <= 10'd0; ref_due <= 1'b0;
			cap0_v <= 1'b0; cap1_v <= 1'b0; cap_kind <= CK_ROM;
			cap_oddw <= 1'b0; cap_todd <= 1'b0;
			b3c_v <= 1'b0; b3c_beat <= 3'd0;
		end else begin
			// defaults every cycle
			cap0_v <= 1'b0; cap1_v <= 1'b0; b3c_v <= 1'b0; rlc_v <= 1'b0;
			cmd <= CMD_NOP;
			rom_ack <= 1'b0; ram_ack <= 1'b0; cram_ack <= 1'b0; tile_ack <= 1'b0;
			cage_ack <= 1'b0;
			dbg_tile_acc <= 1'b0; dbg_tile_pghit <= 1'b0;   // one-shot
			dbg_tile_bankopen <= 1'b0;
			dq_oe <= 1'b0;
			SDRAM_DQML <= 1'b1; SDRAM_DQMH <= 1'b1;

			// release gates: block re-acceptance until req deasserts
			if (!rom_req) rom_served <= 1'b0;
			if (!ram_req) ram_served <= 1'b0;
			if (!cram_req) cram_served <= 1'b0;
			if (!tile_req) tile_served <= 1'b0;
			if (cage_req !== 1'b1) begin
				cage_served <= 1'b0; cl_resume <= 3'd0; cage_yield <= 2'd0;
			end
			if (!cage_want)                cage_age <= 8'd0;
			else if (cage_age != 8'hFF)    cage_age <= cage_age + 8'd1;

			// Download strobe capture (1-deep; the top level holds the CPU
			// during a load). Even byte: hold it and complete at once, no
			// transaction. Odd byte: pair them and queue one word write.
			if (dl_wr && !dl_pend) begin
				if (!dl_addr[0]) begin
					dl_hi     <= dl_data;
					dl_hold_v <= 1'b1;
				end else begin
					dl_pend   <= 1'b1;
					dl_addr_r <= dl_addr;
					dl_data_r <= dl_data;
					// Snapshot the partner now. A stray odd byte with no
					// partner writes 00 rather than stale data, so the failure
					// is visible.
					dl_hi_r   <= dl_hold_v ? dl_hi : 8'd0;
					dl_hold_v <= 1'b0;
				end
			end

			if (ref_cnt == REFRESH_DIV[9:0]) begin
				ref_due <= 1'b1; ref_cnt <= 10'd0;
			end else
				ref_cnt <= ref_cnt + 10'd1;

			case (st)
				// power-up init
				S_INIT_WAIT: begin
					init_cnt <= init_cnt + 14'd1;
					if (init_cnt >= INIT_WAIT[13:0]) st <= S_INIT_PRE;
				end
				S_INIT_PRE: begin
					cmd <= CMD_PRECHG; SDRAM_A <= 13'h400;   // A10: all banks
					st <= S_INIT_RFC1;                       // tRP = 1 cycle
				end
				S_INIT_RFC1: begin
					cmd <= CMD_REFRESH; wcnt <= 3'd4; st <= S_INIT_RFC1W;
				end
				S_INIT_RFC1W: begin
					wcnt <= wcnt - 3'd1;
					if (wcnt == 3'd1) st <= S_INIT_RFC2;
				end
				S_INIT_RFC2: begin
					cmd <= CMD_REFRESH; wcnt <= 3'd4; st <= S_INIT_RFC2W;
				end
				S_INIT_RFC2W: begin
					wcnt <= wcnt - 3'd1;
					if (wcnt == 3'd1) st <= S_INIT_MODE;
				end
				S_INIT_MODE: begin
					cmd <= CMD_LOADMODE; SDRAM_A <= MODE_WORD; SDRAM_BA <= 2'd0;
					st <= S_INIT_MODEW;                       // tMRD = 2
				end
				S_INIT_MODEW: begin
					ready <= 1'b1; st <= S_IDLE;
				end

				// idle / arbitration
				S_IDLE: begin
					// A split cage line's yield ends once neither rom nor tile
					// wants the chip (the arms below count their grants down).
					if (!rom_want && !tile_want) cage_yield <= 2'd0;
					if (ref_due) begin
						// AUTO REFRESH needs all banks precharged and read rows
						// stay open: precharge all first, unless nothing is
						// open.
						ref_due <= 1'b0;
						if (row_open != 4'b0000) begin
							st <= S_PRE_ALL;
						end else begin
							cmd <= CMD_REFRESH;
							wcnt <= 3'd4; st <= S_REFW;
						end
					end else if (rmw_pend) begin
						// Second half of a partial RAM write: the merge is
						// already in acc_wdata, so issue it full-width. Ranked
						// above every port so no other access lands between the
						// read and the write.
						acc_we   <= 1'b1;
						acc_be   <= 4'b1111;
						rmw_pend <= 1'b0;
						st <= S_ACT;
					end else if (dl_pend) begin
						// download: one full word at its exact (possibly odd)
						// word address
						acc_is_dl <= 1'b1; acc_is_ram <= 1'b0; acc_we <= 1'b1;
						acc_is_cram <= 1'b0; acc_is_tile <= 1'b0;
						acc_is_cage <= 1'b0;
						acc_word  <= dl_addr_r[25:1];           // full 64 MB map
						dl_byte   <= dl_data_r;                 // low half
						dl_hi_tx  <= dl_hi_r;                   // high half
						dl_pend   <= 1'b0;
						st <= S_ACT;
					end else if (cage_urgent && !tile_go_first) begin
						// An aged cage request: the same body as the
						// lowest-priority arm below, only earlier in the order.
						// A late line's fetch still goes first.
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b0; acc_is_cram <= 1'b0;
						acc_is_tile <= 1'b0; acc_is_cage <= 1'b1;
						acc_we    <= (cage_we === 1'b1);
						acc_line  <= (cage_line === 1'b1);
						acc_word  <= (cage_line === 1'b1) ? {cage_addr[25:4], 3'd0}
														  : {cage_addr[25:2], 1'b0};
						acc_wdata <= cage_wdata[31:0];
						acc_lwdata <= cage_wdata;
						acc_be    <= 4'b1111;
						rl_issued <= 4'd0; rl_beat <= 4'd0; rl_dly <= 8'd0;
						st <= S_ACT;
					// rom/tile round-robin: when both want the port, the one
					// that did not go last goes now. Alone, either goes at
					// once.
					end else if (rom_want && !(tile_want && last_grant_rom) && !tile_go_first) begin
						last_grant_rom <= 1'b1;
						if (cage_yield != 2'd0) cage_yield <= cage_yield - 2'd1;
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b0; acc_we <= 1'b0;
						acc_is_cram <= 1'b0; acc_is_tile <= 1'b0;
						acc_is_cage <= 1'b0;
						  // Port contract (as the memmap's BRAM stubs): the
						  // address is byte-granular and the server returns the
						  // aligned 32-bit word containing it, so clear the
						  // word-index LSB.
						acc_word  <= {1'b0, rom_addr[20:2], 1'b0};
						st <= S_ACT;
					end else if (tile_want) begin
						last_grant_rom <= 1'b0;
						if (cage_yield != 2'd0) cage_yield <= cage_yield - 2'd1;
						// Compared `=== 1'b1` so an unconnected (X) input reads
						// as 0 rather than sending the state machine down an X
						// branch.
						acc_burst3 <= (tile_burst3 === 1'b1);
						b3_dly <= 8'd0;
						b3_issued  <= 3'd0; b3_beat <= 3'd0;
						// Round-robin with rom (above), as the line renderer
						// must finish inside 3,648 cycles. Still above
						// ram/cram: data accesses can wait.
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b0;
						acc_is_cram <= 1'b0; acc_is_tile <= 1'b1;
						acc_is_cage <= 1'b0;
						acc_we    <= 1'b0;
						acc_word  <= tile_addr[25:1];
						tile_odd  <= tile_addr[0];
						st <= S_ACT;
					end else if (ram_req && !ram_ack && !ram_served) begin
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b1; acc_is_tile <= 1'b0;
						acc_is_cram <= 1'b0; acc_is_cage <= 1'b0;
						acc_we    <= ram_we;
						// RAM region base byte 0x200000 = word 0x100000, above
						// the 18-bit in-region word index
						acc_word  <= 22'h100000
									 + {4'd0, ram_addr[18:2], 1'b0};
						acc_wdata <= ram_wdata;
						acc_be    <= ram_we ? ram_be : 4'b1111;
						// partial write -> read first, then write the merge
						if (ram_we && (ram_be != 4'b1111)) begin
							acc_we    <= 1'b0;          // this pass is a READ
							rmw_pend  <= 1'b1;
							rmw_be    <= ram_be;
							rmw_wdata <= ram_wdata;
						end
						st <= S_ACT;
					end else if (cram_req && !cram_ack && !cram_served) begin
						// colorram: one 16-bit halfword at SDRAM word 0x140000
						// + addr[18:1] (region base byte 0x280000). acc_word
						// keeps the true (possibly odd) word index, so a read
						// knows which beat of the even-aligned BL2 burst is
						// live.
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b0; acc_is_cram <= 1'b1;
						acc_is_tile <= 1'b0; acc_is_cage <= 1'b0;
						acc_we    <= cram_we;
						acc_word  <= 22'h140000 + {3'd0, cram_addr[18:1]};
						acc_wdata <= {cram_wdata, cram_wdata};
						st <= S_ACT;
					end else if (cage_want && !cage_yielding) begin
						// Lowest priority: a whole 32-bit word at a full byte
						// address. No byte enables, so no RMW; the pair base is
						// even, as for every 32-bit access.
						acc_is_dl <= 1'b0; acc_is_ram <= 1'b0; acc_is_cram <= 1'b0;
						acc_is_tile <= 1'b0; acc_is_cage <= 1'b1;
						acc_we    <= (cage_we === 1'b1);
						acc_line  <= (cage_line === 1'b1);
						acc_word  <= (cage_line === 1'b1) ? {cage_addr[25:4], 3'd0}
														  : {cage_addr[25:2], 1'b0};
						acc_wdata <= cage_wdata[31:0];
						acc_lwdata <= cage_wdata;
						acc_be    <= 4'b1111;
						rl_issued <= 4'd0; rl_beat <= 4'd0; rl_dly <= 8'd0;
						st <= S_ACT;
					end
				end

				S_REFW: begin
					wcnt <= wcnt - 3'd1;
					if (wcnt == 3'd1) st <= S_IDLE;
				end

				// Refresh with rows open: precharge all banks, then REFRESH.
				S_PRE_ALL: begin
					cmd <= CMD_PRECHG;
					SDRAM_A[10] <= 1'b1;            // A10 = 1: all banks
					row_open <= 4'b0000;
					st <= S_REF;                    // tRP, then REFRESH
				end
				S_REF: begin
					cmd <= CMD_REFRESH;
					wcnt <= 3'd4; st <= S_REFW;
				end

				// READ on an already-open row: no ACTIVE, no tRCD, and A10 = 0
				// so the row stays open for the next sequential word.
				S_RD_CMD_OPEN: begin
					cmd <= CMD_READ;
					SDRAM_BA <= acc_word[24:23];
					SDRAM_A  <= {2'b00, 1'b0, acc_word[9:1], 1'b0};
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					st <= S_RD_W1;
				end

				// Every arbitration arm comes here; hit/miss routing lives only
				// here. Reads leave their row open (a row holds 512 32-bit
				// words and the RLE stream is read sequentially), and a read
				// that hits skips ACTIVE and tRCD. Writes always take the
				// ACTIVE path and auto-precharge.
				S_ACT: begin
					// Count the first pass only: a miss that must precharge
					// re-enters S_ACT for the same access.
					if (acc_is_tile && !act_second_pass) begin
						dbg_tile_acc      <= 1'b1;
						dbg_tile_pghit    <= page_hit;
						dbg_tile_bankopen <= row_open[acc_word[24:23]];
					end
					if (page_hit && !acc_we) begin
						act_second_pass <= 1'b0;
						cmd <= CMD_NOP;
						wr_idx <= 3'd0; wcnt <= 3'd0;
						st <= (acc_is_tile && acc_burst3) ? S_RD3_CMD
							: acc_cline ? S_RDL_CMD : S_RD_CMD_OPEN;
					end else if (row_open[acc_word[24:23]]) begin
						// A different row is open in this bank: precharge it
						// first (ACTIVATE on an active bank is illegal), then
						// retry.
						cmd <= CMD_PRECHG;
						SDRAM_BA <= acc_word[24:23];
						SDRAM_A[10] <= 1'b0;          // this bank only
						row_open[acc_word[24:23]] <= 1'b0;
						act_second_pass <= 1'b1;      // next S_ACT is the same access
						st <= S_ACT;                  // retry: now a clean miss
					end else begin
						act_second_pass <= 1'b0;
						cmd <= CMD_ACTIVE;
						SDRAM_BA <= acc_word[24:23];
						SDRAM_A  <= acc_word[22:10];  // 13-bit row
						// a split cage line write continues where it stepped aside
						wr_idx <= (acc_cline && acc_we) ? cl_resume : 3'd0;
						wcnt <= 3'd0;
						st <= acc_we ? S_WR_PRE
							: (acc_is_tile && acc_burst3) ? S_RD3_CMD
							: acc_cline ? S_RDL_CMD : S_RD_CMD;
						// a WRITE auto-precharges, so it closes the row it
						// opens; only reads leave one open
						if (acc_we) row_open[acc_word[24:23]] <= 1'b0;
						else begin
							row_open[acc_word[24:23]] <= 1'b1;
							open_row[acc_word[24:23]] <= acc_word[22:10];
						end
					end
				end

				// Three-plane burst on one ACTIVATE: three READs two cycles
				// apart (BL2; a READ in the next cycle would truncate the
				// previous burst's second beat) at columns +0, +2, +4, all with
				// A10 = 0. The six beats stream back from a fixed latency after
				// the first READ, so one beat counter captures them whichever
				// state is current. The access never returns to arbitration, so
				// no other master can open another row in between.
				S_RD3_CMD: begin
					b3_dly <= {b3_dly[6:0], 1'b1};
					if (!b3_issued[0]) begin
						cmd <= CMD_READ;
						SDRAM_BA <= acc_word[24:23];
						// plane N column = pair base + N (each pair = 4 bytes)
						SDRAM_A  <= {2'b00, 1'b0, b3_col, 1'b0};   // 2+1+9+1 = 13 bits
					end else cmd <= CMD_NOP;
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					b3_issued <= b3_issued + 3'd1;
					if (b3_capture) begin
						b3c_v <= 1'b1; b3c_beat <= b3_beat;     // capture at the next edge
						b3_beat <= b3_beat + 3'd1;
					end
					if (b3_issued == 3'd5) st <= S_RD3_D;
				end
				S_RD3_D: begin
					b3_dly <= {b3_dly[6:0], 1'b1};
					cmd <= CMD_NOP;
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					if (b3_capture) begin
						b3c_v <= 1'b1; b3c_beat <= b3_beat;     // capture at the next edge
						if (b3_beat == 3'd5) begin
							// the ack goes with the last beat's capture, one
							// edge on; the gate closes now
							tile_served <= 1'b1;
							acc_burst3 <= 1'b0;
							st <= S_IDLE;
						end
						b3_beat <= b3_beat + 3'd1;
					end
				end

				// A cage line: four READs two cycles apart at columns +0, +2,
				// +4, +6 on one row, as S_RD3_CMD; the eight beats stream back
				// from a fixed latency after the first READ. The ack goes with
				// the last beat.
				S_RDL_CMD: begin
					rl_dly <= {rl_dly[6:0], 1'b1};
					if (!rl_issued[0]) begin
						cmd <= CMD_READ;
						SDRAM_BA <= acc_word[24:23];
						SDRAM_A  <= {2'b00, 1'b0, rl_col, 1'b0};
					end else cmd <= CMD_NOP;
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					rl_issued <= rl_issued + 4'd1;
					if (rl_capture) begin
						rlc_v <= 1'b1; rlc_beat <= rl_beat[2:0];
						rl_beat <= rl_beat + 4'd1;
					end
					if (rl_issued == 4'd7) st <= S_RDL_D;
				end
				S_RDL_D: begin
					rl_dly <= {rl_dly[6:0], 1'b1};
					cmd <= CMD_NOP;
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					if (rl_capture) begin
						rlc_v <= 1'b1; rlc_beat <= rl_beat[2:0];
						if (rl_beat == 4'd7) begin
							cage_served <= 1'b1;
							st <= S_IDLE;
						end
						rl_beat <= rl_beat + 4'd1;
					end
				end

				// read: CL2, two beats
				S_RD_CMD: begin
					cmd <= CMD_READ;
					// BA still holds the bank from S_ACT. The column is forced
					// even: beat 0 is the even word of the pair and beat 1 the
					// odd, so the burst must start at the pair base (an odd
					// cram word is taken from beat 1).
					// A12,A11 = 0; A10 = 0 (row stays open); A9..A0 = column,
					// A0 = 0.
					SDRAM_A  <= {2'b00, 1'b0, acc_word[9:1], 1'b0};
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					st <= S_RD_W1;
				end
				S_RD_W1: begin
					// registered command pipeline: the chip sees the READ one
					// cycle after we issue it, so CL2 data lands two waits from
					// here (pinned by the protocol-checking chip model in
					// tb_sdram)
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					st <= S_RD_W2;
				end
				S_RD_W2: begin
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					if (rd_delay == 2'd0) st <= S_RD_D0;
					else begin wcnt <= {1'b0, rd_delay}; st <= S_RD_WX; end
				end
				S_RD_WX: begin                                 // rd_delay extra waits
					SDRAM_DQML <= 1'b0; SDRAM_DQMH <= 1'b0;
					wcnt <= wcnt - 3'd1;
					if (wcnt == 3'd1) st <= S_RD_D0;
				end
				S_RD_D0: begin                                 // beat 0
					// captured from `dq_q` at the next edge (below)
					cap0_v   <= 1'b1;
					cap_kind <= acc_is_cram ? CK_CRAM :
								acc_is_tile ? CK_TILE :
								acc_is_cage ? CK_CAGE :
								(acc_is_ram && rmw_pend) ? CK_RMW :
								acc_is_ram  ? CK_RAM  : CK_ROM;
					cap_oddw <= acc_word[0];
					cap_todd <= tile_odd;
					st <= S_RD_D1;
				end
				S_RD_D1: begin                                 // beat 1 + ack
					// Beat 1 and the ack land at the next edge (below), but the
					// release gate closes now, so S_IDLE cannot grant the same
					// request again next cycle. The RMW read has no ack and no
					// gate: its merge lands at the next edge while S_IDLE takes
					// the `rmw_pend` arm, and the write reads acc_wdata only
					// from S_ACT on.
					cap1_v <= 1'b1;
					if (acc_is_tile)                    tile_served <= 1'b1;
					else if (acc_is_cram)               cram_served <= 1'b1;
					else if (acc_is_cage)               cage_served <= 1'b1;
					else if (acc_is_ram && !rmw_pend)   ram_served  <= 1'b1;
					else if (!acc_is_ram)               rom_served  <= 1'b1;
					st <= S_IDLE;
				end

				// Write: one single-location command per word, data and DQM
				// held across a window around the command (wr_shift widens it
				// by wr_shift cycles on each side). A 32-bit access issues two
				// commands: word 0 at the even column without auto-precharge,
				// word 1 at the odd column with it.
				S_WR_PRE: begin
					dq_oe <= 1'b1;
					dq_out <= wr_d; SDRAM_DQMH <= wr_mh; SDRAM_DQML <= wr_ml;
					if (wr_shift == 2'd0)      st <= S_WR_CMD;
					else if (wcnt >= {1'b0, wr_shift}) begin
						wcnt <= 3'd0; st <= S_WR_CMD;
					end else
						wcnt <= wcnt + 3'd1;
				end
				S_WR_CMD: begin
					// Re-drive the same word: dq_oe and DQM default off every
					// cycle, so holding means driving the same values again in
					// each state of the window.
					dq_oe <= 1'b1;
					dq_out <= wr_d; SDRAM_DQMH <= wr_mh; SDRAM_DQML <= wr_ml;
					cmd <= CMD_WRITE;
					// A12,A11 = 0; A10 = auto-precharge on the last word, or on
					// the word a cage line steps aside after; A9..A0 = the
					// 10-bit column
					SDRAM_A <= {2'b00, wr_last_word | wr_split_now, wr_col};
					wr_split <= wr_split_now;
					wcnt <= 3'd0;
					st <= S_WR_POST;
				end
				S_WR_POST: begin
					// keep driving the same word: whichever edge the chip
					// latches on, it sees the same data
					dq_oe <= 1'b1;
					dq_out <= wr_d; SDRAM_DQMH <= wr_mh; SDRAM_DQML <= wr_ml;
					if (wr_shift != 2'd0 && wcnt < {1'b0, wr_shift})
						wcnt <= wcnt + 3'd1;
					else if (!wr_last_word && !wr_split) begin
						wr_idx <= wr_idx + 3'd1;   // the access's next word
						wcnt <= 3'd0;
						st <= S_WR_PRE;
					end else begin
						if (wr_split) begin
							// a cage line stepping aside: no ack; rom and tile
							// get up to two grants, then the request resumes at
							// its next word
							cl_resume  <= wr_idx + 3'd1;
							cage_yield <= 2'd2;
						end else if (acc_is_cram) begin
							cram_ack <= 1'b1; cram_served <= 1'b1;
						end else if (acc_is_cage) begin
							cage_ack <= 1'b1; cage_served <= 1'b1;
						end else if (acc_is_ram) begin
							ram_ack <= 1'b1; ram_served <= 1'b1;
						end
						st <= S_WR_REC1;                       // tDPL
					end
				end
				S_WR_REC1: st <= S_WR_REC2;                    // tDPL + tRP
				S_WR_REC2: st <= S_IDLE;

				default: st <= S_IDLE;
			endcase

			// Captures, one edge after the pins were sampled: `dq_q` holds what
			// the pins carried at the edge that ended the sampling state. They
			// come after the case so their acks win over the defaults above.
			// A cage line's beat j: word j/2, high half first.
			if (rlc_v) begin
				cage_rdata[{rlc_beat[2:1], ~rlc_beat[0], 4'd0} +: 16] <= dq_q;
				if (rlc_beat == 3'd7) cage_ack <= 1'b1;
			end
			if (b3c_v) begin
				case (b3c_beat)
					3'd0: tile_data96[95:80] <= dq_q;
					3'd1: tile_data96[79:64] <= dq_q;
					3'd2: tile_data96[63:48] <= dq_q;
					3'd3: tile_data96[47:32] <= dq_q;
					3'd4: tile_data96[31:16] <= dq_q;
					default: tile_data96[15:0] <= dq_q;
				endcase
				if (b3c_beat == 3'd5) tile_ack <= 1'b1;
			end
			if (cap0_v) begin                                  // beat 0
				case (cap_kind)
					CK_CRAM: if (!cap_oddw) cram_rdata <= dq_q;
					CK_TILE: begin
						// beat 0 is the even word of the pair
						tile_data32[31:16] <= dq_q;
						if (!cap_oddw)
							tile_data <= cap_todd ? dq_q[7:0] : dq_q[15:8];
					end
					CK_CAGE: cage_rdata[31:16] <= dq_q;
					CK_RAM, CK_RMW: ram_rdata[31:16] <= dq_q;
					default: rom_rdata[31:16] <= dq_q;
				endcase
			end
			if (cap1_v) begin                                  // beat 1 + ack
				case (cap_kind)
					CK_TILE: begin
						tile_data32[15:0] <= dq_q;   // beat 1 = odd word
						if (cap_oddw)
							tile_data <= cap_todd ? dq_q[7:0] : dq_q[15:8];
						tile_ack <= 1'b1;
					end
					CK_CRAM: begin
						if (cap_oddw) cram_rdata <= dq_q;
						cram_ack <= 1'b1;
					end
					CK_CAGE: begin
						cage_rdata[15:0] <= dq_q;
						cage_ack <= 1'b1;
					end
					CK_RMW: begin
						// merge: enabled lanes take the new data, the rest keep
						// what was just read. Then write all 32 bits.
						acc_wdata[31:24] <= rmw_be[3] ? rmw_wdata[31:24]
													  : ram_rdata[31:24];
						acc_wdata[23:16] <= rmw_be[2] ? rmw_wdata[23:16]
													  : ram_rdata[23:16];
						acc_wdata[15:8]  <= rmw_be[1] ? rmw_wdata[15:8]
													  : dq_q[15:8];
						acc_wdata[7:0]   <= rmw_be[0] ? rmw_wdata[7:0]
													  : dq_q[7:0];
					end
					CK_RAM: begin
						ram_rdata[15:0] <= dq_q;
						ram_ack <= 1'b1;
					end
					default: begin
						rom_rdata[15:0] <= dq_q;
						rom_ack <= 1'b1;
					end
				endcase
			end
		end
	end

endmodule
`default_nettype wire
