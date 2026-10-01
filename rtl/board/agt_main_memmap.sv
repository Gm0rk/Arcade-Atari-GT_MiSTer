// agt_main_memmap.sv -- Atari GT main-CPU address decoder / bus fabric
//
// Transcribed from atarigt.cpp main_map() and the Python model's Mem class
// (cpu68020.py), which ran the real Primal Rage boot against these semantics.
//
//   0x000000-0x1FFFFF  ROM (external port; SDRAM on hardware, BRAM in sim)
//   0xC00000-0xC00003  CAGE sound data (external port to agt_cage_comm)
//   0xD00010-0xD0001F  analog ports (stub: returns 0xFF on selected lanes)
//   0xD20000-0xD20FFF  EEPROM (internal, umask32 0xFF00FF00 byte lanes)
//   0xD40000-0xD4FFFF  EEPROM unlock (arms one write)
//   0xD70000-0xD7FFFF  shared RAM (internal 64KB: playfield, alpha, RLE
//                      object list, mo_command, scratch; snooped by video)
//   0xD80000-0xDFFFFF  colorram + protection window (external port toward
//                      agt_colorram; protection in agt_prage_prot)
//   0xE04000           LED        (write, stored)
//   0xE08000           latch      (write, stored; MOGO/ERASE/CAGE ctl bits)
//   0xE0A000           scanline int ack (clears IRQ6 pending)
//   0xE0C000           video int ack    (clears IRQ4 pending)
//   0xE0E000           watchdog   (write, dropped)
//   0xE80000           P1_P2 input port
//   0xE82000           special port2 (service, live vblank)
//   0xE82004           special port3 (coins + int status, non-clearing read)
//   0xF80000-0xFFFFFF  main RAM 512KB (external port; SDRAM on hardware)
//
// Addresses decode on the low 24 bits (the model's MASK=0xFFFFFF): the CPU
// emits 0xFFFFxxxx for the sign-extended short forms and the board decodes
// only A23-A0.
//
// CPU side: m68020_core's req/ack protocol, size 1/2/4, one transaction at a
// time. Sub-word writes into 32-bit stores use byte-lane enables.
//
// Alignment: any (offset,size) on ROM / main RAM / shared RAM / EEPROM,
// including accesses that cross a 32-bit word boundary (a second beat for
// word1); the real boot does size-4 reads at offset 2. An access occupies
// bytes off..off+size-1 of the 64-bit window {word0, word1}; enables and
// shift amounts follow from (8-off-size). Colorram handles any even address.
// Odd-address colorram and misaligned CAGE/port/analog accesses are not
// supported (none occur); misaligned accesses that cross a region boundary
// (e.g. size-4 at 0xD7FFFE) wrap within the store.
//
// IRQs: video (IRQ4) and scanline (IRQ6) pending bits are set by single-cycle
// pulses from the video side and cleared by the game's ack writes; CAGE IRQ3
// is a level from the sound comm. ipl_out is the priority encode, fed straight
// to m68020_core.ipl_in.
`default_nettype none

module agt_main_memmap (
	input  wire         clk,
	input  wire         rst_n,

	// CPU bus (slave)
	input  wire  [31:0] cpu_addr,
	input  wire  [2:0]  cpu_size,     // 1 / 2 / 4
	input  wire         cpu_we,
	input  wire  [31:0] cpu_wdata,
	input  wire         cpu_req,      // one pulse per transaction;
	output logic        cpu_ack,      // ack pulses when data is valid
	output logic [31:0] cpu_rdata,

	// ROM (external, read-only)
	output logic [20:0] rom_addr,     // byte address within 2MB
	output logic        rom_req,
	input  wire         rom_ack,
	input  wire  [31:0] rom_rdata,    // 32-bit BE word containing the address

	// main RAM 512KB (external)
	output logic [18:0] ram_addr,     // byte address within 512KB
	output logic [3:0]  ram_be,       // byte enables (BE lane 3 = MSB)
	output logic        ram_we,
	output logic [31:0] ram_wdata,
	output logic        ram_req,
	input  wire         ram_ack,
	input  wire  [31:0] ram_rdata,

	// colorram window (external, toward agt_colorram)
	output logic [18:0] cram_addr,    // byte address within 0x80000 window
	output logic        cram_we,
	output logic [15:0] cram_wdata,   // colorram is 16-bit oriented
	output logic        cram_req,
	input  wire         cram_ack,
	input  wire  [15:0] cram_rdata,

	// CAGE comm (toward agt_cage_comm's main-side port). Byte enables are
	// required: sound_data_r/w is lane-sensitive (bits 31:16 = main_r/main_w,
	// bits 15:0 = control_r/control_w, and main_r pops the response latch).
	output logic        cage_req,
	output logic        cage_we,
	output logic [3:0]  cage_be,
	output logic [31:0] cage_wdata,
	input  wire         cage_ack,
	input  wire  [31:0] cage_rdata,

	// interrupts / video status
	input  wire         video_int_set,     // IRQ4 set: pulse at vblank start
	input  wire         scanline_int_set,  // IRQ6 set pulse
	input  wire         cage_irq,          // IRQ3, level from the sound comm

	// COIN port switches, active-low raw levels (idle all ones): bit 7 = COIN1
	// (left), bit 6 = COIN2 (right), per atarigt.cpp. Composed into special
	// port3 (the int status XORs bits 1:0).
	input  wire  [15:0] coin_in,
	// SERVICE / self-test switch, active low (0 = pressed). Tie high when there
	// is no input source; see sport2_temp.
	input  wire         service_n,
	// Pulse when the game acks the video / scanline interrupt: the CPU reached
	// the handler, not merely the source fired.
	output logic        dbg_vid_ack,
	output logic        dbg_scan_ack,

	// Shared-RAM write snoop: every CPU write beat into the 64KB shared region,
	// so the video side can keep live copies of the playfield
	// (0xD72000-0xD75FFF) and alpha (0xD76000-0xD76FFF) tilemaps; filtered at
	// the consumer. Word address within shared (a[15:2]) + byte enables + the
	// 32-bit write data.
	output wire         snoop_we,
	output wire  [13:0] snoop_addr,
	output wire  [3:0]  snoop_be,
	output wire  [31:0] snoop_wd,

	// Checksum write-back: the MO checksum answer writes into the objlist
	// window through this port, serviced only in idle cycles, so the shared
	// arrays keep their single writer.
	input  wire         chkw_req,
	input  wire  [10:0] chkw_half,         // halfword index in the window
	input  wire  [15:0] chkw_data,
	output logic        chkw_ack,          // 1-cycle pulse

	// NVRAM save/restore. The EEPROM is reached only through these service
	// ports, never by a second port on the arrays: eeprom_mem_hi/lo keep one
	// writer and one registered reader, the shape Quartus infers.
	input  wire         nvw_req,
	input  wire  [10:0] nvw_addr,          // EEPROM byte index, 0..2047
	input  wire  [7:0]  nvw_data,
	output logic        nvw_ack,
	input  wire         nvr_req,
	input  wire  [10:0] nvr_addr,
	output logic [7:0]  nvr_data,
	output logic        nvr_valid,
	output logic        eeprom_wr_evt,     // CPU wrote the EEPROM (1-cycle)
	output logic        eeprom_rd_evt,     // CPU read the EEPROM (1-cycle)

	input  wire         vblank_level,      // live vblank (agt_video_timing), sport2 bit 7
	output logic [2:0]  ipl_out,

	// inputs / misc
	input  wire  [31:0] p1_p2_port,        // active-low buttons/sticks
	output logic [31:0] led_value,
	output logic [31:0] latch_value,
	output logic        latch_wr,          // pulse on latch write

	// Counts CPU accesses to the Primal Rage protection magic addresses in the
	// colorram window.
	output logic [15:0] prot_hit_count
);

	// Internal stores. Do not simplify this structure: both stores are
	// single-port, byte-enabled, registered-read arrays, the Quartus M10K
	// inference template. The unaligned 64-bit window is assembled by a
	// multi-beat FSM (M_SHR_*/M_EEP_* below), as M_ROM2/M_RAM2 do for the
	// external ports. Reading or writing both words in one cycle would need a
	// RAM with two write and two asynchronous read ports, which Quartus can
	// only build from registers and mux trees far larger than the device.

	// shared RAM: 64KB = 16K x 32, byte-enabled.
	//
	// Byte enables via separate RAMs, not a RAM feature: Quartus 17.0.2 did not
	// honour either byte-enable template (bit-range writes on [31:0] words, or
	// packed [3:0][7:0] with an indexed loop) and built registers instead. Each
	// lane is a plain 8-bit array with its own write enable and a registered
	// read; the block count is the same as one 32-bit RAM.
	//
	// Lane numbering: lane 3 = bits [31:24] = the byte at the access address
	// (big-endian CPU), matching be[3] on the CPU-side port.
	//
	// Pinned to M10K so the fitter cannot demote them to registers when block
	// memory runs short. Slicing a lane to reach the M10K parity bits gains
	// nothing: those bits are only usable at a declared width above 32.
	(* ramstyle = "M10K" *) logic [7:0] shared_mem_b3 [0:16383];
	(* ramstyle = "M10K" *) logic [7:0] shared_mem_b2 [0:16383];
	(* ramstyle = "M10K" *) logic [7:0] shared_mem_b1 [0:16383];
	(* ramstyle = "M10K" *) logic [7:0] shared_mem_b0 [0:16383];

	// EEPROM: 4KB window; data lives on umask32 0xFF00FF00 lanes, so each
	// 32-bit word carries 2 data bytes -> 0x400 words x 2 lanes. Pinned to
	// M10K: when block memory runs short Quartus otherwise silently converts
	// the array to registers and the fit fails with a misleading LAB error.
	(* ramstyle = "M10K" *) logic [7:0] eeprom_mem_hi [0:1023];  // lane-0 byte
	(* ramstyle = "M10K" *) logic [7:0] eeprom_mem_lo [0:1023];  // lane-1 byte

	initial begin
		// zero-init via $readmemh (Quartus rejects init loops over 5000
		// iterations); one file per lane
$readmemh("mem/memmap_shared_zero_b.hex", shared_mem_b3);
		$readmemh("mem/memmap_shared_zero_b.hex", shared_mem_b2);
		$readmemh("mem/memmap_shared_zero_b.hex", shared_mem_b1);
		$readmemh("mem/memmap_shared_zero_b.hex", shared_mem_b0);
		// Blank is 0xFF: an erased 2816 EEPROM reads all ones, MAME's
		// EEPROM_2816 has no default data and the set ships no EEPROM image, so
		// a fresh boot sees 0xFF everywhere. A 0x00 blank is a different
		// settings block (every option at choice 0, language included).
		$readmemh("mem/memmap_eeprom_blank_b.hex", eeprom_mem_hi);
		$readmemh("mem/memmap_eeprom_blank_b.hex", eeprom_mem_lo);
	end

	// shared RAM port registers (single port, registered read)
	logic [13:0] shr_addr;
	logic        shr_we;
	logic [3:0]  shr_be;
	logic [31:0] shr_wd, shr_q;

	// Each lane is an independent single-port RAM: one write condition, one
	// registered read, nothing else touches it.
always_ff @(posedge clk) begin
		if (shr_we && shr_be[3]) shared_mem_b3[shr_addr] <= shr_wd[31:24];
		shr_q[31:24] <= shared_mem_b3[shr_addr];
	end
	always_ff @(posedge clk) begin
		if (shr_we && shr_be[2]) shared_mem_b2[shr_addr] <= shr_wd[23:16];
		shr_q[23:16] <= shared_mem_b2[shr_addr];
	end
	always_ff @(posedge clk) begin
		if (shr_we && shr_be[1]) shared_mem_b1[shr_addr] <= shr_wd[15:8];
		shr_q[15:8] <= shared_mem_b1[shr_addr];
	end
	always_ff @(posedge clk) begin
		if (shr_we && shr_be[0]) shared_mem_b0[shr_addr] <= shr_wd[7:0];
		shr_q[7:0] <= shared_mem_b0[shr_addr];
	end

	// EEPROM port registers (single port, registered read)
	//
	// 28xx lock/unlock, from eeprompar.cpp with atarigt's lock_after_write(true):
	//
	//   device_reset()   m_oe = 0                  locked out of reset
	//   unlock_write32() m_oe = 1                  a write to 0xd40000
	//   write()  m_oe==0 -> discarded
	//            else    -> stored, then m_oe = 0  re-locks after one byte
	//   read()   m_oe==1 -> space.unmap()          reads fail while unlocked
	//
	// Every saved byte needs its own unlock.
	logic eep_oe;      // 1 = unlocked for one write, 0 = locked
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)                    eep_oe <= 1'b0;   // locked at reset
		else if (sel_unlock && cpu_we) eep_oe <= 1'b1;
		else if (eep_we_raw)           eep_oe <= 1'b0;   // relock after a write
	end

	logic [9:0]  eep_addr;
	logic        eep_we_raw;   // the CPU asked to write
	logic        eep_we;       // ... and the part accepted it
	logic [1:0]  eep_be;       // [1] = lane0 byte, [0] = lane1 byte
	logic [15:0] eep_wd, eep_q;

	always_ff @(posedge clk) begin
		if (eep_we && eep_be[1]) eeprom_mem_hi[eep_addr] <= eep_wd[15:8];
		eep_q[15:8] <= eeprom_mem_hi[eep_addr];
	end
	always_ff @(posedge clk) begin
		if (eep_we && eep_be[0]) eeprom_mem_lo[eep_addr] <= eep_wd[7:0];
		eep_q[7:0] <= eeprom_mem_lo[eep_addr];
	end

	// decode
	wire [23:0] a = cpu_addr[23:0];

	wire sel_rom      = (a < 24'h200000);
	wire sel_cage     = (a >= 24'hC00000) && (a <= 24'hC00003);
	wire sel_analog   = (a >= 24'hD00010) && (a <= 24'hD0001F);
	wire sel_eeprom   = (a >= 24'hD20000) && (a <= 24'hD20FFF);
	wire sel_unlock   = (a >= 24'hD40000) && (a <= 24'hD4FFFF);
	wire sel_shared   = (a >= 24'hD70000) && (a <= 24'hD7FFFF);
	wire sel_cram     = (a >= 24'hD80000) && (a <= 24'hDFFFFF);
	wire sel_led      = (a >= 24'hE04000) && (a <= 24'hE04003);
	wire sel_latch    = (a >= 24'hE08000) && (a <= 24'hE08003);
	wire sel_scan_ack = (a >= 24'hE0A000) && (a <= 24'hE0A003);
	wire sel_vid_ack  = (a >= 24'hE0C000) && (a <= 24'hE0C003);
	wire sel_wdog     = (a >= 24'hE0E000) && (a <= 24'hE0E003);
	wire sel_p1p2     = (a >= 24'hE80000) && (a <= 24'hE80003);
	wire sel_sport2   = (a >= 24'hE82000) && (a <= 24'hE82003);
	wire sel_sport3   = (a >= 24'hE82004) && (a <= 24'hE82007);
	wire sel_mainram  = (a >= 24'hF80000);

	// Primal Rage protection magic addresses (primrage_update_mode sequence
	// members + the mode-1/2 windows), counted in prot_hit_count.
	wire prot_magic = sel_cram && (
		(a == 24'hDC4700) || (a == 24'hDC4010) || (a == 24'hDC4022) ||
		(a == 24'hDCC7C0) || (a == 24'hDCC7C2) || (a == 24'hDCC7C4) ||
		(a == 24'hDCC7C6) || (a == 24'hDCC7CA) ||
		(a == 24'hDC80F2) || (a == 24'hDC7AF2));

	// Primal Rage colorram protection (agt_prage_prot): one pulse per CPU
	// colorram-window transaction at accept time, with the access's 16-bit
	// halves in MAME's order (high/lower-address half first). Tapping at
	// accept keeps the byte-write RMW's internal read and write-back beats
	// away from protection, as in MAME, where a byte write calls protection_w
	// once with the CPU's lane data and does no protection read. The range
	// selects are mutually exclusive, so this guard mirrors the M_IDLE
	// dispatch below. Odd-address cram accesses are not supported (the game's
	// protection traffic is aligned and word-sized).
	// The registered sub0/sub1 results are valid from the cycle after accept,
	// one cycle before the earliest M_CRAM composition, and override the read
	// data there.
	wire        prot_accept = (mstate == M_IDLE) && req_rise && sel_cram;
	// Byte-write lane data: the byte in its lane, the other byte zero. MAME
	// packs sub-width writes as byte << (lane*8) into the 32-bit handler's
	// data; this is not physical-bus replication.
	wire [15:0] prot_h0w =
		(cpu_size == 3'd4) ? cpu_wdata[31:16] :
		(cpu_size == 3'd1) ? (a[0] ? {8'd0, cpu_wdata[7:0]}
								   : {cpu_wdata[7:0], 8'd0}) :
							 cpu_wdata[15:0];
	wire        prot_sub0_v, prot_sub1_v;
	wire [15:0] prot_sub0_d, prot_sub1_d;
	wire [1:0]  prot_mode;

	agt_prage_prot u_prot (
		.clk(clk), .rst_n(rst_n),
		.acc_valid(prot_accept), .acc_we(cpu_we),
		.h0_addr({a[23:1], 1'b0}), .h0_wdata(prot_h0w),
		.h1_valid(cpu_size == 3'd4),
		.h1_addr({a[23:1], 1'b0} + 24'd2), .h1_wdata(cpu_wdata[15:0]),
		.sub0_valid(prot_sub0_v), .sub0_data(prot_sub0_d),
		.sub1_valid(prot_sub1_v), .sub1_data(prot_sub1_d),
		.prot_mode_out(prot_mode)
	);

	// interrupt state (mirrors atarigt.cpp m_video_int_state /
	// m_scanline_int_state + the CAGE IRQ3 level)
	logic video_int_pending, scanline_int_pending;

	always_comb begin
		// priority encode, highest wins: 6 (scanline) > 4 (video) > 3 (CAGE)
		if (scanline_int_pending)      ipl_out = 3'd6;
		else if (video_int_pending)    ipl_out = 3'd4;
		else if (cage_irq)             ipl_out = 3'd3;
		else                           ipl_out = 3'd0;
	end

	// special port3 (atarigt.cpp special_port3_r): temp = COIN port (idle all
	// ones, active low); video int XORs bit 0, scanline XORs bit 1; the 32-bit
	// result is (temp<<16)|temp, so the status shows in both halves. Reading
	// clears nothing.
	wire [15:0] sport3_temp = coin_in ^ {14'd0, scanline_int_pending,
												video_int_pending};
	wire [31:0] sport3_val  = {sport3_temp, sport3_temp};
	// special port2 (atarigt.cpp special_port2_r): temp = SERVICE port, idle
	// active-low bits high. Bit 7 = live vblank (active high): the game spins
	// on it at 0x23BCA waiting for vblank to end before kicking mo_command, so
	// a constant here hangs it. Bit 0 reads 0 (MAME: "/A2DRDY always high for
	// now", XORed with the inactive-high port bit). Result is (temp<<16)|temp
	// like sport3.
	// SERVICE (self-test) is bit 6, active low, per atarigt.cpp:
	//     PORT_SERVICE( 0x0040, IP_ACTIVE_LOW )   /* SELFTEST */
	// Driving service_n low enters the game's EEPROM-backed self-test menu;
	// Primal Rage has no DIP switches, so this is its settings entry point.
	wire [15:0] sport2_temp = (16'hFF7E & {9'b1_1111_1111, service_n, 6'b11_1111})
							| {8'd0, vblank_level, 7'd0};
	wire [31:0] sport2_val  = {sport2_temp, sport2_temp};

	assign snoop_we   = shr_we;
	assign snoop_addr = shr_addr;
	assign snoop_be   = shr_be;
	assign snoop_wd   = shr_wd;

	// transaction FSM
	typedef enum logic [4:0] { M_IDLE, M_ROM, M_RAM, M_CRAM, M_CRAM2, M_CAGE,
							   M_CRAM_RMW, M_CRAM_RMW2,
							   M_ROM2, M_RAM2,
							   // registered-BRAM beat sequencing
							   // (see the internal stores note)
							   M_SHR_R0, M_SHR_R1, M_SHR_R2,
							   M_SHR_W1, M_SHR_W2, M_CHKW,
							   M_NVW, M_NVR0, M_NVR1,
							   M_EEP_R0, M_EEP_R1, M_EEP_R2,
							   M_EEP_W1, M_EEP_W2 } mstate_t;
	mstate_t mstate;

	logic        req_d;
	// req_rise is a one-cycle pulse. Every state except M_CHKW is entered
	// because the CPU asked for it, so only M_CHKW can be busy when a request
	// arrives; the pulse would be lost there and the bus would hang. The rise
	// is latched in req_pend until M_IDLE consumes it (tb_chkw_arb).
	logic        req_pend;
	wire         req_rise = cpu_req && !req_d;

	// byte-position helpers over the two-word window
	// An access at (off, sz) occupies bytes off..off+sz-1 of the 8-byte
	// big-endian window {word0, word1}. be8[7] = word0 lane 3 (MSB) down to
	// be8[0] = word1 lane 0. Everything below follows from one shift amount,
	// (8-off-sz)*8, so aligned and misaligned are the same code.
	function automatic [31:0] szmask(input [2:0] sz);
		case (sz)
			3'd1: szmask = 32'h0000_00FF;
			3'd2: szmask = 32'h0000_FFFF;
			default: szmask = 32'hFFFF_FFFF;
		endcase
	endfunction

	function automatic [7:0] be8_of(input [1:0] off, input [2:0] sz);
		// sz ones, MSB-justified to byte position off (off+sz <= 8 always)
		be8_of = ((8'd1 << sz) - 8'd1) << (4'd8 - {2'd0, off} - {1'd0, sz});
	endfunction

	function automatic [63:0] wval64_of(input [1:0] off, input [2:0] sz,
										input [31:0] v);
		wval64_of = {32'd0, v & szmask(sz)}
					<< ((6'd8 - {4'd0, off} - {3'd0, sz}) * 8);
	endfunction

	function automatic [31:0] rd64_extract(input [1:0] off, input [2:0] sz,
										   input [63:0] w64);
		rd64_extract = (w64 >> ((6'd8 - {4'd0, off} - {3'd0, sz}) * 8))
					   & szmask(sz);
	endfunction

	// aligned-view wrappers (single-word cases). Icarus can't select into a
	// function call result, so use locals.
	function automatic [3:0] lanes(input [1:0] off, input [2:0] sz);
		logic [7:0] t;
		begin t = be8_of(off, sz); lanes = t[7:4]; end
	endfunction
	function automatic [31:0] to_lanes(input [1:0] off, input [2:0] sz,
									   input [31:0] v);
		logic [63:0] t;
		begin t = wval64_of(off, sz, v); to_lanes = t[63:32]; end
	endfunction
	function automatic [31:0] from_lanes(input [1:0] off, input [2:0] sz,
										 input [31:0] w);
		from_lanes = rd64_extract(off, sz, {w, 32'd0});
	endfunction

	// colorram: 16-bit oriented; a 32-bit CPU access becomes two 16-bit ops.
	// Byte writes need read-modify-write: the colorram port has no byte lanes,
	// and a 16-bit write would clobber the partner byte.
	logic        cram_second_half;
	logic [15:0] cram_hi_hold;
	logic [15:0] cram_rmw_hold;

	// hoisted helpers (Icarus can't select into a function call result)
	wire [7:0]  w_be8    = be8_of(cpu_addr[1:0], cpu_size);
	wire        w_cross  = (w_be8[3:0] != 4'd0);   // spills into word1
	wire [63:0] w_wval64 = wval64_of(cpu_addr[1:0], cpu_size, cpu_wdata);
	wire [3:0]  w_lanes  = w_be8[7:4];
	wire [31:0] w_wval   = w_wval64[63:32];
	logic [31:0] beat0_hold;                        // word0 of a 2-beat read

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			mstate <= M_IDLE; cpu_ack <= 1'b0; req_d <= 1'b0;
			shr_addr <= 14'd0; shr_we <= 1'b0; shr_be <= 4'd0; shr_wd <= 32'd0;
			eep_addr <= 10'd0; eep_we <= 1'b0; eep_be <= 2'd0; eep_wd <= 16'd0;
			eep_we_raw <= 1'b0;
			rom_req <= 1'b0; ram_req <= 1'b0; cram_req <= 1'b0; cage_req <= 1'b0;
			cage_be <= 4'd0;
			video_int_pending <= 1'b0; scanline_int_pending <= 1'b0;
			led_value <= 32'd0; latch_value <= 32'd0; latch_wr <= 1'b0;
			dbg_vid_ack <= 1'b0; dbg_scan_ack <= 1'b0;
			prot_hit_count <= 16'd0; cram_second_half <= 1'b0;
			chkw_ack <= 1'b0; req_pend <= 1'b0;
			nvw_ack <= 1'b0; nvr_valid <= 1'b0; nvr_data <= 8'd0;
			eeprom_wr_evt <= 1'b0; eeprom_rd_evt <= 1'b0;
		end else begin
			cpu_ack  <= 1'b0;
			latch_wr <= 1'b0;
			chkw_ack <= 1'b0;
			nvw_ack  <= 1'b0; nvr_valid <= 1'b0;
			eeprom_wr_evt <= 1'b0; eeprom_rd_evt <= 1'b0;
			dbg_vid_ack <= 1'b0; dbg_scan_ack <= 1'b0;   // one-shot, like latch_wr
			req_d    <= cpu_req;
			if (req_rise) req_pend <= 1'b1;

			// pending set has priority over ack-clear only if simultaneous
			if (video_int_set)    video_int_pending    <= 1'b1;
			if (scanline_int_set) scanline_int_pending <= 1'b1;

			case (mstate)
				M_IDLE: if (req_rise || req_pend) begin
					req_pend <= 1'b0;
					if (prot_magic) prot_hit_count <= prot_hit_count + 16'd1;

					if (sel_rom && !cpu_we) begin
						rom_addr <= a[20:0]; rom_req <= 1'b1; mstate <= M_ROM;
					end else if (sel_mainram) begin
						ram_addr  <= {a[18:2], 2'b00};
						ram_be    <= w_be8[7:4];
						ram_we    <= cpu_we;
						ram_wdata <= w_wval64[63:32];
						ram_req   <= 1'b1; mstate <= M_RAM;
					end else if (sel_shared) begin
						// beat 0 of the window; word1 is always
						// sequenced too (its byte enables are zero when
						// the access doesn't cross, so the second write
						// is a no-op). The index wraps inside the 64KB
						// store: region-crossing misalignment is not
						// supported.
						shr_addr <= a[15:2];
						shr_we   <= cpu_we;
						shr_be   <= w_be8[7:4];
						shr_wd   <= w_wval64[63:32];
						mstate   <= cpu_we ? M_SHR_W1 : M_SHR_R0;
					end else if (sel_cram) begin
						// 16-bit-oriented device: run one (or two)
						// halves; byte writes read the word first (RMW)
						cram_second_half <= (cpu_size == 3'd4);
						cram_addr  <= a[18:0] & 19'h7FFFE;
						cram_we    <= cpu_we && !(cpu_size == 3'd1);
						cram_wdata <= (cpu_size == 3'd4) ? cpu_wdata[31:16]
														 : cpu_wdata[15:0];
						cram_req   <= 1'b1;
						mstate <= (cpu_we && cpu_size == 3'd1) ? M_CRAM_RMW
															   : M_CRAM;
					end else if (sel_cage) begin
						cage_we <= cpu_we; cage_wdata <= w_wval;
						cage_be <= w_lanes;
						cage_req <= 1'b1; mstate <= M_CAGE;
					end else if (sel_eeprom) begin
						// umask32 FF00FF00: data bytes on lanes 3 and 1
						// of each word; unused lanes read back FF. Both
						// bytes of a word live in one entry, so a word is
						// one port access; word1 is sequenced as beat 1.
						eep_addr <= a[11:2];
						// The CPU's write only lands while unlocked.
						// eep_we_raw records that a write was attempted,
						// so the relock fires either way, as in
						// eeprompar.cpp.
						eep_we_raw <= cpu_we;
						eep_we     <= cpu_we && eep_oe;
						if (cpu_we) eeprom_wr_evt <= 1'b1;
						else        eeprom_rd_evt <= 1'b1;
						eep_be   <= {w_be8[7], w_be8[5]};
						eep_wd   <= {w_wval64[63:56], w_wval64[47:40]};
						mstate   <= cpu_we ? M_EEP_W1 : M_EEP_R0;
					end else if (sel_led && cpu_we) begin
						led_value <= cpu_wdata; cpu_ack <= 1'b1;
					end else if (sel_latch && cpu_we) begin
						// Position the data by size and address: MAME's
						// latch_w is a 32-bit handler that reads the
						// control bits out of the 32-bit word,
						//     m_rle->control_write((data >> 27) & 7);
						// so a 16-bit write to 0xE08000 puts /MOGO (D11)
						// in bits 31:16.
						latch_value <=
							(cpu_size == 3'd4) ? cpu_wdata
						  : (cpu_size == 3'd2) ? (cpu_addr[1]
								? {16'd0, cpu_wdata[15:0]}
								: {cpu_wdata[15:0], 16'd0})
						  : (cpu_addr[1:0] == 2'd0) ? {cpu_wdata[7:0], 24'd0}
						  : (cpu_addr[1:0] == 2'd1) ? {8'd0, cpu_wdata[7:0], 16'd0}
						  : (cpu_addr[1:0] == 2'd2) ? {16'd0, cpu_wdata[7:0], 8'd0}
													: {24'd0, cpu_wdata[7:0]};
						latch_wr <= 1'b1; cpu_ack <= 1'b1;
					end else if (sel_scan_ack && cpu_we) begin
						scanline_int_pending <= 1'b0; cpu_ack <= 1'b1;
						dbg_scan_ack <= 1'b1;
					end else if (sel_vid_ack && cpu_we) begin
						video_int_pending <= 1'b0; cpu_ack <= 1'b1;
						dbg_vid_ack <= 1'b1;
					end else if (sel_wdog || sel_unlock) begin
						cpu_ack <= 1'b1;                    // accept + drop
					end else if (sel_p1p2 && !cpu_we) begin
						cpu_rdata <= from_lanes(a[1:0], cpu_size, p1_p2_port);
						cpu_ack <= 1'b1;
					end else if (sel_sport2 && !cpu_we) begin
						cpu_rdata <= from_lanes(a[1:0], cpu_size, sport2_val);
						cpu_ack <= 1'b1;
					end else if (sel_sport3 && !cpu_we) begin
						cpu_rdata <= from_lanes(a[1:0], cpu_size, sport3_val);
						cpu_ack <= 1'b1;
					end else if (sel_analog && !cpu_we) begin
						// ADC stub: 0xFF on the umask32 FF00FF00 device
						// lanes, sized like every other read. sport2's
						// /A2DRDY=1 keeps the boot off this path. The
						// ADC0809 itself is not modelled.
						cpu_rdata <= from_lanes(a[1:0], cpu_size, 32'hFF00FF00);
						cpu_ack <= 1'b1;
					end else begin
						// unmapped: reads as FFFFFFFF, writes dropped --
						// and both complete, so a stray access can't
						// hang the bus
						cpu_rdata <= 32'hFFFF_FFFF;
						cpu_ack <= 1'b1;
					end
				end
				else if (nvw_req && !nvw_ack) begin
					// NVRAM restore beat: EEPROM byte n lives in
					// word n>>1, high lane for even n (the umask32
					// FF00FF00 view: lanes 3 and 1, matching MAME's
					// 28C16 mapping). Restore bypasses the lock: it
					// is the save file being loaded back, not the
					// game writing.
					eep_addr <= nvw_addr[10:1];
					eep_we   <= 1'b1;
					eep_be   <= nvw_addr[0] ? 2'b01 : 2'b10;
					eep_wd   <= {nvw_data, nvw_data};
					mstate   <= M_NVW;
				end
				else if (nvr_req && !nvr_valid) begin
					eep_addr <= nvr_addr[10:1];
					eep_we   <= 1'b0;
					eep_we_raw <= 1'b0;
					mstate   <= M_NVR0;
				end
				else if (chkw_req && !chkw_ack) begin
					// checksum answer beat: one halfword into the
					// objlist window. The CPU has priority; the
					// arrays keep their single writer because this
					// reuses the shr_* registers.
					shr_addr <= 14'h2000 + {4'd0, chkw_half[10:1]};
					shr_we   <= 1'b1;
					shr_be   <= chkw_half[0] ? 4'b0011 : 4'b1100;
					shr_wd   <= chkw_half[0] ? {16'h0000, chkw_data}
											 : {chkw_data, 16'h0000};
					mstate   <= M_CHKW;
				end

				// shared RAM: registered-read BRAM beat sequencing.
				// The address register loads at the end of the
				// dispatch cycle, the RAM samples it the next cycle
				// and shr_q is readable the cycle after -- hence R0
				// as a settle state before R1 captures word0.
				M_SHR_R0: begin
					shr_addr <= shr_addr + 14'd1;   // pre-issue word1
					mstate   <= M_SHR_R1;
				end
				M_SHR_R1: begin
					beat0_hold <= shr_q;            // word0
					mstate     <= M_SHR_R2;
				end
				M_SHR_R2: begin
					cpu_rdata <= rd64_extract(a[1:0], cpu_size,
											  {beat0_hold, shr_q});
					cpu_ack   <= 1'b1;
					mstate    <= M_IDLE;
				end
				M_SHR_W1: begin
					// beat 1: word1's enables are zero unless the
					// access crosses, so this is a no-op write in
					// the common case
					shr_addr <= shr_addr + 14'd1;
					shr_we   <= 1'b1;
					shr_be   <= w_be8[3:0];
					shr_wd   <= w_wval64[31:0];
					mstate   <= M_SHR_W2;
				end
				M_SHR_W2: begin
					shr_we  <= 1'b0;
					cpu_ack <= 1'b1;
					mstate  <= M_IDLE;
				end
				M_CHKW: begin
					shr_we   <= 1'b0;
					chkw_ack <= 1'b1;
					mstate   <= M_IDLE;
				end
				M_NVW: begin
					eep_we  <= 1'b0;
					nvw_ack <= 1'b1;
					mstate  <= M_IDLE;
				end
				M_NVR0: begin
					// one settle cycle: the array registers its read
					mstate <= M_NVR1;
				end
				M_NVR1: begin
					nvr_data  <= nvr_addr[0] ? eep_q[7:0] : eep_q[15:8];
					nvr_valid <= 1'b1;
					mstate    <= M_IDLE;
				end

				// EEPROM: same shape, 2 data bytes per entry
				M_EEP_R0: begin
					eep_addr <= eep_addr + 10'd1;
					mstate   <= M_EEP_R1;
				end
				M_EEP_R1: begin
					beat0_hold[15:0] <= eep_q;      // word0's two data bytes
					mstate           <= M_EEP_R2;
				end
				M_EEP_R2: begin
					// rebuild the umask32 view: data bytes on lanes
					// 3 and 1, unused lanes read back FF
					cpu_rdata <= rd64_extract(a[1:0], cpu_size,
								 {beat0_hold[15:8], 8'hFF,
								  beat0_hold[7:0],  8'hFF,
								  eep_q[15:8],      8'hFF,
								  eep_q[7:0],       8'hFF});
					cpu_ack   <= 1'b1;
					mstate    <= M_IDLE;
				end
				M_EEP_W1: begin
					// Gated by the lock like beat 0, so a write to
					// a locked part drives eep_we on neither beat.
					eep_addr <= eep_addr + 10'd1;
					eep_we   <= eep_oe;
					eep_be   <= {w_be8[3], w_be8[1]};
					eep_wd   <= {w_wval64[31:24], w_wval64[15:8]};
					mstate   <= M_EEP_W2;
				end
				M_EEP_W2: begin
					eep_we  <= 1'b0;
					cpu_ack <= 1'b1;
					mstate  <= M_IDLE;
				end

				M_ROM: if (rom_ack) begin
					rom_req <= 1'b0;
					if (w_cross) begin
						beat0_hold <= rom_rdata;
						mstate <= M_ROM2;
					end else begin
						cpu_rdata <= rd64_extract(a[1:0], cpu_size,
												  {rom_rdata, 32'd0});
						cpu_ack <= 1'b1; mstate <= M_IDLE;
					end
				end
				M_ROM2: if (!rom_req) begin
					// issue word1 (one dead cycle after dropping req
					// so the stub's edge logic re-arms; rom stubs
					// ack-then-idle)
					rom_addr <= {a[20:2] + 19'd1, 2'b00};
					rom_req  <= 1'b1;
				end else if (rom_ack) begin
					rom_req <= 1'b0;
					cpu_rdata <= rd64_extract(a[1:0], cpu_size,
											  {beat0_hold, rom_rdata});
					cpu_ack <= 1'b1; mstate <= M_IDLE;
				end

				M_RAM: if (ram_ack) begin
					ram_req <= 1'b0;
					if (w_cross) begin
						beat0_hold <= ram_rdata;   // don't-care on writes
						mstate <= M_RAM2;
					end else begin
						if (!cpu_we)
							cpu_rdata <= rd64_extract(a[1:0], cpu_size,
													  {ram_rdata, 32'd0});
						cpu_ack <= 1'b1; mstate <= M_IDLE;
					end
				end
				M_RAM2: if (!ram_req) begin
					ram_addr  <= {a[18:2] + 17'd1, 2'b00};
					ram_be    <= w_be8[3:0];
					ram_wdata <= w_wval64[31:0];
					ram_req   <= 1'b1;
				end else if (ram_ack) begin
					ram_req <= 1'b0;
					if (!cpu_we)
						cpu_rdata <= rd64_extract(a[1:0], cpu_size,
												  {beat0_hold, ram_rdata});
					cpu_ack <= 1'b1; mstate <= M_IDLE;
				end

				M_CRAM: if (cram_ack) begin
					cram_req <= 1'b0;
					if (cram_second_half) begin
						// first (high) half of a 32-bit access done;
						// hold the read value and issue the low half
						// next cycle
						cram_hi_hold <= prot_sub0_v ? prot_sub0_d
													: cram_rdata;
						mstate <= M_CRAM2;
					end else begin
						if (!cpu_we) begin
							// protection read substitution: 32-bit
							// low half = sub1; 16-bit and byte
							// accesses = sub0 (the RMW read path is
							// untouched: MAME does no protection read
							// for byte writes)
							logic [15:0] cram_fin;
							if (cpu_size == 3'd4)
								cram_fin = prot_sub1_v ? prot_sub1_d
													   : cram_rdata;
							else
								cram_fin = prot_sub0_v ? prot_sub0_d
													   : cram_rdata;
							if (cpu_size == 3'd4)
								cpu_rdata <= {cram_hi_hold, cram_fin};
							else if (cpu_size == 3'd1)
								cpu_rdata <= {24'd0, (a[0] ? cram_fin[7:0]
														  : cram_fin[15:8])};
							else
								cpu_rdata <= {16'd0, cram_fin};
						end
						cpu_ack <= 1'b1; mstate <= M_IDLE;
					end
				end
				M_CRAM2: begin
					// issue the second (low) 16-bit half
					cram_addr  <= (a[18:0] & 19'h7FFFE) + 19'd2;
					cram_wdata <= cpu_wdata[15:0];
					cram_req   <= 1'b1;
					cram_second_half <= 1'b0;
					mstate <= M_CRAM;
				end

				M_CRAM_RMW: if (cram_ack) begin
					// read half of the byte-write RMW: capture the word
					cram_req <= 1'b0;
					cram_rmw_hold <= cram_rdata;
					mstate <= M_CRAM_RMW2;
				end
				M_CRAM_RMW2: begin
					// merge the CPU byte into its half, write it back
					cram_we    <= 1'b1;
					cram_wdata <= a[0] ? {cram_rmw_hold[15:8], cpu_wdata[7:0]}
									   : {cpu_wdata[7:0], cram_rmw_hold[7:0]};
					cram_req   <= 1'b1;
					mstate <= M_CRAM;
				end

				M_CAGE: if (cage_ack) begin
					cage_req <= 1'b0;
					// sized like every other 32-bit device; lane
					// semantics (main vs control halves) live in
					// agt_cage_comm
					if (!cpu_we)
						cpu_rdata <= from_lanes(a[1:0], cpu_size, cage_rdata);
					cpu_ack <= 1'b1; mstate <= M_IDLE;
				end

				default: mstate <= M_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
