// agt_cage_boot.sv -- TMS320C31 on-chip boot loader, in hardware
//
// The CAGE board's DSP boots from the EPROM at 11A, mapped at
// 0x400000-0x47FFFF (cage.cpp `cage_map`). Per the TMS320C31 data sheet
// (SPRS035B) Figure 1, 0x400000 is Boot 2 in microcomputer/boot-loader mode:
// MCBL/MP is tied high and the DSP's own on-chip loader does the loading.
// This module reproduces what that loader deposits, once, before the DSP
// starts.
//
// A C3x boot table is strictly sequential: width, bus-control value, then
// (size, dest, data...) blocks, terminated by a zero size. The ioctl download
// delivers the ROM in file order, so the table is parsed as it arrives and
// the RAM written on the way past, with no buffering and no SDRAM port. The
// loader is finished before `dl_rom_loaded` rises.
//
// Byte order: word 0 reads 8 (an 8-bit boot source), and MAME loads the
// EPROM with ROM_LOAD32_BYTE at stride 4 into a ROM_REGION32_LE, so one DSP
// word is four consecutive file bytes, LSB first.
//
// Primal Rage's table loads 22,641 words to three places:
//   18,799 words  0x001000-0x007FFF   CAGE board RAM
//    3,830 words  0x008000-0x008EF5   CAGE board RAM, above 32K words
//       12 words  0x809FC0-0x809FCB   C31 on-chip RAM
// The last is reset plus the interrupt/trap branch table (data sheet
// Figure 2, microcomputer mode: 809FC1=INT0, 809FC2=INT1, 809FC3=INT2,
// 809FC4=INT3, 809FC5=XINT0, 809FC6=RINT0, 809FC9=TINT0, 809FCA=TINT1,
// 809FCB=DINT). On-chip RAM has its own port because it is separate
// silicon: two 1K x 32 blocks at 0x809800-0x809FFF.
//
// `oor_words` counts words whose destination matched neither window, rather
// than dropping them silently; it must read zero for a correctly sized RAM.
// `wr_hi` is the highest CAGE RAM address the boot table writes: what a RAM
// sized to the boot-loaded range must cover (it says nothing about runtime
// writes).
`default_nettype none

module agt_cage_boot #(
	// CAGE board RAM, cage_map 0x000000-0x00FFFF. Sized by the caller; words
	// the loader cannot place are counted in oor_words.
	parameter int CAGERAM_WORDS = 36864,      // 0x9000, covers the boot table
	// C31 on-chip RAM, data sheet Figure 1: 0x809800-0x809FFF, 2K words
	parameter int IRAM_BASE     = 32'h809800,
	parameter int IRAM_WORDS    = 2048
)(
	input  wire         clk,
	input  wire         rst_n,

	// Boot-ROM byte stream, in file order. `dl_valid` is one cycle per byte.
	// `dl_first` restarts the parse, so a re-download cannot leave the FSM
	// half way through a block.
	input  wire         dl_first,
	input  wire         dl_valid,
	input  wire  [7:0]  dl_byte,

	// CAGE board RAM write port
	output logic        ram_we,
	output logic [17:0] ram_addr,
	output logic [31:0] ram_data,

	// C31 on-chip RAM write port
	output logic        iram_we,
	output logic [10:0] iram_addr,
	output logic [31:0] iram_data,

	// status (each one is checked by the bench)
	output logic        done,           // zero-size block reached: table complete
	output logic        bad,            // width not 8/16/32
	output logic [31:0] boot_width,
	output logic [31:0] boot_bctrl,     // 0x00001058 for Primal Rage
	output logic [23:0] entry,          // destination of the first block
	output logic [15:0] words_loaded,
	output logic [15:0] blocks_loaded,
	output logic [23:0] wr_hi,          // highest CAGE RAM address written
	output logic [15:0] oor_words       // must be zero
);

	typedef enum logic [2:0] {
		S_WIDTH, S_BCTRL, S_SIZE, S_DEST, S_DATA, S_DONE, S_BAD
	} state_t;
	state_t state;

	logic [31:0] wbuf;
	logic [1:0]  bcnt;
	logic        wvalid;                 // a complete 32-bit word this cycle
	// byte index for this cycle: 0 when dl_first comes with the byte,
	// otherwise the running count
	wire [1:0]   bidx = dl_first ? 2'd0 : bcnt;
	logic [31:0] wdata;

	// little-endian byte assembly: first byte is bits 7:0
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wbuf <= 32'd0; bcnt <= 2'd0; wvalid <= 1'b0; wdata <= 32'd0;
		end else begin
			wvalid <= 1'b0;
			// `dl_first` arrives with byte 0 on the same edge (the top level
			// drives it as `cageboot_wr && (cageboot_addr == 0)`), so the
			// byte is taken first; dl_first alone only rewinds.
			if (dl_valid) begin
				case (bidx)
					2'd0: wbuf[7:0]   <= dl_byte;
					2'd1: wbuf[15:8]  <= dl_byte;
					2'd2: wbuf[23:16] <= dl_byte;
					2'd3: begin
						wdata  <= {dl_byte, wbuf[23:0]};
						wvalid <= 1'b1;
					end
				endcase
				bcnt <= bidx + 2'd1;
			end else if (dl_first) begin
				// dl_first without a byte: a restart, so just rewind.
				bcnt <= 2'd0;
			end
		end
	end

	logic [31:0] blk_size, blk_dest;

	wire in_cageram = (blk_dest < CAGERAM_WORDS);
	wire in_iram    = (blk_dest >= IRAM_BASE) &&
					  (blk_dest <  IRAM_BASE + IRAM_WORDS);

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_WIDTH;
			ram_we <= 1'b0; iram_we <= 1'b0;
			ram_addr <= 18'd0; ram_data <= 32'd0;
			iram_addr <= 11'd0; iram_data <= 32'd0;
			done <= 1'b0; bad <= 1'b0;
			boot_width <= 32'd0; boot_bctrl <= 32'd0; entry <= 24'd0;
			words_loaded <= 16'd0; blocks_loaded <= 16'd0;
			wr_hi <= 24'd0; oor_words <= 16'd0;
			blk_size <= 32'd0; blk_dest <= 32'd0;
		end else begin
			ram_we  <= 1'b0;
			iram_we <= 1'b0;

			if (dl_first) begin
				// a re-download must not resume mid-block
				state <= S_WIDTH;
				done <= 1'b0; bad <= 1'b0;
				words_loaded <= 16'd0; blocks_loaded <= 16'd0;
				wr_hi <= 24'd0; oor_words <= 16'd0;
			end else if (wvalid) begin
				unique case (state)
					S_WIDTH: begin
						boot_width <= wdata;
						// data sheet: an 8/16/32-bit boot source. Anything else is
						// not a boot table: stop rather than write noise.
						if (wdata == 32'd8 || wdata == 32'd16 ||
							wdata == 32'd32) state <= S_BCTRL;
						else begin bad <= 1'b1; state <= S_BAD; end
					end
					S_BCTRL: begin
						boot_bctrl <= wdata;
						state      <= S_SIZE;
					end
					S_SIZE: begin
						if (wdata == 32'd0) begin
							done  <= 1'b1;
							state <= S_DONE;
						end else begin
							blk_size <= wdata;
							state    <= S_DEST;
						end
					end
					S_DEST: begin
						blk_dest <= wdata;
						if (blocks_loaded == 16'd0) entry <= wdata[23:0];
						blocks_loaded <= blocks_loaded + 16'd1;
						state <= S_DATA;
					end
					S_DATA: begin
						if (in_cageram) begin
							ram_we   <= 1'b1;
							ram_addr <= blk_dest[17:0];
							ram_data <= wdata;
							if (blk_dest[23:0] > wr_hi) wr_hi <= blk_dest[23:0];
						end else if (in_iram) begin
							iram_we   <= 1'b1;
							iram_addr <= (blk_dest - IRAM_BASE);
							iram_data <= wdata;
						end else begin
							if (oor_words != 16'hFFFF)
								oor_words <= oor_words + 16'd1;
						end
						words_loaded <= words_loaded + 16'd1;
						blk_dest <= blk_dest + 32'd1;
						blk_size <= blk_size - 32'd1;
						if (blk_size == 32'd1) state <= S_SIZE;
					end
					S_DONE: ;   // trailing bytes after the terminator: ignored
					S_BAD:  ;
				endcase
			end
		end
	end

endmodule

`default_nettype wire
