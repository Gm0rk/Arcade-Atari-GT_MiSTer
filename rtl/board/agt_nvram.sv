`timescale 1ns/1ps
// agt_nvram.sv -- EEPROM persistence over the MiSTer ioctl interface
//
// The EEPROM is a 28C16 (EEPROM_2816 in atarigt.cpp:871), 2,048 bytes,
// mapped at 0xd20000-0xd20fff with umask32 0xff00ff00: data on lanes 3 and 1
// of each 32-bit word, so device byte n lives in word n>>1, high lane for
// even n.
//
// Framework side (sys/hps_io.sv):
//   * restore is an ordinary download (ioctl_download, ioctl_wr, ioctl_dout)
//     on its own ioctl_index; agt_rom_download decodes index 0 only, so the
//     two share the bus.
//   * save is a read stream: hps_io captures ioctl_din, then pulses ioctl_rd
//     for the next byte (FIO_FILE_TX_DAT). The first transfer has skip_add
//     set, so byte 0 must already be valid when the upload begins: it is
//     prefetched on the rising edge of ioctl_upload, not on the first
//     ioctl_rd.
//   * the core requests a save by pulsing ioctl_upload_req with
//     ioctl_upload_index.
//
// The EEPROM arrays are not dual-ported for this: both directions go through
// the memmap's single-writer service ports in idle cycles, like the checksum
// write-back. A second port on those arrays breaks RAM inference.
//
// Main_MiSTer sends the <nvram> image while the core is held in reset
// (user_io_init sets status[0] before arcade_send_rom and clears it only after
// the whole MRA), so the module is split across two resets:
//   * capture, in dl_rst_n (hard reset only, same domain as
//     agt_rom_download's rom_loaded): the download is copied into a 2 KB
//     buffer as it arrives, and nv_pending is raised.
//   * replay, in rst_n (board domain): once the board is out of reset and the
//     download has finished, the buffer is written into the EEPROM through
//     the nvw service port, one byte per ack. nv_restore_done stays low
//     until that completes; the top level holds the CPU on it so the game's
//     first EEPROM reads cannot race the replay.
// nv_pending clears when the replay finishes, so a warm reset does not replay
// the boot image over settings the game has since written. The framework never
// resends index 2 without a reset, so one replay per capture is enough.
module agt_nvram #(
	parameter [7:0] NV_INDEX  = 8'd2,      // must match the .mra <nvram index>
	parameter int   NV_BYTES  = 2048,      // 28C16
	// The save waits until the EEPROM has been quiet for 2^SETTLE clk (~0.35 s
	// at 50 MHz), as the game rewrites a page at a time. Benches override it.
	parameter int   SETTLE    = 24
) (
	input  logic        clk,
	input  logic        rst_n,             // board domain: replay side
	input  logic        dl_rst_n,          // hard reset only: capture side

	// hps_io side
	input  logic        ioctl_download,
	input  logic        ioctl_upload,
	input  logic        ioctl_wr,
	input  logic        ioctl_rd,
	input  logic [26:0] ioctl_addr,
	input  logic [7:0]  ioctl_dout,
	input  logic [15:0] ioctl_index,
	output logic [7:0]  ioctl_din,
	output logic        ioctl_upload_req,
	output logic [7:0]  ioctl_upload_index,
	output logic        nv_wait,           // OR into ioctl_wait

	// memmap service ports
	output logic        nvw_req,
	output logic [10:0] nvw_addr,
	output logic [7:0]  nvw_data,
	input  logic        nvw_ack,
	output logic        nvr_req,
	output logic [10:0] nvr_addr,
	input  logic [7:0]  nvr_data,
	input  logic        nvr_valid,

	// activity
	input  logic        eeprom_wr_evt,
	output logic        nv_dirty,          // for the debug overlay
	output logic        nv_restore_done    // hold the CPU until high
);

	localparam logic [10:0] LAST_ADDR = 11'(NV_BYTES - 1);

	wire nv_sel   = (ioctl_index[7:0] == NV_INDEX);
	wire in_range = (ioctl_addr < NV_BYTES[26:0]);

	// Restore capture (dl_rst_n): download -> 2 KB buffer.
	// No reset on the array, so it infers as RAM; the ramstyle keeps Quartus from
	// silently demoting it to registers when memory runs short. no_rw_check is
	// safe: the buffer is written only during an NVRAM download, the replay
	// starts only after the download ends (R_IDLE's guard) and uses rp_q only in
	// R_ISSUE.
	(* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  nv_buf [0:NV_BYTES-1];
	wire         cap_wr = ioctl_download && nv_sel && ioctl_wr && in_range;
	always_ff @(posedge clk) begin
		if (cap_wr) nv_buf[ioctl_addr[10:0]] <= ioctl_dout;
	end

	logic        nv_pending;
	logic        replay_done;      // one-cycle pulse from the replay FSM
	always_ff @(posedge clk or negedge dl_rst_n) begin
		if (!dl_rst_n) begin
			nv_pending <= 1'b0;
		end else begin
			if (cap_wr)           nv_pending <= 1'b1;
			else if (replay_done) nv_pending <= 1'b0;
		end
	end

	// Capture is a one-cycle BRAM write; nothing to stall the HPS for.
	assign nv_wait = 1'b0;

	// Restore replay (rst_n): buffer -> EEPROM via the nvw service port
	typedef enum logic [1:0] { R_IDLE, R_RD, R_ISSUE, R_WAIT } rstate_t;
	rstate_t     rst_st;
	logic [10:0] rp_addr;
	logic [7:0]  rp_q;
	// registered read port, one cycle behind rp_addr
	always_ff @(posedge clk) rp_q <= nv_buf[rp_addr];

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			nvw_req  <= 1'b0;
			nvw_addr <= 11'd0;
			nvw_data <= 8'd0;
			rst_st   <= R_IDLE;
			rp_addr  <= 11'd0;
			replay_done     <= 1'b0;
			nv_restore_done <= 1'b0;
		end else begin
			replay_done <= 1'b0;
			case (rst_st)
				R_IDLE: begin
					// Replay only once the download has ended, so a board reset
					// released mid-transfer cannot replay half an image.
					// !replay_done: the pulse that clears nv_pending lags the
					// return to R_IDLE by a cycle; without the guard the FSM would
					// replay the whole image again on the stale flag.
					if (nv_pending && !ioctl_download && !replay_done) begin
						rp_addr <= 11'd0;
						rst_st  <= R_RD;
					end else if (!nv_pending) begin
						nv_restore_done <= 1'b1;
					end
				end
				R_RD:    rst_st <= R_ISSUE;              // rp_q catches up
				R_ISSUE: begin
					nvw_addr <= rp_addr;
					nvw_data <= rp_q;
					nvw_req  <= 1'b1;
					rst_st   <= R_WAIT;
				end
				R_WAIT: if (nvw_ack) begin
					nvw_req <= 1'b0;
					if (rp_addr == LAST_ADDR) begin
						replay_done <= 1'b1;
						rst_st      <= R_IDLE;            // nv_pending clears; the next pass sets done
					end else begin
						rp_addr <= rp_addr + 11'd1;
						rst_st  <= R_RD;
					end
				end
				default: rst_st <= R_IDLE;
			endcase
		end
	end

	// save: prefetch a byte, hand it over on each ioctl_rd
	logic        upload_d;
	logic [10:0] up_addr;
	logic        fetching;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			upload_d  <= 1'b0;
			up_addr   <= 11'd0;
			fetching  <= 1'b0;
			nvr_req   <= 1'b0;
			nvr_addr  <= 11'd0;
			ioctl_din <= 8'd0;
		end else begin
			upload_d <= ioctl_upload;

			if (ioctl_upload && !upload_d && nv_sel) begin
				// byte 0 must be ready before the first strobe (skip_add)
				up_addr  <= 11'd0;
				nvr_addr <= 11'd0;
				nvr_req  <= 1'b1;
				fetching <= 1'b1;
			end
			else if (fetching && nvr_valid) begin
				ioctl_din <= nvr_data;
				nvr_req   <= 1'b0;
				fetching  <= 1'b0;
			end
			else if (ioctl_upload && nv_sel && ioctl_rd && !fetching) begin
				// Compare with LAST_ADDR: NV_BYTES (2048) does not fit in 11
				// bits, so `up_addr + 1 < NV_BYTES[10:0]` would never pass.
				if (up_addr < LAST_ADDR) begin
					up_addr  <= up_addr + 11'd1;
					nvr_addr <= up_addr + 11'd1;
					nvr_req  <= 1'b1;
					fetching <= 1'b1;
				end
			end
		end
	end

	// dirty tracking and the save request
	logic [SETTLE:0] quiet;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			nv_dirty           <= 1'b0;
			quiet              <= '0;
			ioctl_upload_req   <= 1'b0;
			ioctl_upload_index <= NV_INDEX;
		end else begin
			ioctl_upload_req <= 1'b0;
			ioctl_upload_index <= NV_INDEX;

			if (eeprom_wr_evt) begin
				nv_dirty <= 1'b1;
				quiet    <= '0;                 // restart the settle window
			end
			else if (nv_dirty && !ioctl_upload && !ioctl_download) begin
				if (quiet[SETTLE]) begin
					ioctl_upload_req <= 1'b1;   // one pulse, then clean
					nv_dirty         <= 1'b0;
					quiet            <= '0;
				end else
					quiet <= quiet + 1'b1;
			end
		end
	end

endmodule
