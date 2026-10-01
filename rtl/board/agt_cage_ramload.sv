// agt_cage_ramload.sv -- loads the C31 boot image into cageram in SDRAM
//
// Master of agt_sdram's cage_* port, fed by agt_cage_boot's cageram write
// port (the boot EPROM parsed out of the download stream). Three phases,
// once per download:
//   ZERO   write 0 to all 65,536 cageram words. The DSP program reads two
//          cageram words (0x3EF4, 0x3EF5) that the boot table never loads
//          and the program never writes, and expects zero from both, as
//          cage31.py's RAM starts. SDRAM holds whatever it held.
//   LOAD   drain the loader's words, in order, through a small FIFO. The
//          loader makes at most one word per four download bytes, far slower
//          than this port, so the FIFO rarely holds more than one word. When
//          it fills (words arriving during ZERO, or a fast stream) wait_o
//          stalls the download through ioctl_wait.
//   CHECK  read all 65,536 words back through the port and sum them. The sum
//          equals the one computed from the boot table only if the zeroing,
//          every loaded word and the port's write and read paths are right.
//
// ZERO starts on the download's start, not on reset: a warm reset without a
// download must not wipe an image nothing will reload. LOAD ends when the
// loader reports `done` and at least one word arrived in this run: after a
// re-download, `done` stays high from the previous parse until the boot
// region's first byte arrives with `dl_first`, so a bare `done` would end
// LOAD early.
//
// Port protocol: hold req until ack, then one cycle low before the next
// request. The controller's `cage_served` gate must see req low; a master
// that re-raises req in the ack cycle is served once and never again.
// With the controller acking at edge N:
//   N+1  this FSM sees ack: req <= 0            (controller still sees 1)
//   N+2  GAP: req <= 1 with the next address    (controller sees 0: gate clears)
//   N+3  controller sees the new request and accepts it
//
// `witness` (overlay): when done, the 32-bit sum of all words read back;
// otherwise `F00p xxxx` = {12'hF00, 2'b00, phase, progress}, where progress
// is the word index in ZERO and CHECK, and the count of words taken from the
// loader in LOAD (the index does not move in LOAD).
//
// Reset domain: this module, agt_sdram and the parser that feeds it must all
// be in the download domain (dl_rst_n / ~RESET). A warm reset follows every
// download and clears anything in sys_rst_n.
`default_nettype none

module agt_cage_ramload #(
	parameter logic [25:0] BASE       = 26'h2B20000,  // cageram region (SDRAM byte address)
	parameter int          WORDS      = 65536,        // cage_map 0x0000-0xFFFF
	parameter int          FIFO_DEPTH = 8             // a power of two
) (
	input  wire         clk,
	input  wire         rst_n,          // download domain (dl_rst_n)
	input  wire         sdr_ready,
	input  wire         dl_active,      // agt_rom_download.rom_loading

	// from agt_cage_boot's cageram port
	input  wire         ld_we,
	input  wire  [17:0] ld_addr,        // cageram word index
	input  wire  [31:0] ld_data,
	input  wire         ld_done,

	// backpressure to the download (ORed into ioctl_wait)
	output logic        wait_o,

	// master of agt_sdram's cage port
	output logic [25:0] p_addr,
	output logic        p_we,
	output logic [31:0] p_wdata,
	output logic        p_req,
	input  wire         p_ack,
	input  wire  [31:0] p_rdata,

	// status
	output logic [1:0]  phase,          // 0 idle/done, 1 zero, 2 load, 3 check
	output logic        done,           // CHECK finished; `sum` is final
	output logic [31:0] sum,
	output logic [15:0] n_loaded,       // words taken from the loader this run
	output logic [15:0] n_dropped,      // words that found the FIFO full: must be 0
	output logic [31:0] witness
);

	localparam int AW = $clog2(FIFO_DEPTH);

	// FIFO of {word index, data}. Pinned to registers (8 x 48 bits, read
	// asynchronously) so it takes no M10K: block RAM is short.
	(* ramstyle = "logic" *) logic [15:0]  f_addr [0:FIFO_DEPTH-1];
	(* ramstyle = "logic" *) logic [31:0]  f_data [0:FIFO_DEPTH-1];
	logic [AW-1:0] f_wp, f_rp;
	logic [AW:0]   f_cnt;
	wire           f_empty = (f_cnt == '0);
	wire           f_full  = (f_cnt == FIFO_DEPTH[AW:0]);
	// Stall the download with two words of margin: the stream may deliver a
	// byte or two after ioctl_wait rises, and a word needs four.
	assign wait_o = (f_cnt >= FIFO_DEPTH[AW:0] - 2);

	// sequencing
	typedef enum logic [2:0] { S_IDLE, S_ISSUE, S_WAIT, S_GAP, S_DONE } st_e;
	st_e  st;
	logic [16:0] idx;                   // 0..WORDS
	logic        dl_q, start_pend;
	logic        pop;                   // this cycle's LOAD word leaves the FIFO

	wire  dl_rise = dl_active && !dl_q;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			st <= S_IDLE; phase <= 2'd0; idx <= '0;
			dl_q <= 1'b0; start_pend <= 1'b0;
			p_req <= 1'b0; p_we <= 1'b0; p_addr <= '0; p_wdata <= '0;
			done <= 1'b0; sum <= '0; n_loaded <= '0; n_dropped <= '0;
			f_wp <= '0; f_rp <= '0; f_cnt <= '0;
		end else begin
			dl_q <= dl_active;
			pop  = 1'b0;

			// A download start (re)arms the whole sequence. A new start in
			// the middle of a run is honoured when the current access ends.
			if (dl_rise) start_pend <= 1'b1;

			case (st)
			S_IDLE, S_DONE: if (start_pend && sdr_ready) begin
				start_pend <= 1'b0;
				done <= 1'b0; sum <= '0; n_loaded <= '0; n_dropped <= '0;
				phase <= 2'd1; idx <= '0;
				st <= S_GAP;                        // the GAP state issues
			end

			// Raise the request for the current phase and index. Entered
			// one cycle after req fell, so the controller has seen it low.
			S_GAP: begin
				if (start_pend) begin
					// re-armed mid-run: start over from ZERO. The FIFO is left
					// alone: anything in it is a loaded word, and the new
					// download's table overwrites it anyway.
					start_pend <= 1'b0;
					done <= 1'b0; sum <= '0; n_loaded <= '0; n_dropped <= '0;
					phase <= 2'd1; idx <= '0;
				end else if (phase == 2'd1) begin
					if (idx == WORDS[16:0]) begin
						phase <= 2'd2; idx <= '0;   // ZERO done -> LOAD
					end else begin
						p_addr <= BASE + {8'd0, idx[15:0], 2'b00};
						p_we <= 1'b1; p_wdata <= 32'd0; p_req <= 1'b1;
						st <= S_WAIT;
					end
				end else if (phase == 2'd2) begin
					if (!f_empty) begin
						p_addr  <= BASE + {8'd0, f_addr[f_rp], 2'b00};
						p_we    <= 1'b1;
						p_wdata <= f_data[f_rp];
						p_req   <= 1'b1;
						pop = 1'b1;
						st <= S_WAIT;
					end else if (ld_done && !ld_we && n_loaded != 16'd0) begin
						// `!ld_we`: never leave LOAD in the cycle a word is still
						// being pushed, or it would sit in the FIFO forever.
						// `n_loaded != 0`: see the header.
						phase <= 2'd3; idx <= '0;   // LOAD done -> CHECK
					end
				end else if (phase == 2'd3) begin
					if (idx == WORDS[16:0]) begin
						phase <= 2'd0; done <= 1'b1;
						st <= S_DONE;
					end else begin
						p_addr <= BASE + {8'd0, idx[15:0], 2'b00};
						p_we <= 1'b0; p_req <= 1'b1;
						st <= S_WAIT;
					end
				end else begin
					st <= S_IDLE;
				end
			end

			S_WAIT: if (p_ack) begin
				p_req <= 1'b0;                      // the one low cycle
				if (phase == 2'd3) sum <= sum + p_rdata;
				if (phase != 2'd2) idx <= idx + 17'd1;
				st <= S_GAP;
			end

			default: st <= S_IDLE;
			endcase

			// FIFO bookkeeping, after the FSM's pop decision
			if (ld_we) begin
				if (!f_full || pop) begin
					f_addr[f_wp] <= ld_addr[15:0];
					f_data[f_wp] <= ld_data;
					f_wp <= f_wp + 1'b1;
					if (n_loaded != 16'hFFFF) n_loaded <= n_loaded + 16'd1;
				end else if (n_dropped != 16'hFFFF)
					n_dropped <= n_dropped + 16'd1;
			end
			if (pop) f_rp <= f_rp + 1'b1;
			case ({ld_we && (!f_full || pop), pop})
				2'b10:   f_cnt <= f_cnt + 1'b1;
				2'b01:   f_cnt <= f_cnt - 1'b1;
				default: ;
			endcase
		end
	end

	// Reads as `F00p xxxx` on the overlay: p, the phase, is the last digit of
	// the upper half.
	assign witness = done ? sum
				   : {12'hF00, 2'b00, phase,
					  (phase == 2'd2) ? n_loaded : idx[15:0]};

endmodule

`default_nettype wire
