// agt_cage_bus.sv -- the TMS320C31's memory map, outside the core.
//
// `agt_c31` answers IRAM (0x809800-0x809FFF) and the peripheral file
// (0x808000-0x8080FF) itself. Everything else leaves on its memory port and
// this module routes it as `cage31.py`'s `rmem`/`wmem` (the specification)
// do:
//
//     rmem                               wmem
//     addr < 0x10000   -> cageram        addr < 0x10000   -> cageram
//     0x400000-47FFFF  -> boot ROM byte  (no case)        -> dropped (oob)
//     0xA00000         -> mailbox_in,    0xA00000         -> mailbox_out,
//                         IRQ0 drops                          IOF 0x80
//     0xC00000-FFFFFF  -> sound bank     (no case)        -> dropped (oob)
//     (no case)        -> 0 (oob)        0x200000         -> nopw, dropped
//     anything else    -> 0 (oob)        anything else    -> dropped (oob)
//
// Both asymmetries are the model's: a read of 0x200000 is out of bounds and
// returns zero, and a write to the boot ROM or the sound bank goes nowhere.
//
// Accept rule: `c_req && !c_ack && idle`. `agt_cage_top` uses a `served`
// latch instead because its master holds `req` until the ack and then goes
// idle. `agt_c31` reacts to `ack` at the very next edge, either dropping
// `req` or raising it again with a new address: four wait states (S_3RD1,
// S_MA1, S_STI2, S_PAR_RD) set `i_req` inside the ack branch, with no low
// cycle. A latch waiting for `req` to fall would deadlock the DSP on its
// first three-operand instruction. With a registered ack, the only cycle in
// which `c_req` still shows the old request is the one `c_ack` is high, and
// `!c_ack` blocks exactly that cycle. The gate encodes the master's
// protocol. (`tb_c31_lockstep` acks combinationally, so it has no ack cycle
// and cannot show this; every real slave acks from a register.)
//
// Cageram passes straight through, wires both ways:
//     d_req   = c_req && is_ram && the bus is idle     d_addr/we/wdata = c_*
//     c_ack   = the cache's ack | this module's own     c_rdata = the cache's
//                                                        word on its ack
// so a data-cache hit takes three cycles: accept, look, ack. The other
// targets (mailbox, sound bank, boot ROM, nopw, out of bounds) are rare and
// served from this module's registers. The cache also accepts only on
// `c_req && !c_ack`, so the protocol holds one level down, and `c_ack` here
// is the OR, so neither kind of ack lets the old request in.
//
// Cageram accesses are counted at the cache's ack, while the core still
// presents the request; one in flight at the end of a window counts when it
// completes. With the core held, `d_req` falls at once while the cache may
// still be filling, so `mem_busy` (agt_cage.sv) uses the cache's own `busy`.
`default_nettype none

module agt_cage_bus (
	input  wire         clk,
	input  wire         rst_n,

	// the core: agt_c31's memory port
	input  wire  [23:0] c_addr,
	input  wire         c_req,
	input  wire         c_we,
	input  wire  [31:0] c_wdata,
	input  wire         c_ifetch,
	output wire  [31:0] c_rdata,           // wires on the cageram path
	output wire         c_ack,

	// cageram, through agt_cage_dcache's CPU-side port
	output wire  [15:0] d_addr,            // wires, straight from the core
	output wire         d_req,
	output wire         d_we,
	output wire  [31:0] d_wdata,
	input  wire  [31:0] d_rdata,
	input  wire         d_ack,

	// the mailbox: agt_cage_top's DSP side
	input  wire  [15:0] mb_from_main,
	output logic        mb_cmd_read,       // one cycle per mailbox read
	output logic        mb_resp_we,        // one cycle per mailbox write
	output logic [15:0] mb_resp_data,
	// Hold a mailbox write while high. agt_cage_mbx_cdc raises it while the
	// previous response is still crossing to clk_sys, so a second cannot
	// overwrite the first in flight. Reads never wait (their crossing coalesces
	// exactly). Tie low to never hold.
	input  wire         mb_wait,

	// the sound bank: read-only, 4M words from 0xC00000
	output logic [21:0] s_addr,
	output logic        s_req,
	input  wire  [31:0] s_rdata,
	input  wire         s_ack,

	// the boot ROM: 8 bits wide, 512 KB from 0x400000
	// The C3x loader reads an 8-bit source; `rmem` returns `self.boot[off]`, one
	// byte, and so does this.
	output logic [18:0] b_addr,
	output logic        b_req,
	input  wire  [7:0]  b_rdata,
	input  wire         b_ack,

	// counters, named after `cage31.py`'s own `counts[...]`
	output logic [31:0] n_fetch, n_ram_r, n_ram_w,
	output logic [31:0] n_mail_r, n_mail_w, n_nopw,
	output logic [31:0] n_sound_r, n_boot_r, n_oob
);

	// decode, from rmem/wmem
	wire is_ram  = (c_addr[23:16] == 8'h00);          // addr < 0x10000
	wire is_boot = (c_addr[23:19] == 5'b01000);       // 0x400000-0x47FFFF
	wire is_mail = (c_addr == 24'hA00000);
	wire is_snd  = (c_addr[23:22] == 2'b11);          // 0xC00000-0xFFFFFF
	wire is_nopw = (c_addr == 24'h200000);

	typedef enum logic [1:0] { B_IDLE, B_SND, B_BOOT } bst_e;
	bst_e bst;

	// cageram: wires both ways
	logic        r_ack;                           // every other target's ack
	logic [31:0] r_rdata;
	assign d_req   = c_req && is_ram && (bst == B_IDLE);
	assign d_addr  = c_addr[15:0];
	assign d_we    = c_we;
	assign d_wdata = c_wdata;
	assign c_ack   = r_ack | d_ack;
	assign c_rdata = d_ack ? d_rdata : r_rdata;

	// the other targets: accepted here, from a register
	wire take = c_req && !c_ack && (bst == B_IDLE) && !is_ram
			  && !(is_mail && c_we && mb_wait);       // a response still crossing

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			bst          <= B_IDLE;
			r_ack        <= 1'b0;
			r_rdata      <= 32'd0;
			s_req        <= 1'b0;  s_addr <= 22'd0;
			b_req        <= 1'b0;  b_addr <= 19'd0;
			mb_cmd_read  <= 1'b0;
			mb_resp_we   <= 1'b0;
			mb_resp_data <= 16'd0;
			n_fetch   <= 32'd0; n_ram_r  <= 32'd0; n_ram_w <= 32'd0;
			n_mail_r  <= 32'd0; n_mail_w <= 32'd0; n_nopw  <= 32'd0;
			n_sound_r <= 32'd0; n_boot_r <= 32'd0; n_oob   <= 32'd0;
		end else begin
			r_ack       <= 1'b0;
			mb_cmd_read <= 1'b0;
			mb_resp_we  <= 1'b0;

			// A cageram access is counted when the cache acks it, while the core still
			// presents it.
			if (d_ack && c_req && is_ram) begin
				if (c_we)          n_ram_w <= n_ram_w + 32'd1;
				else if (c_ifetch) n_fetch <= n_fetch + 32'd1;
				else               n_ram_r <= n_ram_r + 32'd1;
			end

			unique case (bst)
			B_IDLE: if (take) begin
				if (is_mail) begin
					// Both directions complete here. The command word is
					// captured before the pulse that clears its ready flag,
					// which is the model's order: `io_iof_clear_in()` then
					// `return self.mailbox_in`, neither affecting the other.
					if (c_we) begin
						mb_resp_we   <= 1'b1;
						mb_resp_data <= c_wdata[15:0];
						n_mail_w     <= n_mail_w + 32'd1;
					end else begin
						mb_cmd_read  <= 1'b1;
						r_rdata      <= {16'd0, mb_from_main};
						n_mail_r     <= n_mail_r + 32'd1;
					end
					r_ack <= 1'b1;
				end else if (is_snd && !c_we) begin
					s_addr    <= c_addr[21:0];
					s_req     <= 1'b1;
					bst       <= B_SND;
					n_sound_r <= n_sound_r + 32'd1;
				end else if (is_boot && !c_we) begin
					b_addr   <= c_addr[18:0];
					b_req    <= 1'b1;
					bst      <= B_BOOT;
					n_boot_r <= n_boot_r + 32'd1;
				end else if (is_nopw && c_we) begin
					r_ack  <= 1'b1;                   // `nopw`: dropped
					n_nopw <= n_nopw + 32'd1;
				end else begin
					// Out of bounds, a read of 0x200000, or a write to the
					// boot ROM or the sound bank: all `oob` in the model.
					r_rdata <= 32'd0;
					r_ack   <= 1'b1;
					n_oob   <= n_oob + 32'd1;
				end
			end

			// Each target acks from a register; drop its request on the edge
			// the ack is seen, as the core does to us.
			B_SND: if (s_ack) begin
				r_rdata <= s_rdata;
				r_ack   <= 1'b1;
				s_req   <= 1'b0;
				bst     <= B_IDLE;
			end

			B_BOOT: if (b_ack) begin
				r_rdata <= {24'd0, b_rdata};
				r_ack   <= 1'b1;
				b_req   <= 1'b0;
				bst     <= B_IDLE;
			end

			default: bst <= B_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
