// agt_cage_mbx_cdc.sv -- the CAGE mailbox across clk_sys / clk_dsp
//
// agt_cage_top (the 68020's side, clk_sys) and agt_cage_bus (the DSP's side,
// clk_dsp) share four things, all zero-latency in cage.cpp:
//   the command word        68020 writes it, DSP reads it
//   IRQ0                    asserted by the write, cleared by the DSP's read
//   the command-read event  clears `cpu_to_cage_ready`
//   the response            DSP writes it, `soundlatch` + `cage_to_cpu_ready`
// Crossed naively, two of these become wrong, not just slow:
//
// 1. IRQ0 would fire twice for one command. agt_c31's IRQs are
//    level-sensitive (ORed into IF every cycle), and the line would stay high
//    for the whole round trip to clk_sys and back. So the DSP side drops its
//    own copy of IRQ0 the cycle after its read, without waiting for clk_sys.
// 2. A late clear could erase a newer command posted while the read of the
//    previous one was crossing. So every post is numbered (`post_seq`,
//    8 bits); the DSP side sends back the number it consumed, and clk_sys
//    clears `cpu_to_cage_ready` only if that is still the current number. The
//    same number limits the local IRQ0 mask to the command that was read, so
//    a newer one still interrupts.
//
// Three crossings, all agt_cdc_hs (covered by the SDC's cdchs_* false paths):
//   down  clk_sys -> clk_dsp  {post_seq, ready, from_main}, re-sent whenever
//         it changes. One bundle, so the DSP never sees a command word with
//         the wrong IRQ state or number. It carries seq_now, which already
//         counts a post in its own pulse cycle, because agt_cage_comm updates
//         `from_main` and `ready` a cycle before post_seq would.
//   up    clk_dsp -> clk_sys  the number each read consumed. Reads that
//         outpace the channel coalesce to the latest, which is exact: an
//         earlier consume changes nothing the latest does not.
//   rsp   clk_dsp -> clk_sys  each response word. The bus holds a mailbox
//         write while `d_wait` is high, so a second response cannot overtake
//         or overwrite one in flight.
//
// agt_cage_bus captures `mb_from_main` at its accept edge and pulses
// `mb_cmd_read` a cycle later, so the consumed number comes from a
// one-cycle-delayed copy: the number that went with the word it returned.
// If the bundle has moved on since, the mask does not match it and the newer
// command interrupts.
//
// Reset: both sides from one reset source, synchronised per domain (as
// agt_cdc_hs requires), not the DSP's own reset: the numbers must survive a
// DSP reset, or a wrapped number could mask a real command.
`default_nettype none

module agt_cage_mbx_cdc (
	// clk_sys: agt_cage_top's DSP-facing side
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	input  wire         s_cmd_post,      // agt_cage_top.cmd_post (the comm's pulse)
	input  wire  [15:0] s_from_main,     // agt_cage_top.from_main
	input  wire         s_ready,         // agt_cage_top.cpu_to_cage_ready
	output wire         s_cmd_read,      // -> agt_cage_top.dsp_cmd_read
	output wire         s_resp_we,       // -> agt_cage_top.dsp_resp_we
	output wire  [15:0] s_resp_data,     // -> agt_cage_top.dsp_resp_data

	// clk_dsp: agt_cage_bus's mailbox side, and the core's IRQ0
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	output wire  [15:0] d_from_main,     // -> agt_cage_bus.mb_from_main
	output wire         d_irq0,          // -> agt_c31.irq_in[0]
	input  wire         d_cmd_read,      // <- agt_cage_bus.mb_cmd_read
	input  wire         d_resp_we,       // <- agt_cage_bus.mb_resp_we
	input  wire  [15:0] d_resp_data,     // <- agt_cage_bus.mb_resp_data
	output wire         d_wait,          // -> agt_cage_bus.mb_wait

	// counters
	output logic [15:0] n_posts,         // clk_sys
	output logic [15:0] n_clears,        // clk_sys: consumes that cleared ready
	output logic [15:0] n_stale          // clk_sys: consumes of a superseded number
);

	// clk_sys
	logic [7:0]  post_seq;
	wire  [7:0]  seq_now = s_cmd_post ? post_seq + 8'd1 : post_seq;
	wire  [24:0] bund    = {seq_now, s_ready, s_from_main};
	logic [24:0] bund_sent;
	logic        bund_valid;
	wire         dn_busy;
	wire         dn_send = !dn_busy && (!bund_valid || bund != bund_sent);

	wire         up_pulse;
	wire  [7:0]  up_seq;

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			post_seq   <= 8'd0;
			bund_sent  <= 25'd0;
			bund_valid <= 1'b0;
			n_posts    <= 16'd0;
			n_clears   <= 16'd0;
			n_stale    <= 16'd0;
		end else begin
			if (s_cmd_post) begin
				post_seq <= post_seq + 8'd1;
				if (n_posts != 16'hFFFF) n_posts <= n_posts + 16'd1;
			end
			if (dn_send) begin
				bund_sent  <= bund;
				bund_valid <= 1'b1;
			end
			if (up_pulse) begin
				if (up_seq == seq_now) begin
					if (n_clears != 16'hFFFF) n_clears <= n_clears + 16'd1;
				end else if (n_stale != 16'hFFFF) n_stale <= n_stale + 16'd1;
			end
		end
	end

	// Combinational, into agt_cage_comm in the same cycle. `seq_now` includes a
	// post pulsing this cycle, and the comm resolves a consume and a post on one
	// edge as consume-then-post, so a command posted at that moment survives.
	assign s_cmd_read = up_pulse && (up_seq == seq_now);

	// the three crossings
	wire         dn_pulse;
	wire  [24:0] dn_q;
	agt_cdc_hs #(.W(25)) u_dn (
		.s_clk(clk_sys), .s_rst_n(rst_sys_n),
		.s_send(dn_send), .s_data(bund), .s_busy(dn_busy),
		.d_clk(clk_dsp), .d_rst_n(rst_dsp_n),
		.d_pulse(dn_pulse), .d_data(dn_q)
	);

	logic [7:0]  cons_seq;               // the number the last read consumed
	logic        cons_pend;              // ... not yet sent
	wire         up_busy;
	wire         up_send = !up_busy && cons_pend;
	agt_cdc_hs #(.W(8)) u_up (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(up_send), .s_data(cons_seq), .s_busy(up_busy),
		.d_clk(clk_sys), .d_rst_n(rst_sys_n),
		.d_pulse(up_pulse), .d_data(up_seq)
	);

	wire rsp_busy;
	agt_cdc_hs #(.W(16)) u_rsp (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(d_resp_we), .s_data(d_resp_data), .s_busy(rsp_busy),
		.d_clk(clk_sys), .d_rst_n(rst_sys_n),
		.d_pulse(s_resp_we), .d_data(s_resp_data)
	);
	assign d_wait = rsp_busy;

	// clk_dsp
	wire  [7:0]  q_seq   = dn_q[24:17];
	wire         q_ready = dn_q[16];
	assign d_from_main   = dn_q[15:0];

	logic [7:0]  q_seq_d;                // the number that went with the word
	logic        mask_on;                // this command has been read
	logic [7:0]  mask_seq;

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			q_seq_d    <= 8'd0;
			mask_on    <= 1'b0;
			mask_seq   <= 8'd0;
			cons_seq   <= 8'd0;
			cons_pend  <= 1'b0;
		end else begin
			q_seq_d <= q_seq;
			if (d_cmd_read) begin
				mask_on    <= 1'b1;
				mask_seq   <= q_seq_d;
				cons_seq   <= q_seq_d;
				cons_pend  <= 1'b1;              // wins over a same-cycle send,
			end else begin                       // which carried the old number
				if (mask_on && (!q_ready || q_seq != mask_seq))
					mask_on <= 1'b0;             // cleared, or superseded
				if (up_send) cons_pend <= 1'b0;
			end
		end
	end

	// A read of an empty mailbox (number 0 after reset, ready low) arrives as
	// the current number and "clears" a flag that is already clear, as the same
	// read does in cage.cpp; it counts in `n_clears`, not `n_stale`.
	assign d_irq0 = q_ready && !(mask_on && q_seq == mask_seq);

endmodule

`default_nettype wire
