// agt_cdc_hs.sv -- one W-bit word across a clock boundary, handshaken
//
// The primitive every CAGE crossing is built from (the cache's memory
// traffic, the mailbox, the boot words into IRAM): a word that must arrive
// whole, once, in order.
//
// Protocol
//   source (s_clk):  present `s_data` with `s_send` high for one cycle while
//                    `s_busy` is low. A send while busy is ignored and the
//                    word is lost; callers must not rely on it.
//   destination (d_clk): `d_pulse` is high for one cycle per delivered word,
//                    with `d_data` valid in that cycle and held after it.
//
// How (a toggle handshake)
//   s: the word goes into `cdchs_hold`, `cdchs_tog` flips.
//   d: `cdchs_tog` through two flops; on a change, `cdchs_hold` is copied to
//      `cdchs_q` and `cdchs_seen` takes the new value (the ack).
//   s: `cdchs_seen` through two flops; busy until it equals `cdchs_tog`.
// `cdchs_hold` does not change while busy, so the destination copies a
// register that has been stable for at least the two synchroniser cycles
// the toggle took to arrive. That is why the data needs no synchroniser of
// its own.
//
// Cost: about 2-3 destination cycles to arrive, 2-3 source cycles more
// before the next word may be sent.
//
// Reset: both sides must be released from the same reset, synchronised into
// each domain. Resetting one side alone leaves the toggles unequal, and the
// destination then delivers one stale word.
//
// SDC: Arcade-Atari-GT.sdc cuts three paths per instance by these register
// names, so keep them:
//   *cdchs_tog*  -> *cdchs_req_s1*     the request toggle
//   *cdchs_seen* -> *cdchs_ack_s1*     the acknowledge toggle
//   *cdchs_hold* -> *cdchs_q*          the data, stable by construction
// clk_sys and clk_dsp come from one PLL, so TimeQuest times these paths at
// the clocks' closest edge pair unless each is cut. Cut by name, never by
// clock group: a group would also silently un-time every future crossing.
`default_nettype none

module agt_cdc_hs #(
	parameter int W = 32
) (
	input  wire          s_clk,
	input  wire          s_rst_n,
	input  wire          s_send,
	input  wire  [W-1:0] s_data,
	output wire          s_busy,

	input  wire          d_clk,
	input  wire          d_rst_n,
	output logic         d_pulse,
	output wire  [W-1:0] d_data
);

	// source
	logic         cdchs_tog;
	logic [W-1:0] cdchs_hold;
	logic         cdchs_ack_s1, cdchs_ack_s2;
	logic         cdchs_seen;                      // destination side, below

	always_ff @(posedge s_clk or negedge s_rst_n) begin
		if (!s_rst_n) begin
			cdchs_tog    <= 1'b0;
			cdchs_hold   <= '0;
			cdchs_ack_s1 <= 1'b0;
			cdchs_ack_s2 <= 1'b0;
		end else begin
			cdchs_ack_s1 <= cdchs_seen;
			cdchs_ack_s2 <= cdchs_ack_s1;
			if (s_send && !s_busy) begin
				cdchs_hold <= s_data;
				cdchs_tog  <= ~cdchs_tog;
			end
		end
	end

	assign s_busy = (cdchs_tog != cdchs_ack_s2);

	// destination
	logic         cdchs_req_s1, cdchs_req_s2;
	logic [W-1:0] cdchs_q;

	always_ff @(posedge d_clk or negedge d_rst_n) begin
		if (!d_rst_n) begin
			cdchs_req_s1 <= 1'b0;
			cdchs_req_s2 <= 1'b0;
			cdchs_seen   <= 1'b0;
			cdchs_q      <= '0;
			d_pulse      <= 1'b0;
		end else begin
			cdchs_req_s1 <= cdchs_tog;
			cdchs_req_s2 <= cdchs_req_s1;
			d_pulse      <= 1'b0;
			if (cdchs_req_s2 != cdchs_seen) begin
				cdchs_q    <= cdchs_hold;
				cdchs_seen <= cdchs_req_s2;
				d_pulse    <= 1'b1;
			end
		end
	end

	assign d_data = cdchs_q;

endmodule

`default_nettype wire
