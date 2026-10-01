// agt_cage_dac.sv -- the C31's serial port and the board's DACs: the words the
// DSP's DMA sends, played in the DSP's own time.
//
// The program (MAME cage.cpp; cage31.py is the specification) mixes into a
// buffer in on-chip RAM and DMAs it to the serial port (DMA_DEST 0x808048),
// 128 words a burst, re-arming the DMA from DINT0 at the end of each burst.
// The words are four interleaved channels (word i of a burst is channel
// i mod 4), which `atarigt_stereo` mixes as
//     left = (ch1 + ch2) / 2        right = (ch0 + ch3) / 2
// (tools/render_cage_audio.py does the same). One word plays every
// `serial_per_word` H1 cycles: 384 in Primal Rage, so 11,025 Hz a channel.
//
// agt_c31 hands over each burst word (`dac_*`) and the model's cycles as they
// advance (`mcyc_*`). A free-running serial clock counts model cycles, not
// clk_dsp's: every `dac_per` of them it moves the next queued word, if any,
// into its channel's register. So the port plays in the DSP's own time, like
// the core's timers. A tick that finds the queue empty holds the last value.
//
// The queue never needs more than one burst: a burst's 128 words take 128
// ticks, the last no later than DINT0, and the program re-arms the DMA only
// after DINT0. `n_ovr` counts words that found it full; the benches require 0.
//
// Crossing (clk_dsp -> clk_sys): each new left/right pair goes through one
// agt_cdc_hs with an 8-bit sequence number (words taken). A pair produced
// while the crossing is busy replaces the waiting one; the sequence number
// still counts every word, so `n_words` is exact.
//
// Witness (clk_sys, for the overlay):
//   [31:16] largest |left| or |right| delivered since reset
//   [15:0]  words delivered in the last WINDOW_VBL vblanks (the chip plays
//           44,100 a second)
//
// Resets: `rst_dsp_n` and `rst_sys_n` are released from the same reset
// (agt_cdc_hs's rule). `flush` (the core held) drops the queue and restarts
// the channel count; the outputs hold.
`default_nettype none

module agt_cage_dac #(
	parameter int WINDOW_VBL = 60
) (
	// clk_dsp: agt_c31's serial-port feed and the model's clock
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	input  wire         flush,          // the core held: drop the queue
	input  wire         dac_stb,
	input  wire  [15:0] dac_word,
	input  wire         dac_first,
	input  wire         mcyc_stb,
	input  wire  [4:0]  mcyc_add,
	input  wire  [24:0] dac_per,

	// clk_sys: the output and the witness
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	input  wire         vblank,
	output logic [15:0] audio_l,
	output logic [15:0] audio_r,
	output wire  [31:0] witness,

	// for the benches
	output logic [31:0] n_push,         // clk_dsp: words queued
	output logic [31:0] n_pop,          // clk_dsp: words played
	output logic [15:0] n_ovr,          // clk_dsp: words that found it full
	output logic [31:0] n_words         // clk_sys: words delivered
);

	// clk_dsp: the queue, 128 x {channel, word}. A word is always read at least
	// one edge after it was written (see below), so `no_rw_check` loses nothing.
	(* ramstyle = "MLAB, no_rw_check" *) logic [17:0] dac_fifo [0:127];

	logic [6:0]  wp, rp;
	logic [7:0]  cnt;                   // 0..128
	logic [1:0]  ch_next;               // the next word's channel
	logic [17:0] fifo_q;

	wire         full = (cnt == 8'd128);
	wire         push = dac_stb && !full && !flush;
	wire  [1:0]  ch_in = dac_first ? 2'd0 : ch_next;

	// The read address runs one ahead during a take, so a take in the very next
	// cycle would still read the right entry (a tick cannot come that often, but
	// the queue does not rely on it).
	logic        take;
	wire  [6:0]  ra = take ? (rp + 7'd1) : rp;

	always_ff @(posedge clk_dsp) begin
		if (push) dac_fifo[wp] <= {ch_in, dac_word};
		fifo_q <= dac_fifo[ra];
	end

	// Serial clock, in model cycles: `sp_cd` is the model cycles left to the next
	// tick. A step earns 1..16 and `dac_per` is at least 32, so a step makes at
	// most one tick; the remainder carries so ticks land on exact multiples.
	logic [25:0] sp_cd;
	wire         tick = mcyc_stb && ({21'd0, mcyc_add} >= sp_cd);

	// Taking a word, two cycles:
	// t   a tick finds a word queued and not already being taken: `take`.
	// t+1 `fifo_q` is that word (queued before t, so written at an earlier edge
	//     than the one that read it). Into its channel; `rp` moves on.
	// t+2 the channel registers hold it: the pair goes to the crossing.
	logic        mixed;
	wire  [7:0]  avail = cnt - {7'd0, take};
	logic [15:0] ch_val [0:3];
	logic [7:0]  seq;                   // words taken, mod 256

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) begin
			wp      <= 7'd0;
			rp      <= 7'd0;
			cnt     <= 8'd0;
			ch_next <= 2'd0;
			sp_cd   <= 26'd0;
			take    <= 1'b0;
			mixed   <= 1'b0;
			seq     <= 8'd0;
			for (int i = 0; i < 4; i = i + 1) ch_val[i] <= 16'd0;
			n_push  <= 32'd0;
			n_pop   <= 32'd0;
			n_ovr   <= 16'd0;
		end else begin
			take  <= 1'b0;
			mixed <= 1'b0;

			if (mcyc_stb)
				sp_cd <= tick ? (sp_cd + {1'b0, dac_per} - {21'd0, mcyc_add})
							  : (sp_cd - {21'd0, mcyc_add});

			if (flush) begin
				rp      <= wp;
				cnt     <= 8'd0;
				ch_next <= 2'd0;
			end else begin
				if (push) begin
					wp      <= wp + 7'd1;
					ch_next <= ch_in + 2'd1;
					n_push  <= n_push + 32'd1;
				end
				if (dac_stb && full && n_ovr != 16'hFFFF)
					n_ovr <= n_ovr + 16'd1;
				if (tick && avail != 8'd0)
					take <= 1'b1;
				if (take) begin
					ch_val[fifo_q[17:16]] <= fifo_q[15:0];
					rp    <= rp + 7'd1;
					seq   <= seq + 8'd1;
					n_pop <= n_pop + 32'd1;
					mixed <= 1'b1;
				end
				cnt <= cnt + {7'd0, push} - {7'd0, take};
			end
		end
	end

	// Mix and crossing. (a + b) / 2 rounds down, as Python's `//` in
	// render_cage_audio.py.
	wire signed [16:0] sum_l = $signed({ch_val[1][15], ch_val[1]}) + $signed({ch_val[2][15], ch_val[2]});
	wire signed [16:0] sum_r = $signed({ch_val[0][15], ch_val[0]}) + $signed({ch_val[3][15], ch_val[3]});

	logic        pend;
	wire         s_busy;
	wire         s_send = pend && !s_busy;
	wire  [39:0] s_data = {seq, sum_l[16:1], sum_r[16:1]};

	always_ff @(posedge clk_dsp or negedge rst_dsp_n) begin
		if (!rst_dsp_n) pend <= 1'b0;
		else if (mixed) pend <= 1'b1;   // the newest pair replaces a waiting one
		else if (s_send) pend <= 1'b0;
	end

	wire         d_pulse;
	wire  [39:0] d_data;

	agt_cdc_hs #(.W(40)) u_out (
		.s_clk(clk_dsp), .s_rst_n(rst_dsp_n),
		.s_send(s_send), .s_data(s_data), .s_busy(s_busy),
		.d_clk(clk_sys), .d_rst_n(rst_sys_n),
		.d_pulse(d_pulse), .d_data(d_data)
	);

	// clk_sys: the output, the word count, the witness
	function automatic [15:0] mag(input [15:0] x);
		mag = x[15] ? (~x + 16'd1) : x;           // |-32768| = 0x8000
	endfunction

	localparam logic [7:0] WIN_LAST = WINDOW_VBL - 1;
	logic [7:0]  last_seq;
	logic [15:0] peak;
	logic        vbl_q;
	logic [7:0]  vcnt;
	logic [31:0] mark;
	logic [15:0] words_last;

	wire  [15:0] mag_l = mag(d_data[31:16]);
	wire  [15:0] mag_r = mag(d_data[15:0]);
	wire  [15:0] mag_max = (mag_l > mag_r) ? mag_l : mag_r;
	wire  [31:0] in_win = n_words - mark;

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			audio_l    <= 16'd0;
			audio_r    <= 16'd0;
			n_words    <= 32'd0;
			last_seq   <= 8'd0;
			peak       <= 16'd0;
			vbl_q      <= 1'b0;
			vcnt       <= 8'd0;
			mark       <= 32'd0;
			words_last <= 16'd0;
		end else begin
			if (d_pulse) begin
				audio_l  <= d_data[31:16];
				audio_r  <= d_data[15:0];
				// every word taken, even when pairs were coalesced
				n_words  <= n_words + {24'd0, d_data[39:32] - last_seq};
				last_seq <= d_data[39:32];
				if (mag_max > peak) peak <= mag_max;
			end
			// A window closes on a vblank's rising edge. This cycle's increment lands
			// in the next window, so nothing is lost at the boundary.
			vbl_q <= vblank;
			if (vblank && !vbl_q) begin
				if (vcnt == WIN_LAST) begin
					vcnt       <= 8'd0;
					words_last <= (in_win[31:16] != 16'd0) ? 16'hFFFF : in_win[15:0];
					mark       <= n_words;
				end else begin
					vcnt <= vcnt + 8'd1;
				end
			end
		end
	end

	assign witness = {peak, words_last};

endmodule

`default_nettype wire
