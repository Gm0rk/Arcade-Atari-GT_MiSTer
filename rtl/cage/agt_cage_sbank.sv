// agt_cage_sbank.sv -- the sound bank: the DSP's reads of 0xC00000-0xFFFFFF,
// served from the sound data in SDRAM.
//
// What the DSP sees (cage.cpp; cage31.py is the specification): `cage_map`
// maps 0xC00000-0xFFFFFF, read only, to the `cage` region. DSP word W reads
// region bytes 4(W - 0xC00000) .. +3 as one little-endian 32-bit word
// (ROM_REGION32_LE). Primal Rage's two sound ROMs sit word-interleaved at
// region byte 0x400000, so the sound data is DSP words 0xD00000-0xDFFFFF and
// the rest of the bank reads zero. All 160 of the program's sample pointers
// fall inside that range.
//
// Byte order: the .mra streams those 4 MB in region byte order to SDRAM
// byte BASE upward. agt_sdram's download port pairs each even byte with the
// next odd one into one 16-bit word, even byte high, and its cage port reads
// a 32-bit word as two beats, bits 31:16 first. So the port returns stream
// bytes b0 b1 b2 b3 as {b0, b1, b2, b3} (big-endian), and the DSP wants
// {b3, b2, b1, b0}: this module reverses the four bytes. (The maincpu region
// needs no swap because the 68020 is big-endian.)
//
// Path: agt_cage_bus's `s_*` port on clk_dsp (it holds `s_req` until `s_ack`
// and drops it on that edge).
//   inside 0xD00000-0xDFFFFF   agt_cage_membridge #(.AW(20)): one word per
//                              crossing to clk_sys, one SDRAM read, back.
//   anywhere else in the bank  acked at once with zero, as the model's
//                              region reads there.
// On clk_sys this is one master of the cage port, beside the cache's bridge.
//
// Not modelled: on the board the `cage` region also holds the boot EPROM,
// one byte per word at region 0x000000 (ROM_LOAD32_BYTE), so DSP words
// 0xC00000-0xC7FFFF would read it in bits 7:0. cage31.py reads zero there
// too, and the program has not been seen reading the bank outside
// 0xD00000-0xDFFFFF (render_cage_audio.py counts reads by range).
//
// Witness (clk_sys, where the data arrives, so no crossing): the first word
// the SDRAM returned since power-on or a download, byte-reversed as the DSP
// sees it; zero until the DSP first reads the bank. Compare it with the first
// sound-bank read tools/render_cage_audio.py prints: the same bytes reversed
// means the byte order is wrong, anything else the base or the region.
// `n_words` counts every word, for the benches.
`default_nettype none

module agt_cage_sbank #(
	parameter logic [25:0] BASE = 26'h2720000        // SDR_BASE_CAGE
) (
	// clk_dsp: agt_cage_bus's sound-bank port
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	input  wire  [21:0] s_addr,          // word offset from DSP 0xC00000
	input  wire         s_req,
	output wire  [31:0] s_rdata,
	output wire         s_ack,

	// clk_sys: one master of the cage port
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	output wire  [25:0] p_addr,
	output wire         p_we,
	output wire  [31:0] p_wdata,
	output wire         p_req,
	input  wire         p_ack,
	input  wire  [31:0] p_rdata,

	output logic [31:0] witness,
	output logic [31:0] n_words
);

	// 0xD00000-0xDFFFFF is offset 0x100000-0x1FFFFF: bits 21:20 == 01
	wire         in_data = (s_addr[21:20] == 2'b01);

	wire  [31:0] m_rdata;
	wire         m_ack;
	wire  [127:0] b_wdata;              // the bridge's port, a word in [31:0]
	assign p_wdata = b_wdata[31:0];

	agt_cage_membridge #(.BASE(BASE), .AW(20)) u_bridge (
		.clk_dsp(clk_dsp), .rst_dsp_n(rst_dsp_n),
		.m_addr(s_addr[19:0]), .m_req(s_req && in_data), .m_we(1'b0),
		.m_wdata(32'd0), .m_rdata(m_rdata), .m_ack(m_ack), .busy(),
		.clk_sys(clk_sys), .rst_sys_n(rst_sys_n),
		.p_addr(p_addr), .p_we(p_we), .p_line(), .p_wdata(b_wdata), .p_req(p_req),
		.p_ack(p_ack), .p_rdata({96'd0, p_rdata}),
		.n_rd(), .n_wr(), .n_rdl(), .n_wrl(), .n_ovr()
	);

	// the port's big-endian word, as the DSP's little-endian one
	wire  [31:0] le = {m_rdata[7:0], m_rdata[15:8], m_rdata[23:16], m_rdata[31:24]};

	assign s_rdata = in_data ? le    : 32'd0;
	assign s_ack   = in_data ? m_ack : s_req;

	// the witness
	logic got_first;
	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			got_first <= 1'b0;
			witness   <= 32'd0;
			n_words   <= 32'd0;
		end else if (p_req && p_ack) begin
			n_words <= n_words + 32'd1;
			if (!got_first) begin
				got_first <= 1'b1;
				witness   <= {p_rdata[7:0], p_rdata[15:8], p_rdata[23:16], p_rdata[31:24]};
			end
		end
	end

endmodule

`default_nettype wire
