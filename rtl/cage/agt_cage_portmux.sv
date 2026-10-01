// agt_cage_portmux.sv -- two masters on agt_sdram's cage port
//   A  agt_cage_ramload     the boot image at download time
//   B  agt_cage_membridge   the DSP's cache, once the game releases it
//
// They never overlap in the game (the DSP is held in reset until well after
// the download), but the mux arbitrates every request rather than handing the
// port over when ramload is done: a set with no boot EPROM in its .mra leaves
// ramload in LOAD for good, and a re-download restarts ramload with the DSP
// perhaps running.
//
// Idle: grant A if it is requesting, else B. Hold the grant until the port
// acks, then one idle cycle with `p_req` low before the next grant. That is
// the low cycle the controller's `cage_served` gate needs between requests;
// without it, B raising its request as A drops its own would look like one
// request held high and B would never be served. Both masters hold `req`
// until their ack and drop it at the ack.
//
// A has priority: a download is short and bounded, and the DSP is not running
// during a real one. A's requests always have gaps between them, so B is
// never shut out for more than one access at a time.
//
// `*_line` and the 128-bit data pass through with the grant: a line master
// (agt_cage_membridge #(.LINE(1))) asks for four words at once, a word master
// uses bits [31:0] with `line` low (agt_sdram's cage port contract).
`default_nettype none

module agt_cage_portmux (
	input  wire         clk,
	input  wire         rst_n,

	input  wire  [25:0] a_addr,
	input  wire         a_we,
	input  wire         a_line,
	input  wire  [127:0] a_wdata,
	input  wire         a_req,
	output wire         a_ack,
	output wire  [127:0] a_rdata,

	input  wire  [25:0] b_addr,
	input  wire         b_we,
	input  wire         b_line,
	input  wire  [127:0] b_wdata,
	input  wire         b_req,
	output wire         b_ack,
	output wire  [127:0] b_rdata,

	// to agt_sdram's cage port
	output wire  [25:0] p_addr,
	output wire         p_we,
	output wire         p_line,
	output wire  [127:0] p_wdata,
	output wire         p_req,
	input  wire         p_ack,
	input  wire  [127:0] p_rdata,

	output logic [31:0] n_a,            // accesses granted to A
	output logic [31:0] n_b             // accesses granted to B
);

	logic act;                          // a grant is held
	logic gnt;                          // 0 = A, 1 = B

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			act <= 1'b0; gnt <= 1'b0;
			n_a <= 32'd0; n_b <= 32'd0;
		end else if (act) begin
			if (p_ack) act <= 1'b0;                 // next cycle: p_req low
		end else if (a_req) begin
			act <= 1'b1; gnt <= 1'b0; n_a <= n_a + 32'd1;
		end else if (b_req) begin
			act <= 1'b1; gnt <= 1'b1; n_b <= n_b + 32'd1;
		end
	end

	assign p_req   = act && (gnt ? b_req : a_req);
	assign p_addr  = gnt ? b_addr  : a_addr;
	assign p_we    = gnt ? b_we    : a_we;
	assign p_line  = gnt ? b_line  : a_line;
	assign p_wdata = gnt ? b_wdata : a_wdata;

	assign a_ack   = act && !gnt && p_ack;
	assign b_ack   = act &&  gnt && p_ack;
	assign a_rdata = p_rdata;
	assign b_rdata = p_rdata;

endmodule

`default_nettype wire
