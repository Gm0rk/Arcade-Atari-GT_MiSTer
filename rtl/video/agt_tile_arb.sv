`timescale 1ns/1ps
// agt_tile_arb.sv -- three-way arbiter for the SDRAM tile port.
//
// Priority: chars, then tiles, then the sprite renderer. The renderer runs
// mostly in vblank, when the line renderers do not need the port.
module agt_tile_arb (
	input  wire        clk,
	input  wire        rst_n,

	input  wire [25:0] cp_addr,        // chars (highest priority)
	input  wire        cp_req,
	output wire        cp_ack,

	input  wire [25:0] tp_addr,        // tiles
	input  wire        tp_req,
	output wire        tp_ack,

	input  wire [25:0] rp_addr,        // sprite renderer (lowest)
	input  wire        rp_req,
	output wire        rp_ack,

	// Sprite fetch yield. Tile fetches (3 accesses) and alpha passes (42) leave
	// a few idle cycles of requester turnaround between accesses; without a
	// yield a sprite transaction slips into each gap and delays the next one.
	// rp_quiet is the number of consecutive cycles cp_req and tp_req must both
	// have been low before a sprite request may claim the port (0 = no yield).
	// A value above the turnaround but below the ~9 idle cycles between tiles
	// blocks only the turnarounds; larger values keep sprites out of the whole
	// playfield pass. Set from an OSD option.
	input  wire [3:0]  rp_quiet,
	// Text hold (D-649): the number of consecutive cycles cp_req alone must
	// have been low before a sprite request may claim the port (0 = none).
	// On a line of text the alpha pass asks for a character, waits for it,
	// then spends 14-23 cycles before asking for the next (tb_video_sdram's
	// ALPHA GAPS). A sprite fetch that claims the port in that gap delays the
	// next character by most of its access, 42 times a line: what made the
	// board's late lines late (D-648). With the hold above the longest gap a
	// line of text runs as if no sprite were drawing. Unlike rp_quiet it does
	// not count tile requests, so the playfield pass is not affected.
	//
	// rp_quiet is not this: it lets the sprite claim once the gap has lasted
	// N cycles, i.e. later in the same gap, nearer the next request, and a
	// sprite access that starts later ends later. Yield 8 and 12 made the
	// alpha and playfield passes slower in the bench (D-649), as Yield 12
	// did on the board at D-534.
	input  wire [4:0]  cp_hold,

	output wire [25:0] sdr_addr,
	output wire        sdr_req,
	input  wire        sdr_ack,

	// Three-plane burst flag. Latched from the winner at claim time like
	// addr_q, so a char or sprite access never inherits the playfield's flag.
	// Only tp asserts it; cp/rp tie it low. The 96-bit result is broadcast and
	// only the owner uses it, as with sdr_ack.
	input  wire        tp_burst3,
	input  wire        cp_burst3,
	input  wire        rp_burst3,
	output wire        sdr_burst3,
	input  wire [95:0] sdr_data96,
	// The request on sdr_req is the line renderer's (chars or tiles), not the
	// sprite renderer's: agt_sdram may put it first when the line is late.
	output wire        sdr_video,
	// The request on sdr_req is the sprite renderer's, and a line-renderer
	// request (chars or tiles) is waiting behind it: whatever delays the
	// sprite access delays the line too (D-648, Video Priority Always+Sprites).
	output wire        sdr_video_wait,
	output wire [95:0] tp_data96,
	output wire [95:0] cp_data96,
	output wire [95:0] rp_data96
);

	// Ownership is a registered claim held for the whole transaction:
	// agt_sdram latches the address at acceptance and answers many cycles
	// later, so the ack must go to the requestor that was granted, not to
	// whoever has priority now. After each ack sdr_req drops for one cycle,
	// because agt_sdram clears its `tile_served` interlock only while
	// `tile_req` is low; without the gap, overlapping requests would keep
	// sdr_req high and the port would stop serving.
	localparam logic [1:0] OWN_CP = 2'd0, OWN_TP = 2'd1, OWN_RP = 2'd2;

	logic [1:0]  owner;
	logic        burst_q;      // granted request's burst flag
	logic [25:0] addr_q;
	logic        busy;
	logic        gap;            // one req-low cycle so tile_served clears

	// Cycles since cp_req or tp_req was last seen, saturating. A gated sprite
	// request is level-held and wins as soon as the gate opens.
	logic [3:0]  hp_idle;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)                  hp_idle <= 4'd0;
		else if (cp_req | tp_req)    hp_idle <= 4'd0;
		else if (hp_idle != 4'd15)   hp_idle <= hp_idle + 4'd1;
	end
	// Cycles since cp_req was last seen, saturating: the text hold's clock.
	logic [4:0]  cp_idle;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n)                  cp_idle <= 5'd0;
		else if (cp_req)             cp_idle <= 5'd0;
		else if (cp_idle != 5'd31)   cp_idle <= cp_idle + 5'd1;
	end
	wire         rp_admit = (hp_idle >= rp_quiet) && (cp_idle >= cp_hold);
	wire         any_req = cp_req | tp_req | (rp_req & rp_admit);
	wire [1:0]   winner  = cp_req ? OWN_CP : tp_req ? OWN_TP : OWN_RP;
	wire [25:0]  win_addr = cp_req ? cp_addr : tp_req ? tp_addr : rp_addr;
	// Same selection as win_addr, so the flag stays with its address.
	wire         win_burst3 = cp_req ? cp_burst3 : tp_req ? tp_burst3 : rp_burst3;
	wire         claim   = !busy && !gap && any_req;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			owner  <= OWN_CP;
			addr_q <= 26'd0;
			burst_q <= 1'b0;
			busy   <= 1'b0;
			gap    <= 1'b0;
		end else begin
			gap <= 1'b0;
			if (busy) begin
				if (sdr_ack) begin
					busy <= 1'b0;
					gap  <= 1'b1;      // hold sdr_req low for one cycle
				end
			end else if (claim) begin
				owner  <= winner;
				addr_q <= win_addr;
				burst_q <= win_burst3;
				busy   <= 1'b1;
			end
		end
	end

	// Request and address go out in the claim cycle, so arbitration adds no
	// latency; only the one-cycle gap after each ack, which the controller's
	// interlock requires.
	assign sdr_req  = busy | claim;
	assign sdr_addr = busy ? addr_q : win_addr;
	// Latched value while busy, the winner's value on the claim cycle.
	assign sdr_burst3 = busy ? burst_q : win_burst3;
	assign sdr_video  = busy ? (owner != OWN_RP) : (winner != OWN_RP);
	assign sdr_video_wait = busy && (owner == OWN_RP) && (cp_req || tp_req);

	// Broadcast; only the owner acts on it, gated by its own ack.
	assign tp_data96 = sdr_data96;
	assign cp_data96 = sdr_data96;
	assign rp_data96 = sdr_data96;

	assign cp_ack = (busy && sdr_ack && owner == OWN_CP);
	assign tp_ack = (busy && sdr_ack && owner == OWN_TP);
	assign rp_ack = (busy && sdr_ack && owner == OWN_RP);

endmodule
