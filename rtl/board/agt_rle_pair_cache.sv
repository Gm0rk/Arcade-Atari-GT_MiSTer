// agt_rle_pair_cache.sv -- RLE pair cache for the sprite blit's SDRAM reads
//
// The blit issues 16-bit word requests; SDRAM returns 32-bit pairs. This holds
// the last fetch so the sibling word is a register read instead of a second
// round trip: for sequential access, one round trip per two words.
//
// BURST is the number of adjacent 32-bit pairs one fetch returns:
//   BURST == 1  one pair per fetch (the default)
//   BURST == 3  agt_tile_sdram's `rom_rd3`, which fetches addr+0/+4/+8 in one
//               controller transaction
//
// With BURST > 1 a hit returns a pair that is not the cache tag but tag +
// offset. The top level's corruption witness (`rle_pair_bad`) checks the
// delivered pair, so it must be fed `data_pair`, not the cache base.
//
// A burst must not cross an SDRAM row. The controller forms a burst's column
// as the pair index within a 2 KB row plus 0/1/2, so a burst starting at pair
// 510 or 511 would carry out of the column. Tile data is laid out so no burst
// straddles a row, but sprite data is linear, so the window base is clamped
// inside the row: a request within BURST-1 of the row end fetches from
// ROW_END-(BURST-1) instead. The requested pair is still inside
// [base, base+BURST-1] and is delivered from offset 1 or 2 instead of 0.
//
// The region base must be row-aligned: the clamp works on the local pair
// index, which equals the physical row position only if the region starts on
// a row (SDR_BASE_RLE = 0x720000 = 3648 x 2 KB). Off a 2 KB boundary this
// clamps at the wrong place.
//
// Callers must use:
//   fetch_pair_base  the pair the memory must fetch from, not req_wpair
//   miss_data        the requested pair out of sdr_data on a return; on a
//                    clamped fetch it is not sdr_data[31:0]
//   data_pair        tag+offset for a hit, the in-flight pair for an SDRAM
//                    return (at BURST=1, `hit ? cache_a : fetch_pair`)

module agt_rle_pair_cache #(
	parameter int BURST    = 1,              // adjacent pairs per fetch
	parameter int ROW_BITS = 9               // 2^ROW_BITS pairs per SDRAM row (512 = 2 KB)
)(
	input  wire         clk,
	input  wire         rst_n,

	// consumer side: 16-bit word requests
	input  wire  [24:0] req_wpair,           // which 32-bit pair is wanted
	input  wire         req_rd,              // request strobe

	// memory side
	output wire         fetch_rd,            // issue a fetch (miss only)
	output wire  [24:0] fetch_pair_base,     // fetch from here, not req_wpair
	input  wire         sdr_valid,           // fetch returned
	input  wire [32*BURST-1:0] sdr_data,     // BURST pairs, lowest first

	// results
	output wire         hit,                 // req_wpair is held
	output logic        hit_valid,           // 1-cycle valid for a hit
	output wire  [31:0] data,                // the selected pair (hit path)
	output wire  [31:0] miss_data,           // the requested pair on a return
	output wire  [24:0] data_pair            // pair being delivered (for the witness)
);

	// The last pair index a BURST-wide window may start at and stay inside the
	// row: 509 for BURST=3, 511 for BURST=1 (where nothing clamps).
	localparam int ROW_PAIRS  = 1 << ROW_BITS;
	localparam int LAST_START = ROW_PAIRS - BURST;
	localparam logic [ROW_BITS-1:0] LAST_COL = LAST_START;   // sized, portable

	reg  [24:0]           cache_a;           // base pair of the window
	reg  [32*BURST-1:0]   cache_d;
	reg                   cache_v;

	// window is [cache_a, cache_a + BURST-1]; a request below the base misses
	wire [24:0] off = req_wpair - cache_a;
	assign hit = cache_v && (req_wpair >= cache_a) && (off < BURST[24:0]);

	assign fetch_rd  = req_rd && !hit;
	assign data      = cache_d[32*off[1:0] +: 32];

	// clamp the fetch base so the window never crosses a row
	wire [ROW_BITS-1:0] req_col  = req_wpair[ROW_BITS-1:0];
	wire                clamp    = (BURST > 1) && (req_col > LAST_COL);
	assign fetch_pair_base = clamp ? {req_wpair[24:ROW_BITS], LAST_COL}
								   : req_wpair;

	// the pair requested when the fetch fired (what a return delivers), and
	// the base actually fetched (what the window will be tagged with)
	reg [24:0] fetch_pair;
	reg [24:0] fetch_base;
	always @(posedge clk) if (fetch_rd) begin
		fetch_pair <= req_wpair;
		fetch_base <= fetch_pair_base;
	end

	// the requested pair's position in the return: 0 unless the fetch was
	// clamped. At BURST=1 there is one pair and nothing to select.
	wire [24:0] miss_off = fetch_pair - fetch_base;
	generate if (BURST == 1) begin : g_miss1
		assign miss_data = sdr_data[31:0];
	end else begin : g_missn
		assign miss_data = sdr_data[32*miss_off[1:0] +: 32];
	end endgenerate

	// tag+offset for a hit; the in-flight pair for a return
	assign data_pair = hit ? (cache_a + off) : fetch_pair;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			cache_a   <= 25'd0;
			cache_d   <= {(32*BURST){1'b0}};
			cache_v   <= 1'b0;
			hit_valid <= 1'b0;
		end else begin
			hit_valid <= req_rd && hit;
			if (sdr_valid) begin
				cache_a <= fetch_base;
				cache_d <= sdr_data;
				cache_v <= 1'b1;
			end
		end
	end

endmodule
