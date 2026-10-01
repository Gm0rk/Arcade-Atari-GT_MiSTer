// agt_tile_sdram.sv -- serve the video tile fetcher from SDRAM
//
// The renderer's tile ROM interface is BRAM-shaped: a one-cycle `rd` pulse
// with the address held, then a `data_valid` pulse carrying the byte (see
// agt_tile_decode's ST_REQ_* / ST_WAIT_* pairs). The SDRAM tile port is a
// req/ack handshake that must also see `req` drop before it accepts the next
// access (`tile_served` clears on `!tile_req`). This adapter translates
// between them and adds the region base, so the renderer is unchanged and
// the controller stays region-free.
//
// SDRAM latency fits because the fetcher's one-tile-row lookahead overlaps a
// row's fetch with the previous row's pixel work. It is not free:
// tb_video_frame's overrun counter is the acceptance test, and the thing to
// watch if a line ever shears.
//
// Plane-adjacent remap (REMAP, tiles only): a tile row is three plane words
// at logical byte offsets B, B+0x100000, B+0x200000 (the pf0l/pf0m/pf0h
// chips, 1 MB apart), always read together. Stored as-is, each would open a
// fresh SDRAM row; interleaved into adjacent columns of one row, accesses 2
// and 3 are page hits. Only the byte->physical map changes: agt_tile_phys
// here and agt_tile_phys_dl in Arcade-Atari-GT.sv (the download side) must
// agree bit for bit. Chars and RLE are not interleaved (REMAP defaults off).
// tb_tile_sdram exercises both REMAP values.
`default_nettype none

// Fold a 22-bit logical tiles byte address (plane 0..2 in [21:20], third-local
// byte in [19:0]) into a physical address where the three planes are
// adjacent 32-bit words of a 16-byte group:
//   logical:  plane*0x100000 + word*4 + bytesel
//   physical: word*16        + plane*4 + bytesel
// 16-byte groups divide a 2048-byte SDRAM row exactly, so a group never
// straddles a row boundary. A straddling 3-plane burst would compute column
// 0x400, and A10 on a READ is auto-precharge: the row would close under the
// next plane read. The price is a 4 MB region for 3 MB of tiles.
// agt_tile_decode forms plane offsets as +0x100000 / +0x200000, so bits
// [21:20] carry the plane index exactly.
function automatic logic [21:0] agt_tile_phys(input logic [21:0] b);
	logic [1:0]  plane;    // 0=L 1=M 2=H
	logic [17:0] wordidx;  // third-local word; must exclude plane bits
	logic [1:0]  bytesel;  // byte within the 32-bit word
	logic [21:0] w12;
	begin
		plane   = b[21:20];
		wordidx = b[19:2];
		bytesel = b[1:0];
		// physical byte = wordidx*16 + plane*4 + bytesel (w12 holds wordidx*16).
		w12 = {4'd0, wordidx} << 4;
		agt_tile_phys = w12 + {18'd0, plane, 2'd0} + {20'd0, bytesel};
	end
endfunction

module agt_tile_sdram #(
	parameter logic [25:0] BASE  = 26'h0300000,  // default: the tiles region
	parameter bit          REMAP = 1'b0,         // plane-adjacent remap (tiles only)
	// Byte-address width of the consumer's region: 22 (4 MB) covers tiles and
	// chars; the 32 MB RLE sprite region needs 25. A parameter rather than a
	// wider port so agt_tile_phys keeps its 22-bit domain on the tile path.
	parameter int          ADDR_W = 22
) (
	input  wire         clk,
	input  wire         rst_n,

	// renderer side (BRAM-shaped)
	input  wire  [ADDR_W-1:0] rom_addr,
	input  wire         rom_rd,
	output logic [7:0]  rom_data,
	output logic        rom_data_valid,

	// SDRAM tile read port
	output logic [25:0] tile_addr,
	output logic        tile_req,
	input  wire         tile_ack,
	input  wire  [7:0]  tile_data,
	// The whole 32-bit word from the same access: a consumer that wants a plane
	// row takes rom_data32 and issues one request instead of four.
	input  wire  [31:0] tile_data32,
	output logic [31:0] rom_data32,
	// Three-plane burst: rom_rd3 requests L/M/H in one controller transaction;
	// rom_data96 returns them as three 32-bit words and rom_data_valid pulses
	// once, at the end. REMAP applies to the plane-L address; M and H sit at +4
	// and +8 from it.
	input  wire         rom_rd3,
	output logic        tile_burst3,
	input  wire  [95:0] tile_data96,
	output logic [95:0] rom_data96
);

	typedef enum logic [1:0] { S_IDLE, S_REQ, S_DROP } state_t;
	state_t state;

	// The renderer's `rd` is a one-cycle pulse and is never retried; it comes
	// the cycle after data_valid, while this adapter is still in S_DROP.
	// Sampling it only in S_IDLE would drop back-to-back requests and hang the
	// fetcher. eff_addr is the address actually presented, mapped if REMAP.
	wire [ADDR_W-1:0] eff_addr   = (rom_rd || rom_rd3) ? rom_addr : rd_addr_q;
	wire [ADDR_W-1:0] eff_mapped = REMAP
								 ? ADDR_W'(agt_tile_phys(eff_addr[21:0]))
								 : eff_addr;

	logic        rd_pending;
	logic        rd3_pending;    // the captured request wants all 3 planes
	logic [ADDR_W-1:0] rd_addr_q;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state          <= S_IDLE;
			tile_addr      <= 26'd0;
			tile_req       <= 1'b0;
			rom_data       <= 8'd0;
			rom_data32     <= 32'd0;
			rom_data96     <= 96'd0;
			tile_burst3    <= 1'b0;
			rom_data_valid <= 1'b0;
			rd_pending     <= 1'b0;
			rd3_pending    <= 1'b0;
			rd_addr_q      <= '0;
		end else begin
			rom_data_valid <= 1'b0;          // one-cycle pulse, like the BRAM

			// Capture the request in every state, so a pulse arriving while the
			// previous access finishes is not lost.
			if (rom_rd || rom_rd3) begin
				rd_pending  <= 1'b1;
				rd3_pending <= rom_rd3;
				rd_addr_q   <= rom_addr;
			end

			unique case (state)
				S_IDLE: if (rd_pending || rom_rd || rom_rd3) begin
							// Zero-extend from ADDR_W so a wider consumer
							// keeps its high bits.
							tile_addr   <= BASE + {{(26-ADDR_W){1'b0}}, eff_mapped};
							tile_burst3 <= (rom_rd || rom_rd3) ? rom_rd3 : rd3_pending;
							tile_req    <= 1'b1;
							rd_pending  <= 1'b0;
							state       <= S_REQ;
						end
				S_REQ:  if (tile_ack) begin
`ifdef AGT_ADAPTER_TRACE
							// Debug: X here means the controller's data is
							// not valid on the ack cycle for this requestor;
							// good here but X in rom_data32 later means
							// something else writes it.
							if (^tile_data32 === 1'bx)
								$display("ADAPTER-X capture: addr=%06h tile_data32=%08h",
										 rd_addr_q, tile_data32);
`endif
							rom_data       <= tile_data;
							rom_data32     <= tile_data32;
							rom_data96     <= tile_data96;
							rom_data_valid <= 1'b1;
							tile_req       <= 1'b0;
							tile_burst3    <= 1'b0;
							state          <= S_DROP;
						end
				// Wait for tile_ack to drop. Defensive: tile_ack is a
				// one-cycle pulse and tile_req is already low here, so
				// tb_tile_sdram passes without it. It costs one cycle of a
				// ~12-cycle access and makes the handshake correct by
				// construction rather than by timing.
				S_DROP: if (!tile_ack) state <= S_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
