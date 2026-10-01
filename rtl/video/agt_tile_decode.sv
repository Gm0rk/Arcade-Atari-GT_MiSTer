// agt_tile_decode.sv -- Atari GT playfield tile pixel decoder.
//
// Given a 16-bit tile `code` and a row (0-7), fetches the needed bytes from
// the "tiles" ROM and produces the row's 8 pixels (6 bits each). Reproduces
// MAME's gfx_layout decode (pflayout + pftoplayout, atarigt.cpp:806-827)
// merged by blend_gfx (0x0f from pflayout | 0x30 from pftoplayout,
// atarigt_v.cpp:81 / atarigen.cpp:82-117).
//
// blend_gfx discards pflayout's plane 0 and pftoplayout's planes 2-5, so they
// are never fetched. The rest is 3 byte pairs per row, one from each third of
// the tile ROM (chips pf0l/pf0m/pf0h at 0x000000/0x100000/0x200000):
//   L0,L1 = tiles[local_byte : local_byte+1]              (pf0l, "local")
//   M0,M1 = tiles[local_byte+0x100000 : +0x100001]        (pf0m, "+1MB")
//   H0,H1 = tiles[local_byte+0x200000 : +0x200001]        (pf0h, "+2MB")
// where local_byte = code*16 + row*2 (code*charincrement/8 + row*yoffset-step/8).
//
// Per-pixel extraction (checked against the full gfx_layout decode by
// tile_golden.py):
//   pf_nibble(x)  = (x even) ? SEL(x)[7:4] : SEL(x)[3:0]
//                   where SEL(x) = M0 (x=0,1), L0 (x=2,3), M1 (x=4,5), L1 (x=6,7)
//   bit5(x),bit4(x) = HSEL(x)[4+xm], HSEL(x)[xm]
//                   where HSEL(x) = H0 (x<4) else H1,  xm = x & 3
//   pixel(x) = {bit5(x), bit4(x), pf_nibble(x)}   -- 6-bit result, 0-63
//
// Tile bank switching (which 4096-tile page a 12-bit playfield code selects)
// is the caller's job; this module decodes the full 16-bit `code` it is given.

module agt_tile_decode #(
	parameter int ROM_ADDR_WIDTH = 22,  // 3MB "tiles" region = 0x300000 bytes
	// WIDE_FETCH=1 takes each plane's two adjacent bytes in one access via
	// rom_data32: 3 accesses per tile row instead of 6.
	// local_byte is always even but not necessarily 4-aligned, and the SDRAM
	// port returns the word pair containing the address, so the plane's two
	// bytes are in the high half when bit 1 is clear and the low half when it
	// is set. Assuming 4-alignment corrupts every odd tile row.
	parameter bit WIDE_FETCH = 1'b0,
	// USE_BURST3 enables the three-plane burst request (rom_rd3),
	// independently of WIDE_FETCH. Only set it when the adapter services
	// rom_rd3: otherwise the decoder waits forever in ST_WAIT_L0.
	// Default 0 = three separate requests.
	parameter bit USE_BURST3 = 1'b0
) (
	input  logic        clk,
	input  logic        rst_n,

	input  logic        start,      // 1-cycle pulse
	input  logic [15:0] code,
	input  logic [2:0]  row,

	// generic synchronous byte-addressed ROM port
	output logic [ROM_ADDR_WIDTH-1:0] rom_addr,
	output logic                      rom_rd,
	// Burst: the three planes (L, M, H) come back from one controller
	// transaction, leaving no gap between plane requests in which the CPU can
	// take the port and open a different row.
	output logic                      rom_rd3,
	input  logic [95:0]               rom_data96,
	input  logic [7:0]                rom_data,
	// Debug: the six plane bytes of the last tile row decoded, as returned by
	// the ROM path, and its first two pixels. Latched together at the decode
	// (valid with `pixels_valid`), so bytes and pens always describe one row.
	output logic [47:0]               dbg_row_bytes,  // {L0,M0,H0,L1,M1,H1}
	output logic [11:0]               dbg_row_pix01,  // {pixel0, pixel1}
	input  logic [31:0]               rom_data32,     // WIDE_FETCH only

	input  logic                      rom_data_valid,

	// decoded row: 8 pixels, 6 bits each (0-63)
	output logic pixels_valid,        // 1-cycle pulse
	output logic [5:0] pixel0, pixel1, pixel2, pixel3,
	output logic [5:0] pixel4, pixel5, pixel6, pixel7,

	output logic busy,
	output logic done
);

	// local_byte = code*16 + row*2; max 0x0FFFF0 + 14, within the first third
	// (0x100000) of the 0x300000 region.
	wire [ROM_ADDR_WIDTH-1:0] local_byte = ({6'd0, code} << 4) + {{(ROM_ADDR_WIDTH-4){1'b0}}, row, 1'b0};

	logic burst_lane;   // logical bit 1 of the requested byte

	logic [7:0] L0, L1, M0, M1, H0, H1;

	typedef enum logic [3:0] {
		ST_IDLE,
		ST_REQ_L0, ST_WAIT_L0, ST_REQ_L1, ST_WAIT_L1,
		ST_REQ_M0, ST_WAIT_M0, ST_REQ_M1, ST_WAIT_M1,
		ST_REQ_H0, ST_WAIT_H0, ST_REQ_H1, ST_WAIT_H1,
		ST_DECODE
	} state_t;
	state_t state;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			dbg_row_bytes <= 48'd0; dbg_row_pix01 <= 12'd0;
			state <= ST_IDLE;
			busy <= 1'b0;
			done <= 1'b0;
			pixels_valid <= 1'b0;
			rom_rd <= 1'b0;
			rom_rd3 <= 1'b0;
		end else begin
			done <= 1'b0;
			pixels_valid <= 1'b0;
			rom_rd <= 1'b0;
			rom_rd3 <= 1'b0;

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy  <= 1'b1;
						state <= ST_REQ_L0;
					end
				end

				ST_REQ_L0: begin
					// A burst must request the 12-byte group base, i.e. the
					// logical byte with bit 1 cleared. agt_tile_phys maps
					// logical 0x..0/0x..2 into one 12-byte group and 0x..4/6
					// into the next, so an address with bit 1 set would start
					// the reads at group_base+2 and the +4/+8 steps would
					// straddle two tile rows (right shapes, wrong pixels).
					// tb_video_frame's BRAM path is not remapped, so it can't
					// catch this.
					rom_addr <= (WIDE_FETCH && USE_BURST3)
								? {local_byte[21:2], 2'b00} : local_byte;
					// rom_addr[1] is cleared for the burst, so remember the
					// lane the pixels actually live in.
					burst_lane <= local_byte[1];
					// one burst request covers L, M and H
					if (WIDE_FETCH && USE_BURST3) rom_rd3 <= 1'b1;
					else                          rom_rd  <= 1'b1;
					state <= ST_WAIT_L0;
				end
				ST_WAIT_L0: if (rom_data_valid) begin
							  if (WIDE_FETCH && USE_BURST3) begin
								  // rom_data96 = {L, M, H}, each a 32-bit word
								  // whose halves are the two bytes of that
								  // plane's row. Byte lane chosen by burst_lane,
								  // as rom_addr[1] does on the other paths.
								  L0 <= burst_lane ? rom_data96[79:72] : rom_data96[95:88];
								  L1 <= burst_lane ? rom_data96[71:64] : rom_data96[87:80];
								  M0 <= burst_lane ? rom_data96[47:40] : rom_data96[63:56];
								  M1 <= burst_lane ? rom_data96[39:32] : rom_data96[55:48];
								  H0 <= burst_lane ? rom_data96[15:8]  : rom_data96[31:24];
								  H1 <= burst_lane ? rom_data96[7:0]   : rom_data96[23:16];
								  state <= ST_DECODE;
							  end else if (WIDE_FETCH) begin
								  L0 <= rom_addr[1] ? rom_data32[15:8] : rom_data32[31:24];
								  L1 <= rom_addr[1] ? rom_data32[7:0]  : rom_data32[23:16];
								  state <= ST_REQ_M0;
							  end else begin
								  L0 <= rom_data; state <= ST_REQ_L1;
							  end
						  end

				ST_REQ_L1: begin rom_addr <= local_byte + 1'b1;                              rom_rd <= 1'b1; state <= ST_WAIT_L1; end
				ST_WAIT_L1: if (rom_data_valid) begin L1 <= rom_data; state <= ST_REQ_M0; end

				ST_REQ_M0: begin rom_addr <= local_byte + 22'h100000; rom_rd <= 1'b1; state <= ST_WAIT_M0; end
				ST_WAIT_M0: if (rom_data_valid) begin
							  if (WIDE_FETCH) begin
								  M0 <= rom_addr[1] ? rom_data32[15:8]  : rom_data32[31:24];
								  M1 <= rom_addr[1] ? rom_data32[7:0]   : rom_data32[23:16];
								  state <= ST_REQ_H0;
							  end else begin
								  M0 <= rom_data; state <= ST_REQ_M1;
							  end
						  end

				ST_REQ_M1: begin rom_addr <= local_byte + 22'h100001;                        rom_rd <= 1'b1; state <= ST_WAIT_M1; end
				ST_WAIT_M1: if (rom_data_valid) begin M1 <= rom_data; state <= ST_REQ_H0; end

				ST_REQ_H0: begin rom_addr <= local_byte + 22'h200000; rom_rd <= 1'b1; state <= ST_WAIT_H0; end
				ST_WAIT_H0: if (rom_data_valid) begin
							  if (WIDE_FETCH) begin
								  H0 <= rom_addr[1] ? rom_data32[15:8]  : rom_data32[31:24];
								  H1 <= rom_addr[1] ? rom_data32[7:0]   : rom_data32[23:16];
								  state <= ST_DECODE;
							  end else begin
								  H0 <= rom_data; state <= ST_REQ_H1;
							  end
						  end

				ST_REQ_H1: begin rom_addr <= local_byte + 22'h200001;                        rom_rd <= 1'b1; state <= ST_WAIT_H1; end
				ST_WAIT_H1: if (rom_data_valid) begin H1 <= rom_data; state <= ST_DECODE; end

				ST_DECODE: begin
					// SEL(x): x=0,1->M0  x=2,3->L0  x=4,5->M1  x=6,7->L1
					// pf_nibble(x): x even -> SEL[7:4], x odd -> SEL[3:0]
					pixel0 <= {H0[4], H0[0], M0[7:4]};
					pixel1 <= {H0[5], H0[1], M0[3:0]};
					pixel2 <= {H0[6], H0[2], L0[7:4]};
					pixel3 <= {H0[7], H0[3], L0[3:0]};
					pixel4 <= {H1[4], H1[0], M1[7:4]};
					pixel5 <= {H1[5], H1[1], M1[3:0]};
					pixel6 <= {H1[6], H1[2], L1[7:4]};
					pixel7 <= {H1[7], H1[3], L1[3:0]};
					// debug: bytes and pens from this row, latched together
					dbg_row_bytes <= {L0, M0, H0, L1, M1, H1};
					dbg_row_pix01 <= {{H0[4], H0[0], M0[7:4]},
									  {H0[5], H0[1], M0[3:0]}};
					pixels_valid <= 1'b1;
					busy  <= 1'b0;
					done  <= 1'b1;
					state <= ST_IDLE;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
