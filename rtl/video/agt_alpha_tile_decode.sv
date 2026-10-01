// agt_alpha_tile_decode.sv -- Atari GT alpha tile row decoder.
//
// Decodes one row (8 pixels x 4 bits) of an alpha tile from the "chars" ROM
// (128KB, 4096 tiles). The format is MAME's gfx_8x8x4_packed_msb: one flat
// region of packed nibbles, high nibble first:
//   base = code*32 + row*4   (bytes)
//   for n in 0..3: byte = chars[base+n]
//     pixel(2n)   = byte[7:4]
//     pixel(2n+1) = byte[3:0]

module agt_alpha_tile_decode #(
	parameter int ROM_ADDR_WIDTH = 17,  // 128KB "chars" region
	// WIDE_FETCH=1 reads the whole character row in one access via rom_data32
	// instead of four byte reads, so a fetch fits in the 8 cycles of emission
	// at SDRAM latency. base is always 4-byte aligned, so the 32-bit word is
	// exactly this row. 0 keeps the byte path.
	parameter bit WIDE_FETCH = 1'b0
) (
	input  logic        clk,
	input  logic        rst_n,

	input  logic        start,
	input  logic [11:0] code,     // 12 bits (4096 tiles)
	input  logic [2:0]  row,

	output logic [ROM_ADDR_WIDTH-1:0] rom_addr,
	output logic                      rom_rd,
	input  logic [7:0]                rom_data,
	input  logic [31:0]               rom_data32,   // WIDE_FETCH only

	input  logic                      rom_data_valid,

	output logic pixels_valid,
	output logic [3:0] pixel0, pixel1, pixel2, pixel3,
	output logic [3:0] pixel4, pixel5, pixel6, pixel7,

	output logic busy,
	output logic done
);

	// base = code*32 + row*4 (bytes)
	wire [ROM_ADDR_WIDTH-1:0] base = ({5'd0, code} << 5) + {{(ROM_ADDR_WIDTH-5){1'b0}}, row, 2'b00};

	logic [7:0] b0, b1, b2, b3;

	typedef enum logic [3:0] {
		ST_IDLE,
		ST_REQ0, ST_WAIT0, ST_REQ1, ST_WAIT1,
		ST_REQ2, ST_WAIT2, ST_REQ3, ST_WAIT3,
		ST_DECODE
	} state_t;
	state_t state;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0; pixels_valid <= 1'b0; rom_rd <= 1'b0;
		end else begin
			done <= 1'b0;
			pixels_valid <= 1'b0;
			rom_rd <= 1'b0;

			unique case (state)
				ST_IDLE: if (start) begin busy <= 1'b1; state <= ST_REQ0; end

				// WIDE_FETCH: one request and wait; otherwise four byte reads

				ST_REQ0: begin rom_addr <= base;       rom_rd <= 1'b1; state <= ST_WAIT0; end
				ST_WAIT0: if (rom_data_valid) begin
							  if (WIDE_FETCH) begin
								  // b0 in the top byte
								  b0 <= rom_data32[31:24];
								  b1 <= rom_data32[23:16];
								  b2 <= rom_data32[15:8];
								  b3 <= rom_data32[7:0];
								  state <= ST_DECODE;
							  end else begin
								  b0 <= rom_data; state <= ST_REQ1;
							  end
						  end

				ST_REQ1: begin rom_addr <= base + 1'b1; rom_rd <= 1'b1; state <= ST_WAIT1; end
				ST_WAIT1: if (rom_data_valid) begin b1 <= rom_data; state <= ST_REQ2; end

				ST_REQ2: begin rom_addr <= base + 2'd2; rom_rd <= 1'b1; state <= ST_WAIT2; end
				ST_WAIT2: if (rom_data_valid) begin b2 <= rom_data; state <= ST_REQ3; end

				ST_REQ3: begin rom_addr <= base + 2'd3; rom_rd <= 1'b1; state <= ST_WAIT3; end
				ST_WAIT3: if (rom_data_valid) begin b3 <= rom_data; state <= ST_DECODE; end

				ST_DECODE: begin
					pixel0 <= b0[7:4]; pixel1 <= b0[3:0];
					pixel2 <= b1[7:4]; pixel3 <= b1[3:0];
					pixel4 <= b2[7:4]; pixel5 <= b2[3:0];
					pixel6 <= b3[7:4]; pixel7 <= b3[3:0];
					pixels_valid <= 1'b1;
					busy <= 1'b0; done <= 1'b1;
					state <= ST_IDLE;
				end
				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
