// agt_alpha_addr.sv -- Atari GT alpha tilemap fetch.
//
// Reads the alpha RAM word for a tile position (col 0-63, row 0-31) and
// decodes it as MAME's get_alpha_tile_info (atarigt_v.cpp) does:
//   code  = data & 0xfff        (4096 "chars" tiles)
//   color = (data >> 12) & 0xf
// Alpha tiles never flip. The map is TILEMAP_SCAN_ROWS, plain row-major:
//   index = row * 64 + col

module agt_alpha_addr (
	input  logic       clk,
	input  logic       rst_n,

	input  logic        start,
	input  logic [5:0]  col,      // 0-63
	input  logic [4:0]  row,      // 0-31

	output logic [10:0] alpharam_addr,   // 64*32 = 2048 words
	output logic        alpharam_rd,
	input  logic [15:0] alpharam_data,
	input  logic         alpharam_data_valid,

	output logic        tile_valid,
	output logic [11:0] tile_code,
	output logic [3:0]  tile_color,

	output logic busy,
	output logic done
);

	typedef enum logic [1:0] { ST_IDLE, ST_REQ, ST_WAIT } state_t;
	state_t state;

	wire [10:0] scan_index = ({5'd0, row} << 6) | {5'd0, col};

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0; tile_valid <= 1'b0; alpharam_rd <= 1'b0;
		end else begin
			done <= 1'b0;
			tile_valid <= 1'b0;
			alpharam_rd <= 1'b0;

			unique case (state)
				ST_IDLE: if (start) begin busy <= 1'b1; state <= ST_REQ; end
				ST_REQ: begin
					alpharam_addr <= scan_index;
					alpharam_rd   <= 1'b1;
					state         <= ST_WAIT;
				end
				ST_WAIT: if (alpharam_data_valid) begin
					tile_code  <= alpharam_data[11:0];
					tile_color <= alpharam_data[15:12];
					tile_valid <= 1'b1;
					busy <= 1'b0; done <= 1'b1;
					state <= ST_IDLE;
				end
				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
