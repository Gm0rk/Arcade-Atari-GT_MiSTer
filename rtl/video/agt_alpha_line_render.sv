// agt_alpha_line_render.sv -- renders one scanline of the alpha (text) layer.
//
// Drives agt_alpha_addr and agt_alpha_tile_decode to emit one 8-bit pen per
// screen column, in ascending order:
//     pen = {tile_color, raw_pixel}   (= color*16 + pix)
// The consumer's mixer rule is "if (pen & 0x8f) alpha wins, cra = pen",
// not "pen != 0": bit 7 is colour bit 3, so a blank pixel in palette >= 8
// wins and a blank in palette 1 does not.
//
// This is a separate pass from the playfield because the playfield scrolls
// and the alpha layer is a fixed screen-aligned grid (column = x >> 3), so
// their tiles do not line up. MAME likewise draws alpha to its own bitmap
// and mixes per pixel.
//
// Character N+1 is fetched while character N's 8 pens are emitted.
`default_nettype none

module agt_alpha_line_render #(
	parameter int VISIBLE_WIDTH  = 336,
	parameter int N_CHARS        = 42,       // 336 / 8
	parameter int ROM_ADDR_WIDTH = 17
) (
	input  wire         clk,
	input  wire         rst_n,

	input  wire         start,
	input  wire  [8:0]  target_scanline,

	// alpha tilemap RAM
	output wire  [10:0] alpharam_addr,
	output wire         alpharam_rd,
	input  wire  [15:0] alpharam_data,
	input  wire         alpharam_data_valid,

	// chars ROM (BRAM or SDRAM, same handshake)
	output wire  [ROM_ADDR_WIDTH-1:0] charrom_addr,
	output wire         charrom_rd,
	input  wire  [7:0]  charrom_data,
	input  wire  [31:0] charrom_data32,
	input  wire         charrom_data_valid,

	// one pen per column, ascending
	output logic        pen_valid,
	output logic [8:0]  pen_col,
	output logic [7:0]  pen_value,

	output logic        busy,
	output logic        done
);

	logic        aa_start;
	logic [5:0]  aa_col;
	logic [4:0]  aa_row;
	wire         aa_done;
	wire [11:0]  aa_code;
	wire [3:0]   aa_color;

	agt_alpha_addr u_addr (
		.clk(clk), .rst_n(rst_n),
		.start(aa_start), .col(aa_col), .row(aa_row),
		.alpharam_addr(alpharam_addr), .alpharam_rd(alpharam_rd),
		.alpharam_data(alpharam_data), .alpharam_data_valid(alpharam_data_valid),
		.tile_valid(), .tile_code(aa_code), .tile_color(aa_color),
		.busy(), .done(aa_done)
	);

	logic        td_start;
	logic [11:0] td_code;
	logic [2:0]  td_row;
	wire         td_done;
	wire [3:0]   td_px [0:7];

	// WIDE_FETCH: one ROM access per character row instead of four, so the
	// fetch fits in the 8 cycles of emission.
	agt_alpha_tile_decode #(.ROM_ADDR_WIDTH(ROM_ADDR_WIDTH),
							.WIDE_FETCH(1'b1)) u_dec (
		.clk(clk), .rst_n(rst_n),
		.start(td_start), .code(td_code), .row(td_row),
		.rom_addr(charrom_addr), .rom_rd(charrom_rd),
		.rom_data(charrom_data), .rom_data32(charrom_data32),
		.rom_data_valid(charrom_data_valid),
		.pixels_valid(),
		.pixel0(td_px[0]), .pixel1(td_px[1]), .pixel2(td_px[2]), .pixel3(td_px[3]),
		.pixel4(td_px[4]), .pixel5(td_px[5]), .pixel6(td_px[6]), .pixel7(td_px[7]),
		.busy(), .done(td_done)
	);

	// Prefetch FSM. It is the only driver of aa_start / td_start.
	//
	// Last-entry cache: alpha is fetch-rate limited (42 characters per line in
	// 336 cycles of emission), but a text layer is mostly blank and neighbouring
	// cells usually share a tilemap entry. The last (code, color) and its eight
	// pixels are cached, so a repeated entry costs only its address read. Exact
	// and data-independent: same entry and same row give the same pixels.
	logic [15:0] last_entry;
	logic        last_valid;
	logic [3:0]  last_px [0:7];
	wire  [15:0] this_entry = {aa_color, aa_code};

	typedef enum logic [1:0] { PF_IDLE, PF_ADDR, PF_DEC, PF_READY } pf_t;
	pf_t         pf_state;
	logic        pf_go, pf_take;
	logic [5:0]  pf_col;
	logic [3:0]  pend_color;
	logic [3:0]  pend_px [0:7];
	wire         pf_ready = (pf_state == PF_READY);

	integer i;
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pf_state <= PF_IDLE;
			aa_start <= 1'b0; td_start <= 1'b0;
			aa_col <= 6'd0; td_code <= 12'd0;
			pend_color <= 4'd0;
			last_valid <= 1'b0; last_entry <= 16'd0;
			for (i = 0; i < 8; i = i + 1) begin
				pend_px[i] <= 4'd0; last_px[i] <= 4'd0;
			end
		end else begin
			aa_start <= 1'b0;
			td_start <= 1'b0;
			// td_row changes every line, so the cache is invalidated on `start`.
			if (start) last_valid <= 1'b0;
			unique case (pf_state)
				PF_IDLE: if (pf_go) begin
							 aa_col   <= pf_col;
							 aa_start <= 1'b1;
							 pf_state <= PF_ADDR;
						 end
				PF_ADDR: if (aa_done) begin
							 pend_color <= aa_color;
							 if (last_valid && this_entry == last_entry) begin
								 // same entry, same row: reuse
								 for (i = 0; i < 8; i = i + 1)
									 pend_px[i] <= last_px[i];
								 pf_state <= PF_READY;
							 end else begin
								 td_code  <= aa_code;
								 td_start <= 1'b1;
								 pf_state <= PF_DEC;
							 end
						 end
				PF_DEC:  if (td_done) begin
							 for (i = 0; i < 8; i = i + 1) begin
								 pend_px[i] <= td_px[i];
								 last_px[i] <= td_px[i];
							 end
							 last_entry <= {pend_color, td_code};
							 last_valid <= 1'b1;
							 pf_state   <= PF_READY;
						 end
				// Take and go arrive in the same cycle (the lookahead); going via PF_IDLE
				// would sample pf_go a cycle after its pulse.
				PF_READY: if (pf_take) begin
							  if (pf_go) begin
								  aa_col   <= pf_col;
								  aa_start <= 1'b1;
								  pf_state <= PF_ADDR;
							  end else pf_state <= PF_IDLE;
						  end
			endcase
		end
	end

	// Main FSM: emit 8 pens per character
	typedef enum logic [1:0] { S_IDLE, S_PRIME, S_EMIT, S_NEXT } st_t;
	st_t         state;
	logic [5:0]  char_idx, fetch_idx;
	logic [2:0]  pix_idx;
	logic [3:0]  cur_color;
	logic [3:0]  cur_px [0:7];

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_IDLE;
			busy <= 1'b0; done <= 1'b0;
			pen_valid <= 1'b0; pen_col <= 9'd0; pen_value <= 8'd0;
			pf_go <= 1'b0; pf_take <= 1'b0;
			char_idx <= 6'd0; fetch_idx <= 6'd0; pix_idx <= 3'd0;
			cur_color <= 4'd0;
			for (i = 0; i < 8; i = i + 1) cur_px[i] <= 4'd0;
			aa_row <= 5'd0; td_row <= 3'd0;
		end else begin
			done      <= 1'b0;
			pen_valid <= 1'b0;
			pf_go     <= 1'b0;
			pf_take   <= 1'b0;

			unique case (state)
				S_IDLE: if (start) begin
							busy      <= 1'b1;
							// screen-aligned: no scroll
							aa_row    <= target_scanline[8:3];
							td_row    <= target_scanline[2:0];
							fetch_idx <= 6'd0;
							pf_col    <= 6'd0;
							pf_go     <= 1'b1;
							state     <= S_PRIME;
						end

				S_PRIME: if (pf_ready) begin
							 cur_color <= pend_color;
							 for (i = 0; i < 8; i = i + 1) cur_px[i] <= pend_px[i];
							 pf_take  <= 1'b1;
							 char_idx <= 6'd0;
							 pix_idx  <= 3'd0;
							 if (N_CHARS[5:0] > 6'd1) begin
								 fetch_idx <= 6'd1;
								 pf_col    <= 6'd1;
								 pf_go     <= 1'b1;
							 end
							 state <= S_EMIT;
						 end

				S_EMIT: begin
					pen_valid <= 1'b1;
					pen_col   <= {char_idx, pix_idx};
					// {color, pix} == color*16 + pix
					pen_value <= {cur_color, cur_px[pix_idx]};
					if (pix_idx == 3'd7) begin
						if (char_idx == N_CHARS[5:0] - 1'b1) begin
							busy  <= 1'b0;
							done  <= 1'b1;
							state <= S_IDLE;
						end else state <= S_NEXT;
					end else pix_idx <= pix_idx + 3'd1;
				end

				S_NEXT: if (pf_ready) begin
							cur_color <= pend_color;
							for (i = 0; i < 8; i = i + 1) cur_px[i] <= pend_px[i];
							pf_take  <= 1'b1;
							char_idx <= fetch_idx;
							pix_idx  <= 3'd0;
							if (fetch_idx + 6'd1 < N_CHARS[5:0]) begin
								fetch_idx <= fetch_idx + 6'd1;
								pf_col    <= fetch_idx + 6'd1;
								pf_go     <= 1'b1;
							end
							state <= S_EMIT;
						end
			endcase
		end
	end

endmodule

`default_nettype wire
