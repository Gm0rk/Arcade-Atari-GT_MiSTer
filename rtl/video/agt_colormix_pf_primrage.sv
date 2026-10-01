// agt_colormix_pf_primrage.sv -- Atari GT / Primal Rage colour mix, playfield
// only.
//
// The Primal Rage branch of MAME's screen_update (atarigt_v.cpp) with no
// alpha or motion objects (an = 0, mo = 0). The full formula:
//   pfpri = (pf>>10)&7;  mopri = mo>>12;  mgep = (mopri>=pfpri)&&!(pfpri&4)
//   if (an & 0x8f)                                  cra = an & 0xff
//   elif (mo&0x3f) && (mo&0x800||mgep||!(pf&0x3f))  cra = 0x1000|(mo&0x7ff)
//   else                                            cra = pf & 0xfff
//   cra = cram[cra]
//   rgb.r = mram[(cra>>10)&0x1f].red_component
//   rgb.g = mram[(cra>>5)&0x1f].green_component
//   rgb.b = mram[cra&0x1f].blue_component
//   if (color_latch&7) and (!(pf&0x3f) || !(pf&0x2000)): rgb = white
// With an = mo = 0 only the last branch is reachable: cra = pf & 0xfff.
//
// pf_pixel arrives already combined (agt_playfield_pixel_combine) with
// MAME's tilemap pen rule, a plain add that may carry between fields:
//   pf_pixel = (playfield_color_bank << 8) + (tile_color << 5) + raw_pixel

module agt_colormix_pf_primrage (
	input  logic clk,
	input  logic rst_n,

	input  logic        start,
	input  logic [15:0] pf_pixel,
	input  logic [15:0] color_latch,

	// CRAM lookup (agt_colorram cram port)
	output logic [13:0] cram_addr,
	input  logic [15:0] cram_data,

	// R/G/B pen lookups (agt_colorram pen ports)
	output logic [14:0] pen_addr_r,
	output logic [14:0] pen_addr_g,
	output logic [14:0] pen_addr_b,
	input  logic [23:0] pen_data_r,
	input  logic [23:0] pen_data_g,
	input  logic [23:0] pen_data_b,

	output logic        rgb_valid,
	output logic [23:0] rgb,        // {red[7:0], green[7:0], blue[7:0]}

	output logic busy,
	output logic done
);

	typedef enum logic [1:0] { ST_IDLE, ST_CRAM, ST_PENS } state_t;
	state_t state;

	logic [11:0] cra_index;   // pf_pixel & 0xfff, latched at start
	logic [15:0] pf_pixel_r;
	logic [15:0] color_latch_r;

	wire [13:0] cram_bank_offset = color_latch_r[3] ? 14'h2000 : 14'h0000;
	wire [14:0] mram_base = {color_latch_r[7:6], 13'd0};  // (color_latch&0xc0)<<7

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0; rgb_valid <= 1'b0;
		end else begin
			done <= 1'b0;
			rgb_valid <= 1'b0;

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy <= 1'b1;
						pf_pixel_r    <= pf_pixel;
						color_latch_r <= color_latch;
						cra_index     <= pf_pixel[11:0];
						state <= ST_CRAM;
					end
				end

				ST_CRAM: begin
					// cram_addr is presented this cycle; cram_data is used in ST_PENS.
					state <= ST_PENS;
				end

				ST_PENS: begin
					logic final_override;
					final_override = (color_latch_r[2:0] != 3'b000) &&
									  (pf_pixel_r[5:0] == 6'd0 || !pf_pixel_r[13]);
					if (final_override) begin
						rgb <= 24'hFFFFFF;
					end else begin
						rgb <= {pen_data_r[23:16], pen_data_g[15:8], pen_data_b[7:0]};
					end
					rgb_valid <= 1'b1;
					busy <= 1'b0;
					done <= 1'b1;
					state <= ST_IDLE;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

	assign cram_addr  = cram_bank_offset + {2'b00, cra_index};
	assign pen_addr_r = mram_base + {10'd0, cram_data[14:10]};
	assign pen_addr_g = mram_base + {10'd0, cram_data[9:5]};
	assign pen_addr_b = mram_base + {10'd0, cram_data[4:0]};

endmodule
