// agt_colormix_primrage.sv -- Atari GT colour mix, Primal Rage branch (alpha,
// playfield and MO layers).
//
// The colour mix is the one place the video path differs between the two
// Atari GT games: MAME selects one of two mixing routines in screen_update
// with `m_is_primrage` (atarigt.cpp, set by init_tmek / init_primrage).
// Everything else (timing, tilemaps, RLE objects, palette) is shared, hence
// the `agt_` prefix elsewhere. This module is the Primal Rage branch only;
// the T-MEK branch is not implemented and would give wrong colours for T-MEK.
//
// Primal Rage branch of screen_update (atarigt_v.cpp:509-540):
//   pfpri = (pf>>10) & 7
//   mopri = mo>>12                       // not masked in MAME either
//   mgep  = (mopri>=pfpri) && !(pfpri&4) // "motion object gets even priority"
//
//   if (an & 0x8f)                     cra = an & 0xff                 // alpha wins
//   elif (mo&0x3f) && (mo&0x800||mgep||!(pf&0x3f))
//                                       cra = 0x1000 | (mo & 0x7ff)     // MO wins
//   else                                cra = pf & 0xfff                // playfield wins
//
//   cra = cram[cra + ((color_latch&0x08)<<10)]
//   mram_base = (color_latch&0xc0)<<7
//   rgb.r = mram[mram_base+((cra>>10)&0x1f)].red
//   rgb.g = mram[mram_base+((cra>>5)&0x1f)].green
//   rgb.b = mram[mram_base+(cra&0x1f)].blue
//   if (color_latch&7) and (!(pf&0x3f) || !(pf&0x2000)): rgb = white
//
// `an` is the alpha tile's 4-bit pixel combined with the tile's colour
// field (same MAME pen convention as the playfield); its exact 16-bit packing
// is not yet confirmed against the source.

module agt_colormix_primrage (
	input  logic clk,
	input  logic rst_n,

	input  logic        start,
	input  logic [15:0] an_pixel,      // alpha layer (see header)
	input  logic [15:0] pf_pixel,      // from agt_playfield_pixel_combine
	input  logic [15:0] mo_pixel,      // from agt_mo_vram read port
	input  logic [15:0] color_latch,

	// CRAM lookup
	output logic [13:0] cram_addr,
	input  logic [15:0] cram_data,
	// Debug: the lookup chain for the last pixel taken from the MO layer.
	//   dbg_mo_cram  : the CRAM word the MO index read
	//   dbg_mo_latch : color_latch_r at the time (bank bits [3] and [7:6])
	//   dbg_mo_rgb   : the colour that pixel resolved to
	output logic [15:0] dbg_mo_cram,
	output logic [15:0] dbg_mo_latch,
	output logic [23:0] dbg_mo_rgb,
	// Debug: how many times the capture above fired, so "MO never
	// selected" can be told from "selected and read zero". The top turns it
	// into a per-frame rate.
	output logic [15:0] dbg_mo_hits,   // free-running, wraps
	// Debug: pixels where the MO buffer held an opaque value
	// (mo_pixel[5:0] != 0), selected or not. hits == nz: priority never
	// rejected a sprite pixel; nz == 0 while sprites are being written: the
	// pixels are not in the buffer when the mixer reads it. Free-running, wraps.
	output logic [15:0] dbg_mo_nz,
	// Debug: the lookup chain for the latest MO pixel that was selected and
	// came out black: cra (CRAM entry), cram[cra] (its contents), color_latch
	// (banks in force). Latest wins.
	output logic [15:0] dbg_blk_cra,
	output logic [15:0] dbg_blk_cram,
	output logic [15:0] dbg_blk_latch,
	// Debug probe: `probe` is asserted with `start` for one screen position
	// per frame (chosen in the top level). Captures that pixel's MO and
	// playfield values, the CRAM entry chosen and what it held.
	input  logic        probe,
	output logic [15:0] dbg_prb_mo,
	output logic [15:0] dbg_prb_pf,
	output logic [15:0] dbg_prb_cra,
	output logic [15:0] dbg_prb_cram,
	output logic [13:0] dbg_mo_cra,    // CRAM address the last MO pixel used

	// 3 independent pen lookups
	output logic [14:0] pen_addr_r,
	output logic [14:0] pen_addr_g,
	output logic [14:0] pen_addr_b,
	input  logic [23:0] pen_data_r,
	input  logic [23:0] pen_data_g,
	input  logic [23:0] pen_data_b,

	output logic        rgb_valid,
	output logic [23:0] rgb,

	output logic busy,
	output logic done
);

	// Two lookups, each through a registered block-RAM read (data valid the
	// cycle after the address). Each address is held stable across its wait
	// state:
	//   ST_IDLE  : latch cra_index -> cram_addr becomes valid
	//   ST_CRAM  : cram_addr stable; cram_data registers at the next edge
	//   ST_PENS  : cram_data valid -> pen_addr valid; pen_data registers next
	//   ST_LATCH : pen_data valid -> compose rgb
	typedef enum logic [1:0] { ST_IDLE, ST_CRAM, ST_PENS, ST_LATCH } state_t;
	state_t state;

	logic [12:0] cra_index;   // 13 bits: the MO case sets bit 12
	logic        mo_sel_r;    // this pixel's index came from the MO layer
	logic [15:0] mo_hits_i;
	assign dbg_mo_hits = mo_hits_i;
	logic [15:0] mo_nz_i;
	assign dbg_mo_nz = mo_nz_i;
	logic [15:0] pf_pixel_r;
	logic [15:0] color_latch_r;
	logic        probe_r;

	wire [13:0] cram_bank_offset = color_latch_r[3] ? 14'h2000 : 14'h0000;
	wire [14:0] mram_base = {color_latch_r[7:6], 13'd0};

	// 3-way mux (combinational, evaluated at `start`)
	wire [2:0] pfpri = pf_pixel[12:10];
	wire [3:0] mopri = mo_pixel[15:12];
	wire mgep = (mopri >= {1'b0, pfpri}) && !pfpri[2];

	wire an_wins = (an_pixel[7:0] & 8'h8f) != 8'h00;
	wire mo_wins = !an_wins &&
				   ((mo_pixel[5:0] != 6'd0) &&
					(mo_pixel[11] || mgep || (pf_pixel[5:0] == 6'd0)));

	// atarigt_v.cpp's primrage path:
	//     if (an[x] & 0x8f)             cra = an[x] & 0xff;      // 0x000..0x0FF
	//     else if (mo wins)             cra = 0x1000 | (mo & 0x7ff);
	//     else                          cra = pf[x] & 0xfff;     // 0x000..0xFFF
	// The MO case sets bit 12 (not bit 11), so the index is 13 bits: `2'b10`
	// is bit 12 = 1, bit 11 = 0. `tb_colormix_cra` checks all three branches.
	wire [12:0] cra_mux = an_wins ? {5'd0, an_pixel[7:0]} :
						  mo_wins ? {2'b10, mo_pixel[10:0]} :
									{1'b0, pf_pixel[11:0]};

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_IDLE;
			busy <= 1'b0; done <= 1'b0; rgb_valid <= 1'b0;
			mo_sel_r <= 1'b0;
			dbg_mo_cram <= 16'd0; dbg_mo_latch <= 16'd0;
			dbg_mo_rgb  <= 24'd0;
			dbg_mo_cra  <= 14'd0;
			mo_hits_i <= 16'd0;
			mo_nz_i   <= 16'd0;
			dbg_blk_cra <= 16'd0; dbg_blk_cram <= 16'd0; dbg_blk_latch <= 16'd0;
			probe_r <= 1'b0; dbg_prb_mo <= 16'd0; dbg_prb_pf <= 16'd0;
			dbg_prb_cra <= 16'd0; dbg_prb_cram <= 16'd0;
		end else begin
			done <= 1'b0;
			rgb_valid <= 1'b0;

			unique case (state)
				ST_IDLE: begin
					if (start) begin
						busy <= 1'b1;
						pf_pixel_r    <= pf_pixel;
						color_latch_r <= color_latch;
						cra_index     <= cra_mux;
						mo_sel_r      <= mo_wins;
						probe_r       <= probe;
						if (probe) begin
							dbg_prb_mo <= mo_pixel;
							dbg_prb_pf <= pf_pixel;
						end
						// Same cycle and same `mo_pixel` as the mux, so nz and hits
						// describe the same pixels.
						if (mo_pixel[5:0] != 6'd0) mo_nz_i <= mo_nz_i + 16'd1;
						state <= ST_CRAM;
					end
				end

				ST_CRAM: begin
					// cram_addr held stable this cycle; cram_data registers now
					state <= ST_PENS;
				end

				ST_PENS: begin
					// cram_data valid -> pen_addr valid; pen_data registers now.
					// cram_data is valid this cycle, so the debug captures happen here.
					if (probe_r) begin
						dbg_prb_cra  <= {2'b00, cram_addr};
						dbg_prb_cram <= cram_data;
					end
					if (mo_sel_r) begin
						dbg_mo_cra   <= cram_addr;
						dbg_mo_cram  <= cram_data;
						dbg_mo_latch <= color_latch_r;
						mo_hits_i <= mo_hits_i + 16'd1;   // wraps; the top takes a 16-bit delta
					end
					state <= ST_LATCH;
				end

				ST_LATCH: begin
					logic final_override;
					final_override = (color_latch_r[2:0] != 3'b000) &&
									  (pf_pixel_r[5:0] == 6'd0 || !pf_pixel_r[13]);
					if (mo_sel_r)
						dbg_mo_rgb <= {pen_data_r[23:16], pen_data_g[15:8],
									   pen_data_b[7:0]};
					// an MO pixel the mixer chose, resolving to black
					if (mo_sel_r && !final_override &&
						pen_data_r[23:16] == 8'd0 && pen_data_g[15:8] == 8'd0 &&
						pen_data_b[7:0] == 8'd0) begin
						dbg_blk_cra   <= {2'b00, cram_addr};
						dbg_blk_cram  <= cram_data;
						dbg_blk_latch <= color_latch_r;
					end
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

	assign cram_addr  = cram_bank_offset + {1'b0, cra_index};
	assign pen_addr_r = mram_base + {10'd0, cram_data[14:10]};
	assign pen_addr_g = mram_base + {10'd0, cram_data[9:5]};
	assign pen_addr_b = mram_base + {10'd0, cram_data[4:0]};

endmodule
