// agt_div32.sv -- sequential unsigned 32/32 restoring divider, 32 cycles.
//
// agt_rle_blit uses it once per object for dx/dy (source step per
// destination pixel/line), outside the per-pixel loop. Division by zero is
// undefined (quotient all 1s); the caller clamps scaled width/height to at
// least 1, as atarirle.cpp's draw_rle_zoom does.

module agt_div32 (
	input  logic        clk,
	input  logic        rst_n,
	input  logic        start,        // 1-cycle pulse
	input  logic [31:0] dividend,
	input  logic [31:0] divisor,
	output logic        busy,
	output logic        done,         // 1-cycle pulse, results valid
	output logic [31:0] quotient,
	output logic [31:0] remainder
);

	logic [31:0] rem_r, quot_r, div_r;
	logic [5:0]  count;

	// One shift-subtract step
	wire [31:0] rem_shifted  = {rem_r[30:0], quot_r[31]};
	wire [31:0] quot_shifted = {quot_r[30:0], 1'b0};
	wire        step_ge      = (rem_shifted >= div_r);
	wire [31:0] rem_next     = step_ge ? (rem_shifted - div_r) : rem_shifted;
	wire [31:0] quot_next    = step_ge ? (quot_shifted | 32'd1) : quot_shifted;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			busy  <= 1'b0;
			done  <= 1'b0;
			count <= '0;
		end else begin
			done <= 1'b0;
			if (start && !busy) begin
				rem_r  <= 32'd0;
				quot_r <= dividend;
				div_r  <= divisor;
				count  <= 6'd32;
				busy   <= 1'b1;
			end else if (busy) begin
				rem_r  <= rem_next;
				quot_r <= quot_next;
				count  <= count - 6'd1;
				if (count == 6'd1) begin
					busy <= 1'b0;
					done <= 1'b1;
				end
			end
		end
	end

	assign quotient  = quot_r;
	assign remainder = rem_r;

endmodule
