// agt_sdram_loopback.sv -- SDRAM write+read self-test, run once after
// `sdr_ready`, before the CPU is released.
//
// `lb_done` gates the ROM readback, which gates cpu_core_rst_n: a hang here
// keeps the CPU in reset for good. Covered by tb/tb_sdram_loopback.sv.
//
// Writes two known halfwords through the cram port (an even and an odd SDRAM
// word) and a 32-bit pattern through the ram port, then reads all three back.
// The scratch locations are memory the game reads later, so their contents
// are saved first and restored afterwards.
`default_nettype none

module agt_sdram_loopback (
	input  wire         clk,
	input  wire         rst_n,
	input  wire         sdr_ready,

	// cram port (16-bit)
	output reg  [18:0]  lb_addr,
	output reg          lb_we,
	output reg          lb_req,
	output reg  [15:0]  lb_wdata,
	input  wire         cram_ack,
	input  wire [15:0]  cram_rdata,

	// ram port (32-bit): the address is a constant in the top-level mux
	output reg          lb_r32_we,
	output reg          lb_r32_req,
	output reg  [31:0]  lb_r32_wdata, // test pattern, then the saved word
	input  wire         ram_ack,
	input  wire [31:0]  ram_rdata,

	output reg          lb_done,
	output reg          lb_even_ok,
	output reg          lb_odd_ok,
	output reg  [15:0]  lb_even_rd,
	output reg  [15:0]  lb_odd_rd,
	output reg  [31:0]  lb_r32_rd,
	output reg          lb_r32_ok
);

	// 0..15: the test; 16..21 save the scratch words before it, 22..27 restore
	// them after.
	reg [4:0] lb_st;
	reg [15:0] save_even, save_odd;
	reg [31:0] save_r32;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			lb_st <= 5'd16; lb_done <= 1'b0;   // start in the save phase
			save_even <= 16'd0; save_odd <= 16'd0; save_r32 <= 32'd0;
			lb_r32_wdata <= 32'hA55A_1234;
			lb_even_ok <= 1'b0; lb_odd_ok <= 1'b0;
			lb_even_rd <= 16'd0; lb_odd_rd <= 16'd0;
			lb_r32_rd <= 32'd0; lb_r32_ok <= 1'b0;
			lb_r32_we <= 1'b0; lb_r32_req <= 1'b0;
			lb_addr <= 19'd0; lb_we <= 1'b0; lb_req <= 1'b0; lb_wdata <= 16'd0;
		end else if (!lb_done && sdr_ready) begin
			case (lb_st)
				// clear
				5'd0:  begin lb_addr <= 19'h7F000; lb_wdata <= 16'h0000;
							 lb_we <= 1'b1; lb_req <= 1'b1; lb_st <= 5'd1; end
				5'd1:  if (cram_ack) begin lb_req <= 1'b0;
							 lb_addr <= 19'h7F002; lb_st <= 5'd2; end
				5'd2:  if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd3; end
				5'd3:  if (cram_ack) begin lb_req <= 1'b0; lb_st <= 5'd4; end
				// write patterns
				5'd4:  if (!cram_ack) begin lb_addr <= 19'h7F000;
							 lb_wdata <= 16'hA55A; lb_req <= 1'b1; lb_st <= 5'd5; end
				5'd5:  if (cram_ack) begin lb_req <= 1'b0;
							 lb_addr <= 19'h7F002; lb_wdata <= 16'h1234; lb_st <= 5'd6; end
				5'd6:  if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd7; end
				5'd7:  if (cram_ack) begin lb_req <= 1'b0; lb_we <= 1'b0;
							 lb_addr <= 19'h7F000; lb_st <= 5'd8; end
				// read back both
				5'd8:  if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd9; end
				5'd9:  if (cram_ack) begin lb_req <= 1'b0;
							 lb_even_ok <= (cram_rdata == 16'hA55A);
							 lb_even_rd <= cram_rdata;
							 lb_addr <= 19'h7F002; lb_st <= 5'd10; end
				5'd10: if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd11; end
				5'd11: if (cram_ack) begin lb_req <= 1'b0;
							 lb_odd_ok <= (cram_rdata == 16'h1234);
							 lb_odd_rd <= cram_rdata;
							 lb_st <= 5'd12; end
				// 32-bit phase, through the ram port. ROM fetches are 32-bit,
				// i.e. two beats; the readback shows how multi-beat capture
				// behaves:
				//   A55A1234 -> 32-bit path good
				//   1234A55A -> the two beats are swapped
				//   A55AA55A / 12341234 -> one beat captured twice
				5'd12: if (!ram_ack) begin
							 lb_r32_we <= 1'b1; lb_r32_req <= 1'b1; lb_st <= 5'd13; end
				5'd13: if (ram_ack) begin lb_r32_req <= 1'b0;
							 lb_r32_we <= 1'b0; lb_st <= 5'd14; end
				5'd14: if (!ram_ack) begin lb_r32_req <= 1'b1; lb_st <= 5'd15; end
				5'd15: if (ram_ack) begin lb_r32_req <= 1'b0;
							 lb_r32_rd <= ram_rdata;
							 lb_r32_ok <= (ram_rdata == 32'hA55A_1234);
							 lb_st <= 5'd22; end

				// Save phase (runs first, states 16-21): the test writes two
				// colorram words and one main-RAM word that the game reads
				// later, so read the originals before disturbing them.
				5'd16: begin lb_addr <= 19'h7F000; lb_we <= 1'b0;
							 lb_req <= 1'b1; lb_st <= 5'd17; end
				5'd17: if (cram_ack) begin lb_req <= 1'b0;
							 save_even <= cram_rdata;
							 lb_addr <= 19'h7F002; lb_st <= 5'd18; end
				5'd18: if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd19; end
				5'd19: if (cram_ack) begin lb_req <= 1'b0;
							 save_odd <= cram_rdata; lb_st <= 5'd20; end
				5'd20: if (!cram_ack) begin lb_r32_we <= 1'b0;
							 lb_r32_req <= 1'b1; lb_st <= 5'd21; end
				5'd21: if (ram_ack) begin lb_r32_req <= 1'b0;
							 save_r32 <= ram_rdata; lb_st <= 5'd0; end

				// Restore phase (states 22-27): put all three back through
				// the same ports, before lb_done releases the machine.
				5'd22: if (!ram_ack) begin lb_addr <= 19'h7F000;
							 lb_wdata <= save_even; lb_we <= 1'b1;
							 lb_req <= 1'b1; lb_st <= 5'd23; end
				5'd23: if (cram_ack) begin lb_req <= 1'b0;
							 lb_addr <= 19'h7F002; lb_wdata <= save_odd;
							 lb_st <= 5'd24; end
				5'd24: if (!cram_ack) begin lb_req <= 1'b1; lb_st <= 5'd25; end
				5'd25: if (cram_ack) begin lb_req <= 1'b0; lb_we <= 1'b0;
							 lb_st <= 5'd26; end
				5'd26: if (!cram_ack) begin lb_r32_wdata <= save_r32;
							 lb_r32_we <= 1'b1; lb_r32_req <= 1'b1;
							 lb_st <= 5'd27; end
				5'd27: if (ram_ack) begin lb_r32_req <= 1'b0; lb_r32_we <= 1'b0;
							 lb_done <= 1'b1; end
				default: ;
			endcase
		end
	end

endmodule

`default_nettype wire
