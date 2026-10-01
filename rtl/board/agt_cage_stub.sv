// agt_cage_stub.sv -- minimal CAGE (TMS320C31 sound board) communication stub.
//
// Not a sound implementation: models only the main-CPU-visible protocol at
// 0xC00000, transcribed from MAME cage.cpp (atari_cage_device), so the game's
// boot/attract logic, which gates on the sound handshake, can run without the
// real CAGE. A stub that only acks and returns zeros is not enough: the game
// then never writes its motion-object list.
//
// Protocol (cage.cpp, function for function):
//   Register at 0xC00000, one 32-bit location, split by byte lanes:
//     [31:16]  main_r / main_w    -- the data latch to/from the DSP
//     [15:0]   control_r/control_w
//   control_r  = {14'b0, cpu_to_cage_ready, cage_to_cpu_ready}
//                bit1: a command the DSP hasn't consumed yet ("TX busy")
//                bit0: DSP data waiting for the main CPU ("RX ready")
//   control_w  : stores control. (control & 3)==0 resets the DSP: both
//                ready flags clear, any pending boot is cancelled;
//                nonzero releases it.
//   main_r     : returns the latch, clears cage_to_cpu_ready.
//   main_w     : latches the command, sets cpu_to_cage_ready; the real
//                DSP consumes it via cage_from_main_r (clearing the flag).
//                The stub auto-consumes after CONSUME_DELAY cycles.
//   IRQ (update_control_lines, both reasons):
//     BUFFER_EMPTY: (control&3)==3 && !cpu_to_cage_ready
//     DATA_READY:   (control&2)   &&  cage_to_cpu_ready
//     irq = |reasons -> M68K IRQ_3 (atarigt.cpp cage_irq_callback)
//
// Boot: when the DSP is released from reset ((control&3) rising from 0), the
// real TMS320C31 boots from its ROM and posts a word to the main latch. The
// stub posts BOOT_VALUE after BOOT_DELAY cycles; it is a parameter because
// the real value comes from the DSP program.
`default_nettype none

module agt_cage_stub #(
	parameter [15:0] BOOT_VALUE    = 16'h0001,
	parameter int    BOOT_DELAY    = 2048,     // clk_sys cycles after release
	parameter int    CONSUME_DELAY = 64        // cycles to "consume" a command
)(
	input  wire         clk,
	input  wire         rst_n,

	// board-core cage port (CPU side)
	input  wire         req,
	input  wire         we,
	input  wire  [3:0]  be,          // [3:2]=main halfword, [1:0]=control
	input  wire  [31:0] wdata,
	output logic        ack,
	output logic [31:0] rdata,

	output logic        irq,         // -> M68K IRQ_3, level
	// Debug: what the game does on this port. The handshake it expects lives
	// in the CAGE DSP's ROM program, not in cage.cpp, so it has to be observed.
	output logic [15:0] dbg_last_cmd,      // last command the CPU posted
	output logic [15:0] dbg_control,       // current control register
	output logic [15:0] dbg_ctrl_writes,   // control writes since reset
	output logic [15:0] dbg_main_reads     // main_r (response pops) since reset
);

	logic [15:0] control;
	logic [15:0] latch;              // DSP -> main data latch
	logic [15:0] cmd;                // main -> DSP command (held for inspection)
	logic        cpu2cage_ready;     // control_r bit 1
	logic        cage2cpu_ready;     // control_r bit 0

	logic        dsp_running;
	assign dbg_control = control;
	logic [15:0] boot_cnt;
	logic        boot_posted;
	logic [7:0]  consume_cnt;

	// update_control_lines, both reasons, combinationally
	wire reason_buffer_empty = (control[1:0] == 2'b11) && !cpu2cage_ready;
	wire reason_data_ready   = control[1] && cage2cpu_ready;
	assign irq = reason_buffer_empty | reason_data_ready;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			control <= 16'd0; latch <= 16'd0; cmd <= 16'd0;
			cpu2cage_ready <= 1'b0; cage2cpu_ready <= 1'b0;
			dsp_running <= 1'b0; boot_cnt <= 16'd0; boot_posted <= 1'b0;
			consume_cnt <= 8'd0;
			ack <= 1'b0; rdata <= 32'd0;
			dbg_last_cmd <= 16'd0; dbg_ctrl_writes <= 16'd0;
			dbg_main_reads <= 16'd0;
		end else begin
			ack <= 1'b0;

			// DSP-side autonomous behaviour
			if (dsp_running) begin
				// boot message: post once, BOOT_DELAY after release
				if (!boot_posted) begin
					boot_cnt <= boot_cnt + 16'd1;
					if (boot_cnt == BOOT_DELAY[15:0]) begin
						latch <= BOOT_VALUE;
						cage2cpu_ready <= 1'b1;
						boot_posted <= 1'b1;
					end
				end
				// command consumption (cage_from_main_r's flag clear)
				if (cpu2cage_ready) begin
					consume_cnt <= consume_cnt + 8'd1;
					if (consume_cnt == CONSUME_DELAY[7:0]) begin
						cpu2cage_ready <= 1'b0;
						consume_cnt <= 8'd0;
					end
				end
			end

			// CPU access
			if (req && !ack) begin
				ack <= 1'b1;
				if (we) begin
					// control_w (low halfword lanes)
					if (be[1] || be[0]) begin
						control <= wdata[15:0];
						if (dbg_ctrl_writes != 16'hFFFF)
							dbg_ctrl_writes <= dbg_ctrl_writes + 16'd1;
						if (wdata[1:0] == 2'b00) begin
							// DSP reset: cage.cpp clears both ready flags
							cpu2cage_ready <= 1'b0;
							cage2cpu_ready <= 1'b0;
							dsp_running <= 1'b0;
							boot_posted <= 1'b0; boot_cnt <= 16'd0;
							consume_cnt <= 8'd0;
						end else begin
							dsp_running <= 1'b1;
						end
					end
					// main_w (high halfword lanes)
					if (be[3] || be[2]) begin
						cmd <= wdata[31:16];
						dbg_last_cmd <= wdata[31:16];
						cpu2cage_ready <= 1'b1;
						consume_cnt <= 8'd0;
					end
				end else begin
					rdata <= {latch, 14'd0, cpu2cage_ready, cage2cpu_ready};
					// main_r side effect: reading the high halfword clears
					// cage_to_cpu_ready (cage.cpp main_r)
					if (be[3] || be[2]) begin
						cage2cpu_ready <= 1'b0;
						if (dbg_main_reads != 16'hFFFF)
							dbg_main_reads <= dbg_main_reads + 16'd1;
					end
				end
			end
		end
	end

endmodule

`default_nettype wire
