// agt_cage_comm.sv -- Atari GT / CAGE sound-board communication interface
//
// The 68020-facing half of the CAGE audio board: the two-way mailbox and the
// handshake/IRQ logic the main CPU uses to talk to the sound board.
// Transcribed from MAME's Atari CAGE device (src/mame/audio/cage.cpp:
// control_r/w, main_r/w, cage_from_main_r, cage_to_main_w,
// update_control_lines) so the 68020 side behaves exactly as the hardware.
// The TMS320C31 DSP is not here; it drives the dsp_* ports from outside (see
// agt_cage_top).
//
// CPU-side register map (atarigt.cpp sound_data_r/w at 0xC00000, a 32-bit
// access split into two 16-bit halves):
//   read  0xC00000, bits 15:0  -> control_r()
//                     bit 1 = cpu_to_cage_ready   (command still pending)
//                     bit 0 = cage_to_cpu_ready   (response waiting)
//   read  0xC00000, bits 31:16 -> main_r()        (pop response; clears
//                                                   cage_to_cpu_ready)
//   write 0xC00000, bits 15:0  -> control_w()      (m_control; bits 1:0
//                                                   gate IRQ + reset)
//   write 0xC00000, bits 31:16 -> main_w()         (push command; sets
//                                                   cpu_to_cage_ready,
//                                                   pulses DSP IRQ0)
//
// IRQ to the 68020 (cage_irq_callback -> M68K_IRQ_3), asserted when either
//   * (control & 3)==3 and the CPU->CAGE buffer is empty   (BUFFER_EMPTY), or
//   * (control & 2)      and a CAGE->CPU response is ready  (DATA_READY).
//
// CAGE reset: while control[1:0]==00 the DSP is held in reset; writing 00
// clears both ready flags (control_w's "both control lines 0" branch).

module agt_cage_comm (
	input  logic        clk,
	input  logic        rst_n,

	// 68020 side: 32-bit access at 0xC00000. Byte enables mirror the
	// ACCESSING_BITS_x checks in sound_data_r/w; cpu_sel pulses for one clk
	// per access.
	input  logic        cpu_sel,
	input  logic        cpu_we,
	input  logic [3:0]  cpu_be,        // [3:2]=high half, [1:0]=low half
	input  logic [31:0] cpu_wdata,
	output logic [31:0] cpu_rdata,

	output logic        irq3,          // -> 68020 IRQ3 (level, active high)

	// CAGE/DSP side: the DSP's reset, and the two-way latches. The DSP drives
	// dsp_* to read commands and post responses.
	output logic        cage_reset,    // 1 = DSP held in reset (control[1:0]==0)
	output logic        dsp_irq0,      // pulses when a new command is posted
	output logic [15:0] from_main,     // latched CPU->CAGE command
	output logic        cpu_to_cage_ready,
	input  logic        dsp_cmd_read,  // DSP pulses when it consumes from_main
	input  logic        dsp_resp_we,   // DSP pulses to post a response
	input  logic [15:0] dsp_resp_data,
	output logic        cage_to_cpu_ready,

	// The stored control word, read-only, for the overlay. cpu_rdata's low half
	// is control_r (the ready flags), not the word the CPU wrote, so the
	// overlay's control register cannot be recovered from the bus. Exported
	// directly rather than shadowed in the wrapper, so it cannot drift.
	output logic [15:0] control_now
);

	// m_control: only bits 1:0 matter to the handshake/IRQ/reset logic
	// modelled here.
	logic [15:0] control;
	logic [15:0] soundlatch;   // CAGE->CPU response latch (m_soundlatch)

	wire acc_low  = cpu_sel && (cpu_be[1:0] != 2'b00);   // ACCESSING_BITS_0_15
	wire acc_high = cpu_sel && (cpu_be[3:2] != 2'b00);   // ACCESSING_BITS_16_31

	// Reads are combinational; the pop side effect is in the sequential block.
	wire [15:0] control_r = {14'd0, cpu_to_cage_ready, cage_to_cpu_ready};
	wire [15:0] main_r    = soundlatch;
	assign cpu_rdata = {main_r, control_r};

	// IRQ3, as update_control_lines: BUFFER_EMPTY or DATA_READY.
	wire buffer_empty = ((control[1:0] == 2'b11) && !cpu_to_cage_ready);
	wire data_ready   = (control[1] && cage_to_cpu_ready);
	assign irq3 = buffer_empty || data_ready;

	assign cage_reset  = (control[1:0] == 2'b00);
	assign control_now = control;
	assign from_main  = from_main_r;
	logic [15:0] from_main_r;

	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			control            <= 16'd0;
			soundlatch         <= 16'd0;
			from_main_r        <= 16'd0;
			cpu_to_cage_ready  <= 1'b0;
			cage_to_cpu_ready  <= 1'b0;
			dsp_irq0           <= 1'b0;
		end else begin
			dsp_irq0 <= 1'b0;

			// DSP-side effects, checked first: on a same-cycle conflict the 68020
			// access below decides the ready flag.
			if (dsp_cmd_read)  cpu_to_cage_ready <= 1'b0;    // cage_from_main_r
			if (dsp_resp_we) begin                           // cage_to_main_w
				soundlatch        <= dsp_resp_data;
				cage_to_cpu_ready <= 1'b1;
			end

			// 68020-side accesses
			if (cpu_sel) begin
				if (cpu_we) begin
					if (acc_low) begin
						// control_w
						control <= cpu_wdata[15:0];
						if (cpu_wdata[1:0] == 2'b00) begin
							// "both control lines 0": DSP reset, clear flags
							cpu_to_cage_ready <= 1'b0;
							cage_to_cpu_ready <= 1'b0;
						end
					end
					if (acc_high) begin
						// main_w: post command to CAGE (data >> 16)
						from_main_r       <= cpu_wdata[31:16];
						cpu_to_cage_ready <= 1'b1;
						dsp_irq0          <= 1'b1;   // TMS3203X_IRQ0 assert
					end
				end else begin
					// reads with pop side effects
					if (acc_high) begin
						// main_r pops the response
						cage_to_cpu_ready <= 1'b0;
					end
					// control_r has no side effect
				end
			end
		end
	end

endmodule
