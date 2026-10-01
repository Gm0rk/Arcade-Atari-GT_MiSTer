// agt_cage_top.sv -- CAGE audio board, the 68020 side.
//
// Wraps `agt_cage_comm` (the 68020-facing mailbox, transcribed from
// `cage.cpp` and checked event for event against a golden model) in the bus
// contract the rest of the core speaks, and brings out the DSP-side signals
// the TMS320C31 needs. The DSP attaches one level up, so every tie-off is
// visible at the instantiation rather than buried in a lint sink.
//
// A read of 0xC00000 bits 31:16 pops the response latch, and
// `agt_main_memmap.sv` holds `req` high until it sees `ack`. So the comm is
// selected once per request, not once per cycle (the gate below), and
// `rdata` is registered on the same edge as `ack`: the memmap samples it on
// the ack cycle, and the comm's read data is combinational off `cpu_sel`.
//
// The four overlay outputs match `agt_cage_stub`'s and feed `cg_last_cmd`,
// `cg_control`, `cg_ctrl_wr`, `cg_main_rd` and the CGCM/CGIO counters. Three
// are counted from the gated bus; the fourth is the comm's `control_now`.
//
// `dsp_irq0_level` is not the comm's `dsp_irq0`. The comm drives that as a
// one-cycle pulse, faithful to `cage.cpp`'s `set_irq_line(0, ASSERT_LINE)`,
// but `agt_c31` writes IF once per step, not per cycle, and misses most such
// pulses. IRQ0 gets `cpu_to_cage_ready` instead: set when a command is
// posted, cleared when the DSP consumes it, a level asserted exactly while a
// command is pending, as the '31's IRQ0 pin is.
`default_nettype none

module agt_cage_top (
	input  logic        clk,
	input  logic        rst_n,

	// 68020 side: the same contract agt_cage_stub presents, so this is a
	// drop-in for it, plus the outputs below.
	input  logic        req,
	input  logic        we,
	input  logic [3:0]  be,          // [3:2]=main halfword, [1:0]=control
	input  logic [31:0] wdata,
	output logic        ack,
	output logic [31:0] rdata,
	output logic        irq,         // -> M68K IRQ_3, level

	// DSP side (the TMS320C31 attaches one level up)
	output logic        dsp_irq0_level,   // -> agt_c31.irq_in[0]; a level, see above
	// The comm's one-cycle "command posted" pulse, kept off IRQ0.
	// agt_cage_mbx_cdc numbers each post with it, so a late consume of an older
	// command cannot clear a newer one. Nothing else reads it.
	output logic        cmd_post,
	output logic [15:0] from_main,         // the pending command word
	output logic        cpu_to_cage_ready,
	output logic        cage_to_cpu_ready,
	input  logic        dsp_cmd_read,      // DSP pulses when it takes from_main
	input  logic        dsp_resp_we,       // DSP pulses to post a response
	input  logic [15:0] dsp_resp_data,
	output logic        cage_reset,        // 1 = DSP held in reset

	// audio out (tied off here)
	output logic [15:0] audio_l,
	output logic [15:0] audio_r,
	output logic        dsp_present,       // always 0 here

	// overlay (the same four agt_cage_stub provides)
	output logic [15:0] dbg_last_cmd,      // last command the CPU posted
	output logic [15:0] dbg_control,       // current control register
	output logic [15:0] dbg_ctrl_writes,   // control writes since reset
	output logic [15:0] dbg_main_reads     // response pops since reset
);

	// The gate: one comm access per bus request, however long `req` is held.
	// The obvious `req && !ack` (as in `agt_cage_stub`) oscillates under a held
	// request:
	//     N   req=1 ack=0 -> sel=1
	//     N+1 req=1 ack=1 -> sel=0
	//     N+2 req=1 ack=0 -> sel=1   <-- again
	// It is safe in the machine only because the memmap drops `cage_req` the
	// cycle it sees `cage_ack`. A destructive read should not rely on that, so
	// `served` latches the request and holds the gate shut until `req` drops.
	//
	// Contract: `req` must go low for at least one cycle between accesses (the
	// memmap returns to `M_IDLE` between them).
	logic served;
	wire  cpu_sel = req && !served;

	logic [31:0] comm_rdata;
	logic [15:0] control_now;

	agt_cage_comm u_comm (
		.clk(clk), .rst_n(rst_n),
		.cpu_sel(cpu_sel), .cpu_we(we), .cpu_be(be),
		.cpu_wdata(wdata), .cpu_rdata(comm_rdata),
		.irq3(irq),
		.cage_reset(cage_reset),
		.dsp_irq0(cmd_post),              // the pulse, not IRQ0; numbers posts
		.from_main(from_main), .cpu_to_cage_ready(cpu_to_cage_ready),
		.dsp_cmd_read(dsp_cmd_read), .dsp_resp_we(dsp_resp_we),
		.dsp_resp_data(dsp_resp_data), .cage_to_cpu_ready(cage_to_cpu_ready),
		.control_now(control_now)
	);

	// The level, not the pulse: asserted for as long as a command is pending.
	assign dsp_irq0_level = cpu_to_cage_ready;

	// ack and the registered read datum
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ack    <= 1'b0;
			rdata  <= 32'd0;
			served <= 1'b0;
		end else begin
			ack <= cpu_sel;
			// Captured on the cycle `cpu_sel` is high, presented on the cycle
			// `ack` is high, which is when the memmap samples it.
			if (cpu_sel) begin
				rdata  <= comm_rdata;
				served <= 1'b1;
			end
			if (!req) served <= 1'b0;       // rearm for the next request
		end
	end

	// The overlay's four counters, counted from the gated access: bus
	// transactions, not cycles, as `agt_cage_stub` counts them.
	always_ff @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			dbg_ctrl_writes <= 16'd0;
			dbg_main_reads  <= 16'd0;
		end else if (cpu_sel) begin
			// control_w: low halfword lanes
			if (we && (be[1:0] != 2'b00) && dbg_ctrl_writes != 16'hFFFF)
				dbg_ctrl_writes <= dbg_ctrl_writes + 16'd1;
			// main_r: high halfword lanes on a read (the pop)
			if (!we && (be[3:2] != 2'b00) && dbg_main_reads != 16'hFFFF)
				dbg_main_reads <= dbg_main_reads + 16'd1;
		end
	end

	assign dbg_last_cmd = from_main;
	assign dbg_control  = control_now;

	// Tied off here; agt_cage's audio comes from agt_cage_dac.
	assign audio_l     = 16'd0;
	assign audio_r     = 16'd0;
	assign dsp_present = 1'b0;

endmodule

`default_nettype wire
