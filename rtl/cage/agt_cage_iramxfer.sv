// agt_cage_iramxfer.sv -- the boot table's IRAM words, from the download
// (clk_sys) into agt_c31's boot port (clk_dsp)
//
// `agt_cage_boot` parses the boot EPROM out of the download in clk_sys. Most
// of the table goes to cageram (agt_cage_ramload); 12 words go to the C31's
// on-chip RAM at 0x809FC0-0x809FCB, which is in clk_dsp. Its boot port writes
// while the core is held in reset, and the DSP's clock runs during the
// download, so the words cross as they arrive.
//
// One word per agt_cdc_hs crossing, plus a one-word skid register for a word
// that arrives while the previous is still crossing. `wait_o` joins
// `ioctl_wait`: it rises as soon as one word is in flight, and the parser needs
// four more bytes to make the next, so the skid only catches a stream that
// ignores the wait for a byte or two. `n_dropped` counts words that found both
// full; it must read 0.
`default_nettype none

module agt_cage_iramxfer (
	// clk_sys: agt_cage_boot's IRAM port
	input  wire         clk_sys,
	input  wire         rst_sys_n,
	input  wire         i_we,
	input  wire  [10:0] i_addr,
	input  wire  [31:0] i_data,
	output wire         wait_o,
	output logic [7:0]  n_dropped, // words lost with the skid full (must be 0)

	// clk_dsp: agt_c31's boot port
	input  wire         clk_dsp,
	input  wire         rst_dsp_n,
	output wire         boot_we,
	output wire  [10:0] boot_addr,
	output wire  [31:0] boot_data
);

	logic        skid_v;
	logic [42:0] skid;
	wire         busy;

	// Send the skid first if it holds a word, else the arriving one.
	wire         send   = !busy && (skid_v || i_we);
	wire  [42:0] s_word = skid_v ? skid : {i_addr, i_data};

	always_ff @(posedge clk_sys or negedge rst_sys_n) begin
		if (!rst_sys_n) begin
			skid_v    <= 1'b0;
			skid      <= 43'd0;
			n_dropped <= 8'd0;
		end else begin
			if (send && skid_v) begin
				// the skid went; an arriving word takes its place
				if (i_we) skid <= {i_addr, i_data};
				else      skid_v <= 1'b0;
			end else if (i_we && !send) begin
				// could not send: into the skid if it is free
				if (!skid_v) begin
					skid   <= {i_addr, i_data};
					skid_v <= 1'b1;
				end else if (n_dropped != 8'hFF)
					n_dropped <= n_dropped + 8'd1;
			end
		end
	end

	assign wait_o = busy || skid_v;

	wire [42:0] q;
	agt_cdc_hs #(.W(43)) u_x (
		.s_clk(clk_sys), .s_rst_n(rst_sys_n),
		.s_send(send), .s_data(s_word), .s_busy(busy),
		.d_clk(clk_dsp), .d_rst_n(rst_dsp_n),
		.d_pulse(boot_we), .d_data(q)
	);
	assign boot_addr = q[42:32];
	assign boot_data = q[31:0];

endmodule

`default_nettype wire
