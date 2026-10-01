`timescale 1ns/10ps
module  pll_0002(

	// interface 'refclk'
	input wire refclk,

	// interface 'reset'
	input wire rst,

	// interface 'outclk0'
	output wire outclk_0,

	// interface 'outclk1'
	output wire outclk_1,

	// interface 'outclk2' -- SDRAM chip clock: same frequency as outclk_0,
	// phase -2500 ps so the chip's sampling edge trails our launch edge by
	// ~15 ns (command setup ~10 ns) and CL2 read data arrives ~3.5 ns after
	// our previous edge (capture setup ~14 ns, hold ~2 ns).
	output wire outclk_2,

	// interface 'outclk3' -- CAGE DSP clock, 630/17 = 37.058823 MHz.
	//
	// The VCO is pinned and this output must not move it: outclk_1 is
	// 14.318181/2 MHz (the arcade board's pixel clock) and outclk_0 is exactly
	// 8 x outclk_1, so the VCO has to stay 630 MHz; this output is a divisor
	// of it.
	//
	// The DSP keeps its own time in model cycles (its timers, its DMA, the
	// serial port), so this clock need not match the chip's 33.8688 MHz; it
	// only sets how fast the core gets through its steps. 630/17 leaves about
	// +2.1 ns of clk_dsp slack; 630/16 (39.375 MHz) would leave about +0.5.
	//
	// `altera_pll` takes these as targets and Quartus re-solves M/N and every
	// C counter for the whole set. After any change, the STA Clocks panel
	// must still read VCO 630.000 MHz (M 63, N 5), outclk_0 divide 11,
	// outclk_1 divide 88 and outclk_3 divide 17.
	output wire outclk_3,

	// interface 'locked'
	output wire locked
);

	altera_pll #(
		.fractional_vco_multiplier("false"),
		.reference_clock_frequency("50.0 MHz"),
		.operation_mode("direct"),
		.number_of_clocks(4),
		.output_clock_frequency0("57.272720 MHz"),
		.phase_shift0("0 ps"),
		.duty_cycle0(50),
		.output_clock_frequency1("7.159090 MHz"),
		.phase_shift1("0 ps"),
		.duty_cycle1(50),
		.output_clock_frequency2("57.272720 MHz"),
		// -2.5 ns expressed as a LEGAL positive shift: the Cyclone V PLL
		// only accepts non-negative multiples of its VCO step (~198.4 ps)
		// within one period, so -2500 ps wraps to 17466-2500 = 14967 ps,
		// whose nearest legal step is 14881 ps (effective lead 2.59 ns --
		// the 86 ps of rounding is noise against multi-ns margins).
		.phase_shift2("14881 ps"),
		.duty_cycle2(50),
		.output_clock_frequency3("37.058823 MHz"),   // 630/17
		.phase_shift3("0 ps"),
		.duty_cycle3(50),
		.output_clock_frequency4("0 MHz"),
		.phase_shift4("0 ps"),
		.duty_cycle4(50),
		.output_clock_frequency5("0 MHz"),
		.phase_shift5("0 ps"),
		.duty_cycle5(50),
		.output_clock_frequency6("0 MHz"),
		.phase_shift6("0 ps"),
		.duty_cycle6(50),
		.output_clock_frequency7("0 MHz"),
		.phase_shift7("0 ps"),
		.duty_cycle7(50),
		.output_clock_frequency8("0 MHz"),
		.phase_shift8("0 ps"),
		.duty_cycle8(50),
		.output_clock_frequency9("0 MHz"),
		.phase_shift9("0 ps"),
		.duty_cycle9(50),
		.output_clock_frequency10("0 MHz"),
		.phase_shift10("0 ps"),
		.duty_cycle10(50),
		.output_clock_frequency11("0 MHz"),
		.phase_shift11("0 ps"),
		.duty_cycle11(50),
		.output_clock_frequency12("0 MHz"),
		.phase_shift12("0 ps"),
		.duty_cycle12(50),
		.output_clock_frequency13("0 MHz"),
		.phase_shift13("0 ps"),
		.duty_cycle13(50),
		.output_clock_frequency14("0 MHz"),
		.phase_shift14("0 ps"),
		.duty_cycle14(50),
		.output_clock_frequency15("0 MHz"),
		.phase_shift15("0 ps"),
		.duty_cycle15(50),
		.output_clock_frequency16("0 MHz"),
		.phase_shift16("0 ps"),
		.duty_cycle16(50),
		.output_clock_frequency17("0 MHz"),
		.phase_shift17("0 ps"),
		.duty_cycle17(50),
		.pll_type("General"),
		.pll_subtype("General")
	) altera_pll_i (
		.rst	(rst),
		.outclk	({outclk_3, outclk_2, outclk_1, outclk_0}),
		.locked	(locked),
		.fboutclk	( ),
		.fbclk	(1'b0),
		.refclk	(refclk)
	);
endmodule

