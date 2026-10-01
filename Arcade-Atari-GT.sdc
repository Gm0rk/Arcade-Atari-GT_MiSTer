derive_pll_clocks
derive_clock_uncertainty

# Atari GT core timing constraints.
#
# Three clock domains, all from the PLL (derive_pll_clocks above creates and
# relates them):
#   outclk_0 -> clk_sys  (57.27272 MHz) : render pipeline
#   outclk_1 -> clk_pix  ( 7.15909 MHz) : video timing + scanout
#   outclk_3 -> clk_dsp  (37.05882 MHz) : CAGE DSP, 630/17
#
# Because they share a PLL, TimeQuest times paths between them. The design
# crosses domains only through explicit synchronizers, so each crossing is cut
# here by register name. Don't replace these with a clock group
# (set_clock_groups -asynchronous): it would also silently un-time any crossing
# added later without a synchronizer.
#
# A pattern that matches nothing is only a Quartus warning, and the crossing is
# then timed. tools/check_sdc.py fails if a pattern here names a register that
# no longer exists.

# CDC cuts: the 4-phase handshake signals between clk_sys and clk_pix.
# Each is captured by a 2-flop synchronizer on the receiving side; the
# source is a stable held level (never a single-cycle pulse), which is what
# makes the crossing safe regardless of the clock ratio.

# clk_sys -> clk_pix: render-complete request
set_false_path -from [get_registers {*u_video|cdc_req*}] -to [get_registers {*u_video|cdc_req_sync_chain*}]

# clk_pix -> clk_sys: swap acknowledge
set_false_path -from [get_registers {*u_video|cdc_ack*}] -to [get_registers {*u_video|cdc_ack_sync_chain*}]

# clk_pix -> clk_sys: the line-0 resync trigger (toggle-encoded)
set_false_path -from [get_registers {*u_video|resync_toggle*}] -to [get_registers {*u_video|frame_start_sync_chain*}]

# clk_dsp -> clk_sys: the DCLK measurement counter.
# A free-running Gray-coded counter in clk_dsp, a 2-flop synchronizer
# (dg_s1 -> dg_s2) in clk_sys, then Gray-to-binary. Only one bit changes per
# increment, so a metastable sample yields the old or the new value, never one
# the counter never held. Uncut, TimeQuest times these paths against the worst,
# nearly coincident clk_dsp/clk_sys edge pair and they fail setup.
# The release build (no overlay) removes the counter and its reset
# synchroniser, and since D-648 the debug build does too (DCLK left the
# overlay; the source keeps it for when a slot shows it again). Both cuts
# are made only where the registers exist, rather than as "Ignored filter"
# warnings in every compile.
if {[get_collection_size [get_registers -nowarn {*dsp_gray*}]] > 0} {
	set_false_path -from [get_registers {*dsp_gray*}] -to [get_registers {*dg_s1*}]
}

# sys_rst_n -> clk_dsp: async-assert / sync-deassert reset synchronizer, the
# DCLK counter's only reset (above).
if {[get_collection_size [get_registers -nowarn {*dsp_rst_sync*}]] > 0} {
	set_false_path -to [get_registers {*dsp_rst_sync*}]
}

# clk_sys <-> clk_dsp: the CAGE sound board.
# Every word that crosses goes through agt_cdc_hs (a toggle handshake: the
# request and acknowledge toggles each cross on two flops, and the data
# register is held stable while busy, so it needs no synchronizer of its own).
# These three patterns cover every instance (SDRAM bridge, mailbox, IRAM boot
# words, entry point). Every level that crosses (the release, the download
# flag, "held and drained", the core's halt, the plumbing reset, IOF's INXF1)
# goes through a 2-flop synchronizer whose register name begins `cagerst_sync`.
# A timed clk_sys/clk_dsp path that is not one of these is a missed crossing.
set_false_path -from [get_registers {*cdchs_tog*}]  -to [get_registers {*cdchs_req_s1*}]
set_false_path -from [get_registers {*cdchs_seen*}] -to [get_registers {*cdchs_ack_s1*}]
set_false_path -from [get_registers {*cdchs_hold*}] -to [get_registers {*cdchs_q*}]
set_false_path -to [get_registers {*cagerst_sync*}]

# Reset-domain crossing: video_rst_n_sysclk is generated in clk_sys and
# synchronized into clk_pix by vrst_sync_pix in Arcade-Atari-GT.sv
# (async assert / sync deassert).
set_false_path -to [get_registers {*vrst_sync_pix*}]

# Asynchronous I/O
set_false_path -from [get_ports *] -to [get_registers *]
set_false_path -from [get_registers *] -to [get_ports *]
