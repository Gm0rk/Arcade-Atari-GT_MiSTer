# post_flow.tcl -- runs at the end of every compile of either build
# (POST_FLOW_SCRIPT_FILE in Arcade-Atari-GT.qsf and Arcade-Atari-GT-Debug.qsf)
#
# 1. The framework's post-flow step, unchanged: sys/build_id.tcl writes the
#    build date for the next compile and jtag.cdf for this revision's .sof.
#    It was the POST_FLOW_SCRIPT_FILE until D-647; Quartus takes only one.
# 2. For the debug build, after a compile that reached timing analysis, the
#    reports every set of photos is scored with:
#      syn/hold.tcl      -> output_files/Arcade-Atari-GT-Debug_hold.txt
#      syn/sdram_dq.tcl  -> output_files/Arcade-Atari-GT-Debug_sdram_dq.txt
#      syn/fmax.tcl      -> output_files/Arcade-Atari-GT-Debug_fmax.txt, and
#                           _paths_clk_sys.txt / _paths_clk_dsp.txt (D-649)
#    They add a few minutes to the compile. A report that fails is a warning
#    in the Messages window, never a failed compile. The syn/ scripts are not
#    in the GitHub tree; without them this step is skipped with a note.
# 3. For the release build (D-649, amended), after a compile that reached
#    timing analysis and met timing, the core is moved and dated:
#      output_files/Arcade-Atari-GT.rbf -> releases/Arcade-Atari-GT_YYYYMMDD.rbf
#    The date is the one compiled into the core (build_id.v, written when this
#    compile started). A second release compile on the same day replaces that
#    day's file. A compile that failed, stopped early or missed timing leaves
#    the .rbf where it is, with a warning saying why.
#
# Each compile's steps are logged in output_files/<revision>_post_flow.log.
#
# To have the release build's timing reports too, add Arcade-Atari-GT to
# TIMING_REVISIONS. To release the debug build as well, add
# Arcade-Atari-GT-Debug to RELEASE_REVISIONS (its file would be
# Arcade-Atari-GT-Debug_YYYYMMDD.rbf, which the release .mra files do not
# load). RELEASE_NEEDS_TIMING 0 releases a compile with negative slack.
set TIMING_REVISIONS     {Arcade-Atari-GT-Debug}
set RELEASE_REVISIONS    {Arcade-Atari-GT}
set RELEASE_DIR          releases
set RELEASE_NEEDS_TIMING 1

# The core's build date, read before step 1 rewrites build_id.v for the next
# compile: `define BUILD_DATE "YYMMDD", from the pre-flow step at the start of
# this one. Empty if it cannot be read (the release step then uses the .rbf's
# own date).
set agt_build_date ""
if {![catch {open build_id.v r} agt_bf]} {
	set agt_bt [read $agt_bf]
	close $agt_bf
	if {[regexp {BUILD_DATE\s+"([0-9]{6})"} $agt_bt -> agt_yymmdd]} {
		set agt_build_date "20$agt_yymmdd"
	}
}

# 1. the framework's step (sets project_name, revision and outpath)
source sys/build_id.tcl

# the log: started fresh for each compile, appended to by the steps
proc agt_log_start {rev outdir} {
	global agt_log
	set agt_log "$outdir/${rev}_post_flow.log"
	if {[catch {
		set lf [open $agt_log w]
		puts $lf "post_flow.tcl: $rev, [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
		close $lf
	}]} { set agt_log "" }
}
proc agt_log {text} {
	global agt_log
	if {$agt_log eq ""} { return }
	catch {
		set lf [open $agt_log a]
		puts $lf $text
		close $lf
	}
}
proc agt_warn {text} {
	post_message -type warning "post_flow.tcl: $text"
	agt_log "WARNING: $text"
}
proc agt_info {text} {
	post_message "post_flow.tcl: $text"
	agt_log $text
}

# This compile reached timing analysis: its summaries are in order, each no
# older than the one before. A compile that stopped in the Fitter leaves a new
# map summary and older fit and sta ones. Returns "" if so, else the reason.
proc agt_reached_sta {rev outdir} {
	set map "$outdir/$rev.map.summary"
	set fit "$outdir/$rev.fit.summary"
	set sta "$outdir/$rev.sta.summary"
	foreach f [list $map $fit $sta] {
		if {![file exists $f]} {
			return "$f is missing"
		}
	}
	if {[file mtime $fit] < [file mtime $map] || [file mtime $sta] < [file mtime $fit]} {
		return "this compile did not reach timing analysis"
	}
	return ""
}

# 2. the timing reports
proc agt_post_timing {rev outdir wanted} {
	global quartus
	if {[lsearch -exact $wanted $rev] < 0} {
		return
	}
	set why [agt_reached_sta $rev $outdir]
	if {$why ne ""} {
		agt_warn "$why; no timing reports"
		return
	}
	set sta_exe [file join $quartus(binpath) quartus_sta]
	foreach script {syn/hold.tcl syn/sdram_dq.tcl syn/fmax.tcl} {
		if {![file exists $script]} {
			agt_warn "$script is not here (syn/ is not in the GitHub tree); skipped"
			continue
		}
		post_message "post_flow.tcl: running $script on $rev"
		set failed [catch {exec $sta_exe -t $script $rev 2>@1} result]
		agt_log "\n==== $script ([expr {$failed ? {FAILED} : {done}}])"
		agt_log $result
		if {$failed} {
			agt_warn "$script did not finish cleanly; see the log"
		} else {
			post_message "post_flow.tcl: $script done"
		}
	}
}

# 3. the release
proc agt_release {rev outdir wanted dir date needs_timing} {
	if {[lsearch -exact $wanted $rev] < 0} {
		return
	}
	set rbf "$outdir/$rev.rbf"
	if {![file exists $rbf]} {
		agt_warn "$rbf is missing; nothing released"
		return
	}
	set why [agt_reached_sta $rev $outdir]
	if {$why ne ""} {
		agt_warn "$why; $rbf not released"
		return
	}
	# written by this compile's Assembler, which runs after the Fitter
	if {[file mtime $rbf] < [file mtime "$outdir/$rev.fit.summary"]} {
		agt_warn "$rbf is older than this compile's fit; not released"
		return
	}
	if {$needs_timing} {
		# every Slack in the timing summary, per clock and check type
		set sf [open "$outdir/$rev.sta.summary" r]
		set st [read $sf]
		close $sf
		set worst ""
		foreach {-> type slack} [regexp -all -inline {Type\s*:\s*([^\n]*?)\s*\nSlack\s*:\s*(-?[0-9.]+)} $st] {
			if {$slack < 0 && ($worst eq "" || $slack < [lindex $worst 1])} {
				set worst [list $type $slack]
			}
		}
		if {$worst ne ""} {
			agt_warn "timing not met ([lindex $worst 0]: slack [lindex $worst 1]); $rbf not released"
			return
		}
	}
	if {$date eq ""} {
		set date [clock format [file mtime $rbf] -format %Y%m%d]
	}
	set dst [file join $dir "${rev}_${date}.rbf"]
	if {[catch {
		file mkdir $dir
		set replaced [file exists $dst]
		file rename -force $rbf $dst
	} err]} {
		agt_warn "could not move $rbf to $dst: $err"
		return
	}
	if {$replaced} {
		agt_info "released $rbf as $dst (replacing an earlier build of the same day)"
	} else {
		agt_info "released $rbf as $dst"
	}
}

if {$revision eq ""} { set revision $project_name }
agt_log_start $revision $outpath
agt_post_timing $revision $outpath $TIMING_REVISIONS
agt_release $revision $outpath $RELEASE_REVISIONS $RELEASE_DIR $agt_build_date $RELEASE_NEEDS_TIMING
