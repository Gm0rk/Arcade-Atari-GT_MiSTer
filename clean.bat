@echo off
del /s *.bak
del /s *.orig
del /s *.rej
del /s *~
rmdir /s /q db
rmdir /s /q incremental_db
rmdir /s /q output_files
rmdir /s /q simulation
rmdir /s /q greybox_tmp
rmdir /s /q hc_output
rmdir /s /q .qsys_edit
rmdir /s /q hps_isw_handoff
rmdir /s /q sys\.qsys_edit
rmdir /s /q sys\vip
rem the syn\ sub-projects compile in their own folders
rmdir /s /q syn\c31\db
rmdir /s /q syn\c31\incremental_db
rmdir /s /q syn\c31\output_files
rmdir /s /q syn\dcache\db
rmdir /s /q syn\dcache\incremental_db
rmdir /s /q syn\dcache\output_files
for /d %%i in (sys\*_sim) do rmdir /s /q "%%i"
for /d %%i in (rtl\*_sim) do rmdir /s /q "%%i"
del build_id.v
del c5_pin_model_dump.txt
del PLLJ_PLLSPE_INFO.txt
del /s *.qws
del /s *.ppf
del /s *.ddb
del /s *.csv
del /s *.cmp
del /s *.sip
del /s *.spd
del /s *.bsf
rem *.f: the top level and sys\ only (tb\cage.f is a source list)
del *.f
del /s sys\*.f
del /s *.sopcinfo
del /s *.xml
del *.cdf
del *.rpt
del /s new_rtl_netlist
del /s old_rtl_netlist
pause
