@echo off
cd /d "%~dp0"
rem LIVE: SO-101 drives the REAL PiPER X. Motors are enabled only after you click
rem "enable and start" in the panel and confirm. ESC / red button = emergency stop.
.venv\Scripts\python.exe -m teleop.teleop_real --live
pause
