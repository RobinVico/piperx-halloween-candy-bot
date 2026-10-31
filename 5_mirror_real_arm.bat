@echo off
cd /d "%~dp0"
rem Read-only: shows the REAL arm's pose in the simulator (move the disabled arm by hand).
.venv\Scripts\python.exe -m sim.mirror_real
pause
