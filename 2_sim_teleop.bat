@echo off
cd /d "%~dp0"
rem SO-101 drives the SIMULATED PiPER X only. Real arm is not touched.
.venv\Scripts\python.exe -m sim.teleop_sim
pause
