@echo off
cd /d "%~dp0"
rem Halloween candy bot on the REAL arm. Close the teleop panel first (CAN is exclusive).
rem Click "start" in the panel and confirm to enable. ESC / red button = emergency stop.
.venv\Scripts\python.exe -m candy.candy_bot --live
pause
