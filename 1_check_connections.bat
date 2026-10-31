@echo off
cd /d "%~dp0"
echo === CAN bus (receive only) ===
.venv\Scripts\python.exe tools\can_probe.py 2
echo.
echo === PiPER X status (read-only) ===
.venv\Scripts\python.exe tools\piper_read.py 2
echo.
echo === SO-101 leader (read-only) ===
.venv\Scripts\python.exe tools\check_so101.py COM3
pause
