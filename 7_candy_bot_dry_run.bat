@echo off
cd /d "%~dp0"
rem Candy bot with the SIMULATED arm only (cameras are real). The real arm never moves.
.venv\Scripts\python.exe -m candy.candy_bot
pause
