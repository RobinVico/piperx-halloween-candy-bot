@echo off
cd /d "%~dp0"
rem Real arm is connected but NEVER enabled. Panel + viewer show what would be sent.
.venv\Scripts\python.exe -m teleop.teleop_real
pause
