@echo off
rem Starts the swarm master (main.py) on this machine.
rem   run_main.bat [bind_ip] [num_bots]
rem bind_ip defaults to auto-detect; for WSL SITL use the Windows-side WSL
rem address (172.24.96.1). num_bots defaults to 10.
cd /d "%~dp0"
if not "%~1"=="" set SWARM_IP=%~1
if not "%~2"=="" set SWARM_NUM_BOTS=%~2
python main.py
pause
