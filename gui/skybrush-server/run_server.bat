@echo off
rem Starts the Skybrush server with the swarm_bridge extension enabled.
rem Config: etc\conf\swarm.jsonc (set command_host / telemetry_url to the
rem swarm-master machine running main.py).
rem Optional: pass a different config file as the first argument.

cd /d "%~dp0"

set CONFIG=%~1
if "%CONFIG%"=="" set CONFIG=etc\conf\swarm.jsonc

echo Starting skybrushd with %CONFIG%
uv run skybrushd --config "%CONFIG%"

echo.
echo Server stopped (exit code %ERRORLEVEL%).
pause
