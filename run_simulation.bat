@echo off
setlocal
cd /d "%~dp0"

echo ========================================================
echo  Building and Running LAVD Scheduler Simulation (C)
echo ========================================================

gcc "Simulated Berkeley Packet Filter\main-bpf.c" "Simulated Berkeley Packet Filter\latency_critical.c" -o "lavd_sim.exe" -I"Simulated Berkeley Packet Filter" -O2
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed!
    exit /b %ERRORLEVEL%
)

if "%~1"=="" (
    lavd_sim.exe
) else (
    lavd_sim.exe "%~1"
)

endlocal
