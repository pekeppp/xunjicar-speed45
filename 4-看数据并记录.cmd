@echo off
chcp 936 >nul
cd /d "%~dp0"
echo ============================================================
echo   4. 看小车数据（USB-TTL 要接在 PA2/PA3/GND 上）
echo ============================================================
echo.
echo 本机串口列表:
python "%~dp0tools\telemetry_monitor.py" --list
echo.
set /p PORT=请输入串口号(例如 COM5)，直接回车退出: 
if "%PORT%"=="" goto end
echo.
echo 开始监视，同时把数据存到 run.csv（按 Ctrl+C 停止）
echo.
python "%~dp0tools\telemetry_monitor.py" --port %PORT% --csv "%~dp0run.csv"
:end
pause
