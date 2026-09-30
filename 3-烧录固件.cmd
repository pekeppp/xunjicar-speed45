@echo off
chcp 936 >nul
cd /d "%~dp0"
echo ============================================================
echo   3. 烧录固件到 STM32
echo   注意: 烧完芯片会自动复位, 3 秒后车会自己起步循迹
echo         所以先让轮子架空, 或者把车放正在黑线上
echo ============================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\flash.ps1"
echo.
pause
