@echo off
chcp 936 >nul
cd /d "%~dp0"
echo ============================================================
echo   2. 检查 ST-Link 有没有认到芯片（不烧录，安全）
echo   接线: DIO -^> SWDIO(PA13)   CLK -^> SWCLK(PA14)
echo         GND -^> GND           3V3 -^> 3.3V
echo         BT0 不要接任何东西
echo ============================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\flash.ps1" -Check
echo.
pause
