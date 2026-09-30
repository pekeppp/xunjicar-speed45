@echo off
chcp 936 >nul
cd /d "%~dp0"
echo ============================================================
echo   1. 编译 XunjiCar 固件
echo   （改完 Core\Inc\line_follow.h 里的参数后要先跑这个）
echo ============================================================
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\build.ps1"
echo.
echo ============================================================
echo   编译结束。有 "编译成功" 就可以去烧录了（3-烧录固件.cmd）
echo ============================================================
pause
