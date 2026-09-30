@echo off
chcp 936 >nul
cd /d "%~dp0"
echo ============================================================
echo   5. 轮子一直慢慢转 —— 单独验证「电机」这条硬件链路
echo ============================================================
echo.
echo   这个固件绕过了 循迹 / 激光避障 / 丢线判断 的全部逻辑,
echo   上电 3 秒后两个轮子会一直往前慢转 (占空比 25%%).
echo.
echo   *** 现在就把两个轮子架空（车翻过来或垫起来）! ***
echo   *** 否则烧完芯片自动复位, 车会一直往前跑.     ***
echo.
pause
echo.
echo [1/2] 编译调试固件（约 1~2 分钟）...
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\build_spin.ps1"
if errorlevel 1 goto failed
if not exist "%~dp0MotorSpin.hex" goto failed
echo.
echo [2/2] 烧录 MotorSpin.hex ...
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\flash.ps1" -Firmware "%~dp0MotorSpin.hex"
echo.
echo ============================================================
echo   怎么看结果:
echo     轮子一直慢转  =^> 电机 / TB6612 / 供电 / PWM 全部正常,
echo                       那"车不动"就是循迹模块或激光测距的问题
echo     轮子不转      =^> 是硬件问题, 按这个顺序查:
echo                       1) TB6612 的 STBY 有没有接 3V3  (最关键)
echo                          STBY 低电平 = 电机全部不转
echo                       2) VM 有没有接电池, 开关有没有打开
echo                       3) 板子自己有没有独立供电
echo ============================================================
echo.
echo   要调转速/反转: powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty 35
echo   测完烧回正常固件:  3-烧录固件.cmd
echo.
pause
exit /b 0

:failed
echo.
echo 编译失败, 没有烧录. 看上面的报错.
pause
exit /b 1