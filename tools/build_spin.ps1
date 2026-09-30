<#
    build_spin.ps1  ——  编译"两个轮子一直慢慢转"的调试固件

    用途: 单独验证 "单片机 -> TIM8 PWM -> TB6612 -> 电机 -> 电池" 这一整条硬件链路。
          它完全绕过 循迹 / 激光避障 / 丢线判断, 上电延时后两个轮子一直往前慢转。

    原理: 临时把 main.c 的 MOTOR_SPIN_TEST 改成 1 -> 编译 -> 导出 MotorSpin.hex/.bin
          -> 还原 main.c -> 重新编译正常固件。
          所以 1-编译固件.cmd / 3-烧录固件.cmd 永远还是正常的循迹固件, 不会烧错。

    用法 (在工程根目录):
        powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1
        powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty 35     # 转快一点
        powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty -25    # 反转
        powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty 20 -DelayMs 5000

    输出: MotorSpin.hex / MotorSpin.bin (工程根目录)
#>

param(
    [int]$Duty    = 0,     # 0 = 沿用 main.c 里现有的 MOTOR_SPIN_DUTY
    [int]$DelayMs = -1     # -1 = 沿用 main.c 里现有的 MOTOR_SPIN_DELAY_MS
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projRoot  = (Resolve-Path (Join-Path $scriptDir '..')).Path
$mainC     = Join-Path $projRoot 'XunjiCar\XunjiCar\Core\Src\main.c'
$buildPs1  = Join-Path $scriptDir 'build.ps1'

if (-not (Test-Path $mainC))    { throw "找不到 main.c: $mainC" }
if (-not (Test-Path $buildPs1)) { throw "找不到 build.ps1: $buildPs1" }

$enc      = New-Object System.Text.UTF8Encoding($false)
$original = [System.IO.File]::ReadAllBytes($mainC)

Write-Host ""
Write-Host "==== 编译『轮子一直慢慢转』调试固件 ====" -ForegroundColor Cyan

try {
    $text = $enc.GetString($original)

    $kTest = '#define MOTOR_SPIN_TEST          (0)'
    if (-not $text.Contains($kTest)) { throw "main.c 里找不到 MOTOR_SPIN_TEST 的 (0) 定义（是不是被改过了？）" }
    $text = $text.Replace($kTest, '#define MOTOR_SPIN_TEST          (1)')

    if ($Duty -ne 0) {
        $kDuty = '#define MOTOR_SPIN_DUTY          (25)'
        if (-not $text.Contains($kDuty)) { throw "找不到 MOTOR_SPIN_DUTY 定义" }
        $text = $text.Replace($kDuty, "#define MOTOR_SPIN_DUTY          ($Duty)")
        Write-Host ("占空比   : {0} %" -f $Duty) -ForegroundColor Yellow
    } else {
        Write-Host "占空比   : 用 main.c 里的默认值"
    }

    if ($DelayMs -ge 0) {
        $kDly = '#define MOTOR_SPIN_DELAY_MS      (3000u)'
        if (-not $text.Contains($kDly)) { throw "找不到 MOTOR_SPIN_DELAY_MS 定义" }
        $text = $text.Replace($kDly, "#define MOTOR_SPIN_DELAY_MS      (${DelayMs}u)")
        Write-Host ("启动延时 : {0} ms" -f $DelayMs) -ForegroundColor Yellow
    }

    [System.IO.File]::WriteAllBytes($mainC, $enc.GetBytes($text))
    Write-Host ""

    # --- 编译调试固件 ---
    & powershell -NoProfile -ExecutionPolicy Bypass -File $buildPs1
    if ($LASTEXITCODE -ne 0) { throw "调试固件编译失败 (exit $LASTEXITCODE)" }

    $outDir = Join-Path $projRoot 'XunjiCar\XunjiCar\build\out'
    foreach ($ext in @('hex', 'bin', 'elf')) {
        Copy-Item (Join-Path $outDir "XunjiCar.$ext") (Join-Path $projRoot "MotorSpin.$ext") -Force
    }

    Write-Host ""
    Write-Host "==== 调试固件已生成 ====" -ForegroundColor Green
    Write-Host ("  " + (Join-Path $projRoot 'MotorSpin.hex'))
    Write-Host ("  " + (Join-Path $projRoot 'MotorSpin.bin'))
}
finally {
    # 无论成功失败, main.c 一定还原
    [System.IO.File]::WriteAllBytes($mainC, $original)
    Write-Host ""
    Write-Host "main.c 已还原 (MOTOR_SPIN_TEST 回到 0)" -ForegroundColor DarkGray
}

# --- 重编译正常固件, 保证 build\out\ 里留的是循迹固件而不是刚才的调试固件 ---
Write-Host ""
Write-Host "重新编译正常循迹固件 (让 build\out\ 恢复正确)..." -ForegroundColor Cyan
& powershell -NoProfile -ExecutionPolicy Bypass -File $buildPs1
if ($LASTEXITCODE -ne 0) { throw "正常固件重编译失败 (exit $LASTEXITCODE)" }

Write-Host ""
Write-Host "全部完成:" -ForegroundColor Green
Write-Host "  调试固件 = MotorSpin.hex                      (轮子一直慢转)"
Write-Host "  正常固件 = XunjiCar\XunjiCar\build\out\XunjiCar.hex   (循迹)"
