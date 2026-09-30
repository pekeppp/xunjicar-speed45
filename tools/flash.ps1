<#
    flash.ps1  ——  用 ST-Link V2 把固件烧进 STM32F407

    用法（在 xunjicar 目录下执行）：
        powershell -ExecutionPolicy Bypass -File tools\flash.ps1
        powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Check      # 只连接芯片，不烧录
        powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Erase      # 全片擦除后再烧
        powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Speed 500  # 克隆版不稳定时降速

    接线（ST-Link V2 -> 开发板）：
        SWDIO -> PA13      SWCLK -> PA14      GND -> GND      3.3V -> 3V3
    烧录工具：OpenOCD（PlatformIO 包，原生支持 STM32F407，不需要额外芯片包）
#>

param(
    [string]$Firmware = '',
    [int]$Speed = 1000,          # SWD 时钟 kHz
    [switch]$Erase,
    [switch]$Check
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projDir   = (Resolve-Path (Join-Path $scriptDir '..\XunjiCar\XunjiCar')).Path
$outDir    = Join-Path $projDir 'build\out'

if (-not $Firmware) { $Firmware = Join-Path $outDir 'XunjiCar.hex' }
if (-not (Test-Path $Firmware)) {
    Write-Host "找不到固件: $Firmware" -ForegroundColor Red
    Write-Host "请先编译：powershell -ExecutionPolicy Bypass -File tools\build.ps1" -ForegroundColor Yellow
    exit 1
}

Write-Host "固件    : $Firmware"
Write-Host "SWD 时钟: $Speed kHz"
Write-Host ""

# --------------------------------------------------------------------------
# 找 OpenOCD
# --------------------------------------------------------------------------
function Find-OpenOcd {
    $onPath = Get-Command openocd -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }

    $roots = @(
        (Join-Path $env:USERPROFILE '.platformio\packages\tool-openocd'),
        (Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\openocd')
    )
    foreach ($r in $roots) {
        if (Test-Path $r) {
            $exe = Get-ChildItem $r -Recurse -Include 'openocd.exe' -File -ErrorAction SilentlyContinue |
                   Select-Object -First 1
            if ($exe) { return $exe.FullName }
        }
    }
    return $null
}

function Find-OpenOcdScripts {
    param([string]$ExePath)
    $dir = Split-Path -Parent $ExePath
    for ($i = 0; $i -lt 4 -and $dir; $i++) {
        $cand = Join-Path $dir 'openocd\scripts'
        if (Test-Path (Join-Path $cand 'target\stm32f4x.cfg')) { return $cand }
        $cand2 = Join-Path $dir 'scripts'
        if (Test-Path (Join-Path $cand2 'target\stm32f4x.cfg')) { return $cand2 }
        $dir = Split-Path -Parent $dir
    }
    return $null
}

$ocd = Find-OpenOcd
if (-not $ocd) {
    Write-Host "没找到 OpenOCD。安装命令：" -ForegroundColor Red
    Write-Host "  python -m platformio pkg install --global --tool platformio/tool-openocd"
    exit 1
}
$ocdScripts = Find-OpenOcdScripts -ExePath $ocd
if (-not $ocdScripts) {
    Write-Host "找到 OpenOCD 但没找到脚本目录（target\stm32f4x.cfg）。" -ForegroundColor Red
    exit 1
}

Write-Host "OpenOCD : $ocd"
Write-Host "脚本目录: $ocdScripts"
Write-Host ""

# OpenOCD 的 Tcl 对反斜杠敏感，路径统一换成正斜杠
$fwForOcd = $Firmware -replace '\\', '/'

# --------------------------------------------------------------------------
# 组装 OpenOCD 参数
# --------------------------------------------------------------------------
$ocdArgs = @(
    '-s', $ocdScripts,
    '-f', 'interface/stlink.cfg',
    '-f', 'target/stm32f4x.cfg',
    '-c', "adapter speed $Speed"
)

if ($Check) {
    $ocdArgs += @('-c', 'init', '-c', 'targets', '-c', 'exit')
    Write-Host "只做连接测试（不烧录）..." -ForegroundColor Cyan
}
else {
    if ($Erase) { $ocdArgs += @('-c', 'init', '-c', 'reset halt', '-c', 'stm32f4x mass_erase 0') }
    $ocdArgs += @('-c', "program $fwForOcd verify reset exit")
    Write-Host "开始烧录..." -ForegroundColor Cyan
}

$eap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$log = & $ocd @ocdArgs 2>&1
$code = $LASTEXITCODE
$ErrorActionPreference = $eap

$log | ForEach-Object { Write-Host "  $_" }
$joined = ($log | Out-String)

# --------------------------------------------------------------------------
# 判断结果
# --------------------------------------------------------------------------
if ($Check) {
    if ($joined -match 'Cortex-M4|stm32f4x|SW-DP|halted') {
        Write-Host ""
        Write-Host "连接成功：ST-Link 已经认到芯片了，可以正常烧录。" -ForegroundColor Green
        exit 0
    }
    Write-Host ""
    Write-Host "连接失败，看下面的排查清单。" -ForegroundColor Red
    $code = 1
}
elseif (($code -eq 0) -and ($joined -match 'Verified OK|verified OK')) {
    Write-Host ""
    Write-Host "烧录成功，芯片已复位运行。" -ForegroundColor Green
    Write-Host "下一步看数据：" -ForegroundColor Cyan
    Write-Host "  python tools\telemetry_monitor.py --list"
    Write-Host "  python tools\telemetry_monitor.py --port COM5 --csv run1.csv"
    Write-Host ""
    Write-Host "提醒：固件上电后等 1 秒会自动开始循迹（LF_START_DELAY_MS=1000），先把车放到线上再上电。" -ForegroundColor DarkGray
    exit 0
}
else {
    Write-Host ""
    Write-Host "烧录失败。按可能性排序排查：" -ForegroundColor Red
}

Write-Host "  1) ST-Link 有没有被电脑认到？设备管理器里看有没有 STLink / STM32 STLink 设备；"
Write-Host "     没有就先装驱动（STM32CubeProgrammer 或 Keil 自带的 ST-Link 驱动）。"
Write-Host "  2) 接线对不对？SWDIO->PA13、SWCLK->PA14、GND->GND、3.3V->3V3。"
Write-Host "     SWDIO/SWCLK 接反不会烧坏，但一定连不上。"
Write-Host "  3) 板子有没有供电？先用 Micro USB 或 5V 给板子上电，ST-Link 的 3.3V 带不动整板。"
Write-Host "  4) 换个 USB 口、换根线重试。"
Write-Host "  5) 别的软件占着调试口：关掉 Keil / STM32CubeIDE / STM32CubeProgrammer，"
Write-Host "     或者上一次没退干净的 openocd（任务管理器里结束它）。"
Write-Host "  6) 克隆版 ST-Link 固件老、线又长：降速重试  -Speed 500"
Write-Host "  7) 芯片被读保护：加 -Erase 全片擦除后重烧。"
Write-Host "  8) 只做连接测试：  powershell -File tools\flash.ps1 -Check"
exit 1
