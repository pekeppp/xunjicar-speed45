<#
    build.ps1  ——  XunjiCar 一键编译脚本（不需要 CMake，也不需要 VSCode 扩展）

    用法（在 xunjicar 目录下执行）：
        powershell -ExecutionPolicy Bypass -File tools\build.ps1
        powershell -ExecutionPolicy Bypass -File tools\build.ps1 -Config Release
        powershell -ExecutionPolicy Bypass -File tools\build.ps1 -Clean

    输出（XunjiCar\XunjiCar\build\out\）：
        XunjiCar.elf   带调试信息，给 pyocd / ST-Link 用
        XunjiCar.hex   给 STM32CubeProgrammer / Keil 用
        XunjiCar.bin   纯二进制，给串口 ISP 用
        XunjiCar.map   链接映射（查 Flash/RAM 占用）

    编译器查找顺序：-ToolchainPath → PATH → PlatformIO 包 → STM32Cube 扩展包
#>

param(
    [ValidateSet('Debug', 'Release')]
    [string]$Config = 'Debug',
    [string]$ToolchainPath = '',
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

# --------------------------------------------------------------------------
# 路径
# --------------------------------------------------------------------------
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projDir   = (Resolve-Path (Join-Path $scriptDir '..\XunjiCar\XunjiCar')).Path
$outDir    = Join-Path $projDir 'build\out'
$objDir    = Join-Path $outDir $Config

if ($projDir -match ' ') {
    throw "工程路径里有空格，本脚本的参数拼接方式不支持：$projDir （把工程挪到没有空格的路径下再试）"
}

Write-Host "工程目录: $projDir"

# --------------------------------------------------------------------------
# 找编译器
# --------------------------------------------------------------------------
function Find-Toolchain {
    param([string]$Explicit)

    if ($Explicit) {
        if (Test-Path (Join-Path $Explicit 'arm-none-eabi-gcc.exe')) { return (Resolve-Path $Explicit).Path }
        throw "指定的工具链目录里没有 arm-none-eabi-gcc.exe: $Explicit"
    }

    $onPath = Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue
    if ($onPath) { return (Split-Path -Parent $onPath.Source) }

    $candidates = @(
        (Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccarmnoneeabi\bin')
    )
    $bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\gnu-tools-for-stm32'
    if (Test-Path $bundleRoot) {
        Get-ChildItem $bundleRoot -Directory -ErrorAction SilentlyContinue | ForEach-Object {
            $candidates += (Join-Path $_.FullName 'bin')
        }
    }
    $candidates += (Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\gnu-tools-for-stm32\13.3.1+st.9\bin')

    foreach ($c in $candidates) {
        if (Test-Path (Join-Path $c 'arm-none-eabi-gcc.exe')) { return (Resolve-Path $c).Path }
    }
    return $null
}

$tc = Find-Toolchain -Explicit $ToolchainPath
if (-not $tc) {
    Write-Host ""
    Write-Host "找不到 arm-none-eabi-gcc。任选一种方式：" -ForegroundColor Red
    Write-Host "  1) python -m platformio pkg install --global --tool platformio/toolchain-gccarmnoneeabi"
    Write-Host "  2) powershell -File tools\build.ps1 -ToolchainPath 'D:\gcc-arm\bin'"
    exit 1
}
$GCC     = Join-Path $tc 'arm-none-eabi-gcc.exe'
$OBJCOPY = Join-Path $tc 'arm-none-eabi-objcopy.exe'
$SIZE    = Join-Path $tc 'arm-none-eabi-size.exe'
Write-Host "编译器  : $GCC"
Write-Host "版本    : $(& $GCC --version | Select-Object -First 1)"
Write-Host ""

# --------------------------------------------------------------------------
# 源文件（与 cmake/stm32cubemx/CMakeLists.txt 一致）
# --------------------------------------------------------------------------
$coreSrc = @(
    'main.c', 'gpio.c', 'i2c.c', 'tim.c', 'usart.c',
    'stm32f4xx_it.c', 'stm32f4xx_hal_msp.c',
    'system_stm32f4xx.c', 'sysmem.c', 'syscalls.c'
) | ForEach-Object { Join-Path $projDir "Core\Src\$_" }

# 自己写的模块（以后新增 .c 文件，记得加到这个列表）
$userSrc = @(
    'track_uart.c', 'vl53l0x.c', 'telemetry.c', 'motor_test.c', 'mpu6050.c',
    'line_follow.c'
) | ForEach-Object { Join-Path $projDir "Core\Src\$_" }

$halSrc = @(
    'stm32f4xx_hal_i2c.c', 'stm32f4xx_hal_i2c_ex.c',
    'stm32f4xx_hal_rcc.c', 'stm32f4xx_hal_rcc_ex.c',
    'stm32f4xx_hal_flash.c', 'stm32f4xx_hal_flash_ex.c', 'stm32f4xx_hal_flash_ramfunc.c',
    'stm32f4xx_hal_gpio.c', 'stm32f4xx_hal_dma.c', 'stm32f4xx_hal_dma_ex.c',
    'stm32f4xx_hal_pwr.c', 'stm32f4xx_hal_pwr_ex.c', 'stm32f4xx_hal_cortex.c',
    'stm32f4xx_hal.c', 'stm32f4xx_hal_exti.c',
    'stm32f4xx_hal_tim.c', 'stm32f4xx_hal_tim_ex.c', 'stm32f4xx_hal_uart.c'
) | ForEach-Object { Join-Path $projDir "Drivers\STM32F4xx_HAL_Driver\Src\$_" }

$asmSrc = Join-Path $projDir 'startup_stm32f407xx.s'
$allSrc = $coreSrc + $userSrc + $halSrc

foreach ($f in ($allSrc + $asmSrc)) {
    if (-not (Test-Path $f)) { throw "源文件不存在: $f" }
}

# --------------------------------------------------------------------------
# 编译参数
# --------------------------------------------------------------------------
$includes = @(
    (Join-Path $projDir 'Core\Inc'),
    (Join-Path $projDir 'Drivers\STM32F4xx_HAL_Driver\Inc'),
    (Join-Path $projDir 'Drivers\STM32F4xx_HAL_Driver\Inc\Legacy'),
    (Join-Path $projDir 'Drivers\CMSIS\Device\ST\STM32F4xx\Include'),
    (Join-Path $projDir 'Drivers\CMSIS\Include')
) | ForEach-Object { "-I$_" }

$targetFlags = @('-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-mthumb')
$defines     = @('-DUSE_HAL_DRIVER', '-DSTM32F407xx')
$warnFlags   = @('-Wall', '-Wextra', '-fdata-sections', '-ffunction-sections')
$optFlags    = if ($Config -eq 'Release') { @('-Os', '-g0', '-DNDEBUG') } else { @('-O0', '-g3', '-DDEBUG') }
$cFlags      = $targetFlags + $defines + $warnFlags + $optFlags + @('-std=gnu11') + $includes
$asmFlags    = $targetFlags + $defines + @('-x', 'assembler-with-cpp') + $includes

$ldScript = Join-Path $projDir 'STM32F407VGTx_FLASH.ld'
$elfFile  = Join-Path $outDir 'XunjiCar.elf'
$mapFile  = Join-Path $outDir 'XunjiCar.map'
$hexFile  = Join-Path $outDir 'XunjiCar.hex'
$binFile  = Join-Path $outDir 'XunjiCar.bin'

# --------------------------------------------------------------------------
# 准备输出目录
# --------------------------------------------------------------------------
if ($Clean -and (Test-Path $outDir)) { Remove-Item $outDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

# --------------------------------------------------------------------------
# 编译
# --------------------------------------------------------------------------
function Compile-One {
    param([string]$Src, [string[]]$Flags, [string]$Tag)

    $objFile = Join-Path $objDir ("{0}_{1}.o" -f $Tag, [IO.Path]::GetFileNameWithoutExtension($Src))
    $gccArgs = $Flags + @('-c', $Src, '-o', $objFile)
    $eap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'   # gcc 的警告走 stderr，别让 PowerShell 把它当成错误中断
    $out = & $GCC @gccArgs 2>&1
    $ErrorActionPreference = $eap
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [失败] $([IO.Path]::GetFileName($Src))" -ForegroundColor Red
        $out | ForEach-Object { Write-Host "        $_" -ForegroundColor DarkRed }
        return $null
    }
    if ($out) { $out | ForEach-Object { Write-Host "  [警告] $_" -ForegroundColor Yellow } }
    return $objFile
}

$objects = @()
$failed  = @()
$total   = $allSrc.Count + 1
$index   = 0

Write-Host "开始编译 $total 个文件（$Config）..." -ForegroundColor Cyan

foreach ($src in $allSrc) {
    $index++
    Write-Progress -Activity '编译中' -Status ([IO.Path]::GetFileName($src)) -PercentComplete (100 * $index / $total)
    $tag = if ($coreSrc -contains $src) { 'core' } elseif ($userSrc -contains $src) { 'user' } else { 'hal' }
    $objFile = Compile-One -Src $src -Flags $cFlags -Tag $tag
    if ($objFile) { $objects += $objFile } else { $failed += $src }
}

$index++
Write-Progress -Activity '编译中' -Status 'startup_stm32f407xx.s' -PercentComplete (100 * $index / $total)
$objFile = Compile-One -Src $asmSrc -Flags $asmFlags -Tag 'asm'
if ($objFile) { $objects += $objFile } else { $failed += $asmSrc }
Write-Progress -Activity '编译中' -Completed

if ($failed.Count -gt 0) {
    Write-Host ""
    Write-Host "编译失败，共 $($failed.Count) 个文件：" -ForegroundColor Red
    $failed | ForEach-Object { Write-Host "  $_" }
    exit 1
}

# --------------------------------------------------------------------------
# 链接
# --------------------------------------------------------------------------
Write-Host ""
Write-Host "链接..." -ForegroundColor Cyan
$ldFlags = $targetFlags + @(
    "-T$ldScript",
    '--specs=nano.specs',
    "-Wl,-Map=$mapFile",
    '-Wl,--gc-sections',
    '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group'
)
$eap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$linkOut = & $GCC @ldFlags @objects -o $elfFile 2>&1
$ErrorActionPreference = $eap
$linkOut | ForEach-Object { Write-Host "  $_" }
if ($LASTEXITCODE -ne 0) {
    Write-Host "链接失败。" -ForegroundColor Red
    exit 1
}

# --------------------------------------------------------------------------
# 生成 hex / bin + 体积
# --------------------------------------------------------------------------
& $OBJCOPY -O ihex   $elfFile $hexFile
& $OBJCOPY -O binary $elfFile $binFile

Write-Host ""
Write-Host "==== 编译成功 ====" -ForegroundColor Green
& $SIZE $elfFile
Write-Host ""
Write-Host "输出："
Write-Host "  $elfFile"
Write-Host "  $hexFile"
Write-Host "  $binFile"
Write-Host ""
Write-Host "下一步：烧录  powershell -ExecutionPolicy Bypass -File tools\flash.ps1" -ForegroundColor Cyan
