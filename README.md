# XunjiCar —— STM32F407 八路灰度巡线小车

基于 **STM32F407** 的循迹小车固件。八路灰度循迹模块（串口版）做主线识别，双路 **VL53L0X** 激光测距做前方避障，**MPU6050** 提供角速度阻尼，**TB6612FNG** 驱动两个 **MG513 GMR 编码器减速电机**，并通过 `$CAR` 文本遥测帧把全部状态实时打到上位机。

> 📖 详细的接线、调试与调参请直接看 **[`接线与调试手册.md`](接线与调试手册.md)**（753 行，含引脚总表、逐脚电平核对、故障排查表、调参顺序）。
> 烧录工具链的原理与路径解析见 **[`烧录工具与路径说明.md`](烧录工具与路径说明.md)**。

---

## 1. 硬件清单

| # | 部件 | 型号 / 备注 |
|---|---|---|
| 1 | 主控板 | STM32F407VGT6 核心板（LQFP100，82×92 mm） |
| 2 | 电机驱动 | TB6612FNG（代码按 AIN1/AIN2/BIN1/BIN2/PWMA/PWMB/STBY 命名） |
| 3 | 电机 ×2 | MG513 GMR 编码器减速电机（6 芯：M+/M- + VCC/GND/A/B） |
| 4 | 循迹模块 | **8 路红外循迹（串口版）** —— I2C 版或并口版需改代码 |
| 5 | 姿态 | MPU6050（GY-521） |
| 6 | 测距 ×2 | **VL53L0X 激光测距**（VL53L1X 寄存器不兼容，不能直接替换） |
| 7 | 上位机串口 | USB-TTL（CH340 / CP2102 / FT232 均可），**必备** |
| 8 | 烧录器 | ST-Link V2（SWD） |
| 9 | 电源 | 2S 锂电 7.4V 或 6×AA 9V 给电机；5V 降压模块 ≥2A 给逻辑 |

**供电要点**：电机电流走 `TB6612 VM` + 电池，**绝不走开发板**（堵转可达 1~2 A）；逻辑部分约 200~300 mA，板载 AMS1117 够用；所有模块 **GND 必须共地**。

---

## 2. 引脚分配（全部插在开发板长排针上）

| 功能 | 引脚 | 外设 |
|---|---|---|
| 左电机调速 PWMA | **PC6** | TIM8_CH1（20 kHz） |
| 右电机调速 PWMB | **PC7** | TIM8_CH2 |
| 左电机方向 AIN1 / AIN2 | **PD0 / PD1** | GPIO |
| 右电机方向 BIN1 / BIN2 | **PD2 / PD3** | GPIO |
| 左编码器 A / B 相 | **PA6 / PA7** | TIM3_CH1 / CH2 |
| 右编码器 A / B 相 | **PD12 / PD13** | TIM4_CH1 / CH2 |
| 循迹模块 TX → 单片机 | **PA10** | USART1_RX（交叉接） |
| 循迹模块 RX ← 单片机 | **PA9** | USART1_TX |
| 上位机遥测 TX | **PA2** | USART2_TX → USB-TTL RXD |
| 上位机遥测 RX | **PA3** | USART2_RX → USB-TTL TXD |
| **左**激光 TOF（帧里 `D1`）SCL / SDA | **PB6 / PB7** | I2C1 |
| **右**激光 TOF（帧里 `D2`）SCL / SDA | **PA8 / PC9** | I2C3 |
| MPU6050 SCL / SDA | **PB10 / PB11** | I2C2 |
| SWD 烧录 | **PA13 (SWDIO) / PA14 (SWCLK)** | 留出插 ST-Link 的空间 |

> 两个激光模块的左右是**装车实测确认**的：`D1`=左边、`D2`=右边（定义在 `line_follow.h` 的
> `LF_TOF_LEFT` / `LF_TOF_RIGHT`，实测方法与改法见手册 15.1）。

> `PA2/PA3/PA6~PA10/PB6/PB7/PC6/PC7/PD0~PD3/PD12/PD13` 全部为 **FT（5V 容忍）** 引脚，已逐脚核对。
> ST-Link 的 **BT0 什么都不要接**；USB-TTL 的 VCC **不要接**（避免双电源打架）。

---

## 3. 快速上手（4 个双击脚本）

在仓库根目录依次双击：

| 脚本 | 作用 |
|---|---|
| `1-编译固件.cmd` | 调用 `tools/build.ps1` 编译，输出 `XunjiCar.elf/.hex/.bin/.map` |
| `2-检查STLink连接.cmd` | 只连芯片不烧录（`flash.ps1 -Check`），安全自检 |
| `3-烧录固件.cmd` | 烧录到 Flash，烧完自动复位 |
| `4-看数据并记录.cmd` | 列串口 → 打开 `tools/telemetry_monitor.py` 实时看 `$CAR` 帧 |
| `5-轮子慢转测试.cmd` | **调试用**：编译并烧录 `MotorSpin.hex`。绕过 循迹/避障/丢线 全部逻辑，上电 3 秒后两个轮子一直慢转（占空比 25）。用来判决"车不动"到底是硬件还是代码。**烧前必须把轮子架空！** |

也可以用 PowerShell 直接调用：

```powershell
powershell -ExecutionPolicy Bypass -File tools\build.ps1              # 默认 Debug
powershell -ExecutionPolicy Bypass -File tools\build.ps1 -Config Release
powershell -ExecutionPolicy Bypass -File tools\build.ps1 -Clean
powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Check       # 只连接
powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Erase       # 全片擦除后烧
powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Speed 500   # 克隆版 ST-Link 不稳时降速
powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1         # 编译"轮子慢转"调试固件 → MotorSpin.hex
powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty 35        # 转快一点
powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty -25       # 反转
powershell -ExecutionPolicy Bypass -File tools\build_spin.ps1 -Duty 20 -DelayMs 5000
```

> **"车不动"怎么判**：先跑 `5-轮子慢转测试.cmd`（记得架空车轮）。
> **轮子转了** → 电机 / TB6612 / 供电 / PWM 这条链路全是好的，问题在循迹模块或激光测距；
> **轮子不转** → 是硬件问题，按顺序查：① TB6612 的 **STBY 有没有接 3V3**（低电平=电机全部不转，最关键）
> ② VM 有没有接电池且开关打开 ③ 板子自己有没有独立供电。
> （注：`LF_ENABLE_TOF_LASER` **默认关**，所以"循迹时车不动"基本与激光测距无关，先查循迹模块和丢线判断。）
>
> 调试开关在 `main.c` 的 `/* USER CODE BEGIN PD */` 里：`MOTOR_SPIN_TEST`（1=慢转调试，0=正常循迹，**默认 0**）、
> `MOTOR_SPIN_DUTY`（转速 %，负数=反转）、`MOTOR_SPIN_DELAY_MS`（上电后等多久起转）。
> `tools/build_spin.ps1` 会临时把它改成 1 编译、导出 `MotorSpin.hex`、再把 `main.c` **逐字节还原**并重编译正常固件，
> 所以 `build\out\` 和 `3-烧录固件.cmd` 永远不会被调试固件污染。

> ⚠️ **烧完立刻自启**：`LF_AUTO_START = 1`，复位后约 1 秒车就起步循迹。第一次烧录请把**轮子架空**，或把车摆正在黑线上。
> ⚠️ 工程路径**不能含空格**（`build.ps1` 的参数拼接方式限制）。

### 构建前置条件

- **arm-none-eabi-gcc**：按 `PATH` → PlatformIO 包（`~/.platformio/packages/toolchain-gccarmnoneeabi`）→ STM32Cube 扩展包（`%LOCALAPPDATA%\stm32cube\bundles\gnu-tools-for-stm32`）顺序自动查找；也可 `-ToolchainPath 'D:\gcc-arm\bin'` 手动指定。
- **OpenOCD**（烧录用）：按 `PATH` → `~/.platformio/packages/tool-openocd` → `%LOCALAPPDATA%\stm32cube\bundles\openocd` 顺序查找。
- **Python**（仅 `4-看数据并记录.cmd` 需要）：`pip install pyserial`。

编译脚本不依赖 CMake，也**不需要任何 VS Code 扩展**。仓库同时保留了 CubeMX 生成的 `CMakeLists.txt` / `CMakePresets.json`，所以 CubeIDE / CLion 也能直接打开。

---

## 4. 遥测帧格式

单片机通过 USART2（115200）持续输出一行文本，`*XX` 为校验：

```
$CAR,N=..,TRK=..,B=..,D1=..,D2=..,AX=..,..,T=..,S=..,MA=..,EA=..,MB=..,EB=..*XX
```

> **`D1` / `D2` 默认是不出现的**：激光测距总开关 `LF_ENABLE_TOF_LASER` 默认 `(0)`（关），
> 关着时 VL53L0X 不初始化、帧里也就没有这两个字段。要恢复激光测距见第 5 节的表。

| 字段 | 含义 |
|---|---|
| `N` | 帧序号 |
| `TRK` / `B` | 循迹模块原始状态 / 计算出的偏差；`TRK=ERR` 表示模块无数据 |
| `D1` / `D2` | 两路 VL53L0X 距离（mm），`---` 表示无有效回波；**仅 `LF_ENABLE_TOF_LASER=1` 时才有**。已实测确认 **`D1`=左边模块、`D2`=右边模块** |
| `A*` | MPU6050 姿态与角速度（`IMS=0` 表示陀螺仪未校准/未接上） |
| `T` / `S` | 目标速度 / 实际速度 |
| `MA` / `EA` | 左电机占空比 / 编码器计数 |
| `MB` / `EB` | 右电机占空比 / 编码器计数 |

---

## 5. 调参入口

**调车主要改 `XunjiCar/XunjiCar/Core/Inc/line_follow.h`**（改完必须重新编译）：

| 宏 | 当前值 | 作用 / 何时改 |
|---|---|---|
| `LF_BASE_DUTY` | 45 | 直道基础速度 %（**只影响直线和缓弯**）；车太慢/太快。**改它要同比改 `LF_KD`/`LF_KGYRO`**（手册 14.7） |
| `LF_KP` | 45 | 比例修正力度；过弯冲出去 → 加大，直道画龙 → 减小 |
| `LF_KD` | 14 | 微分压摆动；摆得厉害 → 加大，发抖 → 减小 |
| `LF_KGYRO` | 20 | 陀螺仪阻尼力度（0.01 %/dps）；小幅摆动 → 加大（16~26），车发木 → 减小。**需 `IMS=1` 才生效**（手册 16.5）。偏差 ≤350 时全量出力，只有急弯才降到 55% 让路（14.6） |
| `LF_CORNER_SPEED_DROP` | 12 | **急弯自动降速**：偏差 >400 时把基础速度最多压下 12%（45 → 33），直道速度不变。急弯还勉强 → 加到 15；不想让急弯减速 → 改 0（手册 14.8） |
| `LF_KI` | 0 | 积分，一般保持 0 |
| `LF_STEER_LIMIT` | 70 | 差速上限 %（同时决定内轮能倒转多少）；快 90° 急弯转不过来 → 加大（70~80），直道一顿一顿 → 减小 |
| `LF_LOST_ACTION` | 0 | 丢线行为：0=停车 1=原地找线 2=陀螺仪保持航向 |
| `LF_USE_IMU` | 1 | 是否用陀螺仪做阻尼（未接 MPU 会自动失效） |
| `LF_AUTO_START` | 1 | 上电自动起步；想用按钮启动改 0 |
| `LF_ENABLE_TOF_LASER` | **0** | **激光测距 / 前方避障总开关**：0=关（不初始化 VL53L0X、帧里无 `D1/D2`、控制环不避障）；1=开，下方三个避障参数才生效 |
| `LF_STOP_DISTANCE_MM` | 150 | 前方小于它就停车（mm）；仅 `LF_ENABLE_TOF_LASER=1` 时生效 |
| `LF_RESUME_DISTANCE_MM` | 220 | 障碍离开到这么远才恢复（mm），迟滞防抖 |

其它标定宏（`LF_BLACK_IS_ZERO` 探头极性、`MOTOR_ENC_PPR` 编码器线数、`TRACK_DATA_WHITE_IS_ONE`、`MPU6050_ADDR7`、`VL53L0X_MEASUREMENT_BUDGET_US` 等）见接线手册的「改哪个文件」表。

**推荐调参顺序**：`BASE_DUTY` → `KP` → `KD` → `STEER_LIMIT` → `LOST_TIMEOUT_MS` → `LOST_ACTION` → 避障距离。

---

## 6. 目录结构

```
xunjicar/
├─ 1-编译固件.cmd / 2-检查STLink连接.cmd / 3-烧录固件.cmd / 4-看数据并记录.cmd
├─ 接线与调试手册.md            ← 接线总表 + 电平核对 + 故障排查 + 调参
├─ 烧录工具与路径说明.md        ← OpenOCD 链路原理与路径查找
├─ docs_引脚位置_*.png          ← LQFP100 引脚定位图
├─ STM32F407VET6数据手册.pdf
├─ STM32F407VX_M核心板-原理图-201811M.pdf
├─ tools/
│  ├─ build.ps1                ← 一键编译（gcc + objcopy + size）
│  ├─ flash.ps1                ← 一键烧录 / 仅连接检查（OpenOCD + ST-Link）
│  └─ telemetry_monitor.py     ← $CAR 遥测帧实时解析与记录
└─ XunjiCar/XunjiCar/          ← CubeMX 工程
   ├─ Core/Inc, Core/Src       ← 自研模块 + CubeMX 生成的初始化代码
   ├─ Drivers/                 ← ST 官方 HAL / CMSIS（不要改）
   ├─ cmake/, CMakeLists.txt, CMakePresets.json
   ├─ XunjiCar.ioc             ← CubeMX 配置（另附 5 个 .ioc.*.bak 演进备份）
   └─ STM32F407VGTx_FLASH.ld, startup_stm32f407xx.s
```

### 自研模块（`Core/Src`）

| 文件 | 职责 |
|---|---|
| `line_follow.c` | 循迹控制环本体（PID + 差速 + 丢线处理 + 避障 + 陀螺仪阻尼） |
| `track_uart.c` | 八路循迹模块的串口协议解析 |
| `vl53l0x.c` | VL53L0X 激光测距驱动 |
| `mpu6050.c` | MPU6050 六轴驱动与上电校准 |
| `telemetry.c` | `$CAR` 文本帧组装与输出 |
| `motor_test.c` | 电机/编码器单测序列（`MOTOR_TEST_AUTO_RUN` 默认 0） |

---

## 7. 常见故障速查

| 症状 | 最可能原因 | 处理 |
|---|---|---|
| 压黑线显示 `·`、白底显示 `█` | 探头极性反了 | `line_follow.h` 的 `LF_BLACK_IS_ZERO` 改 0 |
| `TRK=ERR` 一直无数据 | 循迹模块没通 / 接反 | 查 5V、GND，TX→PA10 交叉接 |
| 压着线完全不动 | `LF_AUTO_START=0` 或还在启动延时 | 等 1 秒，或检查 `B=` 是否有效 |
| 直道画龙（来回摆） | KP 过大 / KD 过小 / 速度过快 | 减 KP、加 KD、降 `BASE_DUTY` |
| 过弯直接冲出去 | KP 过小 / 速度过快 / 差速上限太小 | 加 KP、降速、加 `LF_STEER_LIMIT` |
| 原地打转 / 往一边跑 | 电机线接反或左右轮装反 | 对调两根电机线 |
| 左转时角速度是**负数** | MPU6050 装反（倒扣） | `LF_GYRO_SIGN` 改 `-1` |
| `Error: init mode failed` | 芯片没供电或 SWD 接错 | 查 SWDIO→PA13、SWCLK→PA14；`-Speed 500` 降速 |
| 烧录提示 target not halted | 调试口被占用 | 关掉 Keil / CubeIDE / CubeProgrammer / 残留 openocd |

完整故障表见 `接线与调试手册.md` 第 11 节。

---

## 8. 说明

- `.ioc.before-*.bak` 是 CubeMX 配置在几次关键改动（加电机、改 PB4/PB5、改 USART1）前的备份，保留下来便于回溯配置演进，可按需删除。
- `build/`（约 73 MB）已在 `.gitignore` 中排除，`1-编译固件.cmd` 可随时重新生成。
