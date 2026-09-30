/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴 IMU 驱动 (I2C2: PB10=SCL / PB11=SDA)
  *
  * 说明: 工作区里厂家那份 MPU6050 代码只有编译产物、没有源码, 且是 MSPM0 平台 +
  *       InvenSense DMP 方案。这里按 MPU6050 标准寄存器表写一个寄存器级驱动:
  *       直接读加速度/角速度/温度原始值, 足够做巡线小车的航向修正用。
  *       (以后要 DMP 的四元数输出再说, 那是另一套移植工作)
  *
  * 接线:
  *   VCC -> 3.3V (GY-521 这类带稳压的模块可接 5V, 按模块标注)
  *   GND -> 共地
  *   SCL -> PB10 (I2C2_SCL)
  *   SDA -> PB11 (I2C2_SDA)
  *   AD0 -> 悬空/接地 = 地址 0x68;  接 3.3V = 0x69 (见 MPU6050_ADDR7)
  *
  * 数据去向: 由 telemetry 模块统一组装进 $CAR 遥测帧 (本模块不再自己打印),
  *           对应帧内字段 AX/AY/AZ/GX/GY/GZ/T; 取不到时帧里给 IMU=NA
  *
  * 判断标准 (±2g 量程, 静置水平): ax/ay ≈ 0, az ≈ ±16384, gx/gy/gz ≈ 0, T ≈ 室温
  *   who 为 0x70/0x71 等 -> 兼容芯片(MPU6500/9250), 本驱动的加速度/陀螺仪部分照样能用
  *   完全没应答 -> 帧里 IMU=NA (检查接线/供电/AD0)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 器件参数 ------------------------------------------------------------------*/

/** 7 位从机地址: AD0 接地 = 0x68, 接 3.3V = 0x69 */
#define MPU6050_ADDR7            (0x68u)

/** WHO_AM_I (0x75) 的常见取值 */
#define MPU6050_WHOAMI_MPU6050   (0x68u)
#define MPU6050_WHOAMI_MPU6500   (0x70u)
#define MPU6050_WHOAMI_MPU9250   (0x71u)

/** 加速度量程: 0=±2g  1=±4g  2=±8g  3=±16g */
#define MPU6050_ACCEL_FS_SEL     (0u)

/** 陀螺仪量程: 0=±250  1=±500  2=±1000  3=±2000 dps */
#define MPU6050_GYRO_FS_SEL      (3u)

/** 数字低通滤波: 0=260/256Hz  1=184/188  2=94/98  3=44/42  4=21/20  5=10/10  6=5/5 */
#define MPU6050_DLPF_CFG         (3u)

/** 采样率分频: 采样率 = 1000/(1+div) Hz, 4 -> 200 Hz */
#define MPU6050_SMPLRT_DIV       (4u)

/** 单次 I2C 读写超时 (ms) */
#define MPU6050_I2C_TIMEOUT_MS   (20u)

/** 采样周期 (ms): 循迹控制环要用陀螺仪做阻尼, 所以采到 10 ms (100 Hz).
  * (I2C2 是 100 kHz, 一次 14 字节突发读约 2 ms, 100 Hz 下总线占用约 20%) */
#define MPU6050_SAMPLE_PERIOD_MS (10u)

/** 初始化失败后重试的间隔 (ms) */
#define MPU6050_RETRY_MS         (1000u)

/** 陀螺仪零偏校准采样次数 (取平均作为零偏).
  * 100 次 × (2 ms 读取 + 2 ms 间隔) ≈ 400 ms, 只在第一次初始化成功时做一次 */
#define MPU6050_CALIB_SAMPLES    (100u)

/* 数据结构 ------------------------------------------------------------------*/

/** 一次读取的原始计数 */
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} MPU6050_Raw_t;

/* 对外接口 ------------------------------------------------------------------*/

/** 探测 + 配置 MPU6050 (I2C2), 失败不影响其它模块, 会定时自动重试 */
void MPU6050_Init(void);

/** 主循环周期调用: 定时读取并打印一行 IMU ... (非阻塞) */
void MPU6050_Task(void);

/** 初始化是否成功 */
uint8_t MPU6050_IsReady(void);

/** WHO_AM_I 读到的值 (没读到为 0) */
uint8_t MPU6050_GetWhoAmI(void);

/** 取最近一次读取的原始值
  * @retval 1 = 数据有效 */
uint8_t MPU6050_GetRaw(MPU6050_Raw_t *out);

/** 取"扣掉零偏"后的三轴角速度原始计数 (给控制环用)
  * @param  gx/gy/gz 可为 NULL
  * @retval 1 = 数据有效(已初始化且校准过), 0 = 拿不到 */
uint8_t MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);

/** 取 Z 轴角速度 (dps, 扣零偏后的浮点值; 只在遥测/调试里用, 控制环用 MPU6050_GetGyro)
  * @retval 1 = 数据有效 */
uint8_t MPU6050_GetGyroZDps(float *dps);

/** 陀螺仪零偏是否已校准 (上电静止时自动校准一次) */
uint8_t MPU6050_IsGyroCalibrated(void);

/** 手动重新校准零偏 (要求小车静止不动!). 一般不用调, 上电会自动做一次 */
void MPU6050_CalibrateGyro(void);

/** 原始值 -> 物理量 (供以后控制环用) */
float MPU6050_AccelToG(int16_t raw);
float MPU6050_GyroToDps(int16_t raw);
float MPU6050_TempToC(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
