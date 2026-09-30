/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴 IMU 驱动 (实现, I2C2)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "mpu6050.h"
#include "i2c.h"

/* Private defines -----------------------------------------------------------*/

/* MPU6050 寄存器 (标准寄存器表) */
#define MPU6050_REG_SMPLRT_DIV      (0x19u)
#define MPU6050_REG_CONFIG          (0x1Au)
#define MPU6050_REG_GYRO_CONFIG     (0x1Bu)
#define MPU6050_REG_ACCEL_CONFIG    (0x1Cu)
#define MPU6050_REG_INT_PIN_CFG     (0x37u)
#define MPU6050_REG_ACCEL_XOUT_H    (0x3Bu)   /* 连续 14 字节: ax ay az temp gx gy gz */
#define MPU6050_REG_PWR_MGMT_1      (0x6Bu)
#define MPU6050_REG_PWR_MGMT_2      (0x6Cu)
#define MPU6050_REG_WHO_AM_I        (0x75u)

/** 一次读取 14 字节: 加速度(6) + 温度(2) + 陀螺仪(6) */
#define MPU6050_BURST_LEN           (14u)

/* Private variables ---------------------------------------------------------*/
static uint8_t      s_ready;                       /**< 1 = 初始化成功 */
static uint8_t      s_who;                         /**< WHO_AM_I 值 */
static float        s_accel_lsb_per_g;             /**< 加速度灵敏度 */
static float        s_gyro_lsb_per_dps;            /**< 陀螺仪灵敏度 */
static MPU6050_Raw_t s_last;                       /**< 最近一次读数 */
static uint32_t     s_last_sample_ms;              /**< 上次采样时刻 */
static uint32_t     s_last_retry_ms;               /**< 上次重试初始化时刻 */
static uint8_t      s_gyro_cal_done;               /**< 零偏是否已校准过(只做一次) */
static int16_t      s_gyro_bias[3];                /**< 陀螺仪零偏 (原始计数) */

/* Private function prototypes -----------------------------------------------*/
static uint8_t  mpu_write_reg(uint8_t reg, uint8_t value);
static uint8_t  mpu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t  mpu_configure(void);
static uint8_t  mpu_read_raw(MPU6050_Raw_t *out);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  探测 + 配置 MPU6050
  */
void MPU6050_Init(void)
{
  s_ready = 0u;
  s_who = 0u;
  s_last_sample_ms = HAL_GetTick();
  s_last_retry_ms = HAL_GetTick();
  s_accel_lsb_per_g = 16384.0f;
  s_gyro_lsb_per_dps = 16.4f;

  /* 先读 WHO_AM_I: 既做在线探测, 也顺便看看是不是兼容芯片 */
  if (mpu_read_regs(MPU6050_REG_WHO_AM_I, &s_who, 1u) == 0u)
  {
    return;   /* 没应答: 不动配置, 等 Task 里重试 */
  }

  if (s_who == 0x00u || s_who == 0xFFu)
  {
    return;   /* 总线被拉死/读回全 0 或全 1, 视为无效 */
  }

  if (mpu_configure() != 0u)
  {
    s_ready = 1u;

    /* 上电时小车应该是静止的, 这里做一次陀螺仪零偏校准.
       注意: 只做一次 —— 运行中若因 I2C 抖动重连, 绝不能再校一次
       (那时车在动, 会把运动当成零偏) */
    if (s_gyro_cal_done == 0u)
    {
      MPU6050_CalibrateGyro();
    }
  }
}

/**
  * @brief  主循环周期调用: 定时读取并打印 (非阻塞)
  */
void MPU6050_Task(void)
{
  uint32_t now = HAL_GetTick();

  /* 没初始化成功时定期重试 (模块可能比单片机晚上电) */
  if (s_ready == 0u)
  {
    if ((now - s_last_retry_ms) >= MPU6050_RETRY_MS)
    {
      s_last_retry_ms = now;
      MPU6050_Init();
    }
    return;
  }

  if ((now - s_last_sample_ms) < MPU6050_SAMPLE_PERIOD_MS)
  {
    return;
  }
  s_last_sample_ms = now;

  {
    MPU6050_Raw_t r;

    if (mpu_read_raw(&r) == 0u)
    {
      /* 读失败: 标记失效并尝试重新初始化 */
      s_ready = 0u;
      s_last_retry_ms = now;
      return;
    }

    s_last = r;
  }
}

/**
  * @brief  初始化是否成功
  */
uint8_t MPU6050_IsReady(void)
{
  return s_ready;
}

/**
  * @brief  WHO_AM_I 读到的值
  */
uint8_t MPU6050_GetWhoAmI(void)
{
  return s_who;
}

/**
  * @brief  取最近一次读取的原始值
  * @retval 1 = 数据有效
  */
uint8_t MPU6050_GetRaw(MPU6050_Raw_t *out)
{
  if ((out == NULL) || (s_ready == 0u))
  {
    return 0u;
  }

  *out = s_last;
  return 1u;
}

/**
  * @brief  陀螺仪零偏校准 (要求小车静止!)
  *         做 100 次采样取平均, 存进 s_gyro_bias, 之后读数都会扣掉它
  */
void MPU6050_CalibrateGyro(void)
{
  int32_t sx = 0;
  int32_t sy = 0;
  int32_t sz = 0;
  uint16_t i;
  MPU6050_Raw_t r;

  s_gyro_bias[0] = 0;
  s_gyro_bias[1] = 0;
  s_gyro_bias[2] = 0;
  s_gyro_cal_done = 0u;

  if (s_ready == 0u)
  {
    return;
  }

  for (i = 0u; i < MPU6050_CALIB_SAMPLES; i++)
  {
    if (mpu_read_raw(&r) == 0u)
    {
      return;    /* 读失败: 保持零偏为 0, 下次初始化还会再试 */
    }
    sx += (int32_t)r.gx;
    sy += (int32_t)r.gy;
    sz += (int32_t)r.gz;
    HAL_Delay(2u);
  }

  s_gyro_bias[0] = (int16_t)(sx / (int32_t)MPU6050_CALIB_SAMPLES);
  s_gyro_bias[1] = (int16_t)(sy / (int32_t)MPU6050_CALIB_SAMPLES);
  s_gyro_bias[2] = (int16_t)(sz / (int32_t)MPU6050_CALIB_SAMPLES);
  s_gyro_cal_done = 1u;
}

uint8_t MPU6050_IsGyroCalibrated(void)
{
  return s_gyro_cal_done;
}

/**
  * @brief  取扣掉零偏后的角速度原始计数
  */
uint8_t MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
  if ((s_ready == 0u) || (s_gyro_cal_done == 0u))
  {
    return 0u;
  }

  if (gx != NULL) { *gx = (int16_t)(s_last.gx - s_gyro_bias[0]); }
  if (gy != NULL) { *gy = (int16_t)(s_last.gy - s_gyro_bias[1]); }
  if (gz != NULL) { *gz = (int16_t)(s_last.gz - s_gyro_bias[2]); }

  return 1u;
}

/**
  * @brief  取 Z 轴角速度 (dps)
  */
uint8_t MPU6050_GetGyroZDps(float *dps)
{
  int16_t gz;

  if ((dps == NULL) || (MPU6050_GetGyro(NULL, NULL, &gz) == 0u))
  {
    return 0u;
  }

  *dps = (float)gz / s_gyro_lsb_per_dps;
  return 1u;
}

/**
  * @brief  原始值 -> 加速度 (g)
  */
float MPU6050_AccelToG(int16_t raw)
{
  return (float)raw / s_accel_lsb_per_g;
}

/**
  * @brief  原始值 -> 角速度 (dps)
  */
float MPU6050_GyroToDps(int16_t raw)
{
  return (float)raw / s_gyro_lsb_per_dps;
}

/**
  * @brief  原始值 -> 温度 (摄氏度): T = raw/340 + 36.53
  */
float MPU6050_TempToC(int16_t raw)
{
  return ((float)raw / 340.0f) + 36.53f;
}

/* Private functions ---------------------------------------------------------*/

/** 写一个寄存器 */
static uint8_t mpu_write_reg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(MPU6050_ADDR7 << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, &value, 1u,
                                           MPU6050_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 连续读若干个寄存器 */
static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(MPU6050_ADDR7 << 1), reg,
                                          I2C_MEMADD_SIZE_8BIT, buf, len,
                                          MPU6050_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/**
  * @brief  上电配置: 复位 -> 唤醒 -> 量程/滤波/采样率
  * @retval 1 = 成功
  */
static uint8_t mpu_configure(void)
{
  uint8_t check = 0u;

  /* 1. 软复位, 等内部稳定 */
  if (mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80u) == 0u)
  {
    return 0u;
  }
  HAL_Delay(100u);

  /* 2. 唤醒并选择时钟源: PLL with X axis gyroscope reference (比内部 RC 稳) */
  if (mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u) == 0u)
  {
    return 0u;
  }
  HAL_Delay(10u);

  /* 3. 六轴全部开启 */
  if (mpu_write_reg(MPU6050_REG_PWR_MGMT_2, 0x00u) == 0u)
  {
    return 0u;
  }

  /* 4. 数字低通滤波 + 采样率 */
  if (mpu_write_reg(MPU6050_REG_CONFIG, (uint8_t)MPU6050_DLPF_CFG) == 0u)
  {
    return 0u;
  }
  if (mpu_write_reg(MPU6050_REG_SMPLRT_DIV, (uint8_t)MPU6050_SMPLRT_DIV) == 0u)
  {
    return 0u;
  }

  /* 5. 量程 */
  if (mpu_write_reg(MPU6050_REG_GYRO_CONFIG, (uint8_t)(MPU6050_GYRO_FS_SEL << 3)) == 0u)
  {
    return 0u;
  }
  if (mpu_write_reg(MPU6050_REG_ACCEL_CONFIG, (uint8_t)(MPU6050_ACCEL_FS_SEL << 3)) == 0u)
  {
    return 0u;
  }

  /* 6. 打开 I2C 旁路 (以后接磁力计时可直接访问), 对六轴无影响 */
  (void)mpu_write_reg(MPU6050_REG_INT_PIN_CFG, 0x02u);

  /* 7. 回读校验: PWR_MGMT_1 应等于 0x01 */
  if (mpu_read_regs(MPU6050_REG_PWR_MGMT_1, &check, 1u) == 0u)
  {
    return 0u;
  }
  if ((check & 0x3Fu) != 0x01u)
  {
    return 0u;
  }

  /* 8. 按量程设置换算系数 */
  switch (MPU6050_ACCEL_FS_SEL)
  {
    case 1u:  s_accel_lsb_per_g = 8192.0f;  break;
    case 2u:  s_accel_lsb_per_g = 4096.0f;  break;
    case 3u:  s_accel_lsb_per_g = 2048.0f;  break;
    default:  s_accel_lsb_per_g = 16384.0f; break;
  }

  switch (MPU6050_GYRO_FS_SEL)
  {
    case 0u:  s_gyro_lsb_per_dps = 131.0f;  break;
    case 1u:  s_gyro_lsb_per_dps = 65.5f;   break;
    case 2u:  s_gyro_lsb_per_dps = 32.8f;   break;
    default:  s_gyro_lsb_per_dps = 16.4f;   break;
  }

  return 1u;
}

/**
  * @brief  一次突发读取 14 字节并拆成 7 个 int16
  */
static uint8_t mpu_read_raw(MPU6050_Raw_t *out)
{
  uint8_t buf[MPU6050_BURST_LEN];

  if (mpu_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_BURST_LEN) == 0u)
  {
    return 0u;
  }

  out->ax   = (int16_t)(((uint16_t)buf[0]  << 8) | (uint16_t)buf[1]);
  out->ay   = (int16_t)(((uint16_t)buf[2]  << 8) | (uint16_t)buf[3]);
  out->az   = (int16_t)(((uint16_t)buf[4]  << 8) | (uint16_t)buf[5]);
  out->temp = (int16_t)(((uint16_t)buf[6]  << 8) | (uint16_t)buf[7]);
  out->gx   = (int16_t)(((uint16_t)buf[8]  << 8) | (uint16_t)buf[9]);
  out->gy   = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
  out->gz   = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

  return 1u;
}
