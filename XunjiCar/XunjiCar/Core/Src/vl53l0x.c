/**
  ******************************************************************************
  * @file    vl53l0x.c
  * @brief   VL53L0X-V2 TOF 激光测距模块驱动 (C 语言移植版)
  *
  * 本文件是 VL53L0X-V2 TOF激光红外测距\arduino\vl53l0x\VL53L0X.cpp 的逐段移植,
  * 初始化时序(DataInit / StaticInit / PerformRefCalibration)与调参表均保持一致,
  * 只把 Arduino 的 Wire/millis 换成 STM32 HAL, 并把类改成 C 结构体。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "vl53l0x.h"

/* Private defines -----------------------------------------------------------*/

/** VCSEL 周期解码/编码: 寄存器值 <-> PCLK 数 */
#define VL53L0X_DECODE_VCSEL_PERIOD(reg_val)      (uint8_t)(((reg_val) + 1u) << 1)
#define VL53L0X_ENCODE_VCSEL_PERIOD(period_pclks) (uint8_t)(((period_pclks) >> 1) - 1u)

/* Private types -------------------------------------------------------------*/

/** 初始化用的"调参表"条目 (DefaultTuningSettings from vl53l0x_tuning.h) */
typedef struct
{
  uint8_t reg;
  uint8_t value;
} VL53L0X_RegValue_t;

/** 序列步骤使能标志 */
typedef struct
{
  uint8_t tcc;
  uint8_t msrc;
  uint8_t dss;
  uint8_t pre_range;
  uint8_t final_range;
} VL53L0X_SequenceStepEnables_t;

/** 序列步骤超时 */
typedef struct
{
  uint16_t pre_range_vcsel_period_pclks;
  uint16_t final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks;
  uint16_t pre_range_mclks;
  uint16_t final_range_mclks;

  uint32_t msrc_dss_tcc_us;
  uint32_t pre_range_us;
  uint32_t final_range_us;
} VL53L0X_SequenceStepTimeouts_t;

/* Private variables ---------------------------------------------------------*/

/** VL53L0X 出厂调参表, 顺序不能改 (对应例程 init() 里的 load_tuning_settings 段) */
static const VL53L0X_RegValue_t s_tuning_settings[] =
{
  {0xFFu, 0x01u}, {0x00u, 0x00u}, {0xFFu, 0x00u}, {0x09u, 0x00u}, {0x10u, 0x00u},
  {0x11u, 0x00u}, {0x24u, 0x01u}, {0x25u, 0xFFu}, {0x75u, 0x00u}, {0xFFu, 0x01u},
  {0x4Eu, 0x2Cu}, {0x48u, 0x00u}, {0x30u, 0x20u}, {0xFFu, 0x00u}, {0x30u, 0x09u},
  {0x54u, 0x00u}, {0x31u, 0x04u}, {0x32u, 0x03u}, {0x40u, 0x83u}, {0x46u, 0x25u},
  {0x60u, 0x00u}, {0x27u, 0x00u}, {0x50u, 0x06u}, {0x51u, 0x00u}, {0x52u, 0x96u},
  {0x56u, 0x08u}, {0x57u, 0x30u}, {0x61u, 0x00u}, {0x62u, 0x00u}, {0x64u, 0x00u},
  {0x65u, 0x00u}, {0x66u, 0xA0u}, {0xFFu, 0x01u}, {0x22u, 0x32u}, {0x47u, 0x14u},
  {0x49u, 0xFFu}, {0x4Au, 0x00u}, {0xFFu, 0x00u}, {0x7Au, 0x0Au}, {0x7Bu, 0x00u},
  {0x78u, 0x21u}, {0xFFu, 0x01u}, {0x23u, 0x34u}, {0x42u, 0x00u}, {0x44u, 0xFFu},
  {0x45u, 0x26u}, {0x46u, 0x05u}, {0x40u, 0x40u}, {0x0Eu, 0x06u}, {0x20u, 0x1Au},
  {0x43u, 0x40u}, {0xFFu, 0x00u}, {0x34u, 0x03u}, {0x35u, 0x44u}, {0xFFu, 0x01u},
  {0x31u, 0x04u}, {0x4Bu, 0x09u}, {0x4Cu, 0x05u}, {0x4Du, 0x04u}, {0xFFu, 0x00u},
  {0x44u, 0x00u}, {0x45u, 0x20u}, {0x47u, 0x08u}, {0x48u, 0x28u}, {0x67u, 0x00u},
  {0x70u, 0x04u}, {0x71u, 0x01u}, {0x72u, 0xFEu}, {0x76u, 0x00u}, {0x77u, 0x00u},
  {0xFFu, 0x01u}, {0x0Du, 0x01u}, {0xFFu, 0x00u}, {0x80u, 0x01u}, {0x01u, 0xF8u},
  {0xFFu, 0x01u}, {0x8Eu, 0x01u}, {0x00u, 0x01u}, {0xFFu, 0x00u}, {0x80u, 0x00u}
};

/* Private function prototypes -----------------------------------------------*/
static uint8_t  vl53l0x_write_reg(VL53L0X_t *dev, uint8_t reg, uint8_t value);
static uint8_t  vl53l0x_write_reg16(VL53L0X_t *dev, uint8_t reg, uint16_t value);
static uint8_t  vl53l0x_write_reg32(VL53L0X_t *dev, uint8_t reg, uint32_t value);
static uint8_t  vl53l0x_read_reg(VL53L0X_t *dev, uint8_t reg);
static uint16_t vl53l0x_read_reg16(VL53L0X_t *dev, uint8_t reg);
static uint8_t  vl53l0x_write_multi(VL53L0X_t *dev, uint8_t reg, const uint8_t *src, uint8_t count);
static uint8_t  vl53l0x_read_multi(VL53L0X_t *dev, uint8_t reg, uint8_t *dst, uint8_t count);

static void     vl53l0x_start_timeout(VL53L0X_t *dev);
static uint8_t  vl53l0x_timeout_expired(const VL53L0X_t *dev);

static uint8_t  vl53l0x_get_spad_info(VL53L0X_t *dev, uint8_t *count, uint8_t *type_is_aperture);
static void     vl53l0x_get_sequence_step_enables(VL53L0X_t *dev, VL53L0X_SequenceStepEnables_t *enables);
static void     vl53l0x_get_sequence_step_timeouts(VL53L0X_t *dev,
                                                   const VL53L0X_SequenceStepEnables_t *enables,
                                                   VL53L0X_SequenceStepTimeouts_t *timeouts);
static uint8_t  vl53l0x_perform_single_ref_calibration(VL53L0X_t *dev, uint8_t vhv_init_byte);
static uint8_t  vl53l0x_get_vcsel_pulse_period(VL53L0X_t *dev, VL53L0X_VcselPeriodType type);

static uint16_t vl53l0x_decode_timeout(uint16_t reg_val);
static uint16_t vl53l0x_encode_timeout(uint16_t timeout_mclks);
static uint32_t vl53l0x_calc_macro_period(uint8_t vcsel_period_pclks);
static uint32_t vl53l0x_timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t vl53l0x_timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  绑定 I2C 总线并复位为默认参数
  */
void VL53L0X_InitDevice(VL53L0X_t *dev, I2C_HandleTypeDef *hi2c)
{
  if (dev == NULL)
  {
    return;
  }

  dev->hi2c                        = hi2c;
  dev->address                     = VL53L0X_ADDRESS_DEFAULT;
  dev->io_timeout                  = VL53L0X_IO_TIMEOUT_MS;
  dev->did_timeout                 = 0u;
  dev->timeout_start_ms            = 0u;
  dev->stop_variable               = 0u;
  dev->measurement_timing_budget_us = 0u;
  dev->initialized                 = 0u;
}

/**
  * @brief  探测模块是否应答
  */
uint8_t VL53L0X_IsPresent(VL53L0X_t *dev)
{
  if ((dev == NULL) || (dev->hi2c == NULL))
  {
    return 0u;
  }

  return (HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(dev->address << 1), 2u, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

/**
  * @brief  完整初始化: DataInit + StaticInit + PerformRefCalibration
  *         (不做 reference SPAD calibration, 与例程一致: ST 出厂已在裸模块上做过)
  * @retval 1 = 成功, 0 = 失败(模块没应答或时序超时)
  */
uint8_t VL53L0X_Init(VL53L0X_t *dev)
{
  uint8_t  spad_count = 0u;
  uint8_t  spad_type_is_aperture = 0u;
  uint8_t  ref_spad_map[6];
  uint8_t  first_spad_to_enable;
  uint8_t  spads_enabled = 0u;
  uint8_t  i;
  uint32_t budget;

  if ((dev == NULL) || (dev->hi2c == NULL))
  {
    return 0u;
  }

  dev->initialized = 0u;

  /* ---- VL53L0X_DataInit() begin ---- */

  /* 传感器 I/O 默认为 1V8 模式, 这里切到 2V8 */
  (void)vl53l0x_write_reg(dev, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                          (uint8_t)(vl53l0x_read_reg(dev, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01u));

  /* "Set I2C standard mode" */
  (void)vl53l0x_write_reg(dev, 0x88u, 0x00u);

  (void)vl53l0x_write_reg(dev, 0x80u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x00u);
  dev->stop_variable = vl53l0x_read_reg(dev, 0x91u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x80u, 0x00u);

  /* 关闭 SIGNAL_RATE_MSRC(bit1) 和 SIGNAL_RATE_PRE_RANGE(bit4) 门限检查 */
  (void)vl53l0x_write_reg(dev, VL53L0X_MSRC_CONFIG_CONTROL,
                          (uint8_t)(vl53l0x_read_reg(dev, VL53L0X_MSRC_CONFIG_CONTROL) | 0x12u));

  /* final range 信号速率门限设为 0.25 MCPS */
  (void)VL53L0X_SetSignalRateLimit(dev, 0.25f);

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFFu);

  /* ---- VL53L0X_StaticInit() begin ---- */

  if (vl53l0x_get_spad_info(dev, &spad_count, &spad_type_is_aperture) == 0u)
  {
    return 0u;
  }

  /* 读回参考 SPAD 使能位图 */
  (void)vl53l0x_read_multi(dev, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6u);

  /* ---- VL53L0X_set_reference_spads() begin (假定 NVM 值有效) ---- */

  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00u);
  (void)vl53l0x_write_reg(dev, VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2Cu);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
  (void)vl53l0x_write_reg(dev, VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4u);

  first_spad_to_enable = (spad_type_is_aperture != 0u) ? 12u : 0u;

  for (i = 0u; i < 48u; i++)
  {
    if ((i < first_spad_to_enable) || (spads_enabled == spad_count))
    {
      /* 这一位在首个应使能位之前, 或已经使能够数量, 清零 */
      ref_spad_map[i / 8u] = (uint8_t)(ref_spad_map[i / 8u] & (uint8_t)~(1u << (i % 8u)));
    }
    else if (((ref_spad_map[i / 8u] >> (i % 8u)) & 0x01u) != 0u)
    {
      spads_enabled++;
    }
  }

  (void)vl53l0x_write_multi(dev, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6u);

  /* ---- VL53L0X_set_reference_spads() end ---- */

  /* ---- VL53L0X_load_tuning_settings() begin ---- */
  for (i = 0u; i < (uint8_t)(sizeof(s_tuning_settings) / sizeof(s_tuning_settings[0])); i++)
  {
    (void)vl53l0x_write_reg(dev, s_tuning_settings[i].reg, s_tuning_settings[i].value);
  }
  /* ---- VL53L0X_load_tuning_settings() end ---- */

  /* "Set interrupt config to new sample ready" */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04u);
  (void)vl53l0x_write_reg(dev, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH,
                          (uint8_t)(vl53l0x_read_reg(dev, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH) & (uint8_t)~0x10u)); /* active low */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u);

  budget = VL53L0X_GetMeasurementTimingBudget(dev);

  /* "Disable MSRC and TCC by default" */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8u);

  /* "Recalculate timing budget" */
  (void)VL53L0X_SetMeasurementTimingBudget(dev, budget);

  /* ---- VL53L0X_StaticInit() end ---- */

  /* ---- VL53L0X_PerformRefCalibration() begin ---- */

  /* perform_vhv_calibration */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01u);
  if (vl53l0x_perform_single_ref_calibration(dev, 0x40u) == 0u)
  {
    return 0u;
  }

  /* perform_phase_calibration */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02u);
  if (vl53l0x_perform_single_ref_calibration(dev, 0x00u) == 0u)
  {
    return 0u;
  }

  /* 恢复之前的 Sequence Config */
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8u);

  /* ---- VL53L0X_PerformRefCalibration() end ---- */

  dev->initialized = 1u;
  return 1u;
}

/**
  * @brief  修改从机地址 (多模块时: 上电先把 XSHUT 拉低, 逐个放开并改址)
  */
void VL53L0X_SetAddress(VL53L0X_t *dev, uint8_t new_addr)
{
  if (dev == NULL)
  {
    return;
  }

  (void)vl53l0x_write_reg(dev, VL53L0X_I2C_SLAVE_DEVICE_ADDRESS, (uint8_t)(new_addr & 0x7Fu));
  dev->address = new_addr;
}

/**
  * @brief  启动连续测量
  * @param  period_ms 0 = 背靠背(尽可能快); 非 0 = 定时模式, 按该周期测量
  */
void VL53L0X_StartContinuous(VL53L0X_t *dev, uint32_t period_ms)
{
  uint16_t osc_calibrate_val;

  if (dev == NULL)
  {
    return;
  }

  (void)vl53l0x_write_reg(dev, 0x80u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x91u, dev->stop_variable);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x80u, 0x00u);

  if (period_ms != 0u)
  {
    /* 定时模式 */
    osc_calibrate_val = vl53l0x_read_reg16(dev, VL53L0X_OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0u)
    {
      period_ms *= (uint32_t)osc_calibrate_val;
    }

    (void)vl53l0x_write_reg32(dev, VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, 0x04u); /* timed */
  }
  else
  {
    /* 背靠背模式 */
    (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, 0x02u); /* back-to-back */
  }
}

/**
  * @brief  停止连续测量
  */
void VL53L0X_StopContinuous(VL53L0X_t *dev)
{
  if (dev == NULL)
  {
    return;
  }

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, 0x01u); /* single shot */

  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x91u, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
}

/**
  * @brief  阻塞读取连续测量结果 (mm); 超时返回 65535
  *         注意: 背靠背模式下一次测量约 33 ms, 控制循环里请改用
  *               VL53L0X_ReadRangeIfReady()
  */
uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_t *dev)
{
  uint16_t range;

  if (dev == NULL)
  {
    return 65535u;
  }

  vl53l0x_start_timeout(dev);
  while ((vl53l0x_read_reg(dev, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07u) == 0u)
  {
    if (vl53l0x_timeout_expired(dev) != 0u)
    {
      dev->did_timeout = 1u;
      return 65535u;
    }
  }

  /* 假定 Linearity Corrective Gain = 1000(默认), 未启用小数测距 */
  range = vl53l0x_read_reg16(dev, (uint8_t)(VL53L0X_RESULT_RANGE_STATUS + 10u));

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u);

  return range;
}

/**
  * @brief  单次测量 (阻塞); 超时返回 65535
  */
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_t *dev)
{
  if (dev == NULL)
  {
    return 65535u;
  }

  (void)vl53l0x_write_reg(dev, 0x80u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x91u, dev->stop_variable);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x80u, 0x00u);

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, 0x01u);

  /* "Wait until start bit has been cleared" */
  vl53l0x_start_timeout(dev);
  while ((vl53l0x_read_reg(dev, VL53L0X_SYSRANGE_START) & 0x01u) != 0u)
  {
    if (vl53l0x_timeout_expired(dev) != 0u)
    {
      dev->did_timeout = 1u;
      return 65535u;
    }
  }

  return VL53L0X_ReadRangeContinuousMillimeters(dev);
}

/**
  * @brief  非阻塞读取: 有新结果时写入 *range_mm 并返回 1, 没有则返回 0
  *         主循环/控制循环里应该用这个
  */
uint8_t VL53L0X_ReadRangeIfReady(VL53L0X_t *dev, uint16_t *range_mm)
{
  if ((dev == NULL) || (range_mm == NULL))
  {
    return 0u;
  }

  if ((vl53l0x_read_reg(dev, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07u) == 0u)
  {
    return 0u;
  }

  *range_mm = vl53l0x_read_reg16(dev, (uint8_t)(VL53L0X_RESULT_RANGE_STATUS + 10u));

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u);

  return 1u;
}

/**
  * @brief  查询并清除"发生过超时"标志
  */
uint8_t VL53L0X_TimeoutOccurred(VL53L0X_t *dev)
{
  uint8_t tmp;

  if (dev == NULL)
  {
    return 0u;
  }

  tmp = dev->did_timeout;
  dev->did_timeout = 0u;
  return tmp;
}

/**
  * @brief  设置返回信号速率门限 (MCPS), Q9.7 定点格式
  */
uint8_t VL53L0X_SetSignalRateLimit(VL53L0X_t *dev, float limit_mcps)
{
  if ((dev == NULL) || (limit_mcps < 0.0f) || (limit_mcps > 511.99f))
  {
    return 0u;
  }

  (void)vl53l0x_write_reg16(dev, VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                            (uint16_t)(limit_mcps * 128.0f));
  return 1u;
}

/**
  * @brief  设置测量时间预算 (us), 越大越准; 最小 20000, 默认约 33000
  */
uint8_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_t *dev, uint32_t budget_us)
{
  VL53L0X_SequenceStepEnables_t  enables;
  VL53L0X_SequenceStepTimeouts_t timeouts;

  /* 注意: 与 get_ 版本不同, 这里的 StartOverhead 是 1320 */
  const uint32_t start_overhead      = 1320u;
  const uint32_t end_overhead        = 960u;
  const uint32_t msrc_overhead       = 660u;
  const uint32_t tcc_overhead        = 590u;
  const uint32_t dss_overhead        = 690u;
  const uint32_t pre_range_overhead  = 660u;
  const uint32_t final_range_overhead = 550u;
  const uint32_t min_timing_budget   = 20000u;

  uint32_t used_budget_us;
  uint32_t final_range_timeout_us;
  uint16_t final_range_timeout_mclks;

  if ((dev == NULL) || (budget_us < min_timing_budget))
  {
    return 0u;
  }

  used_budget_us = start_overhead + end_overhead;

  vl53l0x_get_sequence_step_enables(dev, &enables);
  vl53l0x_get_sequence_step_timeouts(dev, &enables, &timeouts);

  if (enables.tcc != 0u)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + tcc_overhead);
  }

  if (enables.dss != 0u)
  {
    used_budget_us += 2u * (timeouts.msrc_dss_tcc_us + dss_overhead);
  }
  else if (enables.msrc != 0u)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + msrc_overhead);
  }

  if (enables.pre_range != 0u)
  {
    used_budget_us += (timeouts.pre_range_us + pre_range_overhead);
  }

  if (enables.final_range != 0u)
  {
    used_budget_us += final_range_overhead;

    if (used_budget_us > budget_us)
    {
      /* 请求的时间预算过大 */
      return 0u;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    /* 最终 range 超时要把 pre-range 超时加上, 两者统一换算成 MClk */
    final_range_timeout_mclks =
      (uint16_t)vl53l0x_timeout_us_to_mclks(final_range_timeout_us,
                                            (uint8_t)timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range != 0u)
    {
      final_range_timeout_mclks = (uint16_t)(final_range_timeout_mclks + timeouts.pre_range_mclks);
    }

    (void)vl53l0x_write_reg16(dev, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              vl53l0x_encode_timeout(final_range_timeout_mclks));

    dev->measurement_timing_budget_us = budget_us;
  }

  return 1u;
}

/**
  * @brief  读取当前测量时间预算 (us)
  */
uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_t *dev)
{
  VL53L0X_SequenceStepEnables_t  enables;
  VL53L0X_SequenceStepTimeouts_t timeouts;

  /* 注意: 与 set_ 版本不同, 这里的 StartOverhead 是 1910 */
  const uint32_t start_overhead      = 1910u;
  const uint32_t end_overhead        = 960u;
  const uint32_t msrc_overhead       = 660u;
  const uint32_t tcc_overhead        = 590u;
  const uint32_t dss_overhead        = 690u;
  const uint32_t pre_range_overhead  = 660u;
  const uint32_t final_range_overhead = 550u;

  uint32_t budget_us;

  if (dev == NULL)
  {
    return 0u;
  }

  budget_us = start_overhead + end_overhead;

  vl53l0x_get_sequence_step_enables(dev, &enables);
  vl53l0x_get_sequence_step_timeouts(dev, &enables, &timeouts);

  if (enables.tcc != 0u)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + tcc_overhead);
  }

  if (enables.dss != 0u)
  {
    budget_us += 2u * (timeouts.msrc_dss_tcc_us + dss_overhead);
  }
  else if (enables.msrc != 0u)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + msrc_overhead);
  }

  if (enables.pre_range != 0u)
  {
    budget_us += (timeouts.pre_range_us + pre_range_overhead);
  }

  if (enables.final_range != 0u)
  {
    budget_us += (timeouts.final_range_us + final_range_overhead);
  }

  dev->measurement_timing_budget_us = budget_us;
  return budget_us;
}

/* Private functions ---------------------------------------------------------*/

/** 写 8 位寄存器 */
static uint8_t vl53l0x_write_reg(VL53L0X_t *dev, uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, &value, 1u, VL53L0X_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 写 16 位寄存器 (高字节在前) */
static uint8_t vl53l0x_write_reg16(VL53L0X_t *dev, uint8_t reg, uint16_t value)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st;

  buf[0] = (uint8_t)((value >> 8) & 0xFFu);
  buf[1] = (uint8_t)(value & 0xFFu);

  st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, 2u, VL53L0X_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 写 32 位寄存器 (最高字节在前) */
static uint8_t vl53l0x_write_reg32(VL53L0X_t *dev, uint8_t reg, uint32_t value)
{
  uint8_t buf[4];
  HAL_StatusTypeDef st;

  buf[0] = (uint8_t)((value >> 24) & 0xFFu);
  buf[1] = (uint8_t)((value >> 16) & 0xFFu);
  buf[2] = (uint8_t)((value >> 8) & 0xFFu);
  buf[3] = (uint8_t)(value & 0xFFu);

  st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, 4u, VL53L0X_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 读 8 位寄存器 */
static uint8_t vl53l0x_read_reg(VL53L0X_t *dev, uint8_t reg)
{
  uint8_t value = 0u;

  (void)HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, &value, 1u, VL53L0X_I2C_TIMEOUT_MS);
  return value;
}

/** 读 16 位寄存器 (高字节在前) */
static uint16_t vl53l0x_read_reg16(VL53L0X_t *dev, uint8_t reg)
{
  uint8_t buf[2] = {0u, 0u};

  (void)HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, 2u, VL53L0X_I2C_TIMEOUT_MS);
  return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/** 从 reg 开始连续写 count 字节 */
static uint8_t vl53l0x_write_multi(VL53L0X_t *dev, uint8_t reg, const uint8_t *src, uint8_t count)
{
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)src, count,
                                           VL53L0X_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 从 reg 开始连续读 count 字节 */
static uint8_t vl53l0x_read_multi(VL53L0X_t *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                                          I2C_MEMADD_SIZE_8BIT, dst, count,
                                          VL53L0X_I2C_TIMEOUT_MS);
  return (st == HAL_OK) ? 1u : 0u;
}

/** 记录超时计时起点 */
static void vl53l0x_start_timeout(VL53L0X_t *dev)
{
  dev->timeout_start_ms = HAL_GetTick();
}

/** 超时是否已到 (io_timeout = 0 表示不超时) */
static uint8_t vl53l0x_timeout_expired(const VL53L0X_t *dev)
{
  if (dev->io_timeout == 0u)
  {
    return 0u;
  }

  return ((HAL_GetTick() - dev->timeout_start_ms) > (uint32_t)dev->io_timeout) ? 1u : 0u;
}

/**
  * @brief  获取参考 SPAD 数量和类型
  *         对应 VL53L0X_get_info_from_device(), 这里只取 SPAD count 和类型
  */
static uint8_t vl53l0x_get_spad_info(VL53L0X_t *dev, uint8_t *count, uint8_t *type_is_aperture)
{
  uint8_t tmp;

  (void)vl53l0x_write_reg(dev, 0x80u, 0x01u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x00u);

  (void)vl53l0x_write_reg(dev, 0xFFu, 0x06u);
  (void)vl53l0x_write_reg(dev, 0x83u, (uint8_t)(vl53l0x_read_reg(dev, 0x83u) | 0x04u));
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x07u);
  (void)vl53l0x_write_reg(dev, 0x81u, 0x01u);

  (void)vl53l0x_write_reg(dev, 0x80u, 0x01u);

  (void)vl53l0x_write_reg(dev, 0x94u, 0x6bu);
  (void)vl53l0x_write_reg(dev, 0x83u, 0x00u);

  vl53l0x_start_timeout(dev);
  while (vl53l0x_read_reg(dev, 0x83u) == 0x00u)
  {
    if (vl53l0x_timeout_expired(dev) != 0u)
    {
      return 0u;
    }
  }

  (void)vl53l0x_write_reg(dev, 0x83u, 0x01u);
  tmp = vl53l0x_read_reg(dev, 0x92u);

  *count = (uint8_t)(tmp & 0x7Fu);
  *type_is_aperture = (uint8_t)((tmp >> 7) & 0x01u);

  (void)vl53l0x_write_reg(dev, 0x81u, 0x00u);
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x06u);
  /* 例程原文是 writeReg(0x83, readReg(0x83 & ~0x04)), 掩码作用在地址上(等于 0x83),
     实际效果是把读到的值原样写回。这里保持一致, 不"修正"为 readReg(0x83) & ~0x04 */
  (void)vl53l0x_write_reg(dev, 0x83u, vl53l0x_read_reg(dev, (uint8_t)(0x83u & (uint8_t)~0x04u)));
  (void)vl53l0x_write_reg(dev, 0xFFu, 0x01u);
  (void)vl53l0x_write_reg(dev, 0x00u, 0x01u);

  (void)vl53l0x_write_reg(dev, 0xFFu, 0x00u);
  (void)vl53l0x_write_reg(dev, 0x80u, 0x00u);

  return 1u;
}

/** 读取各序列步骤的使能位 (对应 VL53L0X_GetSequenceStepEnables) */
static void vl53l0x_get_sequence_step_enables(VL53L0X_t *dev, VL53L0X_SequenceStepEnables_t *enables)
{
  uint8_t sequence_config = vl53l0x_read_reg(dev, VL53L0X_SYSTEM_SEQUENCE_CONFIG);

  enables->tcc         = (uint8_t)((sequence_config >> 4) & 0x01u);
  enables->dss         = (uint8_t)((sequence_config >> 3) & 0x01u);
  enables->msrc        = (uint8_t)((sequence_config >> 2) & 0x01u);
  enables->pre_range   = (uint8_t)((sequence_config >> 6) & 0x01u);
  enables->final_range = (uint8_t)((sequence_config >> 7) & 0x01u);
}

/** 读取各序列步骤的超时 (对应 get_sequence_step_timeout, 这里一次全读) */
static void vl53l0x_get_sequence_step_timeouts(VL53L0X_t *dev,
                                               const VL53L0X_SequenceStepEnables_t *enables,
                                               VL53L0X_SequenceStepTimeouts_t *timeouts)
{
  timeouts->pre_range_vcsel_period_pclks =
    (uint16_t)vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE);

  timeouts->msrc_dss_tcc_mclks = (uint16_t)(vl53l0x_read_reg(dev, VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP) + 1u);
  timeouts->msrc_dss_tcc_us =
    vl53l0x_timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
                                (uint8_t)timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks = vl53l0x_decode_timeout(vl53l0x_read_reg16(dev, VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    vl53l0x_timeout_mclks_to_us(timeouts->pre_range_mclks,
                                (uint8_t)timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks =
    (uint16_t)vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE);

  timeouts->final_range_mclks = vl53l0x_decode_timeout(vl53l0x_read_reg16(dev, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range != 0u)
  {
    timeouts->final_range_mclks = (uint16_t)(timeouts->final_range_mclks - timeouts->pre_range_mclks);
  }

  timeouts->final_range_us =
    vl53l0x_timeout_mclks_to_us(timeouts->final_range_mclks,
                                (uint8_t)timeouts->final_range_vcsel_period_pclks);
}

/**
  * @brief  执行一次参考校准 (对应 VL53L0X_perform_single_ref_calibration)
  */
static uint8_t vl53l0x_perform_single_ref_calibration(VL53L0X_t *dev, uint8_t vhv_init_byte)
{
  (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, (uint8_t)(0x01u | vhv_init_byte));

  vl53l0x_start_timeout(dev);
  while ((vl53l0x_read_reg(dev, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07u) == 0u)
  {
    if (vl53l0x_timeout_expired(dev) != 0u)
    {
      return 0u;
    }
  }

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u);

  (void)vl53l0x_write_reg(dev, VL53L0X_SYSRANGE_START, 0x00u);

  return 1u;
}

/** 读取 VCSEL 脉冲周期 (PCLK) */
static uint8_t vl53l0x_get_vcsel_pulse_period(VL53L0X_t *dev, VL53L0X_VcselPeriodType type)
{
  if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
  {
    return VL53L0X_DECODE_VCSEL_PERIOD(vl53l0x_read_reg(dev, VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }

  if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
  {
    return VL53L0X_DECODE_VCSEL_PERIOD(vl53l0x_read_reg(dev, VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }

  return 255u;
}

/** 从寄存器值解码超时 (格式: (LSByte * 2^MSByte) + 1) */
static uint16_t vl53l0x_decode_timeout(uint16_t reg_val)
{
  return (uint16_t)((uint16_t)((reg_val & 0x00FFu) << (uint16_t)((reg_val & 0xFF00u) >> 8)) + 1u);
}

/** 把超时 (MClk) 编码成寄存器值 */
static uint16_t vl53l0x_encode_timeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte = 0u;
  uint16_t ms_byte = 0u;

  if (timeout_mclks == 0u)
  {
    return 0u;
  }

  ls_byte = (uint32_t)timeout_mclks - 1u;

  while ((ls_byte & 0xFFFFFF00u) > 0u)
  {
    ls_byte >>= 1;
    ms_byte++;
  }

  return (uint16_t)((ms_byte << 8) | (uint16_t)(ls_byte & 0xFFu));
}

/** 宏周期 (ns): PLL_period_ps = 1655, macro_period_vclks = 2304 */
static uint32_t vl53l0x_calc_macro_period(uint8_t vcsel_period_pclks)
{
  return ((((uint32_t)2304u * (uint32_t)vcsel_period_pclks * 1655u) + 500u) / 1000u);
}

/** MClk -> us */
static uint32_t vl53l0x_timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = vl53l0x_calc_macro_period(vcsel_period_pclks);

  return (((uint32_t)timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2u)) / 1000u;
}

/** us -> MClk */
static uint32_t vl53l0x_timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = vl53l0x_calc_macro_period(vcsel_period_pclks);

  return (((timeout_period_us * 1000u) + (macro_period_ns / 2u)) / macro_period_ns);
}
