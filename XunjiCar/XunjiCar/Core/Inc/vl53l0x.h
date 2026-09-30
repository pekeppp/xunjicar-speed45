/**
  ******************************************************************************
  * @file    vl53l0x.h
  * @brief   VL53L0X-V2 TOF 激光测距模块驱动 (C 语言移植版)
  *
  * 移植自例程: VL53L0X-V2 TOF激光红外测距\arduino\vl53l0x\
  *             (Pololu VL53L0X Arduino 库, 其内容基于 ST 官方 API STSW-IMG005)
  * 改动点:
  *   1. C++ 类 -> C 结构体 + 函数, 支持多实例 (每个实例带自己的 I2C 句柄和地址)
  *   2. Wire 库 -> STM32 HAL 的 HAL_I2C_Mem_Read/Write (寄存器地址 8 位)
  *   3. millis() -> HAL_GetTick()
  *   4. 超时时间默认给 500 ms (原库默认 0 = 永不超时, 传感器掉线会死循环)
  *   5. 增加非阻塞接口 VL53L0X_ReadRangeIfReady(), 供主循环里安全调用
  *      (阻塞版 VL53L0X_ReadRangeContinuousMillimeters() 会等约 33 ms, 不推荐在控制循环里用)
  *
  * 硬件: VCC 3.3V(或按模块标注), GND 共地, SCL/SDA 接 I2C1 (见 .ioc: PB6=SCL, PB7=SDA)
  *       模块 I2C 地址: 0x29 (7 位); XSHUT 引脚本驱动不使用
  *       注意: VL53L0X 的 I/O 是 2.8V 电平, 模块板一般自带电平转换, 若是裸芯片则不能直连 3.3V
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VL53L0X_H__
#define __VL53L0X_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 可调参数 ------------------------------------------------------------------*/

/** 模块默认 7 位地址 */
#define VL53L0X_ADDRESS_DEFAULT      (0x29u)

/** 单次 I2C 读写超时 (ms) */
#define VL53L0X_I2C_TIMEOUT_MS       (20u)

/** 轮询等待测距结果的超时 (ms); 0 = 永不超时(不建议) */
#define VL53L0X_IO_TIMEOUT_MS        (500u)

/** 推荐测距周期 (ms): 连续测量模式下两次测量之间的时间 */
#define VL53L0X_MEASUREMENT_BUDGET_US (33000u)

/* 寄存器地址 (与例程 vl53l0x_device.h 一致) ---------------------------------*/
#define VL53L0X_SYSRANGE_START                              0x00u

#define VL53L0X_SYSTEM_THRESH_HIGH                          0x0Cu
#define VL53L0X_SYSTEM_THRESH_LOW                           0x0Eu

#define VL53L0X_SYSTEM_SEQUENCE_CONFIG                      0x01u
#define VL53L0X_SYSTEM_RANGE_CONFIG                         0x09u
#define VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD              0x04u

#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0Au

#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH                     0x84u

#define VL53L0X_SYSTEM_INTERRUPT_CLEAR                      0x0Bu

#define VL53L0X_RESULT_INTERRUPT_STATUS                     0x13u
#define VL53L0X_RESULT_RANGE_STATUS                         0x14u

#define VL53L0X_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28u

#define VL53L0X_I2C_SLAVE_DEVICE_ADDRESS                    0x8Au

#define VL53L0X_MSRC_CONFIG_CONTROL                         0x60u

#define VL53L0X_PRE_RANGE_CONFIG_MIN_SNR                    0x27u
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56u
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57u

#define VL53L0X_FINAL_RANGE_CONFIG_MIN_SNR                  0x67u
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47u
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48u
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44u

#define VL53L0X_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61u
#define VL53L0X_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62u

#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50u
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51u
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52u

#define VL53L0X_SYSTEM_HISTOGRAM_BIN                        0x81u
#define VL53L0X_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33u
#define VL53L0X_HISTOGRAM_CONFIG_READOUT_CTRL               0x55u

#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70u
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71u
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72u
#define VL53L0X_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20u

#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP                  0x46u

#define VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N                 0xBFu
#define VL53L0X_IDENTIFICATION_MODEL_ID                     0xC0u
#define VL53L0X_IDENTIFICATION_REVISION_ID                  0xC2u

#define VL53L0X_OSC_CALIBRATE_VAL                           0xF8u

#define VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4u
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5u

#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6u
#define VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4Eu
#define VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4Fu
#define VL53L0X_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80u

#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89u

#define VL53L0X_ALGO_PHASECAL_LIM                           0x30u
#define VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30u

/* 数据结构 ------------------------------------------------------------------*/

/** VCSEL 脉冲周期类型 */
typedef enum
{
  VL53L0X_VCSEL_PERIOD_PRE_RANGE = 0,
  VL53L0X_VCSEL_PERIOD_FINAL_RANGE = 1
} VL53L0X_VcselPeriodType;

/** 设备实例 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;                        /**< 使用的 I2C 总线 (如 &hi2c1) */
  uint8_t  address;                               /**< 7 位从机地址 */
  uint16_t io_timeout;                            /**< 轮询超时 (ms), 0 = 不超时 */
  uint8_t  did_timeout;                           /**< 上次读取是否超时 */
  uint32_t timeout_start_ms;                      /**< 超时计时起点 */
  uint8_t  stop_variable;                         /**< init 期间读到, 启动测量时要写回 */
  uint32_t measurement_timing_budget_us;          /**< 测量时间预算 */
  uint8_t  initialized;                           /**< 1 = init 成功 */
} VL53L0X_t;

/* 对外接口 ------------------------------------------------------------------*/

/** 绑定 I2C 总线并复位为默认参数 (地址 0x29, 超时 500 ms) */
void VL53L0X_InitDevice(VL53L0X_t *dev, I2C_HandleTypeDef *hi2c);

/** 探测模块是否应答 (不改变任何配置) */
uint8_t VL53L0X_IsPresent(VL53L0X_t *dev);

/** 完整初始化 (DataInit + StaticInit + 参考校准), 耗时约 100~200 ms */
uint8_t VL53L0X_Init(VL53L0X_t *dev);

/** 修改从机地址 (上电默认 0x29; 多模块时配合 XSHUT 使用) */
void VL53L0X_SetAddress(VL53L0X_t *dev, uint8_t new_addr);

/** 启动连续测量; period_ms = 0 表示背靠背(最快), 否则为定时模式周期 */
void VL53L0X_StartContinuous(VL53L0X_t *dev, uint32_t period_ms);

/** 停止连续测量 */
void VL53L0X_StopContinuous(VL53L0X_t *dev);

/** 阻塞读取一次连续测量的结果 (mm); 超时返回 65535 */
uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_t *dev);

/** 单次测量 (阻塞); 超时返回 65535 */
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_t *dev);

/** 非阻塞读取: 有新数据时写入 *range_mm 并返回 1, 否则返回 0
  * 主循环里应该用这个, 不会卡住 */
uint8_t VL53L0X_ReadRangeIfReady(VL53L0X_t *dev, uint16_t *range_mm);

/** 查询并清除"发生过超时"标志 */
uint8_t VL53L0X_TimeoutOccurred(VL53L0X_t *dev);

/** 设置返回信号速率门限 (MCPS), 越大越不容易被杂散反射干扰; 默认 0.25 */
uint8_t VL53L0X_SetSignalRateLimit(VL53L0X_t *dev, float limit_mcps);

/** 设置测量时间预算 (us): 越长越准, 最小 20000, 默认约 33000 */
uint8_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_t *dev, uint32_t budget_us);

/** 读取当前测量时间预算 (us) */
uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_H__ */
