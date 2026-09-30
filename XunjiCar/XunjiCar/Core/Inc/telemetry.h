/**
  ******************************************************************************
  * @file    telemetry.h
  * @brief   统一遥测帧: 循迹 / 双路测距 / IMU / 电机 四类数据合并成一帧发出
  *
  * 数据来源:
  *   循迹  <- USART1(PA9/PA10)  八路巡线模块 (track_uart.c)
  *   测距  <- I2C1(TOF1) / I2C3(TOF2)  VL53L0X (vl53l0x.c)
  *            (总开关是 line_follow.h 的 LF_ENABLE_TOF_LASER, 默认 0 = 关,
  *             关着时 VL53L0X 根本不初始化, 帧里也没有 D1/D2 两个字段)
  *   IMU   <- I2C2(PB10/PB11)   MPU6050 (mpu6050.c)
  *   电机  <- TIM3/TIM4 编码器 + TIM8 PWM (motor_test.c)
  *
  * 帧格式 (ASCII, 一帧一行, 每 TELEMETRY_SEND_PERIOD_MS 一帧):
  *   $CAR,N=123,TRK=1F,B=11101111,D1=235,D2=412,AX=1234,AY=-567,AZ=16384,
  *        GX=123,GY=-45,GZ=8,T=25,S=FWD,MA=+30,EA=+3100,MB=+0,EB=+0*7E
  *
  *   $        帧头, *XX 是校验和(2 位十六进制)
  *   CAR      帧标识
  *   N        帧序号 0~65535 循环 (上位机可据此发现丢帧)
  *   TRK/B    循迹原始字节 / 8 位探头状态 (bit7~bit0 = x1~x8, 1=白底, 0=压线)
  *            读不到时为 TRK=ERR,B=--------
  *   D1/D2    两路 TOF 距离 mm; --- = 无有效回波, NA = 模块未初始化
  *            LF_ENABLE_TOF_LASER = 0 (默认) 时这两个字段整组不出现
  *   AX..GZ   加速度/陀螺仪原始计数 (换算见 mpu6050.h 的 API)
  *   T        温度 (摄氏度); 整组取不到时改用单个字段 IMU=NA
  *   S        电机测试阶段 (WAIT/STOP/A-FWD/A-REV/B-FWD/B-REV/AB-FWD/AB-REV/MANUAL)
  *   MA/MB    两路占空比 (%, 带正负号, 正=正转)
  *   EA/EB    本帧周期内两路编码器的计数增量 (乘换算是速度)
  *   LF       循迹控制环状态 (IDLE/WAIT/RUN/COAST/LOST/BLOCK)
  *   ERR      循迹位置偏差 -1000(线在最左) ~ +1000(线在最右)
  *   ST       本周期差速修正量 (%), 正=往右修正
  *   GZD      Z 轴角速度 (dps×10, 已扣零偏) —— 陀螺仪阻尼用的就是它
  *   YAW      丢线后累计转角 (度×10, 左转为正); 只有 LF_LOST_ACTION=2 时才动
  *   IMS      陀螺仪数据是否有效 (1/0); 0 = 没接 MPU 或没校准成功
  *   *XX      $ 与 * 之间所有字符逐一异或 (与 NMEA 的 XOR 校验同理)
  *
  * 设计要点: 四类数据全部由本模块在一处组装并发出一条完整行,
  *           各驱动模块不再自己打印, 因此不会出现多行交叉/半截帧的问题。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 可调参数 ------------------------------------------------------------------*/

/** 主循环调用 Telemetry_Task() 的节奏 (ms); main 里的 HAL_Delay 用这个值 */
#define TELEMETRY_TASK_PERIOD_MS     (10u)

/** 发帧周期 (ms): 10 Hz */
#define TELEMETRY_SEND_PERIOD_MS     (100u)

/** 向上位机(USART2)发送的超时 (ms) */
#define TELEMETRY_UART_TIMEOUT_MS    (50u)

/** 超过这个距离(mm)视为无效(超量程/无目标), 帧里显示 --- */
#define TELEMETRY_DIST_MAX_MM        (2000u)

/** 帧标识 */
#define TELEMETRY_FRAME_ID           "CAR"

/** 各组数据的开关 (不要哪组就把对应宏改成 0, 帧会变短) */
#define TELEMETRY_ENABLE_TRACK       (1u)
#define TELEMETRY_ENABLE_TOF         (1u)
#define TELEMETRY_ENABLE_IMU         (1u)
#define TELEMETRY_ENABLE_MOTOR       (1u)
#define TELEMETRY_ENABLE_LINEFOLLOW  (1u)

/** 使用几路 TOF: 1 = 只用 D1(I2C1), 2 = D1 + D2(I2C3) */
#define TELEMETRY_TOF_COUNT          (2u)

/** 支持的 TOF 路数上限 (总线映射表长度), 不要改 */
#define TELEMETRY_TOF_MAX            (2u)

/* 对外接口 ------------------------------------------------------------------*/

/** 初始化: 启动巡线串口 + 逐路初始化 VL53L0X (上电调用一次)
  * 某个模块不在线不影响其它模块和整体启动 */
void Telemetry_Init(void);

/** 主循环周期调用: 采样 + 定时发一帧 (非阻塞) */
void Telemetry_Task(void);

/** 取最近一次解析出的循迹原始字节
  * @retval 1 = 数据有效 */
uint8_t Telemetry_GetTrackByte(uint8_t *raw);

/** 取某一路 TOF 距离 (mm)
  * @param  index 0 = D1(I2C1), 1 = D2(I2C3);
  *               帧里 D1 永远排在前面, 和 index 顺序一一对应。
  *               "哪一路在车的左边"见 line_follow.h 的 LF_TOF_LEFT / LF_TOF_RIGHT
  * @retval 1 = 数据有效 */
uint8_t Telemetry_GetDistanceMm(uint8_t index, uint16_t *mm);

/** 指定那一路 VL53L0X 是否初始化成功 */
uint8_t Telemetry_IsTofReady(uint8_t index);

/** 当前帧序号 */
uint16_t Telemetry_GetFrameSeq(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H__ */
