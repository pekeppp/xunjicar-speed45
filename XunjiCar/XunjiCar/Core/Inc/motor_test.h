/**
  ******************************************************************************
  * @file    motor_test.h
  * @brief   双电机接线测试: TIM8_CHx(PWM) + AIN/BIN(方向) + TIM3/TIM4(编码器)
  *
  * 接线:
  *                        电机 A (TB6612 A 通道)      电机 B (TB6612 B 通道)
  *   PWM  调速              PC6  (TIM8_CH1)             PC7  (TIM8_CH2)
  *   I1   方向              PD0  (AIN1)                PD2  (BIN1)
  *   I2   方向              PD1  (AIN2)                PD3  (BIN2)
  *   编码器 A 相            PA6  (TIM3_CH1)             PD12 (TIM4_CH1)
  *   编码器 B 相            PA7  (TIM3_CH2)             PD13 (TIM4_CH2)
  *
  *   STBY: 硬件拉高, 两路共用, 代码不控制
  *   编码器 VCC -> 5V, GND -> 共地
  *   已查数据手册 Table 6 (Doc ID 022152 Rev 2, p.46/47): PA6/PA7 与 PD12/PD13
  *   的 I/O 结构都是 FT(5V 容忍), 所以 5V 编码器的 A/B 相可以直连, 不用分压。
  *   (注意本芯片上是 PA4/PA5 才是 TTa 3.3V-only, 与编码器无关)
  *
  * 串口输出 (USART2, 115200 8-N-1), 每 MOTOR_TEST_SAMPLE_MS 每个电机一行:
  *   MTR A step=A-FWD pwm=+30% enc=+3100 rpm=+310 total=+3100
  *   MTR B step=A-FWD pwm=+0%  enc=+0    rpm=+0   total=+0
  *   step  : 当前测试阶段
  *   pwm   : 占空比, 带符号表示方向
  *   enc   : 本采样周期内累加的编码器计数
  *   rpm   : 按 MOTOR_ENC_COUNTS_PER_REV 换算的转速
  *   total : 上电以来累计计数 (手转一圈可直接用它标定)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_TEST_H__
#define __MOTOR_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 电机编号 ------------------------------------------------------------------*/
#define MOTOR_A                  (0u)
#define MOTOR_B                  (1u)
#define MOTOR_COUNT              (2u)

/* 电机/编码器参数 (按你的电机标签改) ----------------------------------------*/

/** 编码器线数 (每转脉冲数): GMR 版 500 线 (霍尔版是 13 线, 别混)
  * 不确定就用手转一圈看 total 来标定 */
#define MOTOR_ENC_PPR            (500)

/** 减速比: 看电机型号后缀 P10 / P20 / P30 -> 10 / 20 / 30 */
#define MOTOR_GEAR_RATIO         (30)

/** 编码器计数方向取反 (两路独立): 实测正转时计数变小就改成 1 */
#define MOTOR_A_ENC_INVERT       (0)
#define MOTOR_B_ENC_INVERT       (0)

/** 电机转向取反 (两路独立): 电机镜像安装(一正一反)时用这个,
  * 不用改接线. 1 = 把该电机的 I1/I2 逻辑交换, 让"正数=车往前"对它也成立.
  *
  * 【当前设置】A=0, B=1 —— 假设右电机(B)是反装的.
  *   如果实测发现反的是左轮, 把这两行改成  A=1, B=0 即可.
  * 判断方法: 上电跑测试序列, A-FWD 时左轮该往前转, B-FWD 时右轮该往前转. */
#define MOTOR_A_DIR_INVERT       (0)
#define MOTOR_B_DIR_INVERT       (1)

/** 输出轴转一圈的计数值 = PPR x 减速比 x 4 (STM32 编码器模式自带 4 倍频)
  * 默认 500 x 30 x 4 = 60000 */
#define MOTOR_ENC_COUNTS_PER_REV ((int32_t)MOTOR_ENC_PPR * (int32_t)MOTOR_GEAR_RATIO * 4)

/** 与 CubeMX 里 TIM8 的 Counter Period 保持一致 (当前 799, 16 MHz 时钟 -> 20 kHz)
  * 两路电机共用 TIM8, 所以频率必然相同 */
#define MOTOR_PWM_ARR            (799)
#define MOTOR_PWM_PERIOD         ((int32_t)MOTOR_PWM_ARR + 1)

/* 测试流程参数 --------------------------------------------------------------*/

/** 1 = 上电后自动跑测试序列; 0 = 只初始化, 自己调 Motor_SetDuty()
  * 注意: 现在循迹控制环 (line_follow.c) 也在驱动电机, 两个一起开就会互相抢,
  *       所以默认为 0. 要单独测电机接线时改成 1, 并把 LF_AUTO_START 改成 0 */
#define MOTOR_TEST_AUTO_RUN      (0)

/** 串口打印周期 (ms) */
#define MOTOR_TEST_SAMPLE_MS     (100)

/** 测试序列里的占空比 (%), 第一次试转建议不超过 30 */
#define MOTOR_TEST_DUTY_PERCENT  (30)

/* 对外接口 ------------------------------------------------------------------*/

/** 初始化两路: 启动 PWM + 编码器, 全部停车零占空比 (上电调用一次) */
void Motor_Init(void);

/** 设置某个电机的占空比
  * @param  motor   MOTOR_A / MOTOR_B
  * @param  percent -100 ~ +100 (%), 正数正转, 负数反转, 0 = 滑行停车 */
void Motor_SetDuty(uint8_t motor, int16_t percent);

/** 滑行停车 */
void Motor_Stop(uint8_t motor);

/** 短路刹车 */
void Motor_Brake(uint8_t motor);

/** 当前占空比 (%) */
int16_t Motor_GetDuty(uint8_t motor);

/** 取走并清零本周期累加的编码器增量 (遥测帧每周期调用一次; 超出 int16 会限幅) */
int16_t Motor_TakeEncoderDelta(uint8_t motor);

/** 当前测试阶段名 (MANUAL = 未跑自动序列) */
const char *Motor_GetStepName(void);

/** 取上次调用以来的编码器计数增量 (int16 差值, 自动处理 16 位溢出) */
int16_t Motor_GetEncoderDelta(uint8_t motor);

/** 上电以来的累计计数 */
int32_t Motor_GetEncoderTotal(uint8_t motor);

/** 清零计数器与累计值 */
void Motor_ResetEncoder(uint8_t motor);

/** 主循环周期调用: 自动测试序列 + 定时打印 (非阻塞) */
void Motor_TestTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TEST_H__ */
