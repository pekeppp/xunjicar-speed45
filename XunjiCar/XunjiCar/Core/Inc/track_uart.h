/**
  ******************************************************************************
  * @file    track_uart.h
  * @brief   八路巡线模块 —— 串口(USART1)版驱动
  *
  * 接线:
  *   模块 TX  -> PA10 (USART1_RX)     模块 RX -> PA9 (USART1_TX, 发配置命令用)
  *   VCC 5V, GND 共地  (PA9/PA10 是 FT 引脚, 5V 电平可直连)
  *
  * 协议 (来自厂家 Arduino 串口例程 IReight_model.cpp):
  *   串口参数: 115200, 8 数据位, 无校验, 1 停止位
  *   配置命令(单片机->模块): $<adjust>,<ao>,<num>#
  *       例: $0,0,1#  只输出数字型数据
  *   数据帧(模块->单片机): $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
  *       第 i 个探头的数字固定在偏移 6+i*5 处 (例程原码: new_package[6+i*5]-'0')
  *
  * 接收方式: USART1 中断逐字节接收。
  *   115200 下一个字节只有 87 us, 主循环 10 ms 轮询必然丢数据, 所以只能用中断/DMA。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TRACK_UART_H__
#define __TRACK_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 可调参数 ------------------------------------------------------------------*/

/** 超过这么久没收到新帧, 就认为数据无效 (帧里会显示 TRK=ERR) */
#define TRACK_FRAME_TIMEOUT_MS   (200u)

/** 上电后一直收不到帧时, 重发配置命令的间隔 (ms) */
#define TRACK_CFG_RETRY_MS       (500u)

/** 配置命令最多重发几次 */
#define TRACK_CFG_MAX_RETRY      (20u)

/** 1 = 白底对应数字 1 (与原 I2C 版一致: 1=白底, 0=压到黑线)
  *   若实测极性相反, 改成 0 */
#define TRACK_DATA_WHITE_IS_ONE  (1u)

/** 上电后等待模块稳定的时间 (ms), main.c 里会用到 */
#define TRACK_POWER_ON_DELAY_MS  (1000u)

/* 对外接口 ------------------------------------------------------------------*/

/** 初始化: 发送配置命令并启动 USART1 中断接收 (上电调用一次) */
void Track_Init(void);

/** 主循环周期调用: 非阻塞, 负责"没收到数据时重发配置命令" */
void Track_Task(void);

/** 取最新一帧的 8 路状态
  * @param  raw bit7~bit0 依次对应 x1~x8; 1 = 白底, 0 = 压到黑线 (与原来 I2C 版相同)
  * @retval 1 = 数据有效(拿到过帧且未超时), 0 = 无数据/已超时 */
uint8_t Track_GetState(uint8_t *raw);

/** 进入(1) / 退出(0) 模块校准模式 (发 $1,0,1# / $0,0,1#)
  * 流程: 进入校准 -> 把模块在黑白线上左右来回移动 -> 退出校准 (结果存在模块内) */
void Track_SetAdjustMode(uint8_t enable);

/** 累计收到的好帧数 */
uint32_t Track_GetFrameCount(void);

/** 累计坏帧数 (格式不对/超长) */
uint32_t Track_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRACK_UART_H__ */
