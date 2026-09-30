/**
  ******************************************************************************
  * @file    track_uart.c
  * @brief   八路巡线模块 —— 串口(USART1)版驱动 (实现)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "track_uart.h"
#include "usart.h"

/* Private defines -----------------------------------------------------------*/

/** "$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#" 的长度 = 43 */
#define TRACK_FRAME_MIN_LEN   (43u)

/** 组帧缓冲上限 (正常帧 43 字节, 留余量) */
#define TRACK_FRAME_MAX       (64u)

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_rx_byte;                        /**< 中断接收的单字节落点 */
static uint8_t  s_buf[TRACK_FRAME_MAX];           /**< 组帧缓冲 */
static uint16_t s_idx;                            /**< 组帧写指针 */
static uint8_t  s_in_frame;                       /**< 1 = 正在收一帧 */
static uint8_t  s_raw;                            /**< 最新解析出的 8 位状态 */
static uint32_t s_last_frame_ms;                  /**< 最近一次收到好帧的时刻 */
static uint32_t s_frame_count;                    /**< 好帧计数 */
static uint32_t s_error_count;                    /**< 坏帧计数 */
static uint32_t s_cfg_last_ms;                    /**< 上次发配置命令的时刻 */
static uint8_t  s_cfg_retry;                      /**< 配置命令已重发次数 */

/* Private function prototypes -----------------------------------------------*/
static void    track_send_config(void);
static void    track_feed(uint8_t b);
static uint8_t track_parse(const uint8_t *buf, uint16_t len);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化: 发配置命令(数字模式) + 启动 USART1 中断接收
  */
void Track_Init(void)
{
  s_idx           = 0u;
  s_in_frame      = 0u;
  s_raw           = 0xFFu;
  s_frame_count   = 0u;
  s_error_count   = 0u;
  s_cfg_retry     = 0u;
  s_last_frame_ms = HAL_GetTick();

  track_send_config();
  s_cfg_last_ms = HAL_GetTick();

  /* 逐字节中断接收; 每收完 1 字节进一次 HAL_UART_RxCpltCallback, 在回调里重新装载 */
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

/**
  * @brief  主循环周期调用 (非阻塞)
  *         模块可能比单片机晚上电, 所以一直收不到帧就周期性重发配置命令
  */
void Track_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((s_frame_count == 0u) && (s_cfg_retry < TRACK_CFG_MAX_RETRY))
  {
    if ((now - s_cfg_last_ms) >= TRACK_CFG_RETRY_MS)
    {
      s_cfg_last_ms = now;
      s_cfg_retry++;
      track_send_config();
    }
  }
}

/**
  * @brief  取最新一帧的 8 路状态
  * @retval 1 = 有效
  */
uint8_t Track_GetState(uint8_t *raw)
{
  if (raw == NULL)
  {
    return 0u;
  }

  *raw = s_raw;

  if (s_frame_count == 0u)
  {
    return 0u;   /* 还没收到过任何帧 */
  }

  if ((HAL_GetTick() - s_last_frame_ms) > TRACK_FRAME_TIMEOUT_MS)
  {
    return 0u;   /* 超时没新数据 */
  }

  return 1u;
}

/**
  * @brief  进入 / 退出模块校准模式
  *         对应厂家命令 $<adjust>,0,1# : adjust = 1 进入校准, 0 退出校准
  */
void Track_SetAdjustMode(uint8_t enable)
{
  char cmd[8] = "$0,0,1#";

  cmd[1] = (enable != 0u) ? '1' : '0';
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)(sizeof(cmd) - 1u), 50u);
}

/**
  * @brief  累计好帧数
  */
uint32_t Track_GetFrameCount(void)
{
  return s_frame_count;
}

/**
  * @brief  累计坏帧数
  */
uint32_t Track_GetErrorCount(void)
{
  return s_error_count;
}

/* Private functions ---------------------------------------------------------*/

/** 发送配置命令: $0,0,1# = 只输出数字型数据 */
static void track_send_config(void)
{
  static const char cmd[] = "$0,0,1#";

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)(sizeof(cmd) - 1u), 50u);
}

/**
  * @brief  把中断收到的一个字节喂给组帧状态机
  *         帧格式: '$' ... '#' , 收到 '$' 开始, 收到 '#' 结束并解析
  */
static void track_feed(uint8_t b)
{
  if (b == (uint8_t)'$')
  {
    s_idx = 0u;
    s_in_frame = 1u;
  }
  else if (s_in_frame == 0u)
  {
    return;   /* 帧外的杂字节直接丢 */
  }
  else
  {
    /* 继续收 */
  }

  if (s_idx >= TRACK_FRAME_MAX)
  {
    /* 太长: 丢弃这一帧 */
    s_in_frame = 0u;
    s_idx = 0u;
    s_error_count++;
    return;
  }

  s_buf[s_idx] = b;
  s_idx++;

  if (b == (uint8_t)'#')
  {
    s_in_frame = 0u;

    if (track_parse(s_buf, s_idx) != 0u)
    {
      s_frame_count++;
      s_last_frame_ms = HAL_GetTick();
      s_cfg_retry = TRACK_CFG_MAX_RETRY;   /* 收到数据了, 不再重发配置 */
    }
    else
    {
      s_error_count++;
    }

    s_idx = 0u;
  }
}

/**
  * @brief  解析一帧 $D,x1:0,...,x8:0#
  * @retval 1 = 解析成功(已更新 s_raw), 0 = 坏帧
  */
static uint8_t track_parse(const uint8_t *buf, uint16_t len)
{
  uint8_t raw = 0u;
  uint8_t i;

  if ((buf == NULL) || (len < TRACK_FRAME_MIN_LEN))
  {
    return 0u;
  }

  /* 只认数字型帧 (第 2 个字符为 'D') */
  if ((buf[0] != (uint8_t)'$') || (buf[1] != (uint8_t)'D'))
  {
    return 0u;
  }

  for (i = 0u; i < 8u; i++)
  {
    uint8_t v = buf[6u + ((uint16_t)i * 5u)];   /* 例程里的 6, 11, 16, 21, 26, 31, 36, 41 */

    if (v == (uint8_t)'1')
    {
      raw |= (uint8_t)(1u << (7u - i));         /* x1 -> bit7 ... x8 -> bit0 */
    }
    else if (v != (uint8_t)'0')
    {
      return 0u;                                /* 既不是 0 也不是 1, 判为坏帧 */
    }
    else
    {
      /* '0': 该位保持 0 */
    }
  }

#if (TRACK_DATA_WHITE_IS_ONE == 0u)
  raw = (uint8_t)(~raw);
#endif

  s_raw = raw;
  return 1u;
}

/* HAL 回调 ----------------------------------------------------------------*/

/**
  * @brief  串口接收完成回调 (HAL 弱函数, 这里覆盖)
  *         注意: 所有 UART 的 IT 接收都走这个回调, 所以要先判断是不是 USART1
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    track_feed(s_rx_byte);
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);   /* 重新装载, 否则只收一字节 */
  }
}

/**
  * @brief  串口错误回调
  *         溢出(ORE)/帧错误后 HAL 会中止接收, 不重新装载就永久收不到数据了
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    s_idx = 0u;
    s_in_frame = 0u;
    s_error_count++;
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
  }
}
