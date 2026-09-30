/**
  ******************************************************************************
  * @file    telemetry.c
  * @brief   统一遥测帧 (实现): 四类数据 -> 一条 ASCII 帧 -> USART2
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "telemetry.h"
#include "track_uart.h"
#include "vl53l0x.h"
#include "mpu6050.h"
#include "motor_test.h"
#include "line_follow.h"
#include "i2c.h"
#include "usart.h"

/* Private defines -----------------------------------------------------------*/

#if (TELEMETRY_TOF_COUNT < 1u) || (TELEMETRY_TOF_COUNT > TELEMETRY_TOF_MAX)
#error "TELEMETRY_TOF_COUNT must be between 1 and TELEMETRY_TOF_MAX"
#endif

#if (LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 0u)
#error "LF_ENABLE_TOF_LASER=1 需要 TELEMETRY_ENABLE_TOF=1 (避障用的距离来自遥测模块)"
#endif

/** 帧缓冲: 加上陀螺仪字段后最坏情况约 220 字节, 取 256 留足余量
  * 若将来新增字段, 记得同步放大这个值, 否则 Tx_Char 会静默截断 */
#define TELEMETRY_FRAME_BUF   (256u)

/* Private types -------------------------------------------------------------*/

/** 极简文本缓冲区, 避免引入 printf / 堆分配 */
typedef struct
{
  char    buf[TELEMETRY_FRAME_BUF];
  uint8_t len;
} Telemetry_TxBuf_t;

/* Private variables ---------------------------------------------------------*/
static const char s_hex_char[] = "0123456789ABCDEF";

static uint8_t  s_track_valid  = 0u;      /**< 最近一次循迹数据是否有效 */
static uint8_t  s_track_raw    = 0xFFu;   /**< 最近一次循迹原始字节 */
static uint16_t s_seq          = 0u;      /**< 帧序号 */
static uint32_t s_last_poll_ms = 0u;      /**< 上次采样时刻 */
static uint32_t s_last_send_ms = 0u;      /**< 上次发帧时刻 */

#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
/** 各路 TOF 挂在哪条总线上: [0] = I2C1(PB6/PB7), [1] = I2C3(PA8/PC9)
  * 下标就是帧里的路号 (0 -> D1, 1 -> D2), 顺序不能调换;
  * "哪一路装在车左边"是另一回事, 记在 line_follow.h 的 LF_TOF_LEFT / LF_TOF_RIGHT */
static I2C_HandleTypeDef *const s_tof_bus[TELEMETRY_TOF_MAX] = { &hi2c1, &hi2c3 };

static VL53L0X_t s_tof[TELEMETRY_TOF_MAX];        /**< 各路设备实例 */
static uint8_t   s_tof_ready[TELEMETRY_TOF_MAX];  /**< 各路初始化是否成功 */
static uint8_t   s_dist_valid[TELEMETRY_TOF_MAX]; /**< 各路最近一次距离是否有效 */
static uint16_t  s_dist_mm[TELEMETRY_TOF_MAX];    /**< 各路最近一次距离 (mm) */
#endif

/* Private function prototypes -----------------------------------------------*/
static void    Tx_Reset(Telemetry_TxBuf_t *t);
static void    Tx_Char(Telemetry_TxBuf_t *t, char c);
static void    Tx_Str(Telemetry_TxBuf_t *t, const char *s);
static void    Tx_Hex8(Telemetry_TxBuf_t *t, uint8_t v);
static void    Tx_U32(Telemetry_TxBuf_t *t, uint32_t v);
static void    Tx_I32(Telemetry_TxBuf_t *t, int32_t v);
static uint8_t Frame_Checksum(const char *buf, uint8_t len);
static void    Tx_Flush(const Telemetry_TxBuf_t *t);
static void    Telemetry_SendFrame(void);
static void    Telemetry_SampleSensors(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化: 启动巡线串口 + 逐路初始化 VL53L0X
  *         (MPU6050 / 电机由各自模块的 Init 负责)
  */
void Telemetry_Init(void)
{
  s_seq = 0u;

  /* 巡线模块: 发配置命令 + 启动 USART1 中断接收 */
  Track_Init();

#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
  {
    uint8_t i;

    for (i = 0u; i < TELEMETRY_TOF_COUNT; i++)
    {
      s_tof_ready[i]  = 0u;
      s_dist_valid[i] = 0u;
      s_dist_mm[i]    = 0u;

      VL53L0X_InitDevice(&s_tof[i], s_tof_bus[i]);

      if (VL53L0X_IsPresent(&s_tof[i]) == 0u)
      {
        continue;   /* 没应答: 跳过这一路 */
      }

      if (VL53L0X_Init(&s_tof[i]) == 0u)
      {
        continue;   /* 初始化时序失败: 跳过这一路 */
      }

      /* 0 = 背靠背模式(测量尽可能快); 改成 50 之类的值则进入定时模式 */
      VL53L0X_StartContinuous(&s_tof[i], 0u);
      s_tof_ready[i] = 1u;
    }
  }
#endif
}

/**
  * @brief  主循环周期调用: 非阻塞采样 + 定时发一帧
  */
void Telemetry_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_last_poll_ms) < TELEMETRY_TASK_PERIOD_MS)
  {
    return;
  }
  s_last_poll_ms = now;

  Track_Task();
  Telemetry_SampleSensors();

  if ((now - s_last_send_ms) >= TELEMETRY_SEND_PERIOD_MS)
  {
    s_last_send_ms = now;
    Telemetry_SendFrame();
  }
}

/**
  * @brief  取最近一次解析出的循迹原始字节
  */
uint8_t Telemetry_GetTrackByte(uint8_t *raw)
{
  if (raw == NULL)
  {
    return 0u;
  }

  *raw = s_track_raw;
  return s_track_valid;
}

/**
  * @brief  取某一路 TOF 距离 (mm)
  */
uint8_t Telemetry_GetDistanceMm(uint8_t index, uint16_t *mm)
{
  if ((mm == NULL) || (index >= TELEMETRY_TOF_MAX))
  {
    return 0u;
  }

#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
  *mm = s_dist_mm[index];
  return s_dist_valid[index];
#else
  /* 激光测距关着: 永远说"没有有效距离", 循迹控制环也就不会避障 */
  *mm = 0u;
  return 0u;
#endif
}

/**
  * @brief  指定那一路 VL53L0X 是否初始化成功
  */
uint8_t Telemetry_IsTofReady(uint8_t index)
{
  if (index >= TELEMETRY_TOF_MAX)
  {
    return 0u;
  }

#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
  return s_tof_ready[index];
#else
  return 0u;
#endif
}

/**
  * @brief  当前帧序号
  */
uint16_t Telemetry_GetFrameSeq(void)
{
  return s_seq;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  采样循迹与测距 (IMU/电机由各自 Task 采样, 这里只取结果)
  */
static void Telemetry_SampleSensors(void)
{
#if (TELEMETRY_ENABLE_TRACK == 1u)
  {
    uint8_t raw = 0u;

    if (Track_GetState(&raw) != 0u)
    {
      s_track_raw   = raw;
      s_track_valid = 1u;
    }
    else
    {
      s_track_valid = 0u;
    }
  }
#else
  s_track_valid = 0u;
#endif

#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
  {
    uint8_t i;

    for (i = 0u; i < TELEMETRY_TOF_COUNT; i++)
    {
      if (s_tof_ready[i] == 0u)
      {
        s_dist_valid[i] = 0u;
        continue;
      }

      {
        uint16_t range = 0u;

        if (VL53L0X_ReadRangeIfReady(&s_tof[i], &range) != 0u)
        {
          s_dist_mm[i]    = range;
          s_dist_valid[i] = ((range != 0u) && (range <= TELEMETRY_DIST_MAX_MM)) ? 1u : 0u;
        }
      }
    }
  }
#endif
}

/**
  * @brief  组装并发送一帧
  *         $CAR,N=..,TRK=..,B=..,D1=..,D2=..,AX=..,..,T=..,S=..,MA=..,EA=..,MB=..,EB=..*XX
  */
static void Telemetry_SendFrame(void)
{
  Telemetry_TxBuf_t t;
  uint8_t checksum;
  int     b;

  Tx_Reset(&t);

  /* ---- 帧头与序号 ---- */
  Tx_Char(&t, '$');
  Tx_Str(&t, TELEMETRY_FRAME_ID);
  Tx_Str(&t, ",N=");
  Tx_U32(&t, (uint32_t)s_seq);
  s_seq++;

  /* ---- 循迹 ---- */
#if (TELEMETRY_ENABLE_TRACK == 1u)
  if (s_track_valid != 0u)
  {
    Tx_Str(&t, ",TRK=");
    Tx_Hex8(&t, s_track_raw);
    Tx_Str(&t, ",B=");
    for (b = 7; b >= 0; --b)
    {
      /* bit7 ~ bit0 依次是 x1 ~ x8; 1 = 白底, 0 = 压到黑线 */
      Tx_Char(&t, ((s_track_raw >> b) & 0x01u) != 0u ? '1' : '0');
    }
  }
  else
  {
    Tx_Str(&t, ",TRK=ERR,B=--------");
  }
#endif

  /* ---- 双路测距 (关掉时整组字段都不发)
     路号 0 -> D1, 路号 1 -> D2, 顺序固定;
     上位机按"哪一路在左边"来叫名字, 依据 line_follow.h 的 LF_TOF_LEFT/LF_TOF_RIGHT */
#if ((LF_ENABLE_TOF_LASER == 1u) && (TELEMETRY_ENABLE_TOF == 1u))
  {
    uint8_t i;

    for (i = 0u; i < TELEMETRY_TOF_COUNT; i++)
    {
      Tx_Str(&t, ",D");
      Tx_Char(&t, (char)('1' + (char)i));
      Tx_Char(&t, '=');

      if (s_dist_valid[i] != 0u)
      {
        Tx_U32(&t, (uint32_t)s_dist_mm[i]);
      }
      else if (s_tof_ready[i] == 0u)
      {
        Tx_Str(&t, "NA");     /* 模块没应答/初始化失败 */
      }
      else
      {
        Tx_Str(&t, "---");    /* 模块正常, 但这次没有有效回波 */
      }
    }
  }
#endif

  /* ---- IMU ---- */
#if (TELEMETRY_ENABLE_IMU == 1u)
  {
    MPU6050_Raw_t r;

    if (MPU6050_GetRaw(&r) != 0u)
    {
      Tx_Str(&t, ",AX="); Tx_I32(&t, (int32_t)r.ax);
      Tx_Str(&t, ",AY="); Tx_I32(&t, (int32_t)r.ay);
      Tx_Str(&t, ",AZ="); Tx_I32(&t, (int32_t)r.az);
      Tx_Str(&t, ",GX="); Tx_I32(&t, (int32_t)r.gx);
      Tx_Str(&t, ",GY="); Tx_I32(&t, (int32_t)r.gy);
      Tx_Str(&t, ",GZ="); Tx_I32(&t, (int32_t)r.gz);
      Tx_Str(&t, ",T=");  Tx_I32(&t, (int32_t)(MPU6050_TempToC(r.temp)));
    }
    else
    {
      Tx_Str(&t, ",IMU=NA");
    }
  }
#endif

  /* ---- 电机 ---- */
#if (TELEMETRY_ENABLE_MOTOR == 1u)
  {
    uint8_t i;

    Tx_Str(&t, ",S=");
    Tx_Str(&t, Motor_GetStepName());

    for (i = 0u; i < MOTOR_COUNT; i++)
    {
      /* 取走本周期累加的编码器增量 (同时清零, 不会越积越多) */
      int16_t delta = Motor_TakeEncoderDelta(i);

      Tx_Str(&t, (i == MOTOR_A) ? ",MA=" : ",MB=");
      Tx_I32(&t, (int32_t)Motor_GetDuty(i));
      Tx_Str(&t, (i == MOTOR_A) ? ",EA=" : ",EB=");
      Tx_I32(&t, (int32_t)delta);
    }
  }
#endif

  /* ---- 循迹控制环 ---- */
#if (TELEMETRY_ENABLE_LINEFOLLOW == 1u)
  {
    Tx_Str(&t, ",LF=");
    Tx_Str(&t, LineFollow_GetStateName());
    Tx_Str(&t, ",ERR=");
    Tx_I32(&t, (int32_t)LineFollow_GetPosition());
    Tx_Str(&t, ",ST=");
    Tx_I32(&t, (int32_t)LineFollow_GetSteer());
    /* 陀螺仪: 角速度(dps×10) 和 丢线后累计转角(度×10)
       注意字段名不要和上面的原始值 GX/GY/GZ 撞名, 所以用 GZD */
    Tx_Str(&t, ",GZD=");
    Tx_I32(&t, (int32_t)LineFollow_GetGyroDps10());
    Tx_Str(&t, ",YAW=");
    Tx_I32(&t, (int32_t)LineFollow_GetYawX10());
    Tx_Str(&t, ",IMS=");
    Tx_Char(&t, (LineFollow_IsImuOk() != 0u) ? '1' : '0');
  }
#endif

  /* ---- 校验和: $ 与 * 之间所有字符异或 ---- */
  checksum = Frame_Checksum(t.buf, t.len);
  Tx_Char(&t, '*');
  Tx_Hex8(&t, checksum);
  Tx_Str(&t, "\r\n");
  Tx_Flush(&t);
}

/**
  * @brief  异或校验: 跳过开头的 '$'
  */
static uint8_t Frame_Checksum(const char *buf, uint8_t len)
{
  uint8_t x = 0u;
  uint8_t i;

  for (i = 1u; i < len; i++)
  {
    x ^= (uint8_t)buf[i];
  }

  return x;
}

static void Tx_Reset(Telemetry_TxBuf_t *t)
{
  t->len = 0u;
}

static void Tx_Char(Telemetry_TxBuf_t *t, char c)
{
  if (t->len < (uint8_t)(sizeof(t->buf) - 1u))
  {
    t->buf[t->len] = c;
    t->len++;
  }
}

static void Tx_Str(Telemetry_TxBuf_t *t, const char *s)
{
  while (*s != '\0')
  {
    Tx_Char(t, *s);
    s++;
  }
}

static void Tx_Hex8(Telemetry_TxBuf_t *t, uint8_t v)
{
  Tx_Char(t, s_hex_char[(v >> 4) & 0x0Fu]);
  Tx_Char(t, s_hex_char[v & 0x0Fu]);
}

static void Tx_U32(Telemetry_TxBuf_t *t, uint32_t v)
{
  char    tmp[10];
  uint8_t n = 0u;

  if (v == 0u)
  {
    Tx_Char(t, '0');
    return;
  }

  while ((v > 0u) && (n < (uint8_t)sizeof(tmp)))
  {
    tmp[n] = (char)('0' + (v % 10u));
    v /= 10u;
    n++;
  }

  while (n > 0u)
  {
    n--;
    Tx_Char(t, tmp[n]);
  }
}

/** 带符号整数 (正数带 '+', 一眼看出方向) */
static void Tx_I32(Telemetry_TxBuf_t *t, int32_t v)
{
  char     tmp[11];
  uint8_t  n = 0u;
  uint32_t mag;

  if (v < 0)
  {
    Tx_Char(t, '-');
    mag = (uint32_t)(-v);
  }
  else
  {
    Tx_Char(t, '+');
    mag = (uint32_t)v;
  }

  if (mag == 0u)
  {
    Tx_Char(t, '0');
    return;
  }

  while ((mag > 0u) && (n < (uint8_t)sizeof(tmp)))
  {
    tmp[n] = (char)('0' + (mag % 10u));
    mag /= 10u;
    n++;
  }

  while (n > 0u)
  {
    n--;
    Tx_Char(t, tmp[n]);
  }
}

static void Tx_Flush(const Telemetry_TxBuf_t *t)
{
  if (t->len > 0u)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)t->buf, t->len, TELEMETRY_UART_TIMEOUT_MS);
  }
}
