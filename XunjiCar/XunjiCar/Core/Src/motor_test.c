/**
  ******************************************************************************
  * @file    motor_test.c
  * @brief   双电机接线测试 (实现)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_test.h"
#include "tim.h"
#include "usart.h"

/* Private types -------------------------------------------------------------*/

/** 测试序列的一个阶段 */
typedef struct
{
  int16_t     duty_a;       /**< 电机 A 占空比 %, 带符号 */
  int16_t     duty_b;       /**< 电机 B 占空比 %, 带符号 */
  uint16_t    duration_ms;  /**< 持续时长 */
  const char *name;         /**< 串口里显示的名字 */
} Motor_TestStep_t;

/** 单个电机的资源与状态 */
typedef struct
{
  TIM_HandleTypeDef *htim_enc;     /**< 编码器定时器 (TIM3 / TIM4) */
  TIM_HandleTypeDef *htim_pwm;     /**< PWM 定时器 (两路都是 TIM8) */
  uint32_t           pwm_channel;  /**< TIM_CHANNEL_1 / TIM_CHANNEL_2 */
  GPIO_TypeDef      *dir1_port;    /**< 方向脚 I1 */
  uint16_t           dir1_pin;
  GPIO_TypeDef      *dir2_port;    /**< 方向脚 I2 */
  uint16_t           dir2_pin;
  uint8_t            invert;       /**< 编码器计数方向取反 */
  uint8_t            dir_invert;   /**< 电机转向取反 (1 = 这个电机装反了) */
  int16_t            duty;         /**< 当前占空比 */
  int16_t            last_count;   /**< 上次读取的计数 */
  int32_t            total;        /**< 累计计数 */
  int32_t            accum;        /**< 本打印周期内累加的计数 */
} Motor_t;

/* Private variables ---------------------------------------------------------*/

/** 两路电机的资源绑定 (硬件改动时只改这里) */
static Motor_t s_motors[MOTOR_COUNT] =
{
  {
    .htim_enc    = &htim3,
    .htim_pwm    = &htim8,
    .pwm_channel = TIM_CHANNEL_1,
    .dir1_port   = AIN1_GPIO_Port,
    .dir1_pin    = AIN1_Pin,
    .dir2_port   = AIN2_GPIO_Port,
    .dir2_pin    = AIN2_Pin,
    .invert      = MOTOR_A_ENC_INVERT,
    .dir_invert  = MOTOR_A_DIR_INVERT
  },
  {
    .htim_enc    = &htim4,
    .htim_pwm    = &htim8,
    .pwm_channel = TIM_CHANNEL_2,
    .dir1_port   = BIN1_GPIO_Port,
    .dir1_pin    = BIN1_Pin,
    .dir2_port   = BIN2_GPIO_Port,
    .dir2_pin    = BIN2_Pin,
    .invert      = MOTOR_B_ENC_INVERT,
    .dir_invert  = MOTOR_B_DIR_INVERT
  }
};

#if (MOTOR_TEST_AUTO_RUN == 1)
/** 自动测试序列: 先各自单独转(确认方向和编码器), 再一起转
  * WAIT 3 s 是上电安全等待, 循环时跳过 */
static const Motor_TestStep_t s_steps[] =
{
  { 0,                          0,                          3000u, "WAIT"   },
  { 0,                          0,                          1000u, "STOP"   },
  { MOTOR_TEST_DUTY_PERCENT,    0,                          2000u, "A-FWD"  },
  { 0,                          0,                          1000u, "STOP"   },
  { -MOTOR_TEST_DUTY_PERCENT,   0,                          2000u, "A-REV"  },
  { 0,                          0,                          1000u, "STOP"   },
  { 0,                          MOTOR_TEST_DUTY_PERCENT,    2000u, "B-FWD"  },
  { 0,                          0,                          1000u, "STOP"   },
  { 0,                          -MOTOR_TEST_DUTY_PERCENT,   2000u, "B-REV"  },
  { 0,                          0,                          1000u, "STOP"   },
  { MOTOR_TEST_DUTY_PERCENT,    MOTOR_TEST_DUTY_PERCENT,    2000u, "AB-FWD" },
  { 0,                          0,                          1000u, "STOP"   },
  { -MOTOR_TEST_DUTY_PERCENT,   -MOTOR_TEST_DUTY_PERCENT,   2000u, "AB-REV" }
};
#define MOTOR_TEST_STEP_COUNT (sizeof(s_steps) / sizeof(s_steps[0]))

static uint8_t  s_step_index    = 0u;
static uint32_t s_step_start_ms = 0u;
#endif

/* Private function prototypes -----------------------------------------------*/
/* (打印已统一由 telemetry 模块负责, 本模块只提供数据) */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化两路电机: 启动 PWM 与编码器, 全部停在零占空比
  *         (方向脚已在 MX_GPIO_Init 里配好; STBY 硬件拉高, 这里不管)
  */
void Motor_Init(void)
{
  uint8_t i;

  for (i = 0u; i < MOTOR_COUNT; i++)
  {
    Motor_t *m = &s_motors[i];

    m->duty = 0;
    m->total = 0;
    m->accum = 0;

    /* 先确认方向脚为 0、PWM 为 0, 再启动输出 */
    Motor_Stop(i);
    (void)HAL_TIM_PWM_Start(m->htim_pwm, m->pwm_channel);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0u);

    /* 编码器: TI12 模式, 硬件 4 倍频计数 */
    (void)HAL_TIM_Encoder_Start(m->htim_enc, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(m->htim_enc, 0u);
    m->last_count = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
  }

#if (MOTOR_TEST_AUTO_RUN == 1)
  s_step_index    = 0u;
  s_step_start_ms = HAL_GetTick();
#endif
}

/**
  * @brief  设置某个电机的占空比
  * @param  motor   MOTOR_A / MOTOR_B
  * @param  percent -100 ~ +100 (%), 正数正转, 负数反转, 0 = 滑行停车
  */
void Motor_SetDuty(uint8_t motor, int16_t percent)
{
  Motor_t *m;
  int16_t  mag;
  uint32_t pulse;
  uint8_t  forward;
  GPIO_PinState d1;
  GPIO_PinState d2;

  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  m = &s_motors[motor];

  if (percent > 100)
  {
    percent = 100;
  }
  else if (percent < -100)
  {
    percent = -100;
  }

  m->duty = percent;
  mag = (percent < 0) ? (int16_t)(-percent) : percent;

  if (percent == 0)
  {
    d1 = GPIO_PIN_RESET;          /* I1=I2=0 -> 滑行 */
    d2 = GPIO_PIN_RESET;
  }
  else
  {
    forward = (percent > 0) ? 1u : 0u;

    /* 电机镜像安装时, 用 MOTOR_x_DIR_INVERT 把 I1/I2 交换一下,
       这样"正数 = 车往前"对两个轮子都成立, 不用改接线 */
    if (m->dir_invert != 0u)
    {
      forward = (forward != 0u) ? 0u : 1u;
    }

    if (forward != 0u)
    {
      d1 = GPIO_PIN_SET;          /* I1=1 I2=0 -> 正转 */
      d2 = GPIO_PIN_RESET;
    }
    else
    {
      d1 = GPIO_PIN_RESET;        /* I1=0 I2=1 -> 反转 */
      d2 = GPIO_PIN_SET;
    }
  }

  HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, d1);
  HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, d2);

  pulse = (uint32_t)(((int32_t)mag * MOTOR_PWM_PERIOD) / 100);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, pulse);
}

/**
  * @brief  滑行停车
  */
void Motor_Stop(uint8_t motor)
{
  Motor_t *m;

  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  m = &s_motors[motor];

  m->duty = 0;
  HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0u);
}

/**
  * @brief  短路刹车 (需要快速停住时用)
  */
void Motor_Brake(uint8_t motor)
{
  Motor_t *m;

  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  m = &s_motors[motor];

  m->duty = 0;
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0u);
  HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_SET);
}

/**
  * @brief  取某个电机上次调用以来的编码器计数增量
  *         (用 int16_t 相减, 计数器在 0/65535 之间翻转也能算对)
  */
int16_t Motor_GetEncoderDelta(uint8_t motor)
{
  Motor_t *m;
  int16_t  now;
  int16_t  delta;

  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  m = &s_motors[motor];

  now = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
  delta = (int16_t)(now - m->last_count);
  m->last_count = now;

  if (m->invert != 0u)
  {
    delta = (int16_t)(-delta);
  }

  m->total += (int32_t)delta;
  return delta;
}

/**
  * @brief  上电以来的累计计数
  */
int32_t Motor_GetEncoderTotal(uint8_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }

  return s_motors[motor].total;
}

/**
  * @brief  清零计数器与累计值
  */
void Motor_ResetEncoder(uint8_t motor)
{
  Motor_t *m;

  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  m = &s_motors[motor];

  __HAL_TIM_SET_COUNTER(m->htim_enc, 0u);
  m->last_count = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
  m->total = 0;
  m->accum = 0;
}

/**
  * @brief  主循环周期调用: 自动测试序列 + 定时打印 (全部非阻塞)
  */
void Motor_TestTask(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  i;

  /* ---- 每次调用都采一次编码器并累加 ----
     500 线 GMR + 4 倍频后一圈 60000 计数, 若等到 100 ms 才取一次差值,
     高速时增量会超出 int16 范围导致算错; 所以每 10 ms 取一次再累加 */
  for (i = 0u; i < MOTOR_COUNT; i++)
  {
    s_motors[i].accum += (int32_t)Motor_GetEncoderDelta(i);
  }

#if (MOTOR_TEST_AUTO_RUN == 1)
  /* ---- 测试序列状态机 ---- */
  if ((now - s_step_start_ms) >= (uint32_t)s_steps[s_step_index].duration_ms)
  {
    if (s_step_index < (MOTOR_TEST_STEP_COUNT - 1u))
    {
      s_step_index++;
    }
    else
    {
      s_step_index = 1u;   /* 循环时跳过开头的 3 s 等待 */
    }

    s_step_start_ms = now;
    Motor_SetDuty(MOTOR_A, s_steps[s_step_index].duty_a);
    Motor_SetDuty(MOTOR_B, s_steps[s_step_index].duty_b);
  }
#endif
}

/* 供遥测帧/控制环读取的状态 ------------------------------------------------*/

/**
  * @brief  当前占空比 (%)
  */
int16_t Motor_GetDuty(uint8_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }

  return s_motors[motor].duty;
}

/**
  * @brief  取走并清零"本周期内累加的编码器计数"
  *         遥测帧每个周期调用一次, 所以累加值不会越积越大
  */
int16_t Motor_TakeEncoderDelta(uint8_t motor)
{
  int32_t acc;

  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }

  acc = s_motors[motor].accum;
  s_motors[motor].accum = 0;

  /* 上限保护: 正常 100 ms 周期内不可能超出 int16 范围 */
  if (acc > 32767)
  {
    acc = 32767;
  }
  else if (acc < -32768)
  {
    acc = -32768;
  }

  return (int16_t)acc;
}

/**
  * @brief  当前测试阶段名 (MANUAL = 未跑自动序列)
  */
const char *Motor_GetStepName(void)
{
#if (MOTOR_TEST_AUTO_RUN == 1)
  return s_steps[s_step_index].name;
#else
  return "MANUAL";
#endif
}
