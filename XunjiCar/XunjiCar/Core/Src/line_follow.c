/**
  ******************************************************************************
  * @file    line_follow.c
  * @brief   循迹控制环 + 激光测距避障 (实现)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "line_follow.h"
#include "track_uart.h"
#include "telemetry.h"
#include "motor_test.h"
#include "mpu6050.h"

/* Private defines -----------------------------------------------------------*/

/** 探头权重 (x1 在最左): 满偏 35, 与 LF_POS_FULL_SCALE 配套 */
#define LF_POS_FULL_SCALE        (35)

/* Private variables ---------------------------------------------------------*/

static const int8_t s_weight[8] =
{
  -35, -25, -15, -5, 5, 15, 25, 35
};

static LineFollow_State_t s_state       = LF_ST_IDLE;
static uint8_t            s_enabled     = 0u;
static uint32_t           s_init_ms     = 0u;
static uint32_t           s_last_loop_ms = 0u;
static uint32_t           s_lost_ms     = 0u;
static uint32_t           s_lost_count  = 0u;

static int32_t            s_last_pos    = 0;   /**< 上一次的位置 */
static int32_t            s_i_term      = 0;   /**< 积分累加 */
static int16_t            s_pos         = 0;   /**< 本周期位置 */
static int16_t            s_steer       = 0;   /**< 本周期差速 */
static int16_t            s_duty[2]     = { 0, 0 };

/* MPU6050 陀螺仪 (转向阻尼 + 丢线保持航向) */
static uint8_t            s_imu_ok      = 0u;  /**< 本周期陀螺仪数据是否有效 */
static int16_t            s_gyro_dps10  = 0;   /**< 本周期 Z 轴角速度 (dps×10) */
static int32_t            s_yaw_x10     = 0;   /**< 丢线后累计转角 (度×10) */

/* Private function prototypes -----------------------------------------------*/
static int32_t lf_position(uint8_t black, uint8_t *count);
static void    lf_set_duty(int16_t left, int16_t right);
static void    lf_stop(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化
  */
void LineFollow_Init(void)
{
  s_state        = LF_ST_IDLE;
  s_enabled      = 0u;
  s_init_ms      = HAL_GetTick();
  s_last_loop_ms = s_init_ms;
  s_lost_ms      = s_init_ms;
  s_lost_count   = 0u;
  s_last_pos     = 0;
  s_i_term       = 0;
  s_pos          = 0;
  s_steer        = 0;
  s_duty[0]      = 0;
  s_duty[1]      = 0;

  lf_stop();
}

/**
  * @brief  开 / 关循迹 (关闭时立刻停车)
  */
void LineFollow_SetEnable(uint8_t enable)
{
  if (enable != 0u)
  {
    s_enabled = 1u;
    s_state   = LF_ST_WAIT;
    s_i_term  = 0;
    s_lost_ms = HAL_GetTick();
  }
  else
  {
    s_enabled = 0u;
    s_state   = LF_ST_IDLE;
    lf_stop();
  }
}

uint8_t LineFollow_IsEnabled(void)
{
  return s_enabled;
}

/**
  * @brief  主循环周期调用 (非阻塞)
  */
void LineFollow_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  raw = 0xFFu;
  uint8_t  black;
  uint8_t  n_black = 0u;
  int32_t  pos;
  int32_t  dpos;
  int32_t  steer;
  uint32_t dt;
#if ((LF_ENABLE_TOF_LASER == 1u) && (LF_STOP_DISTANCE_MM > 0u))
  uint16_t mm = 0u;
  uint8_t  blocked = 0u;
#endif

  /* ---- 1. 自动启动延时 ---- */
  if (s_enabled == 0u)
  {
#if (LF_AUTO_START == 1)
    if ((now - s_init_ms) >= LF_START_DELAY_MS)
    {
      s_enabled = 1u;
      s_state   = LF_ST_WAIT;
      s_lost_ms = now;
    }
#endif
    if (s_enabled == 0u)
    {
      return;   /* 没使能: 什么都不做 (电机已停) */
    }
  }

  /* ---- 2. 限速到控制周期, 并算出实际间隔 dt ---- */
  dt = now - s_last_loop_ms;

  if (dt < LF_LOOP_PERIOD_MS)
  {
    return;
  }
  s_last_loop_ms = now;

  if (dt > 50u)
  {
    dt = 50u;    /* 卡顿保护: 别让积分项吃进一个很大的 dt */
  }

  /* ---- 2.5 读陀螺仪 (Z 轴角速度) ---- */
#if (LF_USE_IMU == 1u)
  {
    int16_t gz_raw = 0;

    s_imu_ok = MPU6050_GetGyro(NULL, NULL, &gz_raw);

    if (s_imu_ok != 0u)
    {
      /* ±2000 dps 量程: 16.4 计数 = 1 dps, 这里换算成 dps×10 (整数运算) */
      s_gyro_dps10 = (int16_t)(((int32_t)gz_raw * 100) / 164);
    }
    else
    {
      s_gyro_dps10 = 0;
    }
  }
#else
  s_imu_ok     = 0u;
  s_gyro_dps10 = 0;
#endif

  /* ---- 3. 激光测距避障 (优先级最高) ----
     两道门都过了才做避障: LF_ENABLE_TOF_LASER 是激光测距的总开关(默认 0 = 关),
     LF_STOP_DISTANCE_MM 是"多远算障碍"的阈值. 关掉时这一整段连同下面的变量都不编译,
     遥测帧里也不会有 D1/D2 字段 (telemetry.c 用同一个宏控制) */
#if ((LF_ENABLE_TOF_LASER == 1u) && (LF_STOP_DISTANCE_MM > 0u))
  {
    uint16_t d[2];
    uint8_t  i;
    uint8_t  have = 0u;

    /* i 就是帧里的路号: 0 -> D1 (I2C1), 1 -> D2 (I2C3);
       哪一路在车的左边见 line_follow.h 的 LF_TOF_LEFT / LF_TOF_RIGHT.
       下面 mm 只留"最近的那一路", 所以左右叫错也不影响这里的避障行为 */
    for (i = 0u; i < 2u; i++)
    {
#if (LF_USE_FRONT_TOF == 2u)
      if (Telemetry_GetDistanceMm(i, &d[i]) != 0u)
      {
        if (have == 0u) { mm = d[i]; }
        else if (d[i] < mm) { mm = d[i]; }
        have = 1u;
      }
#else
      if (i == (uint8_t)LF_USE_FRONT_TOF)
      {
        if (Telemetry_GetDistanceMm(i, &d[i]) != 0u) { mm = d[i]; have = 1u; }
      }
#endif
    }

    if (have != 0u)
    {
      if (s_state == LF_ST_BLOCKED)
      {
        blocked = (mm < LF_RESUME_DISTANCE_MM) ? 1u : 0u;   /* 迟滞 */
      }
      else
      {
        blocked = (mm < LF_STOP_DISTANCE_MM) ? 1u : 0u;
      }
    }
  }

  if (blocked != 0u)
  {
    if (s_state != LF_ST_BLOCKED)
    {
      s_state = LF_ST_BLOCKED;
    }
    lf_stop();
    return;
  }

  if (s_state == LF_ST_BLOCKED)
  {
    s_state   = LF_ST_WAIT;    /* 障碍走了, 重新开始 */
    s_lost_ms = now;
  }
#endif

  /* ---- 4. 取循迹数据 ---- */
  if (Track_GetState(&raw) == 0u)
  {
    /* 模块没数据: 停车, 但保持使能, 等它回来 */
    lf_stop();
    s_state = LF_ST_WAIT;
    return;
  }

  black = (LF_BLACK_IS_ZERO != 0) ? (uint8_t)(~raw) : raw;

  pos = lf_position(black, &n_black);

  /* ---- 5. 丢线 / 全黑 判定 ---- */
  if (n_black == 0u)
  {
    /* 全白 = 这一刻看不到线 */
    if ((s_state != LF_ST_LOST) && (s_state != LF_ST_COAST))
    {
      s_lost_ms = now;    /* 刚丢, 开始计时 */
      s_lost_count++;
    }

    if ((now - s_lost_ms) < LF_LOST_TIMEOUT_MS)
    {
      s_state = LF_ST_COAST;
      pos = s_last_pos;          /* 容忍时间内: 按上次的位置继续修正 */
    }
    else
    {
#if (LF_LOST_ACTION == 0)
      s_state = LF_ST_LOST;
      lf_stop();                 /* 方案 A: 停车等人工干预 */
      return;
#elif (LF_LOST_ACTION == 1)
      s_state = LF_ST_LOST;
      /* 方案 B: 朝线最后出现的方向原地慢慢转, 找回来 */
      if (s_last_pos <= 0)
      {
        lf_set_duty((int16_t)(-LF_LOST_SEARCH_DUTY / 3), (int16_t)LF_LOST_SEARCH_DUTY);
      }
      else
      {
        lf_set_duty((int16_t)LF_LOST_SEARCH_DUTY, (int16_t)(-LF_LOST_SEARCH_DUTY / 3));
      }
      s_steer = 0;
      return;
#else
      /* 方案 C: 用陀螺仪保持航向, 沿丢线前的方向直行找线 */
      if (s_state != LF_ST_LOST)
      {
        s_state   = LF_ST_LOST;
        s_yaw_x10 = 0;           /* 从这一刻的航向开始算偏了多少 */
      }

      if ((now - s_lost_ms) > (LF_LOST_TIMEOUT_MS + LF_LOST_HOLD_MS) || (s_imu_ok == 0u))
      {
        lf_stop();               /* 找线时间用完, 或者陀螺仪不可用: 停车 */
        return;
      }

      /* 积分求航向角: 度×10 += (dps×10) × ms / 1000 */
      s_yaw_x10 += ((int32_t)s_gyro_dps10 * (int32_t)dt) / 1000;

      steer = -((int32_t)LF_KHOLD * s_yaw_x10) / 100;   /* 偏了就往回修 */

      if (steer > LF_STEER_LIMIT)       { steer = LF_STEER_LIMIT; }
      else if (steer < -LF_STEER_LIMIT) { steer = -LF_STEER_LIMIT; }

      s_pos   = 0;
      s_steer = (int16_t)steer;
      lf_set_duty((int16_t)(LF_LOST_SEARCH_DUTY + steer),
                  (int16_t)(LF_LOST_SEARCH_DUTY - steer));
      return;
#endif
    }
  }
  else if (n_black >= 8u)
  {
    /* 八路全黑: 横线 / 十字 / 全黑地面 */
#if (LF_ALL_BLACK_STRAIGHT == 1)
    pos = 0;                     /* 直行冲过去 */
#else
    pos = s_last_pos;
#endif
    s_state = LF_ST_RUN;
  }
  else
  {
    s_state = LF_ST_RUN;
  }

  /* ---- 6. PD 计算差速 ---- */
  dpos = pos - s_last_pos;

  if (LF_KI != 0)
  {
    s_i_term += pos;
    if (s_i_term > 20000)       { s_i_term = 20000; }    /* 抗积分饱和 */
    else if (s_i_term < -20000) { s_i_term = -20000; }
  }
  else
  {
    s_i_term = 0;
  }

  steer = ((int32_t)LF_KP * pos + (int32_t)LF_KD * dpos) / 1000;
  steer += ((int32_t)LF_KI * s_i_term) / 100000;

  /* ---- 6.5 陀螺仪转向阻尼 ----
     车正在往左转(gz>0)时, 说明它自己正在转过头, 这里给一个正的差速(往右修)
     把它压住. 这一项是"实测转动速度", 比用位置的差分(KD)更直接、更少滞后.

     ★ 急弯衰减 (为什么要有这个):
       陀螺仪的职责是"不许车随便转", 而急弯要求的恰恰是"必须转过去", 两者天生对立.
       更麻烦的是这一项加在限幅之前: 急弯时车转得快(300 dps × KGYRO18 = 54% 差速),
       它会把总指令顶进 LF_STEER_LIMIT 的饱和区, 拿 70 的上限算:
         不衰减 -> 净转向能力只剩 70-54 = 16, 90 度急弯过不去
         衰减到 55% -> 70-29.7 = 40, 够用了
       所以急弯必须给阻尼打折, 但打折的范围要卡准(见下), 否则直道防摆会被一起削掉.

     ★ 打折范围 = 350 (关键):
       8 路探头的归一化偏差是 ±1000, "一个探头的间距"约 100.
       偏差 350 意味着线已经跑到第 4 个探头以外 —— 正常走线(含直道摆动)根本到不了,
       只有真急弯才会超过. 所以在 350 以内阻尼保持 100% 不变:
         偏差 0~350  -> 100% (直道和缓弯的防摆和以前一模一样)
         偏差 350~1000 -> 从 100% 线性降到 55% (只有急弯才让路)
       上一版把起点定在偏差 0, 结果直道摆动刚起来(|pos|≈100~200)阻尼就被削到 70~80%,
       直道又开始摆 —— 这就是那次的教训, 别再把起点往前移. */
#if (LF_USE_IMU == 1u)
  if (s_imu_ok != 0u)
  {
    const int32_t taper_lo = 350;   /* 偏差小于它: 完全不衰减 */
    const int32_t taper_hi = 1000;  /* 偏差到它: 降到最低 */
    int32_t gyro_scale = 100;       /* 百分比: 100 = 不衰减 */

    if ((pos > taper_lo) || (pos < -taper_lo))
    {
      int32_t over = (pos > 0) ? (pos - taper_lo) : ((-pos) - taper_lo);

      gyro_scale = 100 - ((over * 45) / (taper_hi - taper_lo));   /* 100% -> 55% */
    }

    steer += ((int32_t)(LF_KGYRO * LF_GYRO_SIGN) * (int32_t)s_gyro_dps10
              * gyro_scale) / 100000;
  }
#endif

  if (steer > LF_STEER_LIMIT)       { steer = LF_STEER_LIMIT; }
  else if (steer < -LF_STEER_LIMIT) { steer = -LF_STEER_LIMIT; }

  s_pos   = (int16_t)pos;
  s_steer = (int16_t)steer;
  s_last_pos = pos;

  /* ---- 7. 输出 ----
     急弯自动降速: 偏差越大把基础速度压得越低, 但只压"速度", 不压"差速" ——
     基础速度低下来以后, 同样的差速上限能给出更大的速度差, 内轮也更容易被拉成倒转,
     这才是 90° 急弯真正缺的东西 (外轮顶到 100% 之后, 光加 LF_STEER_LIMIT 是没用的)。
     直道和缓弯的偏差到不了 LF_CORNER_SLOW_POS, 所以那里速度保持 LF_BASE_DUTY 不变。 */
  {
    int32_t base = LF_BASE_DUTY;
    int32_t over = 0;

    if (pos > LF_CORNER_SLOW_POS)        { over = pos - LF_CORNER_SLOW_POS; }
    else if (pos < -LF_CORNER_SLOW_POS)  { over = (-pos) - LF_CORNER_SLOW_POS; }

    if (over > 0)
    {
      const int32_t span = 900 - LF_CORNER_SLOW_POS;   /* 降满时对应的偏差 */

      if (over > span) { over = span; }

      base -= (over * LF_CORNER_SPEED_DROP) / span;
    }

    lf_set_duty((int16_t)(base + steer), (int16_t)(base - steer));
  }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  由 8 位黑线掩码算出归一化位置
  * @param  black   bit7~bit0 = x1~x8, 1 = 该探头压到黑线
  * @param  count   输出: 压线的探头个数
  * @retval -1000 ~ +1000 (负数偏左, 正数偏右)
  */
static int32_t lf_position(uint8_t black, uint8_t *count)
{
  int32_t sum = 0;
  uint8_t n   = 0u;
  uint8_t i;

  for (i = 0u; i < 8u; i++)
  {
    if ((black & (uint8_t)(1u << (7u - i))) != 0u)
    {
      sum += (int32_t)s_weight[i];
      n++;
    }
  }

  if (count != NULL)
  {
    *count = n;
  }

  if (n == 0u)
  {
    return 0;
  }

  return (sum * 1000) / ((int32_t)n * LF_POS_FULL_SCALE);
}

/** 设置两轮占空比 (自动限幅 + 记录) */
static void lf_set_duty(int16_t left, int16_t right)
{
  if (left > 100)        { left = 100; }
  else if (left < -100)  { left = -100; }
  if (right > 100)       { right = 100; }
  else if (right < -100) { right = -100; }

  s_duty[MOTOR_A] = left;
  s_duty[MOTOR_B] = right;

  Motor_SetDuty(MOTOR_A, left);
  Motor_SetDuty(MOTOR_B, right);
}

/** 停车 (滑行) 并清掉转向显示 */
static void lf_stop(void)
{
  s_duty[MOTOR_A] = 0;
  s_duty[MOTOR_B] = 0;
  s_steer = 0;
  s_i_term = 0;
  Motor_Stop(MOTOR_A);
  Motor_Stop(MOTOR_B);
}

/* 给遥测帧用的接口 ---------------------------------------------------------*/

const char *LineFollow_GetStateName(void)
{
  switch (s_state)
  {
    case LF_ST_WAIT:    return "WAIT";
    case LF_ST_RUN:     return "RUN";
    case LF_ST_COAST:   return "COAST";
    case LF_ST_LOST:    return "LOST";
    case LF_ST_BLOCKED: return "BLOCK";
    default:            return "IDLE";
  }
}

int16_t LineFollow_GetPosition(void) { return s_pos; }
int16_t LineFollow_GetSteer(void)    { return s_steer; }
uint32_t LineFollow_GetLostCount(void) { return s_lost_count; }
int16_t LineFollow_GetGyroDps10(void) { return s_gyro_dps10; }
int16_t LineFollow_GetYawX10(void) { return (int16_t)s_yaw_x10; }
uint8_t LineFollow_IsImuOk(void) { return s_imu_ok; }

int16_t LineFollow_GetDuty(uint8_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return s_duty[motor];
}
