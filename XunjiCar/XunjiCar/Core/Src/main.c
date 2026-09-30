/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "track_uart.h"
#include "telemetry.h"
#include "motor_test.h"
#include "mpu6050.h"
#include "line_follow.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** ===== 调试开关: 让轮子一直慢慢转 =====
  * 1 = 上电 MOTOR_SPIN_DELAY_MS 之后, 两个轮子一直以 MOTOR_SPIN_DUTY 往前慢转.
  *     完全绕过 循迹 / 激光避障 / 丢线判断, 只用来单独验证
  *     "单片机 -> TIM8 PWM -> TB6612 -> 电机 -> 电池" 这一整条路是否正常.
  *     ★ 烧录前必须把轮子架空, 否则车会一直往前跑!
  * 0 = 正常循迹模式 (参数在 line_follow.h)
  * 想让它反转就把 MOTOR_SPIN_DUTY 写成负数. */
#define MOTOR_SPIN_TEST          (0)

/** 慢转占空比 (%): 20~25 是"慢慢转"; 太小(<15)克服不了静摩擦, 看起来像不转 */
#define MOTOR_SPIN_DUTY          (25)

/** 上电后等多久开始转 (ms): 留时间把车放稳、手拿开 */
#define MOTOR_SPIN_DELAY_MS      (3000u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if (MOTOR_SPIN_TEST == 1)
/** 轮子慢转调试模式: 起转时刻 */
static uint32_t s_spin_t0 = 0u;

/** 到点之后每个主循环刷新一次占空比, 把 PWM 一直保持住 */
static void MotorSpin_Task(void)
{
  if ((HAL_GetTick() - s_spin_t0) >= MOTOR_SPIN_DELAY_MS)
  {
    Motor_SetDuty(MOTOR_A, MOTOR_SPIN_DUTY);
    Motor_SetDuty(MOTOR_B, MOTOR_SPIN_DUTY);
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Wait for the modules to stabilize after power-on
     (vendor line-tracking example uses 3000 ms) */
  HAL_Delay(TRACK_POWER_ON_DELAY_MS);

  /* Probe + init the VL53L0X and start continuous ranging.
     Safe when the sensor is absent: frames will show D=---mm
     注意: 激光测距总开关 LF_ENABLE_TOF_LASER (line_follow.h) 默认是 0(关),
           关着时这里不会去探测 VL53L0X, 帧里也没有 D1/D2 字段 */
  Telemetry_Init();

  /* Single-motor wiring test: TIM8_CH1 PWM + AIN1/AIN2 + TIM3 encoder.
     STBY is tied high in hardware, so it is not driven here.
     注意: MOTOR_TEST_AUTO_RUN 已改成 0, 上电不再自动跑测试序列,
           否则会和下面的循迹控制环抢电机. 要单测电机时再把它改回 1. */
  Motor_Init();

  /* MPU6050 on I2C2 (PB10/PB11): probe + configure; auto-retries if absent */
  MPU6050_Init();

#if (MOTOR_SPIN_TEST == 1)
  /* 调试模式: 不启动循迹控制环
     (它一看到"没线"或"有障碍"就会把电机停掉), 改成延时后让两个轮子一直慢转 */
  s_spin_t0 = HAL_GetTick();
#else
  /* 循迹控制环 + 激光避障: 参数全在 line_follow.h
     上电 LF_START_DELAY_MS 后自动开始 (LF_AUTO_START=1 时) */
  LineFollow_Init();
#endif

  /* To run the line-sensor calibration, uncomment these:
     enter calibrate -> sweep over the black/white line -> exit.
     (Calibration is stored inside the module, survives power-off.) */
  // (void)Track_SetAdjustMode(1u);
  // HAL_Delay(3000);
  // (void)Track_SetAdjustMode(0u);
  // HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Sample first, so the frame carries the freshest data:
       - LineFollow_Task: 循迹控制环 (算差速 -> 直接驱动两个电机)
       - Motor_TestTask: encoder accumulation + auto test sequence
       - MPU6050_Task  : IMU sampling (silent, no UART output) */
#if (MOTOR_SPIN_TEST == 1)
    MotorSpin_Task();     /* 调试模式: 两个轮子一直慢转 */
#else
    LineFollow_Task();    /* 正常模式: 循迹控制环驱动电机 */
#endif
    Motor_TestTask();
    MPU6050_Task();

    /* Single sender: builds and sends one unified $CAR frame on USART2 */
    Telemetry_Task();

    HAL_Delay(TELEMETRY_TASK_PERIOD_MS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
