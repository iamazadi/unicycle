/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMIT_LENGTH 220
#define RECEIVE_FRAME_LENGTH 23
#define TRANSMIT_FRAME_LENGTH 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART2_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x12, 0xC1};
uint8_t UART2_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x12, 0xC1};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
// response: a4030812f93dfbce0028000100010002ddae16b9ffe227
int receive_ok1 = 0;
int receive_ok2 = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // HAL_UART_Receive_DMA(&huart, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
    receive_ok1 = 1;
  }
  if (huart->Instance == USART2)
  {
    // HAL_UART_Receive_DMA(&huart, UART2_rxBuffer, RECEIVE_FRAME_LENGTH);
    receive_ok2 = 1;
  }
}
typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float calibrated_acc_x;
  float calibrated_acc_y;
  float calibrated_acc_z;
  float calibrated_gyro_x;
  float calibrated_gyro_y;
  float calibrated_gyro_z;
  float acc_x_offset;
  float acc_y_offset;
  float acc_z_offset;
  float gyro_x_offset;
  float gyro_y_offset;
  float gyro_z_offset;
} gy;
typedef struct
{
  float kp;
  float ki;
  float kd;
  float windup;
  float safety_angle;
  float target_angle;
  float setpoint;
  float integrator;
  float output;
  int active;
} controller;
typedef struct
{
  float x1k;
  float x2k;
  float u1k;
  float u2k;
  float x1k1;
  float x2k1;
  float u1k1;
  float u2k1;
} systemStates;
void setServoAngle(uint32_t angle)
{
  if (angle > 180)
    angle = 180;
  uint32_t minPulseWidth = 1000; // 1ms pulse width at a 1MHz clock
  uint32_t maxPulseWidth = 2000; // 2ms pulse width
  uint32_t pulse = ((angle * (maxPulseWidth - minPulseWidth)) / 180) + minPulseWidth;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); // Changed to TIM2, Channel 1
}
gy parsedata(gy sensor, float theta, uint8_t data[])
{
  sensor.acc_x = (data[4] << 8) | data[5];
  sensor.acc_y = (data[6] << 8) | data[7];
  sensor.acc_z = (data[8] << 8) | data[9];
  sensor.gyro_x = (data[10] << 8) | data[11];
  sensor.gyro_y = (data[12] << 8) | data[13];
  sensor.gyro_z = (data[14] << 8) | data[15];
  sensor.calibrated_acc_x = (float)sensor.acc_x / 100.0f - sensor.acc_x_offset;
  sensor.calibrated_acc_y = (float)sensor.acc_y / 100.0f - sensor.acc_y_offset;
  sensor.calibrated_acc_z = (float)sensor.acc_z / 100.0f - sensor.acc_z_offset;
  sensor.calibrated_gyro_x = (float)sensor.gyro_x / 100.0f - sensor.gyro_x_offset;
  sensor.calibrated_gyro_y = (float)sensor.gyro_y / 100.0f - sensor.gyro_y_offset;
  sensor.calibrated_gyro_z = (float)sensor.gyro_z / 100.0f - sensor.gyro_z_offset;
  sensor.calibrated_acc_x = cos(theta) * sensor.calibrated_acc_x + -sin(theta) * sensor.calibrated_acc_y;
  sensor.calibrated_acc_y = sin(theta) * sensor.calibrated_acc_x + cos(theta) * sensor.calibrated_acc_y;
  sensor.calibrated_gyro_x = cos(theta) * sensor.calibrated_gyro_x + -sin(theta) * sensor.calibrated_gyro_y;
  sensor.calibrated_gyro_y = sin(theta) * sensor.calibrated_gyro_x + cos(theta) * sensor.calibrated_gyro_y;
  return sensor;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint8_t MSG[TRANSMIT_LENGTH] = {'\0'};
  const float CPU_CLOCK = 84000000.0;
  const int LOG_CYCLE = 100;
  const float alpha = 0.95;                  // for alpha-smoothing of the controller output
  const float theta = -30.0 / 180.0 * 3.14; // sensor frame rotation in X-Y plane
  const float reaction_dithering_angle = 2.0;
  const float reaction_dithering_scale = 1.0 / 10.0;
  const float max_reaction_speed = 40.0;
  const float max_rolling_speed = 250.0;
  // sampling time
  float dt = 0.0;
  int transmit = 0;
  int log_counter = 0;
  unsigned long t1 = 0;
  unsigned long t2 = 0;
  unsigned long diff = 0;
  uint8_t sum = 0, i = 0;
  // fields: acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw,
  //         calibrated_acc_x, calibrated_acc_y, calibrated_acc_z,
  //         calibrated_gyro_x, calibrated_gyro_y, calibrated_gyro_z,
  //         acc_x_offset, acc_y_offset, acc_z_offset,
  //         gyro_x_offset, gyro_y_offset, gyro_z_offset.
  gy my_25t1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.51, -0.60, -0.06, 28.25, 137.0, 7.88};
  gy my_25t2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  uint8_t printstr[2 * RECEIVE_FRAME_LENGTH + 1] = {0};
  // fields: p, i, d, windup, safety angle, target angle, setpoint, integrator, output and active.
  controller reaction_ctrl = {5.0, 20.0, 1.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0};
  controller rolling_ctrl = {9.0, 20.0, 1.0, 29.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0};
  systemStates states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float roll = 0.0;
  float roll_velocity = 0.0;
  float pitch = 0.0;
  float pitch_velocity = 0.0;
  float reaction_accumulator = 0.0;
  float reaction_servoangle = 0.0;
  int sensor_updated = 0;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
  // HAL_Delay(10);
  // HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, RECEIVE_FRAME_LENGTH);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_Delay(30);
  setServoAngle(90.0);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    t1 = DWT->CYCCNT;

    if (receive_ok1 == 1)
    {
      for (sum = 0, i = 0; i < (UART1_rxBuffer[3] + 4); i++)
      {
        sum += UART1_rxBuffer[i];
      }
      // Check sum and frame ID
      if (sum == UART1_rxBuffer[i] && UART1_rxBuffer[0] == UART1_txBuffer[0])
      {
        my_25t1 = parsedata(my_25t1, theta, UART1_rxBuffer);
        receive_ok1 = 0;
        sensor_updated = 1;
      }
    }

    if (receive_ok2 == 1)
    {
      for (sum = 0, i = 0; i < (UART2_rxBuffer[3] + 4); i++)
      {
        sum += UART2_rxBuffer[i];
      }
      // Check sum and frame ID
      // if (sum == UART2_rxBuffer[i] && UART2_rxBuffer[0] == UART2_txBuffer[0])
      if (sum == UART2_rxBuffer[i])
      {
        my_25t2 = parsedata(my_25t2, theta, UART2_rxBuffer);
        receive_ok2 = 0;
      }
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0)
    {
      reaction_ctrl.active = 1;
    }
    else
    {
      reaction_ctrl.active = 0;
      setServoAngle(90.0);
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
    {
      rolling_ctrl.active = 1;
    }
    else
    {
      rolling_ctrl.active = 0;
    }

    if (sensor_updated == 1)
    {
      roll_velocity = my_25t1.calibrated_acc_y - roll;
      roll = my_25t1.calibrated_acc_y;
      pitch_velocity = my_25t1.calibrated_acc_x - pitch;
      pitch = my_25t1.calibrated_acc_x;

      reaction_ctrl.setpoint = reaction_ctrl.target_angle + reaction_accumulator;
      reaction_accumulator = reaction_accumulator - (reaction_ctrl.output * reaction_dithering_scale);
      reaction_accumulator = fmin(reaction_dithering_angle, reaction_accumulator);
      reaction_accumulator = fmax(-reaction_dithering_angle, reaction_accumulator);

      reaction_ctrl.integrator = reaction_ctrl.integrator + (roll - reaction_ctrl.setpoint) * reaction_ctrl.ki;
      if (reaction_ctrl.integrator > reaction_ctrl.windup)
      {
        reaction_ctrl.integrator = reaction_ctrl.windup;
      }
      if (reaction_ctrl.integrator < -reaction_ctrl.windup)
      {
        reaction_ctrl.integrator = -reaction_ctrl.windup;
      }
      // PID function
      reaction_ctrl.output = reaction_ctrl.kp * (roll - reaction_ctrl.setpoint) + reaction_ctrl.kd * roll_velocity + reaction_ctrl.integrator;
      reaction_ctrl.output = fmin(max_reaction_speed, reaction_ctrl.output);
      reaction_ctrl.output = fmax(-max_reaction_speed, reaction_ctrl.output);

      rolling_ctrl.integrator = rolling_ctrl.integrator + (pitch - rolling_ctrl.setpoint) * rolling_ctrl.ki;
      if (rolling_ctrl.integrator > rolling_ctrl.windup)
      {
        rolling_ctrl.integrator = rolling_ctrl.windup;
      }
      if (rolling_ctrl.integrator < -rolling_ctrl.windup)
      {
        rolling_ctrl.integrator = -rolling_ctrl.windup;
      }
      // PID function
      rolling_ctrl.output = rolling_ctrl.kp * (pitch - rolling_ctrl.setpoint) + rolling_ctrl.kd * pitch_velocity + rolling_ctrl.integrator;
      rolling_ctrl.output = fmin(max_rolling_speed, rolling_ctrl.output);
      rolling_ctrl.output = fmax(-max_rolling_speed, rolling_ctrl.output);

      sensor_updated = 0;
    }

    if (reaction_ctrl.active == 1)
    {
      if (fabs(roll) < reaction_ctrl.safety_angle)
      {
        // 0-90 clockwise
        // 90-135 anti-clockwise
        reaction_servoangle = alpha * reaction_servoangle + (1.0 - alpha) * reaction_ctrl.output;
        setServoAngle(reaction_servoangle < 0 ? 90.0 + 2.0 * reaction_servoangle : reaction_servoangle + 90.0);
      }
      else
      {
        setServoAngle(90.0);
      }
    }

    if (rolling_ctrl.active == 1)
    {
      if (fabs(pitch) < rolling_ctrl.safety_angle)
      {
        if (rolling_ctrl.output < 0)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255 * (int)fabs(rolling_ctrl.output));
        }
        else
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255 * (int)fabs(rolling_ctrl.output));
        }
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      }
    }

    log_counter++;
    if (log_counter > LOG_CYCLE)
    {
      transmit = 1;
    }
    if (transmit == 1)
    {
      transmit = 0;
      log_counter = 0;

    states.x1k = states.x1k1;
    states.x2k = states.x2k1;
    states.u1k = states.u1k1;
    states.u2k = states.u2k1;
    states.x1k1 = roll;
    states.x2k1 = pitch;
    states.u1k1 = reaction_ctrl.output;
    states.u2k1 = rolling_ctrl.output;
    
      sprintf(MSG, "x1k: %0.2f, x2k: %0.2f, u1k: %0.2f, u2k: %0.2f, x1k+: %0.2f, x2k+: %0.2f, u1k+: %0.2f, u2k+: %0.2f, dt: %0.6f\r\n",
        states.x1k, states.x2k, states.u1k, states.u2k, states.x1k1, states.x2k1, states.u1k1, states.u2k1, dt);
      // sprintf(MSG, "ID:%d, ACC_X:%0.2f, ACC_Y:%0.2f, ACC_Z:%0.2f, GYRO_X:%0.2f, GYRO_Y:%0.2f, GYRO_Z:%0.2f, count:%d, dt: %0.6f\r\n",
      //         UART1_rxBuffer[0],
      //         my_25t.calibrated_acc_x, my_25t.calibrated_acc_y, my_25t.calibrated_acc_z,
      //         my_25t.calibrated_gyro_x, my_25t.calibrated_gyro_y, my_25t.calibrated_gyro_z, UART1_rxBuffer[3] + 4, dt);
      // sprintf(MSG, "ID:%d, roll:%0.3f, velocity:%0.6f, acceleration:%0.6f, ACC_X:%0.2f, ACC_Y:%0.2f, GYRO_X:%0.2f, GYRO_Y:%0.2f, count:%d, dt: %0.6f\r\n",
      //         UART1_rxBuffer[0], roll, roll_velocity, roll_acceleration,
      //         my_25t1.calibrated_acc_x, my_25t1.calibrated_acc_y,
      //         my_25t1.calibrated_gyro_x, my_25t1.calibrated_gyro_y, UART1_rxBuffer[3] + 4, dt);

      // Toggle the LED
      // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      // for (int x = 0; x < RECEIVE_FRAME_LENGTH; x++)
      // {
      //   sprintf(printstr + (x * 2), "%02x", UART1_rxBuffer[x]);
      // }
      // printstr[2 * RECEIVE_FRAME_LENGTH] = '\0';
      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);

      // else
      // {
      //   sprintf(MSG, "sum %d, checksum:%d, count %d, dt: %0.6f\r\n", sum, UART1_rxBuffer[i], UART1_rxBuffer[3] + 4, dt);
      // }
    }

    t2 = DWT->CYCCNT;
    diff = t2 - t1;
    dt = (float)diff / CPU_CLOCK;

    // Rinse and repeat :)
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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

#ifdef USE_FULL_ASSERT
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
