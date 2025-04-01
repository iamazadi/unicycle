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
#define N 3
#define M 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
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
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x12, 0xC1};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
// response: a4030812f93dfbce0028000100010002ddae16b9ffe227
int receive_ok = 0;
int sensor_updated = 0;
const int dim_n = N;
const int dim_m = M;
const int max_episode_length = 50000;
const float sensor_rotation = -30.0 / 180.0 * M_PI; // sensor frame rotation in X-Y plane
const float reaction_wheel_safety_angle = 10.0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // HAL_UART_Receive_DMA(&huart, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
    receive_ok = 1;
  }
}
typedef struct
{
  int value;                  // the vlue of the timer counter
  int change;                 // instantanious change in encoder value
  int accumulator;            // accumultes the instantanoius changes over a number of intervals
  int interval_counter;       // the number of intervals accumulated
  int intervals;              // maximum number intervals before averaging
  float pulse_per_revolution; // the number of pulses per revolution
  float velocity;             // the wheel velocity
} Encoder;

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
  float calibrated_acc_x_velocity;
  float calibrated_acc_y_velocity;
  float calibrated_acc_z_velocity;
  float calibrated_gyro_x;
  float calibrated_gyro_y;
  float calibrated_gyro_z;
  float acc_x_offset;
  float acc_y_offset;
  float acc_z_offset;
  float gyro_x_offset;
  float gyro_y_offset;
  float gyro_z_offset;
} IMU;
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
} Controller;

Encoder updateEncoder(Encoder encoder)
{
  if (encoder.interval_counter > encoder.intervals)
  {
    encoder.velocity = (float)encoder.accumulator / (float)encoder.interval_counter;
    // convert the unit to radian per sample time
    encoder.velocity = encoder.velocity / encoder.pulse_per_revolution * 2.0 * M_PI;
    encoder.interval_counter = 0;
    encoder.accumulator = 0;
  }
  else
  {
    encoder.interval_counter++;
    encoder.change = TIM3->CNT - encoder.value;
    encoder.value = TIM3->CNT;
    encoder.accumulator += encoder.change;
  }
  return encoder;
}

void setServoAngle(int angle)
{
  if (angle > 180)
  {
    angle = 180;
  }
  if (angle < 0)
  {
    angle = 0;
  }
  uint32_t minPulseWidth = 1000; // 1ms pulse width at a 1MHz clock
  uint32_t maxPulseWidth = 2000; // 2ms pulse width
  uint32_t pulse = ((angle * (maxPulseWidth - minPulseWidth)) / 180) + minPulseWidth;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); // Changed to TIM2, Channel 1
}

IMU parsedata(IMU sensor, float theta, uint8_t data[])
{
  float calibrated_acc_x = sensor.calibrated_acc_x;
  float calibrated_acc_y = sensor.calibrated_acc_y;
  float calibrated_acc_z = sensor.calibrated_acc_z;
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
  sensor.calibrated_acc_x_velocity = sensor.calibrated_acc_x - calibrated_acc_x;
  sensor.calibrated_acc_y_velocity = sensor.calibrated_acc_y - calibrated_acc_y;
  sensor.calibrated_acc_z_velocity = sensor.calibrated_acc_z - calibrated_acc_z;
  return sensor;
}

IMU updateIMU(IMU gy_25t)
{
  uint8_t sum = 0, i = 0;
  if (receive_ok == 1)
  {
    for (sum = 0, i = 0; i < (UART1_rxBuffer[3] + 4); i++)
    {
      sum += UART1_rxBuffer[i];
    }
    // Check sum and frame ID
    if (sum == UART1_rxBuffer[i] && UART1_rxBuffer[0] == UART1_txBuffer[0])
    {
      gy_25t = parsedata(gy_25t, sensor_rotation, UART1_rxBuffer);
      receive_ok = 0;
      sensor_updated = 1;
    }
  }
  return gy_25t;
}

float sigmoid(float x)
{
  return 1.0 / (1.0 + exp(-x));
}

int argmax(float *array, int length)
{
  int index = 0;
  float value = array[0];
  for (int i = 0; i < length; i++)
  {
    if (array[i] > value)
    {
      index = i;
      value = array[index];
    }
  }
  return index;
}

typedef struct
{
  float x1;
  float x2;
  float x3;
} Vec3f;
typedef struct
{
  float x1;
  float x2;
  float x3;
  float x4;
  float x5;
  float x6;
  float x7;
  float x8;
  float x9;
  float x10;
} Vec10f;
typedef struct
{
  float x11;
  float x12;
  float x13;
  float x21;
  float x22;
  float x23;
  float x31;
  float x32;
  float x33;
} Mat3f;
typedef struct
{
  float x11;
  float x12;
  float x13;
  float x21;
  float x22;
  float x23;
} Mat23f;
typedef struct
{
  float x11;
  float x12;
  float x13;
  float x14;
  float x15;
  float x21;
  float x22;
  float x23;
  float x24;
  float x25;
  float x31;
  float x32;
  float x33;
  float x34;
  float x35;
  float x41;
  float x42;
  float x43;
  float x44;
  float x45;
  float x51;
  float x52;
  float x53;
  float x54;
  float x55;
} Mat5f;
// Represents a Linear Quadratic Regulator (LQR) model.
typedef struct
{
  Mat5f W_n;      // filter matrix
  Mat5f P_n;      // inverse autocorrelation matrix
  Mat23f K_j;     // feedback policy
  Vec10f dataset; // (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
  int j;          // step number
  int k;          // time k
  float reward;   // the cumulative reward
  int n;          // xₖ ∈ ℝⁿ
  int m;          // uₖ ∈ ℝᵐ
  float lambda;   // exponential wighting factor
  float delta;    // value used to intialize P(0)
  int terminated; // has the environment been reset
  int updated;    // whether the policy has been updated since episode termination and parameter convegence
  int active;     // is the model controller active
  IMU imu;
  Encoder encoder;
} LinearQuadraticRegulator;

// Initialize the randomizer using the current timestamp as a seed
// (The time() function is provided by the <time.h> header file)
// srand(time(NULL));

LinearQuadraticRegulator initialize(LinearQuadraticRegulator model)
{
  model.j = 1;
  model.k = 1;
  model.reward = 0.0;
  model.n = dim_n;
  model.m = dim_m;
  model.lambda = 0.999;
  model.delta = 0.001;
  model.terminated = 0;
  model.updated = 0;
  model.active = 0;

  model.W_n.x11 = (float)(rand() % 100) / 100.0;
  model.W_n.x12 = (float)(rand() % 100) / 100.0;
  model.W_n.x13 = (float)(rand() % 100) / 100.0;
  model.W_n.x14 = (float)(rand() % 100) / 100.0;
  model.W_n.x15 = (float)(rand() % 100) / 100.0;
  model.W_n.x21 = (float)(rand() % 100) / 100.0;
  model.W_n.x22 = (float)(rand() % 100) / 100.0;
  model.W_n.x23 = (float)(rand() % 100) / 100.0;
  model.W_n.x24 = (float)(rand() % 100) / 100.0;
  model.W_n.x25 = (float)(rand() % 100) / 100.0;
  model.W_n.x31 = (float)(rand() % 100) / 100.0;
  model.W_n.x32 = (float)(rand() % 100) / 100.0;
  model.W_n.x33 = (float)(rand() % 100) / 100.0;
  model.W_n.x34 = (float)(rand() % 100) / 100.0;
  model.W_n.x35 = (float)(rand() % 100) / 100.0;
  model.W_n.x41 = (float)(rand() % 100) / 100.0;
  model.W_n.x42 = (float)(rand() % 100) / 100.0;
  model.W_n.x43 = (float)(rand() % 100) / 100.0;
  model.W_n.x44 = (float)(rand() % 100) / 100.0;
  model.W_n.x45 = (float)(rand() % 100) / 100.0;
  model.W_n.x51 = (float)(rand() % 100) / 100.0;
  model.W_n.x52 = (float)(rand() % 100) / 100.0;
  model.W_n.x53 = (float)(rand() % 100) / 100.0;
  model.W_n.x54 = (float)(rand() % 100) / 100.0;
  model.W_n.x55 = (float)(rand() % 100) / 100.0;

  model.P_n.x11 = 1.0 / model.delta;
  model.P_n.x12 = 0.0;
  model.P_n.x13 = 0.0;
  model.P_n.x14 = 0.0;
  model.P_n.x15 = 0.0;
  model.P_n.x21 = 0.0;
  model.P_n.x22 = 1.0 / model.delta;
  model.P_n.x23 = 0.0;
  model.P_n.x24 = 0.0;
  model.P_n.x25 = 0.0;
  model.P_n.x31 = 0.0;
  model.P_n.x32 = 0.0;
  model.P_n.x33 = 1.0 / model.delta;
  model.P_n.x34 = 0.0;
  model.P_n.x35 = 0.0;
  model.P_n.x41 = 0.0;
  model.P_n.x42 = 0.0;
  model.P_n.x43 = 0.0;
  model.P_n.x44 = 1.0 / model.delta;
  model.P_n.x45 = 0.0;
  model.P_n.x51 = 0.0;
  model.P_n.x52 = 0.0;
  model.P_n.x53 = 0.0;
  model.P_n.x54 = 0.0;
  model.P_n.x55 = 1.0 / model.delta;
  model.K_j.x11 = 1.0;
  model.K_j.x12 = 1.0;
  model.K_j.x13 = 1.0;
  model.K_j.x21 = 1.0;
  model.K_j.x22 = 1.0;
  model.K_j.x23 = 1.0;
  model.dataset.x1 = 0.0;
  model.dataset.x2 = 0.0;
  model.dataset.x3 = 0.0;
  model.dataset.x4 = 0.0;
  model.dataset.x5 = 0.0;
  model.dataset.x6 = 0.0;
  model.dataset.x7 = 0.0;
  model.dataset.x8 = 0.0;
  model.dataset.x9 = 0.0;
  model.dataset.x10 = 0.0;
  IMU imu = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.51, -0.60, -0.06, 28.25, 137.0, 7.88};
  Encoder encoder = {0, 0, 0, 0, 100, 50.0, 0.0};
  model.imu = imu;
  model.encoder = encoder;
  return model;
}
/*
Identify the Q function using RLS and update the control policy,
with the given `environment` and `model`.
The algorithm is terminated when there are no further updates
to the Q function or the control policy at each step.
*/
LinearQuadraticRegulator stepForward(LinearQuadraticRegulator model)
{
  int k = model.k;
  float x_k[N] = {model.dataset.x1, model.dataset.x2, model.dataset.x3};
  float K_j[M][N] = {{model.K_j.x11, model.K_j.x12, model.K_j.x13}, {model.K_j.x21, model.K_j.x22, model.K_j.x23}};
  // feeback policy
  float u_k[M] = {0.0, 0.0};
  for (int i = 0; i < model.n; i++)
  {
    for (int j = 0; j < model.m; j++)
    {
      u_k[i] += -K_j[i][j] * x_k[j];
      u_k[i] += -K_j[i][j] * x_k[j];
    }
  }
  for (int i = 0; i < model.m; i++)
  {
    u_k[i] = sigmoid(u_k[i]);
  }
  // act!
  model.dataset.x1 = model.imu.calibrated_acc_y;
  model.dataset.x2 = model.imu.calibrated_acc_y_velocity;
  model.dataset.x3 = model.imu.calibrated_acc_y * model.imu.calibrated_acc_y;
  // model.dataset.x3 = model.encoder.velocity;
  model.dataset.x4 = u_k[0];
  model.dataset.x5 = u_k[1];
  int index = argmax(u_k, M);
  float action = u_k[index] * 45.0;
  if (model.active == 1)
  {
    // 0-90 clockwise
    // 90-135 anti-clockwise
    if (rand() % 100 > 5)
    {
      setServoAngle(index == 0 ? 90.0 + action : 90.0 - 2.0 * action);
    }
    else
    {
      setServoAngle(model.imu.calibrated_acc_y > 0.0 ? 90.0 + 22.5 : 90.0 - 2.0 * 22.5);
      if (model.imu.calibrated_acc_y > 0.0)
      {
        model.dataset.x4 = 1.0;
        model.dataset.x5 = 0.0;
      }
      else
      {
        model.dataset.x4 = 0.0;
        model.dataset.x5 = 1.0;
      }
    }
  }
  else
  {
    setServoAngle(90.0);
  }
  // dataset = (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  model.dataset.x6 = model.imu.calibrated_acc_y;
  model.dataset.x7 = model.imu.calibrated_acc_y_velocity;
  model.dataset.x8 = model.imu.calibrated_acc_y * model.imu.calibrated_acc_y;
  // model.dataset.x8 = model.encoder.velocity;
  float x_k1[N] = {model.dataset.x6, model.dataset.x7, model.dataset.x8};
  float u_k1[M] = {0.0, 0.0};
  for (int i = 0; i < model.n; i++)
  {
    for (int j = 0; j < model.m; j++)
    {
      u_k1[i] += -K_j[i][j] * x_k1[j];
      u_k1[i] += -K_j[i][j] * x_k1[j];
    }
  }
  for (int i = 0; i < model.m; i++)
  {
    u_k1[i] = sigmoid(u_k1[i]);
  }
  model.dataset.x9 = u_k1[0];
  model.dataset.x10 = u_k1[1];
  // Compute the quadratic basis sets ϕ(zₖ), ϕ(zₖ₊₁).
  float z_k[N + M] = {model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5};
  float z_k1[N + M] = {model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, model.dataset.x10};
  float basisset1[N + M];
  float basisset2[N + M];
  for (int i = 0; i < N + M; i++)
  {
    basisset1[i] = sigmoid(z_k[i]);
    basisset2[i] = sigmoid(z_k1[i]);
  }
  // Now perform a one-step update in the parameter vector W by applying RLS to equation (S27).
  float z_n[N + M];
  // initialize z_n
  for (int i = 0; i < model.n + model.m; i++)
  {
    z_n[i] = 0.0;
  }
  float P_n[N + M][N + M] = {{model.P_n.x11, model.P_n.x12, model.P_n.x13, model.P_n.x14, model.P_n.x15},
                             {model.P_n.x21, model.P_n.x22, model.P_n.x23, model.P_n.x24, model.P_n.x25},
                             {model.P_n.x31, model.P_n.x32, model.P_n.x33, model.P_n.x34, model.P_n.x35},
                             {model.P_n.x41, model.P_n.x42, model.P_n.x43, model.P_n.x44, model.P_n.x45},
                             {model.P_n.x51, model.P_n.x52, model.P_n.x53, model.P_n.x54, model.P_n.x55}};
  float W_n[N + M][N + M] = {{model.W_n.x11, model.W_n.x12, model.W_n.x13, model.W_n.x14, model.W_n.x15},
                             {model.W_n.x21, model.W_n.x22, model.W_n.x23, model.W_n.x24, model.W_n.x25},
                             {model.W_n.x31, model.W_n.x32, model.W_n.x33, model.W_n.x34, model.W_n.x35},
                             {model.W_n.x41, model.W_n.x42, model.W_n.x43, model.W_n.x44, model.W_n.x45},
                             {model.W_n.x51, model.W_n.x52, model.W_n.x53, model.W_n.x54, model.W_n.x55}};
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      z_n[i] += P_n[i][j] * z_k[j];
    }
  }
  float g_n[N + M];
  for (int i = 0; i < N + M; i++)
  {
    g_n[i] = 1.0 / (model.lambda + z_k[i] * z_n[i]) * z_n[i];
  }
  // αₙ = dₙ - transpose(wₙ₋₁) * xₙ
  float alpha_n[N + M];
  // initialize alpha_n
  for (int i = 0; i < model.n + model.m; i++)
  {
    alpha_n[i] = 0.0;
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      alpha_n[i] += W_n[i][j] * (basisset1[j] - basisset2[j]);
    }
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      W_n[i][j] = W_n[i][j] + (alpha_n[i] - g_n[j]);
    }
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      P_n[i][j] = (1.0 / model.lambda) * (P_n[i][j] - g_n[i] * z_n[j]);
    }
  }
  model.W_n.x11 = W_n[1][1];
  model.W_n.x12 = W_n[1][2];
  model.W_n.x13 = W_n[1][3];
  model.W_n.x14 = W_n[1][4];
  model.W_n.x15 = W_n[1][5];
  model.W_n.x21 = W_n[2][1];
  model.W_n.x22 = W_n[2][2];
  model.W_n.x23 = W_n[2][3];
  model.W_n.x24 = W_n[2][4];
  model.W_n.x25 = W_n[2][5];
  model.W_n.x31 = W_n[3][1];
  model.W_n.x32 = W_n[3][2];
  model.W_n.x33 = W_n[3][3];
  model.W_n.x34 = W_n[3][4];
  model.W_n.x35 = W_n[3][5];
  model.W_n.x41 = W_n[4][1];
  model.W_n.x42 = W_n[4][2];
  model.W_n.x43 = W_n[4][3];
  model.W_n.x44 = W_n[4][4];
  model.W_n.x45 = W_n[4][5];
  model.W_n.x51 = W_n[5][1];
  model.W_n.x52 = W_n[5][2];
  model.W_n.x53 = W_n[5][3];
  model.W_n.x54 = W_n[5][4];
  model.W_n.x55 = W_n[5][5];
  model.P_n.x11 = P_n[1][1];
  model.P_n.x12 = P_n[1][2];
  model.P_n.x13 = P_n[1][3];
  model.P_n.x14 = P_n[1][4];
  model.P_n.x15 = P_n[1][5];
  model.P_n.x21 = P_n[2][1];
  model.P_n.x22 = P_n[2][2];
  model.P_n.x23 = P_n[2][3];
  model.P_n.x24 = P_n[2][4];
  model.P_n.x25 = P_n[2][5];
  model.P_n.x31 = P_n[3][1];
  model.P_n.x32 = P_n[3][2];
  model.P_n.x33 = P_n[3][3];
  model.P_n.x34 = P_n[3][4];
  model.P_n.x35 = P_n[3][5];
  model.P_n.x41 = P_n[4][1];
  model.P_n.x42 = P_n[4][2];
  model.P_n.x43 = P_n[4][3];
  model.P_n.x44 = P_n[4][4];
  model.P_n.x45 = P_n[4][5];
  model.P_n.x51 = P_n[5][1];
  model.P_n.x52 = P_n[5][2];
  model.P_n.x53 = P_n[5][3];
  model.P_n.x54 = P_n[5][4];
  model.P_n.x55 = P_n[5][5];
  // Repeat at the next time k + 1 and continue until RLS converges and the new parameter vector Wⱼ₊₁ is found.
  model.k = k + 1;
  return model;
}

LinearQuadraticRegulator updateControlPolicy(LinearQuadraticRegulator model)
{
  // npack the vector Wⱼ₊₁ into the kernel matrix
  // Q(xₖ, uₖ) ≡ 0.5 * transpose([xₖ; uₖ]) * S * [xₖ; uₖ] = 0.5 * transpose([xₖ; uₖ]) * [Sₓₓ Sₓᵤ; Sᵤₓ Sᵤᵤ] * [xₖ; uₖ]
  model.k = 1;
  model.j = model.j + 1;
  float S_ux[M][N];
  float S_uu[M][M];
  float W_n[N + M][N + M] = {{model.W_n.x11, model.W_n.x12, model.W_n.x13, model.W_n.x14, model.W_n.x15},
                             {model.W_n.x21, model.W_n.x22, model.W_n.x23, model.W_n.x24, model.W_n.x25},
                             {model.W_n.x31, model.W_n.x32, model.W_n.x33, model.W_n.x34, model.W_n.x35},
                             {model.W_n.x41, model.W_n.x42, model.W_n.x43, model.W_n.x44, model.W_n.x45},
                             {model.W_n.x51, model.W_n.x52, model.W_n.x53, model.W_n.x54, model.W_n.x55}};
  for (int i = 0; i < model.m; i++)
  {
    for (int j = 0; j < model.n; j++)
    {
      S_ux[i][j] = W_n[model.n + i][j];
    }
  }
  for (int i = 0; i < model.m; i++)
  {
    for (int j = 0; j < model.m; j++)
    {
      S_uu[i][j] = W_n[model.n + i][model.n + j];
    }
  }
  // Perform the control update using (S24), which is uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  // uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  float determinant = S_uu[1][1] * S_uu[2][2] - S_uu[1][2] * S_uu[2][1];
  // check the rank S_uu to see if it's equal to 2 (invertible matrix)
  if (fabs(determinant) > 0.001) // greater than zero
  {
    float S_uu_inverse[M][M] = {{S_uu[2][2] / determinant, -S_uu[1][2] / determinant},
                                {-S_uu[2][1] / determinant, S_uu[1][1] / determinant}};
    float K_j[M][N];
    for (int i = 0; i < model.m; i++)
    {
      for (int j = 0; j < model.n; j++)
      {
        K_j[i][j] = 0.0;
        for (int k = 0; k < model.m; k++)
        {
          K_j[i][j] += S_uu_inverse[i][k] * S_ux[k][j];
        }
      }
    }
    model.K_j.x11 = K_j[1][1];
    model.K_j.x12 = K_j[1][2];
    model.K_j.x13 = K_j[1][3];
    model.K_j.x21 = K_j[2][1];
    model.K_j.x22 = K_j[2][2];
    model.K_j.x23 = K_j[2][3];
  }
  model.updated = 1;
  return model;
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
  const float max_rolling_speed = 250.0;
  // const float epsilon = 10.0; // convergence threshold
  // const int numStates = 3;
  // sampling time
  float dt = 0.0;
  int transmit = 0;
  int log_counter = 0;
  unsigned long t1 = 0;
  unsigned long t2 = 0;
  unsigned long diff = 0;
  // fields: p, i, d, windup, safety angle, target angle, setpoint, integrator, output and active.
  Controller rolling_wheel_controller = {9.0, 20.0, 1.0, 29.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0};
  // instantiate a model and initialize it
  LinearQuadraticRegulator model;
  model = initialize(model);
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
  MX_TIM3_Init();
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
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_Delay(30);
  setServoAngle(90.0);
  // initialize the Encoder and IMU
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  HAL_Delay(1);
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    t1 = DWT->CYCCNT;

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0)
    {
      model.active = 1;
    }
    else
    {
      model.active = 0;
      setServoAngle(90.0);
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
    {
      rolling_wheel_controller.active = 1;
    }
    else
    {
      rolling_wheel_controller.active = 0;
    }

    if (sensor_updated == 1 && rolling_wheel_controller.active == 1)
    {
      rolling_wheel_controller.integrator = rolling_wheel_controller.integrator + (model.imu.calibrated_acc_x - rolling_wheel_controller.setpoint) * rolling_wheel_controller.ki;
      if (rolling_wheel_controller.integrator > rolling_wheel_controller.windup)
      {
        rolling_wheel_controller.integrator = rolling_wheel_controller.windup;
      }
      if (rolling_wheel_controller.integrator < -rolling_wheel_controller.windup)
      {
        rolling_wheel_controller.integrator = -rolling_wheel_controller.windup;
      }
      // PID function
      rolling_wheel_controller.output = rolling_wheel_controller.kp * (model.imu.calibrated_acc_x - rolling_wheel_controller.setpoint) + rolling_wheel_controller.kd * model.imu.calibrated_acc_x_velocity + rolling_wheel_controller.integrator;
      rolling_wheel_controller.output = fmin(max_rolling_speed, rolling_wheel_controller.output);
      rolling_wheel_controller.output = fmax(-max_rolling_speed, rolling_wheel_controller.output);

      sensor_updated = 0;
    }

    if (fabs(model.imu.calibrated_acc_y) > reaction_wheel_safety_angle || fabs(model.imu.calibrated_acc_x) > rolling_wheel_controller.safety_angle || model.k > max_episode_length)
    {
      model.terminated = 1;
      model.active = 0;
      rolling_wheel_controller.active = 0;
      rolling_wheel_controller.output = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      // HAL_Delay(1000);
      model.terminated = 0;
      model.active = 1;
      rolling_wheel_controller.active = 1;
      model.updated = 0;
    }

    if (rolling_wheel_controller.active == 1)
    {
      if (fabs(model.imu.calibrated_acc_x) < rolling_wheel_controller.safety_angle)
      {
        if (rolling_wheel_controller.output < 0)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255 * (int)fabs(rolling_wheel_controller.output));
        }
        else
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255 * (int)fabs(rolling_wheel_controller.output));
        }
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      }
    }
    if (model.terminated == 0)
    {
      model = stepForward(model);
    }
    else
    {
      setServoAngle(90.0);
    }
    // Rinse and repeat :)
    if (model.terminated == 1 && model.updated == 0)
    {
      model = updateControlPolicy(model);
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

      sprintf(MSG, "x1: %0.2f, x2: %0.2f, x3: %0.2f, u1: %0.2f, u2: %0.2f, x1k: %0.2f, x2k: %0.2f, x3k: %0.2f, u1k: %0.2f, u2k: %0.2f, j: %d, k: %d, enc: %d, dt: %0.6f\r\n",
              model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5,
              model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, model.dataset.x10,
              model.j, model.k, TIM3->CNT, dt);
      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);
    }

    t2 = DWT->CYCCNT;
    diff = t2 - t1;
    dt = (float)diff / CPU_CLOCK;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
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
