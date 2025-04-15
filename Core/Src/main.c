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
#define TRANSMIT_LENGTH 300
#define RECEIVE_FRAME_LENGTH 23
#define TRANSMIT_FRAME_LENGTH 5
#define N 6
#define M 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

uint32_t value_adc;

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
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x12, 0xC1};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
// response: a4030812f93dfbce0028000100010002ddae16b9ffe227
int uart_receive_ok = 0;
int adc_receive_ok = 0;
int sensor_updated = 0;
const int dim_n = N;
const int dim_m = M;
const int max_episode_length = 50000;
const float sensor_rotation = -30.0 / 180.0 * M_PI; // sensor frame rotation in X-Y plane
const float reaction_wheel_safety_angle = 10.0;
const float clip_value = 10000.0;
float servoAngle = 0.0;
float reaction_wheel_speed = 0.0;
float rolling_wheel_speed = 0.0;
int encoder0 = 0;
int encoder1 = 0;
int encoderState = 0;
int encoderLastState = 0;
int encoderCounter = 0;
int encoderChange = 0;
int encoderVelocity = 0;
uint32_t AD_RES_BUFFER[2];
// define arrays for matrix-matrix and matrix-vector multiplication
float x_k[N];
float u_k[M];
float x_k1[N];
float u_k1[M];
float z_k[N + M];
float z_k1[N + M];
float basisset0[N + M];
float basisset1[N + M];
float z_n[N + M];
float W_n[N + M][N + M];
float P_n[N + M][N + M];
float K_j[M][N];
float g_n[N + M];
float alpha_n[N + M];
float S_ux[M][N];
float S_uu[M][M];
float S_uu_inverse[M][M];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // HAL_UART_Receive_DMA(&huart, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
    uart_receive_ok = 1;
  }
}
typedef struct
{
  int value0; // the value of the adc measurement on channel A
  int value1; // the value of the adc measurement on channel B
  int channelA;
  int channelB;
  int counter;
  float angle;
  int aState;
  int aLastState;
  int change;                 // instantanious change in encoder value
  int accumulator;            // accumultes the instantanoius changes over a number of intervals
  int interval_counter;       // the number of intervals accumulated
  int intervals;              // maximum number intervals before averaging
  float pulse_per_revolution; // the number of pulses per revolution
  float threshold;            // the threshold for binarizing sensor readings
  float velocity;             // the angular velocity
  float acceleration;         // the angular acceleration
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
  float calibrated_acc_x_acceleration;
  float calibrated_acc_y_acceleration;
  float calibrated_acc_z_acceleration;
  float calibrated_acc_x_jerk;
  float calibrated_acc_y_jerk;
  float calibrated_acc_z_jerk;
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

float sigmoid(float x)
{
  return 1.0 / (1.0 + exp(-x));
}

Encoder updateEncoder(Encoder encoder)
{
  if (adc_receive_ok == 1)
  {
    encoder.value0 = (AD_RES_BUFFER[0] << 4);
    encoder.value1 = (AD_RES_BUFFER[1] << 4);
    float valueA = ((float)encoder.value0 - 32512.0) / 65025.0;
    float valueB = ((float)encoder.value1 - 32512.0) / 65025.0;
    if (valueA < encoder.threshold)
    {
      encoder.channelA = 0;
    }
    else
    {
      encoder.channelA = 1;
    }
    if (valueB < encoder.threshold)
    {
      encoder.channelB = 0;
    }
    else
    {
      encoder.channelB = 1;
    }
    adc_receive_ok = 0;
  }
  encoder.aState = encoder.channelA;
  if (encoder.aState != encoder.aLastState)
  {
    if (encoder.channelB != encoder.aState)
    {
      encoder.angle = encoder.angle + (2.0 * M_PI / encoder.pulse_per_revolution);
    }
    else
    {
      encoder.angle = encoder.angle - (2.0 * M_PI / encoder.pulse_per_revolution);
    }
  }
  encoder.aLastState = encoder.aState;
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
  float calibrated_acc_x_velocity = sensor.calibrated_acc_x_velocity;
  float calibrated_acc_y_velocity = sensor.calibrated_acc_y_velocity;
  float calibrated_acc_z_velocity = sensor.calibrated_acc_z_velocity;
  float calibrated_acc_x_acceleration = sensor.calibrated_acc_x_acceleration;
  float calibrated_acc_y_acceleration = sensor.calibrated_acc_y_acceleration;
  float calibrated_acc_z_acceleration = sensor.calibrated_acc_z_acceleration;
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
  sensor.calibrated_acc_x_acceleration = sensor.calibrated_acc_x_velocity - calibrated_acc_x_velocity;
  sensor.calibrated_acc_y_acceleration = sensor.calibrated_acc_y_velocity - calibrated_acc_y_velocity;
  sensor.calibrated_acc_z_acceleration = sensor.calibrated_acc_z_velocity - calibrated_acc_z_velocity;
  sensor.calibrated_acc_x_jerk = sensor.calibrated_acc_x_acceleration - calibrated_acc_x_acceleration;
  sensor.calibrated_acc_y_jerk = sensor.calibrated_acc_y_acceleration - calibrated_acc_y_acceleration;
  sensor.calibrated_acc_z_jerk = sensor.calibrated_acc_z_acceleration - calibrated_acc_z_acceleration;
  return sensor;
}

IMU updateIMU(IMU gy_25t)
{
  uint8_t sum = 0, i = 0;
  if (uart_receive_ok == 1)
  {
    for (sum = 0, i = 0; i < (UART1_rxBuffer[3] + 4); i++)
    {
      sum += UART1_rxBuffer[i];
    }
    // Check sum and frame ID
    if (sum == UART1_rxBuffer[i] && UART1_rxBuffer[0] == UART1_txBuffer[0])
    {
      gy_25t = parsedata(gy_25t, sensor_rotation, UART1_rxBuffer);
      uart_receive_ok = 0;
      sensor_updated = 1;
    }
  }
  return gy_25t;
}

float clip_by_value(float x, float clip_value)
{
  if (x < -clip_value)
  {
    return -clip_value;
  }
  if (x > clip_value)
  {
    return clip_value;
  }
  return x;
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
  float x0;
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
  float x11;
  float x12;
  float x13;
  float x14;
  float x15;
} Vec16f;
typedef struct
{
  float x00;
  float x01;
  float x02;
  float x03;
  float x04;
  float x05;
  float x10;
  float x11;
  float x12;
  float x13;
  float x14;
  float x15;
} Mat26f;
typedef struct
{
  float x00;
  float x01;
  float x02;
  float x03;
  float x04;
  float x05;
  float x06;
  float x07;
  float x10;
  float x11;
  float x12;
  float x13;
  float x14;
  float x15;
  float x16;
  float x17;
  float x20;
  float x21;
  float x22;
  float x23;
  float x24;
  float x25;
  float x26;
  float x27;
  float x30;
  float x31;
  float x32;
  float x33;
  float x34;
  float x35;
  float x36;
  float x37;
  float x40;
  float x41;
  float x42;
  float x43;
  float x44;
  float x45;
  float x46;
  float x47;
  float x50;
  float x51;
  float x52;
  float x53;
  float x54;
  float x55;
  float x56;
  float x57;
  float x60;
  float x61;
  float x62;
  float x63;
  float x64;
  float x65;
  float x66;
  float x67;
  float x70;
  float x71;
  float x72;
  float x73;
  float x74;
  float x75;
  float x76;
  float x77;
} Mat8f;
// Represents a Linear Quadratic Regulator (LQR) model.
typedef struct
{
  Mat8f W_n;      // filter matrix
  Mat8f P_n;      // inverse autocorrelation matrix
  Mat26f K_j;     // feedback policy
  Vec16f dataset; // (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
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

// instantiate a model and initialize it
LinearQuadraticRegulator model;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Conversion Complete & DMA Transfer Complete As Well
  // So The AD_RES_BUFFER Is Now Updated
  adc_receive_ok = 1;
}

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
  model.lambda = 0.99;
  model.delta = 0.001;
  model.terminated = 0;
  model.updated = 0;
  model.active = 0;

  model.W_n.x00 = (float)(rand() % 100) / 100.0;
  model.W_n.x01 = (float)(rand() % 100) / 100.0;
  model.W_n.x02 = (float)(rand() % 100) / 100.0;
  model.W_n.x03 = (float)(rand() % 100) / 100.0;
  model.W_n.x04 = (float)(rand() % 100) / 100.0;
  model.W_n.x05 = (float)(rand() % 100) / 100.0;
  model.W_n.x06 = (float)(rand() % 100) / 100.0;
  model.W_n.x07 = (float)(rand() % 100) / 100.0;
  model.W_n.x10 = (float)(rand() % 100) / 100.0;
  model.W_n.x11 = (float)(rand() % 100) / 100.0;
  model.W_n.x12 = (float)(rand() % 100) / 100.0;
  model.W_n.x13 = (float)(rand() % 100) / 100.0;
  model.W_n.x14 = (float)(rand() % 100) / 100.0;
  model.W_n.x15 = (float)(rand() % 100) / 100.0;
  model.W_n.x16 = (float)(rand() % 100) / 100.0;
  model.W_n.x17 = (float)(rand() % 100) / 100.0;
  model.W_n.x20 = (float)(rand() % 100) / 100.0;
  model.W_n.x21 = (float)(rand() % 100) / 100.0;
  model.W_n.x22 = (float)(rand() % 100) / 100.0;
  model.W_n.x23 = (float)(rand() % 100) / 100.0;
  model.W_n.x24 = (float)(rand() % 100) / 100.0;
  model.W_n.x25 = (float)(rand() % 100) / 100.0;
  model.W_n.x26 = (float)(rand() % 100) / 100.0;
  model.W_n.x27 = (float)(rand() % 100) / 100.0;
  model.W_n.x30 = (float)(rand() % 100) / 100.0;
  model.W_n.x31 = (float)(rand() % 100) / 100.0;
  model.W_n.x32 = (float)(rand() % 100) / 100.0;
  model.W_n.x33 = (float)(rand() % 100) / 100.0;
  model.W_n.x34 = (float)(rand() % 100) / 100.0;
  model.W_n.x35 = (float)(rand() % 100) / 100.0;
  model.W_n.x36 = (float)(rand() % 100) / 100.0;
  model.W_n.x37 = (float)(rand() % 100) / 100.0;
  model.W_n.x40 = (float)(rand() % 100) / 100.0;
  model.W_n.x41 = (float)(rand() % 100) / 100.0;
  model.W_n.x42 = (float)(rand() % 100) / 100.0;
  model.W_n.x43 = (float)(rand() % 100) / 100.0;
  model.W_n.x44 = (float)(rand() % 100) / 100.0;
  model.W_n.x45 = (float)(rand() % 100) / 100.0;
  model.W_n.x46 = (float)(rand() % 100) / 100.0;
  model.W_n.x47 = (float)(rand() % 100) / 100.0;
  model.W_n.x50 = (float)(rand() % 100) / 100.0;
  model.W_n.x51 = (float)(rand() % 100) / 100.0;
  model.W_n.x52 = (float)(rand() % 100) / 100.0;
  model.W_n.x53 = (float)(rand() % 100) / 100.0;
  model.W_n.x54 = (float)(rand() % 100) / 100.0;
  model.W_n.x55 = (float)(rand() % 100) / 100.0;
  model.W_n.x56 = (float)(rand() % 100) / 100.0;
  model.W_n.x57 = (float)(rand() % 100) / 100.0;
  model.W_n.x60 = (float)(rand() % 100) / 100.0;
  model.W_n.x61 = (float)(rand() % 100) / 100.0;
  model.W_n.x62 = (float)(rand() % 100) / 100.0;
  model.W_n.x63 = (float)(rand() % 100) / 100.0;
  model.W_n.x64 = (float)(rand() % 100) / 100.0;
  model.W_n.x65 = (float)(rand() % 100) / 100.0;
  model.W_n.x66 = (float)(rand() % 100) / 100.0;
  model.W_n.x67 = (float)(rand() % 100) / 100.0;
  model.W_n.x70 = (float)(rand() % 100) / 100.0;
  model.W_n.x71 = (float)(rand() % 100) / 100.0;
  model.W_n.x72 = (float)(rand() % 100) / 100.0;
  model.W_n.x73 = (float)(rand() % 100) / 100.0;
  model.W_n.x74 = (float)(rand() % 100) / 100.0;
  model.W_n.x75 = (float)(rand() % 100) / 100.0;
  model.W_n.x76 = (float)(rand() % 100) / 100.0;
  model.W_n.x77 = (float)(rand() % 100) / 100.0;

  model.P_n.x00 = 1.0 / model.delta;
  model.P_n.x01 = 0.0;
  model.P_n.x02 = 0.0;
  model.P_n.x03 = 0.0;
  model.P_n.x04 = 0.0;
  model.P_n.x05 = 0.0;
  model.P_n.x06 = 0.0;
  model.P_n.x07 = 0.0;
  model.P_n.x10 = 0.0;
  model.P_n.x11 = 1.0 / model.delta;
  model.P_n.x12 = 0.0;
  model.P_n.x13 = 0.0;
  model.P_n.x14 = 0.0;
  model.P_n.x15 = 0.0;
  model.P_n.x16 = 0.0;
  model.P_n.x17 = 0.0;
  model.P_n.x20 = 0.0;
  model.P_n.x21 = 0.0;
  model.P_n.x22 = 1.0 / model.delta;
  model.P_n.x23 = 0.0;
  model.P_n.x24 = 0.0;
  model.P_n.x25 = 0.0;
  model.P_n.x26 = 0.0;
  model.P_n.x27 = 0.0;
  model.P_n.x30 = 0.0;
  model.P_n.x31 = 0.0;
  model.P_n.x32 = 0.0;
  model.P_n.x33 = 1.0 / model.delta;
  model.P_n.x34 = 0.0;
  model.P_n.x35 = 0.0;
  model.P_n.x36 = 0.0;
  model.P_n.x37 = 0.0;
  model.P_n.x40 = 0.0;
  model.P_n.x41 = 0.0;
  model.P_n.x42 = 0.0;
  model.P_n.x43 = 0.0;
  model.P_n.x44 = 1.0 / model.delta;
  model.P_n.x45 = 0.0;
  model.P_n.x46 = 0.0;
  model.P_n.x47 = 0.0;
  model.P_n.x50 = 0.0;
  model.P_n.x51 = 0.0;
  model.P_n.x52 = 0.0;
  model.P_n.x53 = 0.0;
  model.P_n.x54 = 0.0;
  model.P_n.x55 = 1.0 / model.delta;
  model.P_n.x56 = 0.0;
  model.P_n.x57 = 0.0;
  model.P_n.x60 = 0.0;
  model.P_n.x61 = 0.0;
  model.P_n.x62 = 0.0;
  model.P_n.x63 = 0.0;
  model.P_n.x64 = 0.0;
  model.P_n.x65 = 0.0;
  model.P_n.x66 = 1.0 / model.delta;
  model.P_n.x67 = 0.0;
  model.P_n.x70 = 0.0;
  model.P_n.x71 = 0.0;
  model.P_n.x72 = 0.0;
  model.P_n.x73 = 0.0;
  model.P_n.x74 = 0.0;
  model.P_n.x75 = 0.0;
  model.P_n.x76 = 0.0;
  model.P_n.x77 = 1.0 / model.delta;

  model.K_j.x00 = (float)(rand() % 100) / 100.0;
  model.K_j.x01 = (float)(rand() % 100) / 100.0;
  model.K_j.x02 = (float)(rand() % 100) / 100.0;
  model.K_j.x03 = (float)(rand() % 100) / 100.0;
  model.K_j.x04 = (float)(rand() % 100) / 100.0;
  model.K_j.x05 = (float)(rand() % 100) / 100.0;
  model.K_j.x10 = (float)(rand() % 100) / 100.0;
  model.K_j.x11 = (float)(rand() % 100) / 100.0;
  model.K_j.x12 = (float)(rand() % 100) / 100.0;
  model.K_j.x13 = (float)(rand() % 100) / 100.0;
  model.K_j.x14 = (float)(rand() % 100) / 100.0;
  model.K_j.x15 = (float)(rand() % 100) / 100.0;

  model.dataset.x0 = 0.0;
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
  model.dataset.x11 = 0.0;
  model.dataset.x12 = 0.0;
  model.dataset.x13 = 0.0;
  model.dataset.x14 = 0.0;
  model.dataset.x15 = 0.0;
  IMU imu = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.51, -0.60, -0.06, 28.25, 137.0, 7.88};
  Encoder encoder = {0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 30, 50.0, 0.0, 0.0, 0.0};
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
  x_k[0] = model.dataset.x0;
  x_k[1] = model.dataset.x1;
  x_k[2] = model.dataset.x2;
  x_k[3] = model.dataset.x3;
  x_k[4] = model.dataset.x4;
  x_k[5] = model.dataset.x5;
  K_j[0][0] = model.K_j.x00;
  K_j[0][1] = model.K_j.x01;
  K_j[0][2] = model.K_j.x02;
  K_j[0][3] = model.K_j.x03;
  K_j[0][4] = model.K_j.x04;
  K_j[0][5] = model.K_j.x05;
  K_j[1][0] = model.K_j.x10;
  K_j[1][1] = model.K_j.x11;
  K_j[1][2] = model.K_j.x12;
  K_j[1][3] = model.K_j.x13;
  K_j[1][4] = model.K_j.x14;
  K_j[1][5] = model.K_j.x15;
  u_k[0] = 0.0;
  u_k[1] = 0.0;
  // feeback policy
  for (int i = 0; i < model.m; i++)
  {
    for (int j = 0; j < model.n; j++)
    {
      u_k[i] += -K_j[i][j] * x_k[j];
    }
  }
  for (int i = 0; i < model.m; i++)
  {
    u_k[i] = sigmoid(u_k[i]);
  }
  // act!
  model.dataset.x0 = -model.imu.calibrated_acc_y;
  model.dataset.x1 = model.imu.calibrated_acc_y_velocity;
  model.dataset.x2 = model.imu.calibrated_acc_y_acceleration;
  model.dataset.x3 = -pow(model.imu.calibrated_acc_y, 2);
  model.dataset.x4 = pow(model.imu.calibrated_acc_y_velocity, 2);
  model.dataset.x5 = pow(model.imu.calibrated_acc_y_acceleration, 2);
  model.dataset.x6 = u_k[0];
  model.dataset.x7 = u_k[1];
  int index = argmax(u_k, M);
  float action = sigmoid(u_k[index]);
  if (model.active == 1)
  {
    // 0-90 clockwise
    // 90-135 anti-clockwise
    servoAngle = index == 0 ? servoAngle + action : servoAngle - action;
    servoAngle = fmin(servoAngle, 125.0);
    servoAngle = fmax(servoAngle, 20.0);
    setServoAngle(servoAngle);
  }
  else
  {
    servoAngle = 90.0;
    setServoAngle(servoAngle);
  }
  // dataset = (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  model.dataset.x8 = -model.imu.calibrated_acc_y;
  model.dataset.x9 = model.imu.calibrated_acc_y_velocity;
  model.dataset.x10 = model.imu.calibrated_acc_y_acceleration;
  model.dataset.x11 = -pow(model.imu.calibrated_acc_y, 2);
  model.dataset.x12 = pow(model.imu.calibrated_acc_y_velocity, 2);
  model.dataset.x13 = pow(model.imu.calibrated_acc_y_acceleration, 2);
  x_k1[0] = model.dataset.x8;
  x_k1[1] = model.dataset.x9;
  x_k1[2] = model.dataset.x10;
  x_k1[3] = model.dataset.x11;
  x_k1[4] = model.dataset.x12;
  x_k1[5] = model.dataset.x13;
  u_k1[0] = 0.0;
  u_k1[1] = 0.0;
  for (int i = 0; i < model.m; i++)
  {
    for (int j = 0; j < model.n; j++)
    {
      u_k1[i] += -K_j[i][j] * x_k1[j];
    }
  }
  for (int i = 0; i < model.m; i++)
  {
    u_k1[i] = sigmoid(u_k1[i]);
  }
  model.dataset.x14 = u_k1[0];
  model.dataset.x15 = u_k1[1];
  // Compute the quadratic basis sets ϕ(zₖ), ϕ(zₖ₊₁).
  z_k[0] = model.dataset.x0;
  z_k[1] = model.dataset.x1;
  z_k[2] = model.dataset.x2;
  z_k[3] = model.dataset.x3;
  z_k[4] = model.dataset.x4;
  z_k[5] = model.dataset.x5;
  z_k[6] = model.dataset.x6;
  z_k[7] = model.dataset.x7;
  z_k1[0] = model.dataset.x8;
  z_k1[1] = model.dataset.x9;
  z_k1[2] = model.dataset.x10;
  z_k1[3] = model.dataset.x11;
  z_k1[4] = model.dataset.x12;
  z_k1[5] = model.dataset.x13;
  z_k1[6] = model.dataset.x14;
  z_k1[7] = model.dataset.x15;
  for (int i = 0; i < model.n + model.m; i++)
  {
    basisset0[i] = sigmoid(z_k[i]);
    basisset1[i] = sigmoid(z_k1[i]);
  }
  // Now perform a one-step update in the parameter vector W by applying RLS to equation (S27).
  P_n[0][0] = model.P_n.x00;
  P_n[0][1] = model.P_n.x01;
  P_n[0][2] = model.P_n.x02;
  P_n[0][3] = model.P_n.x03;
  P_n[0][4] = model.P_n.x04;
  P_n[0][5] = model.P_n.x05;
  P_n[0][6] = model.P_n.x06;
  P_n[0][7] = model.P_n.x07;
  P_n[1][0] = model.P_n.x10;
  P_n[1][1] = model.P_n.x11;
  P_n[1][2] = model.P_n.x12;
  P_n[1][3] = model.P_n.x13;
  P_n[1][4] = model.P_n.x14;
  P_n[1][5] = model.P_n.x15;
  P_n[1][6] = model.P_n.x16;
  P_n[1][7] = model.P_n.x17;
  P_n[2][0] = model.P_n.x20;
  P_n[2][1] = model.P_n.x21;
  P_n[2][2] = model.P_n.x22;
  P_n[2][3] = model.P_n.x23;
  P_n[2][4] = model.P_n.x24;
  P_n[2][5] = model.P_n.x25;
  P_n[2][6] = model.P_n.x26;
  P_n[2][7] = model.P_n.x27;
  P_n[3][0] = model.P_n.x30;
  P_n[3][1] = model.P_n.x31;
  P_n[3][2] = model.P_n.x32;
  P_n[3][3] = model.P_n.x33;
  P_n[3][4] = model.P_n.x34;
  P_n[3][5] = model.P_n.x35;
  P_n[3][6] = model.P_n.x36;
  P_n[3][7] = model.P_n.x37;
  P_n[4][0] = model.P_n.x40;
  P_n[4][1] = model.P_n.x41;
  P_n[4][2] = model.P_n.x42;
  P_n[4][3] = model.P_n.x43;
  P_n[4][4] = model.P_n.x44;
  P_n[4][5] = model.P_n.x45;
  P_n[4][6] = model.P_n.x46;
  P_n[4][7] = model.P_n.x47;
  P_n[5][0] = model.P_n.x50;
  P_n[5][1] = model.P_n.x51;
  P_n[5][2] = model.P_n.x52;
  P_n[5][3] = model.P_n.x53;
  P_n[5][4] = model.P_n.x54;
  P_n[5][5] = model.P_n.x55;
  P_n[5][6] = model.P_n.x56;
  P_n[5][7] = model.P_n.x57;
  P_n[6][0] = model.P_n.x60;
  P_n[6][1] = model.P_n.x61;
  P_n[6][2] = model.P_n.x62;
  P_n[6][3] = model.P_n.x63;
  P_n[6][4] = model.P_n.x64;
  P_n[6][5] = model.P_n.x65;
  P_n[6][6] = model.P_n.x66;
  P_n[6][7] = model.P_n.x67;
  P_n[7][0] = model.P_n.x70;
  P_n[7][1] = model.P_n.x71;
  P_n[7][2] = model.P_n.x72;
  P_n[7][3] = model.P_n.x73;
  P_n[7][4] = model.P_n.x74;
  P_n[7][5] = model.P_n.x75;
  P_n[7][6] = model.P_n.x76;
  P_n[7][7] = model.P_n.x77;

  W_n[0][0] = model.W_n.x00;
  W_n[0][1] = model.W_n.x01;
  W_n[0][2] = model.W_n.x02;
  W_n[0][3] = model.W_n.x03;
  W_n[0][4] = model.W_n.x04;
  W_n[0][5] = model.W_n.x05;
  W_n[0][6] = model.W_n.x06;
  W_n[0][7] = model.W_n.x07;
  W_n[1][0] = model.W_n.x10;
  W_n[1][1] = model.W_n.x11;
  W_n[1][2] = model.W_n.x12;
  W_n[1][3] = model.W_n.x13;
  W_n[1][4] = model.W_n.x14;
  W_n[1][5] = model.W_n.x15;
  W_n[1][6] = model.W_n.x16;
  W_n[1][7] = model.W_n.x17;
  W_n[2][0] = model.W_n.x20;
  W_n[2][1] = model.W_n.x21;
  W_n[2][2] = model.W_n.x22;
  W_n[2][3] = model.W_n.x23;
  W_n[2][4] = model.W_n.x24;
  W_n[2][5] = model.W_n.x25;
  W_n[2][6] = model.W_n.x26;
  W_n[2][7] = model.W_n.x27;
  W_n[3][0] = model.W_n.x30;
  W_n[3][1] = model.W_n.x31;
  W_n[3][2] = model.W_n.x32;
  W_n[3][3] = model.W_n.x33;
  W_n[3][4] = model.W_n.x34;
  W_n[3][5] = model.W_n.x35;
  W_n[3][6] = model.W_n.x36;
  W_n[3][7] = model.W_n.x37;
  W_n[4][0] = model.W_n.x40;
  W_n[4][1] = model.W_n.x41;
  W_n[4][2] = model.W_n.x42;
  W_n[4][3] = model.W_n.x43;
  W_n[4][4] = model.W_n.x44;
  W_n[4][5] = model.W_n.x45;
  W_n[4][6] = model.W_n.x46;
  W_n[4][7] = model.W_n.x47;
  W_n[5][0] = model.W_n.x50;
  W_n[5][1] = model.W_n.x51;
  W_n[5][2] = model.W_n.x52;
  W_n[5][3] = model.W_n.x53;
  W_n[5][4] = model.W_n.x54;
  W_n[5][5] = model.W_n.x55;
  W_n[5][6] = model.W_n.x56;
  W_n[5][7] = model.W_n.x57;
  W_n[6][0] = model.W_n.x60;
  W_n[6][1] = model.W_n.x61;
  W_n[6][2] = model.W_n.x62;
  W_n[6][3] = model.W_n.x63;
  W_n[6][4] = model.W_n.x64;
  W_n[6][5] = model.W_n.x65;
  W_n[6][6] = model.W_n.x66;
  W_n[6][7] = model.W_n.x67;
  W_n[7][0] = model.W_n.x70;
  W_n[7][1] = model.W_n.x71;
  W_n[7][2] = model.W_n.x72;
  W_n[7][3] = model.W_n.x73;
  W_n[7][4] = model.W_n.x74;
  W_n[7][5] = model.W_n.x75;
  W_n[7][6] = model.W_n.x76;
  W_n[7][7] = model.W_n.x77;

  // initialize z_n
  for (int i = 0; i < model.n + model.m; i++)
  {
    z_n[i] = 0.0;
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      z_n[i] += P_n[i][j] * z_k[j];
    }
  }
  float z_k_dot_z_n = 0.0;
  for (int i = 0; i < model.n + model.m; i++)
  {
    z_k_dot_z_n += z_k[i] * z_n[i];
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    g_n[i] = 1.0 / (model.lambda + z_k_dot_z_n) * z_n[i];
  }
  // αₙ = dₙ - transpose(wₙ₋₁) * xₙ
  // initialize alpha_n
  for (int i = 0; i < model.n + model.m; i++)
  {
    alpha_n[i] = 0.0;
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      alpha_n[i] += W_n[i][j] * (basisset0[j] - basisset1[j]); // checked manually
    }
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      W_n[i][j] = W_n[i][j] + (alpha_n[i] * g_n[j]); // checked manually
    }
  }
  for (int i = 0; i < model.n + model.m; i++)
  {
    for (int j = 0; j < model.n + model.m; j++)
    {
      P_n[i][j] = (1.0 / model.lambda) * (P_n[i][j] - g_n[i] * z_n[j]); // checked manually
    }
  }
  model.W_n.x00 = clip_by_value(W_n[0][0], clip_value);
  model.W_n.x01 = clip_by_value(W_n[0][1], clip_value);
  model.W_n.x02 = clip_by_value(W_n[0][2], clip_value);
  model.W_n.x03 = clip_by_value(W_n[0][3], clip_value);
  model.W_n.x04 = clip_by_value(W_n[0][4], clip_value);
  model.W_n.x05 = clip_by_value(W_n[0][5], clip_value);
  model.W_n.x06 = clip_by_value(W_n[0][6], clip_value);
  model.W_n.x07 = clip_by_value(W_n[0][7], clip_value);
  model.W_n.x10 = clip_by_value(W_n[1][0], clip_value);
  model.W_n.x11 = clip_by_value(W_n[1][1], clip_value);
  model.W_n.x12 = clip_by_value(W_n[1][2], clip_value);
  model.W_n.x13 = clip_by_value(W_n[1][3], clip_value);
  model.W_n.x14 = clip_by_value(W_n[1][4], clip_value);
  model.W_n.x15 = clip_by_value(W_n[1][5], clip_value);
  model.W_n.x16 = clip_by_value(W_n[1][6], clip_value);
  model.W_n.x17 = clip_by_value(W_n[1][7], clip_value);
  model.W_n.x20 = clip_by_value(W_n[2][0], clip_value);
  model.W_n.x21 = clip_by_value(W_n[2][1], clip_value);
  model.W_n.x22 = clip_by_value(W_n[2][2], clip_value);
  model.W_n.x23 = clip_by_value(W_n[2][3], clip_value);
  model.W_n.x24 = clip_by_value(W_n[2][4], clip_value);
  model.W_n.x25 = clip_by_value(W_n[2][5], clip_value);
  model.W_n.x26 = clip_by_value(W_n[2][6], clip_value);
  model.W_n.x27 = clip_by_value(W_n[2][7], clip_value);
  model.W_n.x30 = clip_by_value(W_n[3][0], clip_value);
  model.W_n.x31 = clip_by_value(W_n[3][1], clip_value);
  model.W_n.x32 = clip_by_value(W_n[3][2], clip_value);
  model.W_n.x33 = clip_by_value(W_n[3][3], clip_value);
  model.W_n.x34 = clip_by_value(W_n[3][4], clip_value);
  model.W_n.x35 = clip_by_value(W_n[3][5], clip_value);
  model.W_n.x36 = clip_by_value(W_n[3][6], clip_value);
  model.W_n.x37 = clip_by_value(W_n[3][7], clip_value);
  model.W_n.x40 = clip_by_value(W_n[4][0], clip_value);
  model.W_n.x41 = clip_by_value(W_n[4][1], clip_value);
  model.W_n.x42 = clip_by_value(W_n[4][2], clip_value);
  model.W_n.x43 = clip_by_value(W_n[4][3], clip_value);
  model.W_n.x44 = clip_by_value(W_n[4][4], clip_value);
  model.W_n.x45 = clip_by_value(W_n[4][5], clip_value);
  model.W_n.x46 = clip_by_value(W_n[4][6], clip_value);
  model.W_n.x47 = clip_by_value(W_n[4][7], clip_value);
  model.W_n.x50 = clip_by_value(W_n[5][0], clip_value);
  model.W_n.x51 = clip_by_value(W_n[5][1], clip_value);
  model.W_n.x52 = clip_by_value(W_n[5][2], clip_value);
  model.W_n.x53 = clip_by_value(W_n[5][3], clip_value);
  model.W_n.x54 = clip_by_value(W_n[5][4], clip_value);
  model.W_n.x55 = clip_by_value(W_n[5][5], clip_value);
  model.W_n.x56 = clip_by_value(W_n[5][6], clip_value);
  model.W_n.x57 = clip_by_value(W_n[5][7], clip_value);
  model.W_n.x60 = clip_by_value(W_n[6][0], clip_value);
  model.W_n.x61 = clip_by_value(W_n[6][1], clip_value);
  model.W_n.x62 = clip_by_value(W_n[6][2], clip_value);
  model.W_n.x63 = clip_by_value(W_n[6][3], clip_value);
  model.W_n.x64 = clip_by_value(W_n[6][4], clip_value);
  model.W_n.x65 = clip_by_value(W_n[6][5], clip_value);
  model.W_n.x66 = clip_by_value(W_n[6][6], clip_value);
  model.W_n.x67 = clip_by_value(W_n[6][7], clip_value);
  model.W_n.x70 = clip_by_value(W_n[7][0], clip_value);
  model.W_n.x71 = clip_by_value(W_n[7][1], clip_value);
  model.W_n.x72 = clip_by_value(W_n[7][2], clip_value);
  model.W_n.x73 = clip_by_value(W_n[7][3], clip_value);
  model.W_n.x74 = clip_by_value(W_n[7][4], clip_value);
  model.W_n.x75 = clip_by_value(W_n[7][5], clip_value);
  model.W_n.x76 = clip_by_value(W_n[7][6], clip_value);
  model.W_n.x77 = clip_by_value(W_n[7][7], clip_value);

  model.P_n.x00 = clip_by_value(P_n[0][0], clip_value);
  model.P_n.x01 = clip_by_value(P_n[0][1], clip_value);
  model.P_n.x02 = clip_by_value(P_n[0][2], clip_value);
  model.P_n.x03 = clip_by_value(P_n[0][3], clip_value);
  model.P_n.x04 = clip_by_value(P_n[0][4], clip_value);
  model.P_n.x05 = clip_by_value(P_n[0][5], clip_value);
  model.P_n.x06 = clip_by_value(P_n[0][6], clip_value);
  model.P_n.x07 = clip_by_value(P_n[0][7], clip_value);
  model.P_n.x10 = clip_by_value(P_n[1][0], clip_value);
  model.P_n.x11 = clip_by_value(P_n[1][1], clip_value);
  model.P_n.x12 = clip_by_value(P_n[1][2], clip_value);
  model.P_n.x13 = clip_by_value(P_n[1][3], clip_value);
  model.P_n.x14 = clip_by_value(P_n[1][4], clip_value);
  model.P_n.x15 = clip_by_value(P_n[1][5], clip_value);
  model.P_n.x16 = clip_by_value(P_n[1][6], clip_value);
  model.P_n.x17 = clip_by_value(P_n[1][7], clip_value);
  model.P_n.x20 = clip_by_value(P_n[2][0], clip_value);
  model.P_n.x21 = clip_by_value(P_n[2][1], clip_value);
  model.P_n.x22 = clip_by_value(P_n[2][2], clip_value);
  model.P_n.x23 = clip_by_value(P_n[2][3], clip_value);
  model.P_n.x24 = clip_by_value(P_n[2][4], clip_value);
  model.P_n.x25 = clip_by_value(P_n[2][5], clip_value);
  model.P_n.x26 = clip_by_value(P_n[2][6], clip_value);
  model.P_n.x27 = clip_by_value(P_n[2][7], clip_value);
  model.P_n.x30 = clip_by_value(P_n[3][0], clip_value);
  model.P_n.x31 = clip_by_value(P_n[3][1], clip_value);
  model.P_n.x32 = clip_by_value(P_n[3][2], clip_value);
  model.P_n.x33 = clip_by_value(P_n[3][3], clip_value);
  model.P_n.x34 = clip_by_value(P_n[3][4], clip_value);
  model.P_n.x35 = clip_by_value(P_n[3][5], clip_value);
  model.P_n.x36 = clip_by_value(P_n[3][6], clip_value);
  model.P_n.x37 = clip_by_value(P_n[3][7], clip_value);
  model.P_n.x40 = clip_by_value(P_n[4][0], clip_value);
  model.P_n.x41 = clip_by_value(P_n[4][1], clip_value);
  model.P_n.x42 = clip_by_value(P_n[4][2], clip_value);
  model.P_n.x43 = clip_by_value(P_n[4][3], clip_value);
  model.P_n.x44 = clip_by_value(P_n[4][4], clip_value);
  model.P_n.x45 = clip_by_value(P_n[4][5], clip_value);
  model.P_n.x46 = clip_by_value(P_n[4][6], clip_value);
  model.P_n.x47 = clip_by_value(P_n[4][7], clip_value);
  model.P_n.x50 = clip_by_value(P_n[5][0], clip_value);
  model.P_n.x51 = clip_by_value(P_n[5][1], clip_value);
  model.P_n.x52 = clip_by_value(P_n[5][2], clip_value);
  model.P_n.x53 = clip_by_value(P_n[5][3], clip_value);
  model.P_n.x54 = clip_by_value(P_n[5][4], clip_value);
  model.P_n.x55 = clip_by_value(P_n[5][5], clip_value);
  model.P_n.x56 = clip_by_value(P_n[5][6], clip_value);
  model.P_n.x57 = clip_by_value(P_n[5][7], clip_value);
  model.P_n.x60 = clip_by_value(P_n[6][0], clip_value);
  model.P_n.x61 = clip_by_value(P_n[6][1], clip_value);
  model.P_n.x62 = clip_by_value(P_n[6][2], clip_value);
  model.P_n.x63 = clip_by_value(P_n[6][3], clip_value);
  model.P_n.x64 = clip_by_value(P_n[6][4], clip_value);
  model.P_n.x65 = clip_by_value(P_n[6][5], clip_value);
  model.P_n.x66 = clip_by_value(P_n[6][6], clip_value);
  model.P_n.x67 = clip_by_value(P_n[6][7], clip_value);
  model.P_n.x70 = clip_by_value(P_n[7][0], clip_value);
  model.P_n.x71 = clip_by_value(P_n[7][1], clip_value);
  model.P_n.x72 = clip_by_value(P_n[7][2], clip_value);
  model.P_n.x73 = clip_by_value(P_n[7][3], clip_value);
  model.P_n.x74 = clip_by_value(P_n[7][4], clip_value);
  model.P_n.x75 = clip_by_value(P_n[7][5], clip_value);
  model.P_n.x76 = clip_by_value(P_n[7][6], clip_value);
  model.P_n.x77 = clip_by_value(P_n[7][7], clip_value);
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
  // initialize the filter matrix
  W_n[0][0] = model.W_n.x00;
  W_n[0][1] = model.W_n.x01;
  W_n[0][2] = model.W_n.x02;
  W_n[0][3] = model.W_n.x03;
  W_n[0][4] = model.W_n.x04;
  W_n[0][5] = model.W_n.x05;
  W_n[0][6] = model.W_n.x06;
  W_n[0][7] = model.W_n.x07;
  W_n[1][0] = model.W_n.x10;
  W_n[1][1] = model.W_n.x11;
  W_n[1][2] = model.W_n.x12;
  W_n[1][3] = model.W_n.x13;
  W_n[1][4] = model.W_n.x14;
  W_n[1][5] = model.W_n.x15;
  W_n[1][6] = model.W_n.x16;
  W_n[1][7] = model.W_n.x17;
  W_n[2][0] = model.W_n.x20;
  W_n[2][1] = model.W_n.x21;
  W_n[2][2] = model.W_n.x22;
  W_n[2][3] = model.W_n.x23;
  W_n[2][4] = model.W_n.x24;
  W_n[2][5] = model.W_n.x25;
  W_n[2][6] = model.W_n.x26;
  W_n[2][7] = model.W_n.x27;
  W_n[3][0] = model.W_n.x30;
  W_n[3][1] = model.W_n.x31;
  W_n[3][2] = model.W_n.x32;
  W_n[3][3] = model.W_n.x33;
  W_n[3][4] = model.W_n.x34;
  W_n[3][5] = model.W_n.x35;
  W_n[3][6] = model.W_n.x36;
  W_n[3][7] = model.W_n.x37;
  W_n[4][0] = model.W_n.x40;
  W_n[4][1] = model.W_n.x41;
  W_n[4][2] = model.W_n.x42;
  W_n[4][3] = model.W_n.x43;
  W_n[4][4] = model.W_n.x44;
  W_n[4][5] = model.W_n.x45;
  W_n[4][6] = model.W_n.x46;
  W_n[4][7] = model.W_n.x47;
  W_n[5][0] = model.W_n.x50;
  W_n[5][1] = model.W_n.x51;
  W_n[5][2] = model.W_n.x52;
  W_n[5][3] = model.W_n.x53;
  W_n[5][4] = model.W_n.x54;
  W_n[5][5] = model.W_n.x55;
  W_n[5][6] = model.W_n.x56;
  W_n[5][7] = model.W_n.x57;
  W_n[6][0] = model.W_n.x60;
  W_n[6][1] = model.W_n.x61;
  W_n[6][2] = model.W_n.x62;
  W_n[6][3] = model.W_n.x63;
  W_n[6][4] = model.W_n.x64;
  W_n[6][5] = model.W_n.x65;
  W_n[6][6] = model.W_n.x66;
  W_n[6][7] = model.W_n.x67;
  W_n[7][0] = model.W_n.x70;
  W_n[7][1] = model.W_n.x71;
  W_n[7][2] = model.W_n.x72;
  W_n[7][3] = model.W_n.x73;
  W_n[7][4] = model.W_n.x74;
  W_n[7][5] = model.W_n.x75;
  W_n[7][6] = model.W_n.x76;
  W_n[7][7] = model.W_n.x77;

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
    S_uu_inverse[0][0] = S_uu[1][1] / determinant;
    S_uu_inverse[0][1] = -S_uu[0][1] / determinant;
    S_uu_inverse[1][0] = -S_uu[1][0] / determinant;
    S_uu_inverse[1][1] = S_uu[0][0] / determinant;
    // initialize the gain matrix
    for (int i = 0; i < model.m; i++)
    {
      for (int j = 0; j < model.n; j++)
      {
        K_j[i][j] = 0.0;
      }
    }
    for (int i = 0; i < model.m; i++)
    {
      for (int j = 0; j < model.n; j++)
      {
        for (int k = 0; k < model.m; k++)
        {
          K_j[i][j] += S_uu_inverse[i][k] * S_ux[k][j];
        }
      }
    }
    model.K_j.x00 = K_j[0][0];
    model.K_j.x01 = K_j[0][1];
    model.K_j.x02 = K_j[0][2];
    model.K_j.x03 = K_j[0][3];
    model.K_j.x04 = K_j[0][4];
    model.K_j.x05 = K_j[0][5];
    model.K_j.x10 = K_j[1][0];
    model.K_j.x11 = K_j[1][1];
    model.K_j.x12 = K_j[1][2];
    model.K_j.x13 = K_j[1][3];
    model.K_j.x14 = K_j[1][4];
    model.K_j.x15 = K_j[1][5];
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
  int log_status = 0;
  unsigned long t1 = 0;
  unsigned long t2 = 0;
  unsigned long diff = 0;
  // fields: p, i, d, windup, safety angle, target angle, setpoint, integrator, output and active.
  Controller rolling_wheel_controller = {9.0, 20.0, 1.0, 29.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0};
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
  MX_ADC1_Init();
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
  // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_Delay(30);
  setServoAngle(90.0);
  // initialize the Encoder and IMU
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  HAL_Delay(1);
  model.encoder = updateEncoder(model.encoder);
  model.imu = updateIMU(model.imu);
  HAL_Delay(1);
  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 2);
  HAL_Delay(3000);
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
    // Rinse and repeat :)

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
      model.encoder = updateEncoder(model.encoder);
      model.imu = updateIMU(model.imu);
    }
    if (model.terminated == 1 && model.updated == 0)
    {
      model = updateControlPolicy(model);
    }

    reaction_wheel_speed = servoAngle > 90.0 ? (servoAngle - 90.0) / 45.0 : -(90.0 - servoAngle) / 90.0;
    rolling_wheel_speed = rolling_wheel_controller.output / 255.0;

    log_counter++;
    if (log_counter > LOG_CYCLE)
    {
      transmit = 1;
    }
    if (transmit == 1)
    {
      transmit = 0;
      log_counter = 0;

      if (log_status == 0)
      {
        sprintf(MSG,
                "z: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, j: %d, k: %d, roll: %0.2f, pitch: %0.2f, angle: %0.2f, enc: %d, v1: %0.2f, v2: %0.2f, dt: %0.6f\r\n",
                model.dataset.x0, model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5,
                model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, model.dataset.x10, model.dataset.x11,
                model.j, model.k, model.imu.calibrated_acc_y, model.imu.calibrated_acc_x, model.encoder.angle,
                model.encoder.value0, reaction_wheel_speed, rolling_wheel_speed, dt);
        log_status = 0;
      }
      // else if (log_status == 1)
      // {
      //   sprintf(MSG, "w0: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, w1: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, w2: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, w3: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, w4: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, end\r\n",
      //           model.W_n.x00, model.W_n.x01, model.W_n.x02, model.W_n.x03, model.W_n.x04,
      //           model.W_n.x10, model.W_n.x11, model.W_n.x12, model.W_n.x13, model.W_n.x14,
      //           model.W_n.x20, model.W_n.x21, model.W_n.x22, model.W_n.x23, model.W_n.x24,
      //           model.W_n.x30, model.W_n.x31, model.W_n.x32, model.W_n.x33, model.W_n.x34,
      //           model.W_n.x40, model.W_n.x41, model.W_n.x42, model.W_n.x43, model.W_n.x44);
      //   log_status = 2;
      // }
      // else if (log_status == 2)
      // {
      //   sprintf(MSG, "p0: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, p1: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, p2: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, p3: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, p4: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, end\r\n",
      //           model.P_n.x00, model.P_n.x01, model.P_n.x02, model.P_n.x03, model.P_n.x04,
      //           model.P_n.x10, model.P_n.x11, model.P_n.x12, model.P_n.x13, model.P_n.x14,
      //           model.P_n.x20, model.P_n.x21, model.P_n.x22, model.P_n.x23, model.P_n.x24,
      //           model.P_n.x30, model.P_n.x31, model.P_n.x32, model.P_n.x33, model.P_n.x34,
      //           model.P_n.x40, model.P_n.x41, model.P_n.x42, model.P_n.x43, model.P_n.x44);
      //   log_status = 3;
      // }
      // else if (log_status == 3)
      // {
      //   sprintf(MSG, "k0: %0.2f, %0.2f, %0.2f, k1: %0.2f, %0.2f, %0.2f, end\r\n",
      //           model.K_j.x00, model.K_j.x01, model.K_j.x02,
      //           model.K_j.x10, model.K_j.x11, model.K_j.x12);
      //   log_status = 0;
      // }
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
