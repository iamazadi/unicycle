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
#define N 10
#define M 2
#define WINDOWLENGTH 5
#define SLAVE_ADDRESS 0xA4
#define TRANSFER_REQUEST 0x02
#define MASTER_REQ_ROLL_H 0x14
#define MASTER_REQ_ROLL_L 0x15
#define MASTER_REQ_ACC_X_H 0x08
#define SLAVE_ADDRESS_ICM 0x68 // 0b1101001
#define SLAVE_ADDRESS_ICM_READ 0xD1 // 0b11010011
#define SLAVE_ADDRESS_ICM_WRITE 0xD0 // 0b11010010
#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

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
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x12, 0xC1};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
// response: a4030812f93dfbce0028000100010002ddae16b9ffe227
int deviceReady = 0;
int uart_receive_ok = 0;
int adc_receive_ok = 0;
int sensor_updated = 0;
const float CPU_CLOCK = 84000000.0;
const int dim_n = N;
const int dim_m = M;
const int max_episode_length = 50000;
const float sensor_rotation = -30.0 / 180.0 * M_PI; // sensor frame rotation in X-Y plane
const float roll_coefficient = 10.0;
const float pitch_coefficient = 10.0;
const float yaw_coefficient = 360.0;
const float roll_safety_angle = 0.30;
const float pitch_safety_angle = 0.10;
const int encoderWindowLength = WINDOWLENGTH;
const int currentWindowLength = WINDOWLENGTH;
const float angle = -30.0 / 180.0 * M_PI;
const float imuScale = 2048.0;
float reaction_wheel_pwm = 0.0;
float rolling_wheel_pwm = 0.0;
float reaction_wheel_speed = 0.0;
float rolling_wheel_speed = 0.0;
float gyro_tilt_y = 0.0;
float gyro_velocity_y = 0.0;
// uint8_t transferRequest = MASTER_REQ_ROLL_H;
uint8_t transferRequest = MASTER_REQ_ACC_X_H;
// sampling time
float dt = 0.0;
int encoder0 = 0;
int encoder1 = 0;
int encoderState = 0;
int encoderLastState = 0;
int encoderCounter = 0;
int encoderChange = 0;
int encoderVelocity = 0;
unsigned long interruptTime = 0;
unsigned long encoderTime = 0;
uint8_t raw_data[14] = {0};
// uint8_t i2cAddress[128] = {0};
int connectedDevices = 0;
uint16_t AD_RES = 0;
uint32_t AD_RES_BUFFER[2];
int reactionEncoderWindow[WINDOWLENGTH];
int rollingEncoderWindow[WINDOWLENGTH];
int reactionCurrentWindow[WINDOWLENGTH];
int rollingCurrentWindow[WINDOWLENGTH];
float reaction_wheel_current0 = 0.0;
float reaction_wheel_current1 = 0.0;
float reaction_wheel_current_velocity = 0.0;
float reaction_wheel_current_acceleration = 0.0;
float rolling_wheel_current0 = 0.0;
float rolling_wheel_current1 = 0.0;
float rolling_wheel_current_velocity = 0.0;
float rolling_wheel_current_acceleration = 0.0;
int reaction_wheel_accumulator = 0;
int rolling_wheel_accumulator = 0;
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
// tilt estimation
// the pivot point B̂ in the inertial frame Ô
float pivot[3] = {-0.097, -0.1, -0.032};
// the position of sensors mounted on the body in the body frame of reference
float p1[3] = {-0.035, -0.19, -0.04};
float p2[3] = {0.025, -0.144, -0.07};
// the vectors of the standard basis for the input space ℝ³
float e1[3] = {1.0, 0.0, 0.0};
float e2[3] = {0.0, 1.0, 0.0};
float e3[3] = {0.0, 0.0, 1.0};
// The rotation of the inertial frame Ô to the body frame B̂
float O_B_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
float B_O_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
// The rotation of the local frame of the sensor i to the robot frame B̂
float A1_B_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // [ê[2] ê[1] ê[3]]
float A2_B_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // [ê[1] ê[2] ê[3]]
float B_A1_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // LinearAlgebra.inv(A1_B_R)
float B_A2_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // LinearAlgebra.inv(A2_B_R)
// The matrix of unknown parameters
float Q[3][4] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
// The matrix of sensor locations (known parameters)
float P[4][2] = {{1.0, 1.0}, {-0.043, 0.057}, {0.035, 0.04}, {-0.03, -0.028}}; // [[1.0; vec(p1 - pivot)] [1.0; vec(p2 - pivot)] [1.0; vec(p3 - pivot)] [1.0; vec(p4 - pivot)]]
// The optimal fusion matrix
float X[2][4] = {{0.586913, -11.3087, 0.747681, 0.0}, {0.446183, 8.92749, -3.54337, 0.0}}; // transpose(P) * LinearAlgebra.inv(P * transpose(P))
// accelerometer sensor measurements in the local frame of the sensors
float R1[3] = {0.0, 0.0, 0.0};
float R2[3] = {0.0, 0.0, 0.0};
// accelerometer sensor measurements in the robot body frame
float _R1[3] = {0.0, 0.0, 0.0};
float _R2[3] = {0.0, 0.0, 0.0};
// all sensor measurements combined
float Matrix[3][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
// The gravity vector
float g[3] = {0.0, 0.0, 0.0};
// y-Euler angle (pitch)
float beta = 0.0;
float fused_beta = 0.0;
// x-Euler angle (roll)
float gamma1 = 0.0;
float fused_gamma = 0.0;
// tuning parameters to minimize estimate variance
float kappa1 = 0.05;
float kappa2 = 0.05;
// the average of the body angular rate from rate gyro
float r[3] = {0.0, 0.0, 0.0};
// the average of the body angular rate in Euler angles
float r_dot[3] = {0.0, 0.0, 0.0};
// gyro sensor measurements in the local frame of the sensors
float G1[3] = {0.0, 0.0, 0.0};
float G2[3] = {0.0, 0.0, 0.0};
// gyro sensor measurements in the robot body frame
float _G1[3] = {0.0, 0.0, 0.0};
float _G2[3] = {0.0, 0.0, 0.0};
// a matrix transfom from body rates to Euler angular rates
float E[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_receive_ok == 0 && huart->Instance == USART1)
  {
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
  int direction;              // clockwise or anticlockwise rotation
  int accumulator;            // accumultes the instantanoius changes over a number of intervals
  int interval_counter;       // the number of intervals accumulated
  int intervals;              // maximum number intervals before averaging
  float pulse_per_revolution; // the number of pulses per revolution
  float threshold;            // the threshold for binarizing sensor readings
  float velocity;             // the angular velocity
  float acceleration;         // the angular acceleration
  float jerk;                 // the angular jerk
} Encoder;

typedef struct
{
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
  int16_t accX_offset;
  int16_t accY_offset;
  int16_t accZ_offset;
  int16_t gyrX_offset;
  int16_t gyrY_offset;
  int16_t gyrZ_offset;
  float roll;
  float pitch;
  float yaw;
  float roll_velocity;
  float pitch_velocity;
  float yaw_velocity;
  float roll_acceleration;
  float pitch_acceleration;
  float yaw_acceleration;
} IMU;

void updateEncoder(Encoder *encoder, int *window, int newValue)
{
  encoder->value0 = encoder->value1;
  encoder->value1 = newValue;
  // shift the window one step
  for (int i = 0; i < encoderWindowLength - 1; i++)
  {
    window[i] = window[i + 1];
  }
  window[encoderWindowLength - 1] = encoder->value0 - encoder->value1;
  encoder->accumulator = 0;
  for (int i = 0; i < encoderWindowLength; i++)
  {
    encoder->accumulator += window[i];
  }
  // compute the angular velocity and acceleration
  float velocity = 0.05 * (float)encoder->accumulator / (float)encoderWindowLength;
  velocity = fmin(1.0, velocity);
  velocity = fmax(-1.0, velocity);
  float acceleration = encoder->velocity - velocity;
  encoder->jerk = encoder->acceleration - acceleration;
  encoder->acceleration = acceleration;
  encoder->velocity = velocity;
  encoder->angle = sin((float)(encoder->value1 % 3072) / 3072.0 * 2.0 * M_PI);
  return;
}

void updateCurrentSensing()
{
  // Start ADC Conversion in DMA Mode (Periodically Every 1ms)
  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 2);
  reaction_wheel_current1 = reaction_wheel_current0;
  rolling_wheel_current1 = rolling_wheel_current0;
  reaction_wheel_current0 = (AD_RES_BUFFER[0] << 4);
  rolling_wheel_current0 = (AD_RES_BUFFER[1] << 4);

  // shift the window one step
  for (int i = 0; i < currentWindowLength - 1; i++)
  {
    reactionCurrentWindow[i] = reactionCurrentWindow[i + 1];
    rollingCurrentWindow[i] = rollingCurrentWindow[i + 1];
  }
  reactionCurrentWindow[currentWindowLength - 1] = reaction_wheel_current0 - reaction_wheel_current1;
  rollingCurrentWindow[currentWindowLength - 1] = rolling_wheel_current0 - rolling_wheel_current1;
  reaction_wheel_accumulator = 0;
  rolling_wheel_accumulator = 0;
  for (int i = 0; i < currentWindowLength; i++)
  {
    reaction_wheel_accumulator += reactionCurrentWindow[i];
    rolling_wheel_accumulator += rollingCurrentWindow[i];
  }
  // compute the angular velocity and acceleration
  reaction_wheel_current_acceleration = (float)reaction_wheel_accumulator / (float)currentWindowLength / 20000.0 - reaction_wheel_current_velocity;
  rolling_wheel_current_acceleration = (float)rolling_wheel_accumulator / (float)currentWindowLength / 20000.0 - rolling_wheel_current_velocity;
  reaction_wheel_current_velocity = (float)reaction_wheel_accumulator / (float)currentWindowLength / 20000.0;
  rolling_wheel_current_velocity = (float)rolling_wheel_accumulator / (float)currentWindowLength / 20000.0;
  reaction_wheel_current_velocity = fmin(1.0, reaction_wheel_current_velocity);
  reaction_wheel_current_velocity = fmax(-1.0, reaction_wheel_current_velocity);
  rolling_wheel_current_velocity = fmin(1.0, rolling_wheel_current_velocity);
  rolling_wheel_current_velocity = fmax(-1.0, rolling_wheel_current_velocity);
}


void updateIMU1(IMU *sensor) // GY-25 I2C
{
  do
  {
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_ADDRESS, (uint8_t *)&transferRequest, 1, 10);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      ;
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  do
  {
    HAL_I2C_Master_Receive(&hi2c1, (uint16_t)SLAVE_ADDRESS, (uint8_t *)raw_data, 12, 10);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      ;
    int16_t rawX = (raw_data[0] << 8) | raw_data[1];
    int16_t rawY = (raw_data[2] << 8) | raw_data[3];
    int16_t rawZ = (raw_data[4] << 8) | raw_data[5];
    sensor->accX = (1.0 * rawX) / imuScale;
    sensor->accY = (1.0 * rawY) / imuScale;
    sensor->accZ = (1.0 * rawZ) / imuScale;
    rawX = (raw_data[6] << 8) | raw_data[7];
    rawY = (raw_data[8] << 8) | raw_data[9];
    rawZ = (raw_data[10] << 8) | raw_data[11];
    sensor->gyrX = (1.0 * rawX) / 180.0 * M_PI;
    sensor->gyrY = (1.0 * rawY) / 180.0 * M_PI;
    sensor->gyrZ = (1.0 * rawZ) / 180.0 * M_PI;
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  return;
}


void updateIMU2(IMU *sensor) // GY-25 USART
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
      int16_t rawX = (UART1_rxBuffer[4] << 8) | UART1_rxBuffer[5];
      int16_t rawY = (UART1_rxBuffer[6] << 8) | UART1_rxBuffer[7];
      int16_t rawZ = (UART1_rxBuffer[8] << 8) | UART1_rxBuffer[9];
      float scaledX = (1.0 * rawX) / (5.0 * imuScale);
      float scaledY = (1.0 * rawY) / (5.0 * imuScale);
      float scaledZ = (1.0 * rawZ) / (5.0 * imuScale);
      sensor->accX = -(sin(angle) * scaledX + cos(angle) * scaledY);
      sensor->accY = -(cos(angle) * scaledY - sin(angle) * scaledX);
      sensor->accZ = -scaledZ;
      rawX = (UART1_rxBuffer[10] << 8) | UART1_rxBuffer[11];
      rawY = (UART1_rxBuffer[12] << 8) | UART1_rxBuffer[13];
      rawZ = (UART1_rxBuffer[14] << 8) | UART1_rxBuffer[15];
      scaledX = (1.0 * rawX) / (1024.0) / 180.0 * M_PI;
      scaledY = (1.0 * rawY) / (1024.0) / 180.0 * M_PI;
      scaledZ = (1.0 * rawZ) / (1024.0) / 180.0 * M_PI;
      sensor->gyrX = -(sin(angle) * scaledX + cos(angle) * scaledY);
      sensor->gyrY = -(cos(angle) * scaledY - sin(angle) * scaledX);
      sensor->gyrZ = -scaledZ;
      uart_receive_ok = 0;
      sensor_updated = 1;
    }
  }
  return;
}


// void updateIMU3(IMU *sensor) // MPU9250 I2C
// {
//   uint8_t _transferRequest = 0x3B;
//   // do
//   // {
//   //   HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68 << 1 + 0), _transferRequest, 1, (uint8_t *)raw_data, 14, 10);
//   //   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//   //     ;
//   //     sensor->accX = ((raw_data[0] << 8) | raw_data[1]);
//   //     sensor->accY = ((raw_data[2] << 8) | raw_data[3]);
//   //     sensor->accZ = ((raw_data[4] << 8) | raw_data[5]);
//   //     sensor->gyrX = ((raw_data[8] << 8) | raw_data[9]);
//   //     sensor->gyrY = ((raw_data[10] << 8) | raw_data[11]);
//   //     sensor->gyrZ = ((raw_data[12] << 8) | raw_data[13]);
//   // } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

//   do
//   {
//     HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68 << 1 + 0), _transferRequest, 1, (uint8_t *)raw_data, 6, 100);
//     while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//       ;
//       sensor->accX = ((raw_data[0] << 8) | raw_data[1]);
//       sensor->accY = ((raw_data[2] << 8) | raw_data[3]);
//       sensor->accZ = ((raw_data[4] << 8) | raw_data[5]);
//   } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

//   _transferRequest = 0x43;
//   do
//   {
//     HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68 << 1 + 0), _transferRequest, 1, (uint8_t *)raw_data, 6, 100);
//     while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//       ;
//       sensor->gyrX = ((raw_data[0] << 8) | raw_data[1]);
//       sensor->gyrY = ((raw_data[2] << 8) | raw_data[3]);
//       sensor->gyrZ = ((raw_data[4] << 8) | raw_data[5]);
//   } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
// }


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
  float x16;
  float x17;
  float x18;
  float x19;
  float x20;
  float x21;
  float x22;
  float x23;
} Vec24f;
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
  float x08;
  float x09;
  float x10;
  float x11;
  float x12;
  float x13;
  float x14;
  float x15;
  float x16;
  float x17;
  float x18;
  float x19;
} Mat210f;
typedef struct
{
  float x0000;
  float x0001;
  float x0002;
  float x0003;
  float x0004;
  float x0005;
  float x0006;
  float x0007;
  float x0008;
  float x0009;
  float x0010;
  float x0011;
  float x0100;
  float x0101;
  float x0102;
  float x0103;
  float x0104;
  float x0105;
  float x0106;
  float x0107;
  float x0108;
  float x0109;
  float x0110;
  float x0111;
  float x0200;
  float x0201;
  float x0202;
  float x0203;
  float x0204;
  float x0205;
  float x0206;
  float x0207;
  float x0208;
  float x0209;
  float x0210;
  float x0211;
  float x0300;
  float x0301;
  float x0302;
  float x0303;
  float x0304;
  float x0305;
  float x0306;
  float x0307;
  float x0308;
  float x0309;
  float x0310;
  float x0311;
  float x0400;
  float x0401;
  float x0402;
  float x0403;
  float x0404;
  float x0405;
  float x0406;
  float x0407;
  float x0408;
  float x0409;
  float x0410;
  float x0411;
  float x0500;
  float x0501;
  float x0502;
  float x0503;
  float x0504;
  float x0505;
  float x0506;
  float x0507;
  float x0508;
  float x0509;
  float x0510;
  float x0511;
  float x0600;
  float x0601;
  float x0602;
  float x0603;
  float x0604;
  float x0605;
  float x0606;
  float x0607;
  float x0608;
  float x0609;
  float x0610;
  float x0611;
  float x0700;
  float x0701;
  float x0702;
  float x0703;
  float x0704;
  float x0705;
  float x0706;
  float x0707;
  float x0708;
  float x0709;
  float x0710;
  float x0711;
  float x0800;
  float x0801;
  float x0802;
  float x0803;
  float x0804;
  float x0805;
  float x0806;
  float x0807;
  float x0808;
  float x0809;
  float x0810;
  float x0811;
  float x0900;
  float x0901;
  float x0902;
  float x0903;
  float x0904;
  float x0905;
  float x0906;
  float x0907;
  float x0908;
  float x0909;
  float x0910;
  float x0911;
  float x1000;
  float x1001;
  float x1002;
  float x1003;
  float x1004;
  float x1005;
  float x1006;
  float x1007;
  float x1008;
  float x1009;
  float x1010;
  float x1011;
  float x1100;
  float x1101;
  float x1102;
  float x1103;
  float x1104;
  float x1105;
  float x1106;
  float x1107;
  float x1108;
  float x1109;
  float x1110;
  float x1111;
} Mat12f;
// Represents a Linear Quadratic Regulator (LQR) model.
typedef struct
{
  Mat12f W_n;     // filter matrix
  Mat12f P_n;     // inverse autocorrelation matrix
  Mat210f K_j;    // feedback policy
  Vec24f dataset; // (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
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
  float dt;       // period in seconds
  IMU imu1;
  IMU imu2;
  IMU imu3;
  Encoder ReactionEncoder;
  Encoder RollingEncoder;
} LinearQuadraticRegulator;


void updateIMU(LinearQuadraticRegulator *model)
{
  updateIMU1(&(model->imu1));
  updateIMU2(&(model->imu2));
  // updateIMU3(&(model->imu3)); // causes too large of a delay
  R1[0] = model->imu1.accX;
  R1[1] = model->imu1.accY;
  R1[2] = model->imu1.accZ;
  R2[0] = model->imu2.accX;
  R2[1] = model->imu2.accY;
  R2[2] = model->imu2.accZ;

  _R1[0] = 0.0;
  _R1[1] = 0.0;
  _R1[2] = 0.0;
  _R2[0] = 0.0;
  _R2[1] = 0.0;
  _R2[2] = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      _R1[i] += B_A1_R[i][j] * R1[j];
      _R2[i] += B_A2_R[i][j] * R2[j];
    }
  }

  for (int i = 0; i < 3; i++) {
    Matrix[i][0] = _R1[i];
    Matrix[i][1] = _R2[i];
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      Q[i][j] = 0.0;
      for (int k = 0; k < 2; k++) {
        Q[i][j] += Matrix[i][k] * X[k][j];
      }
    }
  }
  g[0] = Q[0][0];
  g[1] = Q[1][0];
  g[2] = Q[2][0];
  beta = atan2(-g[0], sqrt(pow(g[1], 2) + pow(g[2], 2)));
  gamma1 = atan2(g[1], g[2]);

  G1[0] = model->imu1.gyrX;
  G1[1] = model->imu1.gyrY;
  G1[2] = model->imu1.gyrZ;
  G2[0] = model->imu2.gyrX;
  G2[1] = model->imu2.gyrY;
  G2[2] = model->imu2.gyrZ;

  _G1[0] = 0.0;
  _G1[1] = 0.0;
  _G1[2] = 0.0;
  _G2[0] = 0.0;
  _G2[1] = 0.0;
  _G2[2] = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      _G1[i] += B_A1_R[i][j] * G1[j];
      _G2[i] += B_A2_R[i][j] * G2[j];
    }
  }
  for (int i = 0; i < 3; i++) {
    // r[i] = (_G1[i] + _G2[i]) / 2.0;
    r[i] = _G1[i];
  }

  E[0][0] = 0.0;
  E[0][1] = sin(gamma1) / cos(beta);
  E[0][2] = cos(gamma1) / cos(beta);
  E[1][0] = 0.0;
  E[1][1] = cos(gamma1);
  E[1][2] = -sin(gamma1);
  E[2][0] = 1.0;
  E[2][1] = sin(gamma1) * tan(beta);
  E[2][2] = cos(gamma1) * tan(beta);

  r_dot[0] = 0.0;
  r_dot[1] = 0.0;
  r_dot[2] = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
    }
  }

  fused_beta = kappa1 * beta + (1.0 - kappa1) * (fused_beta + model->dt * (r_dot[1] / 180.0 * M_PI));
  fused_gamma = kappa2 * gamma1 + (1.0 - kappa2) * (fused_gamma + model->dt * (r_dot[2] / 180.0 * M_PI));

  float _roll = fused_beta;
  float _pitch = -fused_gamma;
  float _roll_velocity = r_dot[1] / 180.0 * M_PI;
  float _pitch_velocity = -r_dot[2] / 180.0 * M_PI;
  model->imu1.roll_acceleration = _roll_velocity - model->imu1.roll_velocity;
  model->imu1.pitch_acceleration = _pitch_velocity - model->imu1.pitch_velocity;
  model->imu1.roll_velocity = _roll_velocity;
  model->imu1.pitch_velocity = _pitch_velocity;
  model->imu1.roll = _roll;
  model->imu1.pitch = _pitch;
  model->imu1.roll_velocity = fmax(-1.0, model->imu1.roll_velocity);
  model->imu1.roll_velocity = fmin(1.0, model->imu1.roll_velocity);
  model->imu1.pitch_velocity = fmax(-1.0, model->imu1.pitch_velocity);
  model->imu1.pitch_velocity = fmin(1.0, model->imu1.pitch_velocity);
}


void putBuffer(int m, int n, float buffer[][n], Mat12f matrix) // function prototype
{
  buffer[0][0] = matrix.x0000;
  buffer[0][1] = matrix.x0001;
  buffer[0][2] = matrix.x0002;
  buffer[0][3] = matrix.x0003;
  buffer[0][4] = matrix.x0004;
  buffer[0][5] = matrix.x0005;
  buffer[0][6] = matrix.x0006;
  buffer[0][7] = matrix.x0007;
  buffer[0][8] = matrix.x0008;
  buffer[0][9] = matrix.x0009;
  buffer[0][10] = matrix.x0010;
  buffer[0][11] = matrix.x0011;
  buffer[1][0] = matrix.x0100;
  buffer[1][1] = matrix.x0101;
  buffer[1][2] = matrix.x0102;
  buffer[1][3] = matrix.x0103;
  buffer[1][4] = matrix.x0104;
  buffer[1][5] = matrix.x0105;
  buffer[1][6] = matrix.x0106;
  buffer[1][7] = matrix.x0107;
  buffer[1][8] = matrix.x0108;
  buffer[1][9] = matrix.x0109;
  buffer[1][10] = matrix.x0110;
  buffer[1][11] = matrix.x0111;
  buffer[2][0] = matrix.x0200;
  buffer[2][1] = matrix.x0201;
  buffer[2][2] = matrix.x0202;
  buffer[2][3] = matrix.x0203;
  buffer[2][4] = matrix.x0204;
  buffer[2][5] = matrix.x0205;
  buffer[2][6] = matrix.x0206;
  buffer[2][7] = matrix.x0207;
  buffer[2][8] = matrix.x0208;
  buffer[2][9] = matrix.x0209;
  buffer[2][10] = matrix.x0210;
  buffer[2][11] = matrix.x0211;
  buffer[3][0] = matrix.x0300;
  buffer[3][1] = matrix.x0301;
  buffer[3][2] = matrix.x0302;
  buffer[3][3] = matrix.x0303;
  buffer[3][4] = matrix.x0304;
  buffer[3][5] = matrix.x0305;
  buffer[3][6] = matrix.x0306;
  buffer[3][7] = matrix.x0307;
  buffer[3][8] = matrix.x0308;
  buffer[3][9] = matrix.x0309;
  buffer[3][10] = matrix.x0310;
  buffer[3][11] = matrix.x0311;
  buffer[4][0] = matrix.x0400;
  buffer[4][1] = matrix.x0401;
  buffer[4][2] = matrix.x0402;
  buffer[4][3] = matrix.x0403;
  buffer[4][4] = matrix.x0404;
  buffer[4][5] = matrix.x0405;
  buffer[4][6] = matrix.x0406;
  buffer[4][7] = matrix.x0407;
  buffer[4][8] = matrix.x0408;
  buffer[4][9] = matrix.x0409;
  buffer[4][10] = matrix.x0410;
  buffer[4][11] = matrix.x0411;
  buffer[5][0] = matrix.x0500;
  buffer[5][1] = matrix.x0501;
  buffer[5][2] = matrix.x0502;
  buffer[5][3] = matrix.x0503;
  buffer[5][4] = matrix.x0504;
  buffer[5][5] = matrix.x0505;
  buffer[5][6] = matrix.x0506;
  buffer[5][7] = matrix.x0507;
  buffer[5][8] = matrix.x0508;
  buffer[5][9] = matrix.x0509;
  buffer[5][10] = matrix.x0510;
  buffer[5][11] = matrix.x0511;
  buffer[6][0] = matrix.x0600;
  buffer[6][1] = matrix.x0601;
  buffer[6][2] = matrix.x0602;
  buffer[6][3] = matrix.x0603;
  buffer[6][4] = matrix.x0604;
  buffer[6][5] = matrix.x0605;
  buffer[6][6] = matrix.x0606;
  buffer[6][7] = matrix.x0607;
  buffer[6][8] = matrix.x0608;
  buffer[6][9] = matrix.x0609;
  buffer[6][10] = matrix.x0610;
  buffer[6][11] = matrix.x0611;
  buffer[7][0] = matrix.x0700;
  buffer[7][1] = matrix.x0701;
  buffer[7][2] = matrix.x0702;
  buffer[7][3] = matrix.x0703;
  buffer[7][4] = matrix.x0704;
  buffer[7][5] = matrix.x0705;
  buffer[7][6] = matrix.x0706;
  buffer[7][7] = matrix.x0707;
  buffer[7][8] = matrix.x0708;
  buffer[7][9] = matrix.x0709;
  buffer[7][10] = matrix.x0710;
  buffer[7][11] = matrix.x0711;
  buffer[8][0] = matrix.x0800;
  buffer[8][1] = matrix.x0801;
  buffer[8][2] = matrix.x0802;
  buffer[8][3] = matrix.x0803;
  buffer[8][4] = matrix.x0804;
  buffer[8][5] = matrix.x0805;
  buffer[8][6] = matrix.x0806;
  buffer[8][7] = matrix.x0807;
  buffer[8][8] = matrix.x0808;
  buffer[8][9] = matrix.x0809;
  buffer[8][10] = matrix.x0810;
  buffer[8][11] = matrix.x0811;
  buffer[9][0] = matrix.x0900;
  buffer[9][1] = matrix.x0901;
  buffer[9][2] = matrix.x0902;
  buffer[9][3] = matrix.x0903;
  buffer[9][4] = matrix.x0904;
  buffer[9][5] = matrix.x0905;
  buffer[9][6] = matrix.x0906;
  buffer[9][7] = matrix.x0907;
  buffer[9][8] = matrix.x0908;
  buffer[9][9] = matrix.x0909;
  buffer[9][10] = matrix.x0910;
  buffer[9][11] = matrix.x0911;
  buffer[10][0] = matrix.x1000;
  buffer[10][1] = matrix.x1001;
  buffer[10][2] = matrix.x1002;
  buffer[10][3] = matrix.x1003;
  buffer[10][4] = matrix.x1004;
  buffer[10][5] = matrix.x1005;
  buffer[10][6] = matrix.x1006;
  buffer[10][7] = matrix.x1007;
  buffer[10][8] = matrix.x1008;
  buffer[10][9] = matrix.x1009;
  buffer[10][10] = matrix.x1010;
  buffer[10][11] = matrix.x1011;
  buffer[11][0] = matrix.x1100;
  buffer[11][1] = matrix.x1101;
  buffer[11][2] = matrix.x1102;
  buffer[11][3] = matrix.x1103;
  buffer[11][4] = matrix.x1104;
  buffer[11][5] = matrix.x1105;
  buffer[11][6] = matrix.x1106;
  buffer[11][7] = matrix.x1107;
  buffer[11][8] = matrix.x1108;
  buffer[11][9] = matrix.x1109;
  buffer[11][10] = matrix.x1110;
  buffer[11][11] = matrix.x1111;

  return;
}

void getBuffer(int m, int n, float buffer[][n], Mat12f *matrix) // function prototype
{
  matrix->x0000 = buffer[0][0];
  matrix->x0001 = buffer[0][1];
  matrix->x0002 = buffer[0][2];
  matrix->x0003 = buffer[0][3];
  matrix->x0004 = buffer[0][4];
  matrix->x0005 = buffer[0][5];
  matrix->x0006 = buffer[0][6];
  matrix->x0007 = buffer[0][7];
  matrix->x0008 = buffer[0][8];
  matrix->x0009 = buffer[0][9];
  matrix->x0010 = buffer[0][10];
  matrix->x0011 = buffer[0][11];
  matrix->x0100 = buffer[1][0];
  matrix->x0101 = buffer[1][1];
  matrix->x0102 = buffer[1][2];
  matrix->x0103 = buffer[1][3];
  matrix->x0104 = buffer[1][4];
  matrix->x0105 = buffer[1][5];
  matrix->x0106 = buffer[1][6];
  matrix->x0107 = buffer[1][7];
  matrix->x0108 = buffer[1][8];
  matrix->x0109 = buffer[1][9];
  matrix->x0110 = buffer[1][10];
  matrix->x0111 = buffer[1][11];
  matrix->x0200 = buffer[2][0];
  matrix->x0201 = buffer[2][1];
  matrix->x0202 = buffer[2][2];
  matrix->x0203 = buffer[2][3];
  matrix->x0204 = buffer[2][4];
  matrix->x0205 = buffer[2][5];
  matrix->x0206 = buffer[2][6];
  matrix->x0207 = buffer[2][7];
  matrix->x0208 = buffer[2][8];
  matrix->x0209 = buffer[2][9];
  matrix->x0210 = buffer[2][10];
  matrix->x0211 = buffer[2][11];
  matrix->x0300 = buffer[3][0];
  matrix->x0301 = buffer[3][1];
  matrix->x0302 = buffer[3][2];
  matrix->x0303 = buffer[3][3];
  matrix->x0304 = buffer[3][4];
  matrix->x0305 = buffer[3][5];
  matrix->x0306 = buffer[3][6];
  matrix->x0307 = buffer[3][7];
  matrix->x0308 = buffer[3][8];
  matrix->x0309 = buffer[3][9];
  matrix->x0310 = buffer[3][10];
  matrix->x0311 = buffer[3][11];
  matrix->x0400 = buffer[4][0];
  matrix->x0401 = buffer[4][1];
  matrix->x0402 = buffer[4][2];
  matrix->x0403 = buffer[4][3];
  matrix->x0404 = buffer[4][4];
  matrix->x0405 = buffer[4][5];
  matrix->x0406 = buffer[4][6];
  matrix->x0407 = buffer[4][7];
  matrix->x0408 = buffer[4][8];
  matrix->x0409 = buffer[4][9];
  matrix->x0410 = buffer[4][10];
  matrix->x0411 = buffer[4][11];
  matrix->x0500 = buffer[5][0];
  matrix->x0501 = buffer[5][1];
  matrix->x0502 = buffer[5][2];
  matrix->x0503 = buffer[5][3];
  matrix->x0504 = buffer[5][4];
  matrix->x0505 = buffer[5][5];
  matrix->x0506 = buffer[5][6];
  matrix->x0507 = buffer[5][7];
  matrix->x0508 = buffer[5][8];
  matrix->x0509 = buffer[5][9];
  matrix->x0510 = buffer[5][10];
  matrix->x0511 = buffer[5][11];
  matrix->x0600 = buffer[6][0];
  matrix->x0601 = buffer[6][1];
  matrix->x0602 = buffer[6][2];
  matrix->x0603 = buffer[6][3];
  matrix->x0604 = buffer[6][4];
  matrix->x0605 = buffer[6][5];
  matrix->x0606 = buffer[6][6];
  matrix->x0607 = buffer[6][7];
  matrix->x0608 = buffer[6][8];
  matrix->x0609 = buffer[6][9];
  matrix->x0610 = buffer[6][10];
  matrix->x0611 = buffer[6][11];
  matrix->x0700 = buffer[7][0];
  matrix->x0701 = buffer[7][1];
  matrix->x0702 = buffer[7][2];
  matrix->x0703 = buffer[7][3];
  matrix->x0704 = buffer[7][4];
  matrix->x0705 = buffer[7][5];
  matrix->x0706 = buffer[7][6];
  matrix->x0707 = buffer[7][7];
  matrix->x0708 = buffer[7][8];
  matrix->x0709 = buffer[7][9];
  matrix->x0710 = buffer[7][10];
  matrix->x0711 = buffer[7][11];
  matrix->x0800 = buffer[8][0];
  matrix->x0801 = buffer[8][1];
  matrix->x0802 = buffer[8][2];
  matrix->x0803 = buffer[8][3];
  matrix->x0804 = buffer[8][4];
  matrix->x0805 = buffer[8][5];
  matrix->x0806 = buffer[8][6];
  matrix->x0807 = buffer[8][7];
  matrix->x0808 = buffer[8][8];
  matrix->x0809 = buffer[8][9];
  matrix->x0810 = buffer[8][10];
  matrix->x0811 = buffer[8][11];
  matrix->x0900 = buffer[9][0];
  matrix->x0901 = buffer[9][1];
  matrix->x0902 = buffer[9][2];
  matrix->x0903 = buffer[9][3];
  matrix->x0904 = buffer[9][4];
  matrix->x0905 = buffer[9][5];
  matrix->x0906 = buffer[9][6];
  matrix->x0907 = buffer[9][7];
  matrix->x0908 = buffer[9][8];
  matrix->x0909 = buffer[9][9];
  matrix->x0910 = buffer[9][10];
  matrix->x0911 = buffer[9][11];
  matrix->x1000 = buffer[10][0];
  matrix->x1001 = buffer[10][1];
  matrix->x1002 = buffer[10][2];
  matrix->x1003 = buffer[10][3];
  matrix->x1004 = buffer[10][4];
  matrix->x1005 = buffer[10][5];
  matrix->x1006 = buffer[10][6];
  matrix->x1007 = buffer[10][7];
  matrix->x1008 = buffer[10][8];
  matrix->x1009 = buffer[10][9];
  matrix->x1010 = buffer[10][10];
  matrix->x1011 = buffer[10][11];
  matrix->x1100 = buffer[11][0];
  matrix->x1101 = buffer[11][1];
  matrix->x1102 = buffer[11][2];
  matrix->x1103 = buffer[11][3];
  matrix->x1104 = buffer[11][4];
  matrix->x1105 = buffer[11][5];
  matrix->x1106 = buffer[11][6];
  matrix->x1107 = buffer[11][7];
  matrix->x1108 = buffer[11][8];
  matrix->x1109 = buffer[11][9];
  matrix->x1110 = buffer[11][10];
  matrix->x1111 = buffer[11][11];

  return;
}

// instantiate a model and initialize it
LinearQuadraticRegulator model;

// Initialize the randomizer using the current timestamp as a seed
// (The time() function is provided by the <time.h> header file)
// srand(time(NULL));

void initialize(LinearQuadraticRegulator *model)
{
  model->j = 1;
  model->k = 1;
  model->reward = 0.0;
  model->n = dim_n;
  model->m = dim_m;
  model->lambda = 0.97;
  model->delta = 0.001;
  model->terminated = 0;
  model->updated = 0;
  model->active = 0;
  model->dt = 0.0;

  model->W_n.x0000 = (float)(rand() % 100) / 100.0;
  model->W_n.x0001 = (float)(rand() % 100) / 100.0;
  model->W_n.x0002 = (float)(rand() % 100) / 100.0;
  model->W_n.x0003 = (float)(rand() % 100) / 100.0;
  model->W_n.x0004 = (float)(rand() % 100) / 100.0;
  model->W_n.x0005 = (float)(rand() % 100) / 100.0;
  model->W_n.x0006 = (float)(rand() % 100) / 100.0;
  model->W_n.x0007 = (float)(rand() % 100) / 100.0;
  model->W_n.x0008 = (float)(rand() % 100) / 100.0;
  model->W_n.x0009 = (float)(rand() % 100) / 100.0;
  model->W_n.x0010 = (float)(rand() % 100) / 100.0;
  model->W_n.x0011 = (float)(rand() % 100) / 100.0;
  model->W_n.x0100 = (float)(rand() % 100) / 100.0;
  model->W_n.x0101 = (float)(rand() % 100) / 100.0;
  model->W_n.x0102 = (float)(rand() % 100) / 100.0;
  model->W_n.x0103 = (float)(rand() % 100) / 100.0;
  model->W_n.x0104 = (float)(rand() % 100) / 100.0;
  model->W_n.x0105 = (float)(rand() % 100) / 100.0;
  model->W_n.x0106 = (float)(rand() % 100) / 100.0;
  model->W_n.x0107 = (float)(rand() % 100) / 100.0;
  model->W_n.x0108 = (float)(rand() % 100) / 100.0;
  model->W_n.x0109 = (float)(rand() % 100) / 100.0;
  model->W_n.x0110 = (float)(rand() % 100) / 100.0;
  model->W_n.x0111 = (float)(rand() % 100) / 100.0;
  model->W_n.x0200 = (float)(rand() % 100) / 100.0;
  model->W_n.x0201 = (float)(rand() % 100) / 100.0;
  model->W_n.x0202 = (float)(rand() % 100) / 100.0;
  model->W_n.x0203 = (float)(rand() % 100) / 100.0;
  model->W_n.x0204 = (float)(rand() % 100) / 100.0;
  model->W_n.x0205 = (float)(rand() % 100) / 100.0;
  model->W_n.x0206 = (float)(rand() % 100) / 100.0;
  model->W_n.x0207 = (float)(rand() % 100) / 100.0;
  model->W_n.x0208 = (float)(rand() % 100) / 100.0;
  model->W_n.x0209 = (float)(rand() % 100) / 100.0;
  model->W_n.x0210 = (float)(rand() % 100) / 100.0;
  model->W_n.x0211 = (float)(rand() % 100) / 100.0;
  model->W_n.x0300 = (float)(rand() % 100) / 100.0;
  model->W_n.x0301 = (float)(rand() % 100) / 100.0;
  model->W_n.x0302 = (float)(rand() % 100) / 100.0;
  model->W_n.x0303 = (float)(rand() % 100) / 100.0;
  model->W_n.x0304 = (float)(rand() % 100) / 100.0;
  model->W_n.x0305 = (float)(rand() % 100) / 100.0;
  model->W_n.x0306 = (float)(rand() % 100) / 100.0;
  model->W_n.x0307 = (float)(rand() % 100) / 100.0;
  model->W_n.x0308 = (float)(rand() % 100) / 100.0;
  model->W_n.x0309 = (float)(rand() % 100) / 100.0;
  model->W_n.x0310 = (float)(rand() % 100) / 100.0;
  model->W_n.x0311 = (float)(rand() % 100) / 100.0;
  model->W_n.x0400 = (float)(rand() % 100) / 100.0;
  model->W_n.x0401 = (float)(rand() % 100) / 100.0;
  model->W_n.x0402 = (float)(rand() % 100) / 100.0;
  model->W_n.x0403 = (float)(rand() % 100) / 100.0;
  model->W_n.x0404 = (float)(rand() % 100) / 100.0;
  model->W_n.x0405 = (float)(rand() % 100) / 100.0;
  model->W_n.x0406 = (float)(rand() % 100) / 100.0;
  model->W_n.x0407 = (float)(rand() % 100) / 100.0;
  model->W_n.x0408 = (float)(rand() % 100) / 100.0;
  model->W_n.x0409 = (float)(rand() % 100) / 100.0;
  model->W_n.x0410 = (float)(rand() % 100) / 100.0;
  model->W_n.x0411 = (float)(rand() % 100) / 100.0;
  model->W_n.x0500 = (float)(rand() % 100) / 100.0;
  model->W_n.x0501 = (float)(rand() % 100) / 100.0;
  model->W_n.x0502 = (float)(rand() % 100) / 100.0;
  model->W_n.x0503 = (float)(rand() % 100) / 100.0;
  model->W_n.x0504 = (float)(rand() % 100) / 100.0;
  model->W_n.x0505 = (float)(rand() % 100) / 100.0;
  model->W_n.x0506 = (float)(rand() % 100) / 100.0;
  model->W_n.x0507 = (float)(rand() % 100) / 100.0;
  model->W_n.x0508 = (float)(rand() % 100) / 100.0;
  model->W_n.x0509 = (float)(rand() % 100) / 100.0;
  model->W_n.x0510 = (float)(rand() % 100) / 100.0;
  model->W_n.x0511 = (float)(rand() % 100) / 100.0;
  model->W_n.x0600 = (float)(rand() % 100) / 100.0;
  model->W_n.x0601 = (float)(rand() % 100) / 100.0;
  model->W_n.x0602 = (float)(rand() % 100) / 100.0;
  model->W_n.x0603 = (float)(rand() % 100) / 100.0;
  model->W_n.x0604 = (float)(rand() % 100) / 100.0;
  model->W_n.x0605 = (float)(rand() % 100) / 100.0;
  model->W_n.x0606 = (float)(rand() % 100) / 100.0;
  model->W_n.x0607 = (float)(rand() % 100) / 100.0;
  model->W_n.x0608 = (float)(rand() % 100) / 100.0;
  model->W_n.x0609 = (float)(rand() % 100) / 100.0;
  model->W_n.x0610 = (float)(rand() % 100) / 100.0;
  model->W_n.x0611 = (float)(rand() % 100) / 100.0;
  model->W_n.x0700 = (float)(rand() % 100) / 100.0;
  model->W_n.x0701 = (float)(rand() % 100) / 100.0;
  model->W_n.x0702 = (float)(rand() % 100) / 100.0;
  model->W_n.x0703 = (float)(rand() % 100) / 100.0;
  model->W_n.x0704 = (float)(rand() % 100) / 100.0;
  model->W_n.x0705 = (float)(rand() % 100) / 100.0;
  model->W_n.x0706 = (float)(rand() % 100) / 100.0;
  model->W_n.x0707 = (float)(rand() % 100) / 100.0;
  model->W_n.x0708 = (float)(rand() % 100) / 100.0;
  model->W_n.x0709 = (float)(rand() % 100) / 100.0;
  model->W_n.x0710 = (float)(rand() % 100) / 100.0;
  model->W_n.x0711 = (float)(rand() % 100) / 100.0;
  model->W_n.x0800 = (float)(rand() % 100) / 100.0;
  model->W_n.x0801 = (float)(rand() % 100) / 100.0;
  model->W_n.x0802 = (float)(rand() % 100) / 100.0;
  model->W_n.x0803 = (float)(rand() % 100) / 100.0;
  model->W_n.x0804 = (float)(rand() % 100) / 100.0;
  model->W_n.x0805 = (float)(rand() % 100) / 100.0;
  model->W_n.x0806 = (float)(rand() % 100) / 100.0;
  model->W_n.x0807 = (float)(rand() % 100) / 100.0;
  model->W_n.x0808 = (float)(rand() % 100) / 100.0;
  model->W_n.x0809 = (float)(rand() % 100) / 100.0;
  model->W_n.x0810 = (float)(rand() % 100) / 100.0;
  model->W_n.x0811 = (float)(rand() % 100) / 100.0;
  model->W_n.x0900 = (float)(rand() % 100) / 100.0;
  model->W_n.x0901 = (float)(rand() % 100) / 100.0;
  model->W_n.x0902 = (float)(rand() % 100) / 100.0;
  model->W_n.x0903 = (float)(rand() % 100) / 100.0;
  model->W_n.x0904 = (float)(rand() % 100) / 100.0;
  model->W_n.x0905 = (float)(rand() % 100) / 100.0;
  model->W_n.x0906 = (float)(rand() % 100) / 100.0;
  model->W_n.x0907 = (float)(rand() % 100) / 100.0;
  model->W_n.x0908 = (float)(rand() % 100) / 100.0;
  model->W_n.x0909 = (float)(rand() % 100) / 100.0;
  model->W_n.x0910 = (float)(rand() % 100) / 100.0;
  model->W_n.x0911 = (float)(rand() % 100) / 100.0;
  model->W_n.x1000 = (float)(rand() % 100) / 100.0;
  model->W_n.x1001 = (float)(rand() % 100) / 100.0;
  model->W_n.x1002 = (float)(rand() % 100) / 100.0;
  model->W_n.x1003 = (float)(rand() % 100) / 100.0;
  model->W_n.x1004 = (float)(rand() % 100) / 100.0;
  model->W_n.x1005 = (float)(rand() % 100) / 100.0;
  model->W_n.x1006 = (float)(rand() % 100) / 100.0;
  model->W_n.x1007 = (float)(rand() % 100) / 100.0;
  model->W_n.x1008 = (float)(rand() % 100) / 100.0;
  model->W_n.x1009 = (float)(rand() % 100) / 100.0;
  model->W_n.x1010 = (float)(rand() % 100) / 100.0;
  model->W_n.x1011 = (float)(rand() % 100) / 100.0;
  model->W_n.x1100 = (float)(rand() % 100) / 100.0;
  model->W_n.x1101 = (float)(rand() % 100) / 100.0;
  model->W_n.x1102 = (float)(rand() % 100) / 100.0;
  model->W_n.x1103 = (float)(rand() % 100) / 100.0;
  model->W_n.x1104 = (float)(rand() % 100) / 100.0;
  model->W_n.x1105 = (float)(rand() % 100) / 100.0;
  model->W_n.x1106 = (float)(rand() % 100) / 100.0;
  model->W_n.x1107 = (float)(rand() % 100) / 100.0;
  model->W_n.x1108 = (float)(rand() % 100) / 100.0;
  model->W_n.x1109 = (float)(rand() % 100) / 100.0;
  model->W_n.x1110 = (float)(rand() % 100) / 100.0;
  model->W_n.x1111 = (float)(rand() % 100) / 100.0;

  model->P_n.x0000 = 1.0 / model->delta;
  model->P_n.x0001 = 0.0;
  model->P_n.x0002 = 0.0;
  model->P_n.x0003 = 0.0;
  model->P_n.x0004 = 0.0;
  model->P_n.x0005 = 0.0;
  model->P_n.x0006 = 0.0;
  model->P_n.x0007 = 0.0;
  model->P_n.x0008 = 0.0;
  model->P_n.x0009 = 0.0;
  model->P_n.x0010 = 0.0;
  model->P_n.x0011 = 0.0;
  model->P_n.x0100 = 0.0;
  model->P_n.x0101 = 1.0 / model->delta;
  model->P_n.x0102 = 0.0;
  model->P_n.x0103 = 0.0;
  model->P_n.x0104 = 0.0;
  model->P_n.x0105 = 0.0;
  model->P_n.x0106 = 0.0;
  model->P_n.x0107 = 0.0;
  model->P_n.x0108 = 0.0;
  model->P_n.x0109 = 0.0;
  model->P_n.x0110 = 0.0;
  model->P_n.x0111 = 0.0;
  model->P_n.x0200 = 0.0;
  model->P_n.x0201 = 0.0;
  model->P_n.x0202 = 1.0 / model->delta;
  model->P_n.x0203 = 0.0;
  model->P_n.x0204 = 0.0;
  model->P_n.x0205 = 0.0;
  model->P_n.x0206 = 0.0;
  model->P_n.x0207 = 0.0;
  model->P_n.x0208 = 0.0;
  model->P_n.x0209 = 0.0;
  model->P_n.x0210 = 0.0;
  model->P_n.x0211 = 0.0;
  model->P_n.x0300 = 0.0;
  model->P_n.x0301 = 0.0;
  model->P_n.x0302 = 0.0;
  model->P_n.x0303 = 1.0 / model->delta;
  model->P_n.x0304 = 0.0;
  model->P_n.x0305 = 0.0;
  model->P_n.x0306 = 0.0;
  model->P_n.x0307 = 0.0;
  model->P_n.x0308 = 0.0;
  model->P_n.x0309 = 0.0;
  model->P_n.x0310 = 0.0;
  model->P_n.x0311 = 0.0;
  model->P_n.x0400 = 0.0;
  model->P_n.x0401 = 0.0;
  model->P_n.x0402 = 0.0;
  model->P_n.x0403 = 0.0;
  model->P_n.x0404 = 1.0 / model->delta;
  model->P_n.x0405 = 0.0;
  model->P_n.x0406 = 0.0;
  model->P_n.x0407 = 0.0;
  model->P_n.x0408 = 0.0;
  model->P_n.x0409 = 0.0;
  model->P_n.x0410 = 0.0;
  model->P_n.x0411 = 0.0;
  model->P_n.x0500 = 0.0;
  model->P_n.x0501 = 0.0;
  model->P_n.x0502 = 0.0;
  model->P_n.x0503 = 0.0;
  model->P_n.x0504 = 0.0;
  model->P_n.x0505 = 1.0 / model->delta;
  model->P_n.x0506 = 0.0;
  model->P_n.x0507 = 0.0;
  model->P_n.x0508 = 0.0;
  model->P_n.x0509 = 0.0;
  model->P_n.x0510 = 0.0;
  model->P_n.x0511 = 0.0;
  model->P_n.x0600 = 0.0;
  model->P_n.x0601 = 0.0;
  model->P_n.x0602 = 0.0;
  model->P_n.x0603 = 0.0;
  model->P_n.x0604 = 0.0;
  model->P_n.x0605 = 0.0;
  model->P_n.x0606 = 1.0 / model->delta;
  model->P_n.x0607 = 0.0;
  model->P_n.x0608 = 0.0;
  model->P_n.x0609 = 0.0;
  model->P_n.x0610 = 0.0;
  model->P_n.x0611 = 0.0;
  model->P_n.x0700 = 0.0;
  model->P_n.x0701 = 0.0;
  model->P_n.x0702 = 0.0;
  model->P_n.x0703 = 0.0;
  model->P_n.x0704 = 0.0;
  model->P_n.x0705 = 0.0;
  model->P_n.x0706 = 0.0;
  model->P_n.x0707 = 1.0 / model->delta;
  model->P_n.x0708 = 0.0;
  model->P_n.x0709 = 0.0;
  model->P_n.x0710 = 0.0;
  model->P_n.x0711 = 0.0;
  model->P_n.x0800 = 0.0;
  model->P_n.x0801 = 0.0;
  model->P_n.x0802 = 0.0;
  model->P_n.x0803 = 0.0;
  model->P_n.x0804 = 0.0;
  model->P_n.x0805 = 0.0;
  model->P_n.x0806 = 0.0;
  model->P_n.x0807 = 0.0;
  model->P_n.x0808 = 1.0 / model->delta;
  model->P_n.x0809 = 0.0;
  model->P_n.x0810 = 0.0;
  model->P_n.x0811 = 0.0;
  model->P_n.x0900 = 0.0;
  model->P_n.x0901 = 0.0;
  model->P_n.x0902 = 0.0;
  model->P_n.x0903 = 0.0;
  model->P_n.x0904 = 0.0;
  model->P_n.x0905 = 0.0;
  model->P_n.x0906 = 0.0;
  model->P_n.x0907 = 0.0;
  model->P_n.x0908 = 0.0;
  model->P_n.x0909 = 1.0 / model->delta;
  model->P_n.x0910 = 0.0;
  model->P_n.x0911 = 0.0;
  model->P_n.x1000 = 0.0;
  model->P_n.x1001 = 0.0;
  model->P_n.x1002 = 0.0;
  model->P_n.x1003 = 0.0;
  model->P_n.x1004 = 0.0;
  model->P_n.x1005 = 0.0;
  model->P_n.x1006 = 0.0;
  model->P_n.x1007 = 0.0;
  model->P_n.x1008 = 0.0;
  model->P_n.x1009 = 0.0;
  model->P_n.x1010 = 1.0 / model->delta;
  model->P_n.x1011 = 0.0;
  model->P_n.x1100 = 0.0;
  model->P_n.x1101 = 0.0;
  model->P_n.x1102 = 0.0;
  model->P_n.x1103 = 0.0;
  model->P_n.x1104 = 0.0;
  model->P_n.x1105 = 0.0;
  model->P_n.x1106 = 0.0;
  model->P_n.x1107 = 0.0;
  model->P_n.x1108 = 0.0;
  model->P_n.x1109 = 0.0;
  model->P_n.x1110 = 0.0;
  model->P_n.x1111 = 1.0 / model->delta;

  model->K_j.x00 = (float)(rand() % 100) / 100.0;
  model->K_j.x01 = (float)(rand() % 100) / 100.0;
  model->K_j.x02 = (float)(rand() % 100) / 100.0;
  model->K_j.x03 = (float)(rand() % 100) / 100.0;
  model->K_j.x04 = (float)(rand() % 100) / 100.0;
  model->K_j.x05 = (float)(rand() % 100) / 100.0;
  model->K_j.x06 = (float)(rand() % 100) / 100.0;
  model->K_j.x07 = (float)(rand() % 100) / 100.0;
  model->K_j.x08 = (float)(rand() % 100) / 100.0;
  model->K_j.x09 = (float)(rand() % 100) / 100.0;
  model->K_j.x10 = (float)(rand() % 100) / 100.0;
  model->K_j.x11 = (float)(rand() % 100) / 100.0;
  model->K_j.x12 = (float)(rand() % 100) / 100.0;
  model->K_j.x13 = (float)(rand() % 100) / 100.0;
  model->K_j.x14 = (float)(rand() % 100) / 100.0;
  model->K_j.x15 = (float)(rand() % 100) / 100.0;
  model->K_j.x16 = (float)(rand() % 100) / 100.0;
  model->K_j.x17 = (float)(rand() % 100) / 100.0;
  model->K_j.x18 = (float)(rand() % 100) / 100.0;
  model->K_j.x19 = (float)(rand() % 100) / 100.0;

  model->dataset.x0 = 0.0;
  model->dataset.x1 = 0.0;
  model->dataset.x2 = 0.0;
  model->dataset.x3 = 0.0;
  model->dataset.x4 = 0.0;
  model->dataset.x5 = 0.0;
  model->dataset.x6 = 0.0;
  model->dataset.x7 = 0.0;
  model->dataset.x8 = 0.0;
  model->dataset.x9 = 0.0;
  model->dataset.x10 = 0.0;
  model->dataset.x11 = 0.0;
  model->dataset.x12 = 0.0;
  model->dataset.x13 = 0.0;
  model->dataset.x14 = 0.0;
  model->dataset.x15 = 0.0;
  model->dataset.x16 = 0.0;
  model->dataset.x17 = 0.0;
  model->dataset.x18 = 0.0;
  model->dataset.x19 = 0.0;
  model->dataset.x20 = 0.0;
  model->dataset.x21 = 0.0;
  model->dataset.x22 = 0.0;
  model->dataset.x23 = 0.0;
  IMU imu1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  IMU imu2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  IMU imu3 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Encoder ReactionEncoder = {0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 50, 2.0, 900.0, 0.0, 0.0, 0.0};
  Encoder RollingEncoder = {0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 50, 2.0, 900.0, 0.0, 0.0, 0.0};
  model->imu1 = imu1;
  model->imu2 = imu2;
  model->imu3 = imu3;
  model->ReactionEncoder = ReactionEncoder;
  model->RollingEncoder = RollingEncoder;
  return;
}
/*
Identify the Q function using RLS and update the control policy,
with the given `environment` and `model`.
The algorithm is terminated when there are no further updates
to the Q function or the control policy at each step.
*/
void stepForward(LinearQuadraticRegulator *model)
{
  int k = model->k;
  x_k[0] = model->dataset.x0;
  x_k[1] = model->dataset.x1;
  x_k[2] = model->dataset.x2;
  x_k[3] = model->dataset.x3;
  x_k[4] = model->dataset.x4;
  x_k[5] = model->dataset.x5;
  x_k[6] = model->dataset.x6;
  x_k[7] = model->dataset.x7;
  x_k[8] = model->dataset.x8;
  x_k[9] = model->dataset.x9;
  K_j[0][0] = model->K_j.x00;
  K_j[0][1] = model->K_j.x01;
  K_j[0][2] = model->K_j.x02;
  K_j[0][3] = model->K_j.x03;
  K_j[0][4] = model->K_j.x04;
  K_j[0][5] = model->K_j.x05;
  K_j[0][6] = model->K_j.x06;
  K_j[0][7] = model->K_j.x07;
  K_j[0][8] = model->K_j.x08;
  K_j[0][9] = model->K_j.x09;
  K_j[1][0] = model->K_j.x10;
  K_j[1][1] = model->K_j.x11;
  K_j[1][2] = model->K_j.x12;
  K_j[1][3] = model->K_j.x13;
  K_j[1][4] = model->K_j.x14;
  K_j[1][5] = model->K_j.x15;
  K_j[1][6] = model->K_j.x16;
  K_j[1][7] = model->K_j.x17;
  K_j[1][8] = model->K_j.x18;
  K_j[1][9] = model->K_j.x19;
  u_k[0] = 0.0;
  u_k[1] = 0.0;
  // feeback policy
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      u_k[i] += -K_j[i][j] * x_k[j];
    }
  }
  // act!
  model->dataset.x0 = model->imu1.roll;
  model->dataset.x1 = model->imu1.roll_velocity;
  model->dataset.x2 = model->imu1.pitch_acceleration * model->imu1.roll_velocity - model->imu1.roll_acceleration * model->imu1.pitch_velocity;
  model->dataset.x3 = model->imu1.pitch;
  model->dataset.x4 = model->imu1.pitch_velocity;
  model->dataset.x5 = model->RollingEncoder.acceleration;
  model->dataset.x6 = model->ReactionEncoder.velocity;
  model->dataset.x7 = model->RollingEncoder.angle;
  model->dataset.x8 = reaction_wheel_current_acceleration;
  model->dataset.x9 = rolling_wheel_current_acceleration;
  model->dataset.x10 = u_k[0];
  model->dataset.x11 = u_k[1];

  if (model->active == 1)
  {
    reaction_wheel_pwm += 32.0 * u_k[0];
    rolling_wheel_pwm += 8.0 * u_k[1];
    reaction_wheel_pwm = fmin(255.0, reaction_wheel_pwm);
    reaction_wheel_pwm = fmax(-255.0, reaction_wheel_pwm);
    rolling_wheel_pwm = fmin(255.0, rolling_wheel_pwm);
    rolling_wheel_pwm = fmax(-255.0, rolling_wheel_pwm);
    TIM2->CCR1 = 255 * (int)fabs(rolling_wheel_pwm);
    TIM2->CCR2 = 255 * (int)fabs(reaction_wheel_pwm);
    if (reaction_wheel_pwm < 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }
    if (rolling_wheel_pwm < 0)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
  }
  else
  {
    reaction_wheel_pwm = 0.0;
    rolling_wheel_pwm = 0.0;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  }
  // dataset = (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
  updateEncoder(&(model->ReactionEncoder), reactionEncoderWindow, TIM3->CNT);
  updateEncoder(&(model->RollingEncoder), rollingEncoderWindow, TIM4->CNT);
  updateIMU(model);
  updateCurrentSensing();
  model->dataset.x12 = model->imu1.roll;
  model->dataset.x13 = model->imu1.roll_velocity;
  model->dataset.x14 = model->imu1.pitch_acceleration * model->imu1.roll_velocity - model->imu1.roll_acceleration * model->imu1.pitch_velocity;
  model->dataset.x15 = model->imu1.pitch;
  model->dataset.x16 = model->imu1.pitch_velocity;
  model->dataset.x17 = model->RollingEncoder.acceleration;
  model->dataset.x18 = model->ReactionEncoder.velocity;
  model->dataset.x19 = model->RollingEncoder.angle;
  model->dataset.x20 = reaction_wheel_current_acceleration;
  model->dataset.x21 = rolling_wheel_current_acceleration;
  x_k1[0] = model->dataset.x12;
  x_k1[1] = model->dataset.x13;
  x_k1[2] = model->dataset.x14;
  x_k1[3] = model->dataset.x15;
  x_k1[4] = model->dataset.x16;
  x_k1[5] = model->dataset.x17;
  x_k1[6] = model->dataset.x18;
  x_k1[7] = model->dataset.x19;
  x_k1[8] = model->dataset.x20;
  x_k1[9] = model->dataset.x21;
  u_k1[0] = 0.0;
  u_k1[1] = 0.0;
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      u_k1[i] += -K_j[i][j] * x_k1[j];
    }
  }
  model->dataset.x22 = u_k1[0];
  model->dataset.x23 = u_k1[1];
  // Compute the quadratic basis sets ϕ(zₖ), ϕ(zₖ₊₁).
  z_k[0] = model->dataset.x0;
  z_k[1] = model->dataset.x1;
  z_k[2] = model->dataset.x2;
  z_k[3] = model->dataset.x3;
  z_k[4] = model->dataset.x4;
  z_k[5] = model->dataset.x5;
  z_k[6] = model->dataset.x6;
  z_k[7] = model->dataset.x7;
  z_k[8] = model->dataset.x8;
  z_k[9] = model->dataset.x9;
  z_k[10] = model->dataset.x10;
  z_k[11] = model->dataset.x11;
  z_k1[0] = model->dataset.x12;
  z_k1[1] = model->dataset.x13;
  z_k1[2] = model->dataset.x14;
  z_k1[3] = model->dataset.x15;
  z_k1[4] = model->dataset.x16;
  z_k1[5] = model->dataset.x17;
  z_k1[6] = model->dataset.x18;
  z_k1[7] = model->dataset.x19;
  z_k1[8] = model->dataset.x20;
  z_k1[9] = model->dataset.x21;
  z_k1[10] = model->dataset.x22;
  z_k1[11] = model->dataset.x23;
  for (int i = 0; i < model->n + model->m; i++)
  {
    basisset0[i] = z_k[i];
    basisset1[i] = z_k1[i];
  }
  // Now perform a one-step update in the parameter vector W by applying RLS to equation (S27).
  putBuffer(model->m + model->n, model->m + model->n, P_n, model->P_n);
  putBuffer(model->m + model->n, model->m + model->n, W_n, model->W_n);
  // initialize z_n
  for (int i = 0; i < model->n + model->m; i++)
  {
    z_n[i] = 0.0;
  }
  for (int i = 0; i < model->n + model->m; i++)
  {
    for (int j = 0; j < model->n + model->m; j++)
    {
      z_n[i] += P_n[i][j] * z_k[j];
    }
  }
  float z_k_dot_z_n = 0.0;
  for (int i = 0; i < model->n + model->m; i++)
  {
    z_k_dot_z_n += z_k[i] * z_n[i];
  }
  for (int i = 0; i < model->n + model->m; i++)
  {
    g_n[i] = 1.0 / (model->lambda + z_k_dot_z_n) * z_n[i];
  }
  // αₙ = dₙ - transpose(wₙ₋₁) * xₙ
  // initialize alpha_n
  for (int i = 0; i < model->n + model->m; i++)
  {
    alpha_n[i] = 0.0;
  }
  for (int i = 0; i < model->n + model->m; i++)
  {
    for (int j = 0; j < model->n + model->m; j++)
    {
      alpha_n[i] += W_n[i][j] * (basisset0[j] - basisset1[j]); // checked manually
    }
  }
  for (int i = 0; i < model->n + model->m; i++)
  {
    for (int j = 0; j < model->n + model->m; j++)
    {
      W_n[i][j] = W_n[i][j] + (alpha_n[i] * g_n[j]); // checked manually
    }
  }
  for (int i = 0; i < model->n + model->m; i++)
  {
    for (int j = 0; j < model->n + model->m; j++)
    {
      P_n[i][j] = (1.0 / model->lambda) * (P_n[i][j] - g_n[i] * z_n[j]); // checked manually
    }
  }
  getBuffer(model->m + model->n, model->m + model->n, W_n, &(model->W_n));
  getBuffer(model->m + model->n, model->m + model->n, P_n, &(model->P_n));

  // Repeat at the next time k + 1 and continue until RLS converges and the new parameter vector Wⱼ₊₁ is found.
  model->k = k + 1;
  return;
}

void updateControlPolicy(LinearQuadraticRegulator *model)
{
  // npack the vector Wⱼ₊₁ into the kernel matrix
  // Q(xₖ, uₖ) ≡ 0.5 * transpose([xₖ; uₖ]) * S * [xₖ; uₖ] = 0.5 * transpose([xₖ; uₖ]) * [Sₓₓ Sₓᵤ; Sᵤₓ Sᵤᵤ] * [xₖ; uₖ]
  model->k = 1;
  model->j = model->j + 1;
  // initialize the filter matrix
  putBuffer(model->m + model->n, model->m + model->n, W_n, model->W_n);

  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      S_ux[i][j] = W_n[model->n + i][j];
    }
  }
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->m; j++)
    {
      S_uu[i][j] = W_n[model->n + i][model->n + j];
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
    for (int i = 0; i < model->m; i++)
    {
      for (int j = 0; j < model->n; j++)
      {
        K_j[i][j] = 0.0;
      }
    }
    for (int i = 0; i < model->m; i++)
    {
      for (int j = 0; j < model->n; j++)
      {
        for (int k = 0; k < model->m; k++)
        {
          K_j[i][j] += S_uu_inverse[i][k] * S_ux[k][j];
        }
      }
    }
    model->K_j.x00 = K_j[0][0];
    model->K_j.x01 = K_j[0][1];
    model->K_j.x02 = K_j[0][2];
    model->K_j.x03 = K_j[0][3];
    model->K_j.x04 = K_j[0][4];
    model->K_j.x05 = K_j[0][5];
    model->K_j.x06 = K_j[0][6];
    model->K_j.x07 = K_j[0][7];
    model->K_j.x08 = K_j[0][8];
    model->K_j.x09 = K_j[0][9];
    model->K_j.x10 = K_j[1][0];
    model->K_j.x11 = K_j[1][1];
    model->K_j.x12 = K_j[1][2];
    model->K_j.x13 = K_j[1][3];
    model->K_j.x14 = K_j[1][4];
    model->K_j.x15 = K_j[1][5];
    model->K_j.x16 = K_j[1][6];
    model->K_j.x17 = K_j[1][7];
    model->K_j.x18 = K_j[1][8];
    model->K_j.x19 = K_j[1][9];
  }
  model->updated = 1;
  return;
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
  const int LOG_CYCLE = 100;
  int transmit = 0;
  int log_counter = 0;
  int log_status = 0;
  unsigned long t1 = 0;
  unsigned long t2 = 0;
  unsigned long diff = 0;
  initialize(&model);
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_Delay(10);

  // initialize the Encoder and IMU
  HAL_Delay(10);
  updateEncoder(&model.ReactionEncoder, reactionEncoderWindow, TIM3->CNT);
  updateEncoder(&model.RollingEncoder, rollingEncoderWindow, TIM4->CNT);
  updateIMU(&model);
  HAL_Delay(10);
  updateEncoder(&model.ReactionEncoder, reactionEncoderWindow, TIM3->CNT);
  updateEncoder(&model.RollingEncoder, rollingEncoderWindow, TIM4->CNT);
  updateIMU(&model);
  for (int i = 0; i < encoderWindowLength; i++)
  {
    reactionEncoderWindow[i] = 0;
    rollingEncoderWindow[i] = 0;
  }

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
      reaction_wheel_pwm = 0.0;
      rolling_wheel_pwm = 0.0;
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
    }

    if (fabs(model.imu1.roll) > roll_safety_angle || fabs(model.imu1.pitch) > pitch_safety_angle || model.k > max_episode_length)
    {
      model.terminated = 1;
      model.active = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      model.terminated = 0;
      model.active = 1;
      model.updated = 0;
    }
    // Rinse and repeat :)

    if (model.terminated == 0)
    {
      stepForward(&model);
    }
    else
    {
      reaction_wheel_pwm = 0.0;
      rolling_wheel_pwm = 0.0;
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      updateEncoder(&model.ReactionEncoder, reactionEncoderWindow, TIM3->CNT);
      updateEncoder(&model.RollingEncoder, rollingEncoderWindow, TIM4->CNT);
      updateIMU(&model);
      updateCurrentSensing();
    }
    if (model.terminated == 1 && model.updated == 0)
    {
      updateControlPolicy(&model);
    }

    model.imu1.yaw += dt * r_dot[2];

    log_counter++;
    if (log_counter > LOG_CYCLE && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
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
          "roll: %0.2f, rollv: %0.2f, pitch: %0.2f, pitchv: %0.2f, | aX1: %0.2f, aY1: %0.2f, aZ1: %0.2f, | aX2: %0.2f, aY2: %0.2f, aZ2: %0.2f, | encB: %0.2f, encT: %0.2f, dt: %0.6f\r\n",
          model.imu1.roll, model.imu1.roll_velocity, model.imu1.pitch, model.imu1.pitch_velocity, model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.RollingEncoder.angle, model.ReactionEncoder.angle, dt);
          // sprintf(MSG,
          //   "gX1: %0.2f, gY1: %0.2f, gZ1: %0.2f, | gX2: %0.2f, gY2: %0.2f, gZ2: %0.2f, dt: %0.6f\r\n",
          //   model.imu1.gyrX, model.imu1.gyrY, model.imu1.gyrZ, model.imu2.gyrX, model.imu2.gyrY, model.imu2.gyrZ, dt);
          log_status = 0;
      }

      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);
    }

    t2 = DWT->CYCCNT;
    diff = t2 - t1;
    dt = (float)diff / CPU_CLOCK;
    model.dt = dt;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC5 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
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

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
