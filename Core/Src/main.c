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
#define TRANSMIT_LENGTH 500
#define RECEIVE_FRAME_LENGTH 32
#define TRANSMIT_FRAME_LENGTH 5
#define N 10
#define M 2
#define SLAVE_ADDRESS 0xA4
#define TRANSFER_REQUEST 0x02
#define MASTER_REQ_ROLL_H 0x14
#define MASTER_REQ_ROLL_L 0x15
#define MASTER_REQ_ACC_X_H 0x08
#define SLAVE_ADDRESS_ICM 0x68       // 0b1101001
#define SLAVE_ADDRESS_ICM_READ 0xD1  // 0b11010011
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
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x1B, 0xCA};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
int uart_receive_ok = 0;
const float CPU_CLOCK = 84000000.0;
const int dim_n = N;
const int dim_m = M;
const int max_episode_length = 50000;
const int updatePolicyPeriod = 1;
const int LOG_CYCLE = 20;
const float roll_safety_angle = 0.28;
const float pitch_safety_angle = 0.20;
const float sensorAngle = -30.0 / 180.0 * M_PI;
const float clipping = 1000.0;
const float clippingFactor = 0.95;
uint8_t transferRequest = MASTER_REQ_ACC_X_H;
// maximum PWM step size for each control cycle
float reactionPulseStep = 255.0 * 48.0;
float rollingPulseStep = 255.0 * 48.0;
float updateChange = 0.0; // corrections to the filter coefficients
float minimumChange = 60.0; // the minimum correction to filter coefficients
float triggerUpdate = 0; // trigger a policy update
// sampling time
float dt = 0.0;
uint8_t raw_data[14] = {0};
uint16_t AD_RES = 0;
uint32_t AD_RES_BUFFER[2];
// define arrays for matrix-matrix and matrix-vector multiplication
// float W_n[N + M][N + M];
// float P_n[N + M][N + M];
float x_k[N];
float u_k[M];
// float x_k1[N];
// float u_k1[M];
float z_k[N + M];
// float z_k1[N + M];
// float basisset0[N + M];
// float basisset1[N + M];
float z_n[N + M];
float K_j[M][N];
float g_n[N + M];
float alpha_n[N + M];
float S_ux[M][N];
float S_uu[M][M];
float S_uu_inverse[M][M];
float z_k_dot_z_n = 0.0;
// tilt estimation
// the pivot point B̂ in the inertial frame Ô
float pivot[3] = {-0.097, -0.1, -0.032};
// the position of sensors mounted on the body in the body frame of reference
float p1[3] = {-0.1400, -0.0650, -0.0620};
float p2[3] = {-0.0400, -0.0600, -0.0600};
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
float kappa1 = 0.03;
float kappa2 = 0.03;
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
  float row0[N + M];
  float row1[N + M];
  float row2[N + M];
  float row3[N + M];
  float row4[N + M];
  float row5[N + M];
  float row6[N + M];
  float row7[N + M];
  float row8[N + M];
  float row9[N + M];
  float row10[N + M];
  float row11[N + M];
} Mat12;

float getIndex(Mat12 matrix, int i, int j)
{
  switch (i)
  {
  case 0:
    return matrix.row0[j];
    break;
  case 1:
    return matrix.row1[j];
    break;
  case 2:
    return matrix.row2[j];
    break;
  case 3:
    return matrix.row3[j];
    break;
  case 4:
    return matrix.row4[j];
    break;
  case 5:
    return matrix.row5[j];
    break;
  case 6:
    return matrix.row6[j];
    break;
  case 7:
    return matrix.row7[j];
    break;
  case 8:
    return matrix.row8[j];
    break;
  case 9:
    return matrix.row9[j];
    break;
  case 10:
    return matrix.row10[j];
    break;
  case 11:
    return matrix.row11[j];
    break;
  default:
    break;
  }
}

void setIndex(Mat12 *matrix, int i, int j, float value)
{
  switch (i)
  {
  case 0:
    matrix->row0[j] = value;
    break;
  case 1:
    matrix->row1[j] = value;
    break;
  case 2:
    matrix->row2[j] = value;
    break;
  case 3:
    matrix->row3[j] = value;
    break;
  case 4:
    matrix->row4[j] = value;
    break;
  case 5:
    matrix->row5[j] = value;
    break;
  case 6:
    matrix->row6[j] = value;
    break;
  case 7:
    matrix->row7[j] = value;
    break;
  case 8:
    matrix->row8[j] = value;
    break;
  case 9:
    matrix->row9[j] = value;
    break;
  case 10:
    matrix->row10[j] = value;
    break;
  case 11:
    matrix->row11[j] = value;
    break;
  default:
    break;
  }
}

typedef struct
{
  int pulse_per_revolution; // the number of pulses per revolution
  int value;                // the counter
  float radianAngle;        // the angle in radian
  float angle;              // the absolute angle
  float velocity;           // the angular velocity
  float acceleration;       // the angular acceleration
} Encoder;

typedef struct
{
  float currentScale;
  int current0;
  int current1;
  float currentVelocity;
} CurrentSensor;

typedef struct
{
  int16_t accX_offset;
  int16_t accY_offset;
  int16_t accZ_offset;
  float accX_scale;
  float accY_scale;
  float accZ_scale;
  int16_t gyrX_offset;
  int16_t gyrY_offset;
  int16_t gyrZ_offset;
  float gyrX_scale;
  float gyrY_scale;
  float gyrZ_scale;
  int16_t rawAccX;
  int16_t rawAccY;
  int16_t rawAccZ;
  int16_t rawGyrX;
  int16_t rawGyrY;
  int16_t rawGyrZ;
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
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

void encodeWheel(Encoder *encoder, int newValue)
{
  encoder->value = newValue;
  encoder->radianAngle = (float)(encoder->value % encoder->pulse_per_revolution) / (float)encoder->pulse_per_revolution * 2.0 * M_PI;
  float angle = sin(encoder->radianAngle);
  float velocity = angle - encoder->angle;
  float acceleration = velocity - encoder->velocity;
  encoder->angle = angle;
  encoder->velocity = velocity;
  encoder->acceleration = acceleration;
  return;
}

void senseCurrent(CurrentSensor *reactionCurrentSensor, CurrentSensor *rollingCurrentSensor)
{
  // Start ADC Conversion in DMA Mode (Periodically Every 1ms)
  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 2);
  reactionCurrentSensor->current1 = reactionCurrentSensor->current0;
  rollingCurrentSensor->current1 = rollingCurrentSensor->current0;
  reactionCurrentSensor->current0 = (AD_RES_BUFFER[0] << 4);
  rollingCurrentSensor->current0 = (AD_RES_BUFFER[1] << 4);
  reactionCurrentSensor->currentVelocity = (float)(reactionCurrentSensor->current0 - reactionCurrentSensor->current1) / reactionCurrentSensor->currentScale;
  rollingCurrentSensor->currentVelocity = (float)(rollingCurrentSensor->current0 - rollingCurrentSensor->current1) / rollingCurrentSensor->currentScale;
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
    sensor->rawAccX = (raw_data[0] << 8) | raw_data[1];
    sensor->rawAccY = (raw_data[2] << 8) | raw_data[3];
    sensor->rawAccZ = (raw_data[4] << 8) | raw_data[5];
    sensor->rawGyrX = (raw_data[6] << 8) | raw_data[7];
    sensor->rawGyrY = (raw_data[8] << 8) | raw_data[9];
    sensor->rawGyrZ = (raw_data[10] << 8) | raw_data[11];
    sensor->accX = sensor->accX_scale * (sensor->rawAccX - sensor->accX_offset);
    sensor->accY = sensor->accY_scale * (sensor->rawAccY - sensor->accY_offset);
    sensor->accZ = sensor->accZ_scale * (sensor->rawAccZ - sensor->accZ_offset);
    sensor->gyrX = sensor->gyrX_scale * (sensor->rawGyrX - sensor->gyrX_offset);
    sensor->gyrY = sensor->gyrY_scale * (sensor->rawGyrY - sensor->gyrY_offset);
    sensor->gyrZ = sensor->gyrZ_scale * (sensor->rawGyrZ - sensor->gyrZ_offset);
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  return;
}

void updateIMU2(IMU *sensor) // GY-95 USART
{
  if (uart_receive_ok == 1)
  {
    if (UART1_rxBuffer[0] == UART1_txBuffer[0] && UART1_rxBuffer[1] == UART1_txBuffer[1] && UART1_rxBuffer[2] == UART1_txBuffer[2] && UART1_rxBuffer[3] == UART1_txBuffer[3])
    {
      sensor->rawAccX = (UART1_rxBuffer[5] << 8) | UART1_rxBuffer[4];
      sensor->rawAccY = (UART1_rxBuffer[7] << 8) | UART1_rxBuffer[6];
      sensor->rawAccZ = (UART1_rxBuffer[9] << 8) | UART1_rxBuffer[8];
      sensor->rawGyrX = (UART1_rxBuffer[11] << 8) | UART1_rxBuffer[10];
      sensor->rawGyrY = (UART1_rxBuffer[13] << 8) | UART1_rxBuffer[12];
      sensor->rawGyrZ = (UART1_rxBuffer[15] << 8) | UART1_rxBuffer[14];
      sensor->accX = sensor->accX_scale * (sensor->rawAccX - sensor->accX_offset);
      sensor->accY = sensor->accY_scale * (sensor->rawAccY - sensor->accY_offset);
      sensor->accZ = sensor->accZ_scale * (sensor->rawAccZ - sensor->accZ_offset);
      sensor->gyrX = sensor->gyrX_scale * (sensor->rawGyrX - sensor->gyrX_offset);
      sensor->gyrY = sensor->gyrY_scale * (sensor->rawGyrY - sensor->gyrY_offset);
      sensor->gyrZ = sensor->gyrZ_scale * (sensor->rawGyrZ - sensor->gyrZ_offset);
      float dummyx = cos(sensorAngle) * sensor->accX - sin(sensorAngle) * sensor->accY;
      float dummyy = sin(sensorAngle) * sensor->accX + cos(sensorAngle) * sensor->accY;
      sensor->accX = -dummyy;
      sensor->accY = dummyx;
      dummyx = cos(sensorAngle) * sensor->gyrX - sin(sensorAngle) * sensor->gyrY;
      dummyy = sin(sensorAngle) * sensor->gyrX + cos(sensorAngle) * sensor->gyrY;
      sensor->gyrX = -dummyy;
      sensor->gyrY = dummyx;
      uart_receive_ok = 0;
    }
  }
  return;
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

// Represents a Linear Quadratic Regulator (LQR) model.
typedef struct
{
  Mat12 W_n;                           // filter matrix
  Mat12 P_n;                           // inverse autocorrelation matrix
  Mat210f K_j;                         // feedback policy
  Vec24f dataset;                      // (xₖ, uₖ, xₖ₊₁, uₖ₊₁)
  int j;                               // step number
  int k;                               // time k
  int n;                               // xₖ ∈ ℝⁿ
  int m;                               // uₖ ∈ ℝᵐ
  float lambda;                        // exponential wighting factor
  float delta;                         // value used to intialize P(0)
  int active;                          // is the model controller active
  float dt;                            // period in seconds
  float reactionPWM;                   // reaction wheel's motor PWM duty cycle
  float rollingPWM;                    // rolling wheel's motor PWM duty cycle
  IMU imu1;                            // the first inertial measurement unit
  IMU imu2;                            // the second inertial measurement unit
  Encoder reactionEncoder;             // the reaction wheel encoder
  Encoder rollingEncoder;              // the rolling wheel encoder
  CurrentSensor reactionCurrentSensor; // the reaction wheel's motor current sensor
  CurrentSensor rollingCurrentSensor;  // the rolling wheel's motor current sensor
} LinearQuadraticRegulator;

void updateIMU(LinearQuadraticRegulator *model)
{
  updateIMU1(&(model->imu1));
  updateIMU2(&(model->imu2));
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
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      _R1[i] += B_A1_R[i][j] * R1[j];
      _R2[i] += B_A2_R[i][j] * R2[j];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    Matrix[i][0] = _R1[i];
    Matrix[i][1] = _R2[i];
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Q[i][j] = 0.0;
      for (int k = 0; k < 2; k++)
      {
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
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      _G1[i] += B_A1_R[i][j] * G1[j];
      _G2[i] += B_A2_R[i][j] * G2[j];
    }
  }
  for (int i = 0; i < 3; i++)
  {
    r[i] = (_G1[i] + _G2[i]) / 2.0;
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
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
      r_dot[i] += E[i][j] * r[j];
    }
  }

  fused_beta = kappa1 * beta + (1.0 - kappa1) * (fused_beta + model->dt * (r_dot[1] / 180.0 * M_PI));
  fused_gamma = kappa2 * gamma1 + (1.0 - kappa2) * (fused_gamma + model->dt * (r_dot[2] / 180.0 * M_PI));
  model->imu1.yaw += model->dt * r_dot[0];

  float _roll = fused_beta;
  float _pitch = -fused_gamma;
  float _roll_velocity = ((r_dot[1] / 180.0 * M_PI) + (_roll - model->imu1.roll) / model->dt) / 2.0;
  float _pitch_velocity = ((-r_dot[2] / 180.0 * M_PI) + (_pitch - model->imu1.pitch) / model->dt) / 2.0;
  model->imu1.roll_acceleration = _roll_velocity - model->imu1.roll_velocity;
  model->imu1.pitch_acceleration = _pitch_velocity - model->imu1.pitch_velocity;
  model->imu1.roll_velocity = _roll_velocity;
  model->imu1.pitch_velocity = _pitch_velocity;
  model->imu1.roll = _roll;
  model->imu1.pitch = _pitch;
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
  model->n = dim_n;
  model->m = dim_m;
  model->lambda = 0.9;
  model->delta = 0.01;
  model->active = 0;
  model->dt = 0.0;

  for (int i = 0; i < (model->n + model->n); i++)
  {
    for (int j = 0; j < (model->n + model->n); j++)
    {
      setIndex(&(model->W_n), i, j, (float)(rand() % 100) / 100.0);
      if (i == j)
      {
        setIndex(&(model->P_n), i, j, 1.0);
      }
      else
      {
        setIndex(&(model->P_n), i, j, 0.0);
      }
    }
  }

  int seed = DWT->CYCCNT;
  srand(seed);
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
  // scale : 1 / 2048
  IMU imu1 = {-24, -60, 27, 0.000488281, 0.000488281, 0.000488281, 0, 0, 0, 0.017444444, 0.017444444, 0.017444444, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  IMU imu2 = {75, -25, -18, 0.000488281, 0.000488281, 0.000488281, 0, 0, 0, 0.017444444, 0.017444444, 0.017444444, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Encoder reactionEncoder = {1736, 0, 0, 0, 0, 0};
  Encoder rollingEncoder = {3020, 0, 0, 0, 0, 0};
  CurrentSensor reactionCurrentSensor = {30000.0, 0, 0, 0};
  CurrentSensor rollingCurrentSensor = {30000.0, 0, 0, 0};
  model->imu1 = imu1;
  model->imu2 = imu2;
  model->reactionEncoder = reactionEncoder;
  model->rollingEncoder = rollingEncoder;
  model->reactionCurrentSensor = reactionCurrentSensor;
  model->rollingCurrentSensor = rollingCurrentSensor;
  model->reactionPWM = 0.0;
  model->rollingPWM = 0.0;
  return;
}
/*
Identify the Q function using RLS with the given pointer to the `model`.
The algorithm is finished when there are no further updates
to the Q function or the control policy at each step.
*/
void stepForward(LinearQuadraticRegulator *model)
{
  // measure
  encodeWheel(&(model->reactionEncoder), TIM3->CNT);
  encodeWheel(&(model->rollingEncoder), TIM4->CNT);
  senseCurrent(&(model->reactionCurrentSensor), &(model->rollingCurrentSensor));
  updateIMU(model);
  // dataset = (xₖ, uₖ)
  model->dataset.x0 = model->imu1.roll;
  model->dataset.x1 = model->imu1.roll_velocity;
  model->dataset.x2 = model->imu1.roll_acceleration;
  model->dataset.x3 = model->imu1.pitch;
  model->dataset.x4 = model->imu1.pitch_velocity;
  model->dataset.x5 = model->imu1.pitch_acceleration;
  model->dataset.x6 = model->reactionEncoder.velocity;
  model->dataset.x7 = model->rollingEncoder.velocity;
  model->dataset.x8 = model->reactionCurrentSensor.currentVelocity;
  model->dataset.x9 = model->rollingCurrentSensor.currentVelocity;
  model->dataset.x10 = u_k[0];
  model->dataset.x11 = u_k[1];

  // act!
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
  // add probing noise to guarantee persistence of excitation
  int seed = DWT->CYCCNT;
  srand(seed);
  u_k[0] += (float)(rand() % 1000) / 10000000.0;
  seed = DWT->CYCCNT;
  srand(seed);
  u_k[0] -= (float)(rand() % 1000) / 10000000.0;
  seed = DWT->CYCCNT;
  srand(seed);
  u_k[1] += (float)(rand() % 1000) / 10000000.0;
  seed = DWT->CYCCNT;
  srand(seed);
  u_k[1] -= (float)(rand() % 1000) / 10000000.0;

  model->reactionPWM += reactionPulseStep * u_k[0];
  model->rollingPWM += rollingPulseStep * u_k[1];
  model->reactionPWM = fmin(255.0 * 255.0, model->reactionPWM);
  model->reactionPWM = fmax(-255.0 * 255.0, model->reactionPWM);
  model->rollingPWM = fmin(255.0 * 255.0, model->rollingPWM);
  model->rollingPWM = fmax(-255.0 * 255.0, model->rollingPWM);
  TIM2->CCR1 = (int)fabs(model->rollingPWM);
  TIM2->CCR2 = (int)fabs(model->reactionPWM);
  if (model->reactionPWM < 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  }
  if (model->rollingPWM < 0)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  }

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
  // Now perform a one-step update in the parameter vector W by applying RLS to equation (S27).
  // initialize z_n
  for (int i = 0; i < (model->n + model->m); i++)
  {
    z_n[i] = 0.0;
  }
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      z_n[i] += getIndex(model->P_n, i, j) * z_k[j];
    }
  }
  z_k_dot_z_n = 0.0;
  float buffer = 0.0;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    buffer = z_k[i] * z_n[i];
    if (isnanf(buffer) == 0)
    {
      z_k_dot_z_n += buffer;
    }
  }
  if (fabs(model->lambda + z_k_dot_z_n) > 0)
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      g_n[i] = (1.0 / (model->lambda + z_k_dot_z_n)) * z_n[i];
    }
  }
  else
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      g_n[i] = (1.0 / model->lambda) * z_n[i];
    }
  }
  // αₙ = dₙ - transpose(wₙ₋₁) * xₙ
  // initialize alpha_n
  for (int i = 0; i < (model->n + model->m); i++)
  {
    alpha_n[i] = 0.0;
  }
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      alpha_n[i] += 0.0 - getIndex(model->W_n, i, j) * z_k[j];
    }
  }
  updateChange = 0.0;
  float correction = 0.0;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      correction = alpha_n[i] * g_n[j];
      // sum the absolute value of the corrections to the filter coefficients to see if there is any update
      updateChange += fabs(getIndex(model->W_n, i, j) - correction);
      buffer = getIndex(model->W_n, i, j) + correction;
      if (isnanf(buffer) == 0)
      {
        setIndex(&(model->W_n), i, j, buffer); 
      }
    }
  }
  // trigger a policy update if the RLS algorithm has converged
  if (fabs(updateChange) > minimumChange) {
    // triggerUpdate = 1;
  }
  int scaleFlag = 0;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      buffer = (1.0 / model->lambda) * (getIndex(model->P_n, i, j) - g_n[i] * z_n[j]);
      if (isnanf(buffer) == 0)
      {
        if (fabs(buffer) > clipping)
        {
          scaleFlag = 1;
        }
        setIndex(&(model->P_n), i, j, buffer);
      }
    }
  }
  if (scaleFlag == 1)
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      for (int j = 0; j < (model->n + model->m); j++)
      {
        setIndex(&(model->P_n), i, j, clippingFactor * getIndex(model->P_n, i, j));
      }
    }
  }
  // Repeat at the next time k + 1 and continue until RLS converges and the new parameter vector Wⱼ₊₁ is found.
  model->k = model->k + 1;
  return;
}

void updateControlPolicy(LinearQuadraticRegulator *model)
{
  // unpack the vector Wⱼ₊₁ into the kernel matrix
  // Q(xₖ, uₖ) ≡ 0.5 * transpose([xₖ; uₖ]) * S * [xₖ; uₖ] = 0.5 * transpose([xₖ; uₖ]) * [Sₓₓ Sₓᵤ; Sᵤₓ Sᵤᵤ] * [xₖ; uₖ]
  model->k = 1;
  model->j = model->j + 1;
  // initialize the filter matrix
  // putBuffer(model->m + model->n, model->m + model->n, W_n, model->W_n);

  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      S_ux[i][j] = getIndex(model->W_n, model->n + i, j);
    }
  }
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->m; j++)
    {
      S_uu[i][j] = getIndex(model->W_n, model->n + i, model->n + j);
    }
  }

  // Perform the control update using (S24), which is uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  // uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  float determinant = S_uu[1][1] * S_uu[2][2] - S_uu[1][2] * S_uu[2][1];
  // check the rank of S_uu to see if it's equal to 2 (invertible matrix)
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
  int transmit = 0;
  int log_counter = 0;
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
  encodeWheel(&model.reactionEncoder, TIM3->CNT);
  encodeWheel(&model.rollingEncoder, TIM4->CNT);
  senseCurrent(&(model.reactionCurrentSensor), &(model.rollingCurrentSensor));
  updateIMU(&model);

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
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        model.active = 1;
      }
    }
    else
    {
      model.active = 0;
      model.reactionPWM = 0.0;
      model.rollingPWM = 0.0;
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
    }

    if (fabs(model.imu1.roll) > roll_safety_angle || fabs(model.imu1.pitch) > pitch_safety_angle || model.k > max_episode_length)
    {
      model.active = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (model.active == 1)
    {
      stepForward(&model);
    }
    else
    {
      model.reactionPWM = 0.0;
      model.rollingPWM = 0.0;
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      encodeWheel(&model.reactionEncoder, TIM3->CNT);
      encodeWheel(&model.rollingEncoder, TIM4->CNT);
      senseCurrent(&(model.reactionCurrentSensor), &(model.rollingCurrentSensor));
      updateIMU(&model);
    }
    if (model.k % updatePolicyPeriod == 0 || triggerUpdate == 1)
    {
      updateControlPolicy(&model);
      triggerUpdate = 0;
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

      sprintf(MSG,
              "AX1: %0.2f, AY1: %0.2f, AZ1: %0.2f, | AX2: %0.2f, AY2: %0.2f, AZ2: %0.2f, | roll: %0.2f, pitch: %0.2f, | encT: %0.2f, encB: %0.2f, | k: %f, j: %f, | x0: %0.2f, x1: %0.2f, x2: %0.2f, x3: %0.2f, x4: %0.2f, x5: %0.2f, x6: %0.2f, x7: %0.2f, x8: %0.2f, x9: %0.2f, x10: %0.2f, x11: %0.2f, | P0: %0.2f, P1: %0.2f, P2: %0.2f, P3: %0.2f, P4: %0.2f, P5: %0.2f, P6: %0.2f, P7: %0.2f, P8: %0.2f, P9: %0.2f, P10: %0.2f, P11: %0.2f, change: %0.2f, dt: %0.6f\r\n",
              model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.imu1.roll, model.imu1.pitch, model.reactionEncoder.radianAngle, model.rollingEncoder.radianAngle, (float) model.k, (float) model.j, model.dataset.x0, model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5, model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, model.dataset.x10, model.dataset.x11, getIndex(model.P_n, 0, 0), getIndex(model.P_n, 1, 1), getIndex(model.P_n, 2, 2), getIndex(model.P_n, 3, 3), getIndex(model.P_n, 4, 4), getIndex(model.P_n, 5, 5), getIndex(model.P_n, 6, 6), getIndex(model.P_n, 7, 7), getIndex(model.P_n, 8, 8), getIndex(model.P_n, 9, 9), getIndex(model.P_n, 10, 10), getIndex(model.P_n, 11, 11), updateChange, dt);

      // sprintf(MSG,
      //         "x0: %0.2f, x1: %0.2f, x2: %0.2f, x3: %0.2f, x4: %0.2f, x5: %0.2f, x6: %0.2f, x7: %0.2f, x8: %0.2f, x9: %0.2f, dt: %0.6f\r\n",
      //         model.dataset.x0, model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5, model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, dt);

      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | P11: %0.2f, P22: %0.2f, P33: %0.2f, P44: %0.2f, P55: %0.2f, P66: %0.2f, P77: %0.2f, P88: %0.2f, P99: %0.2f, P1010: %0.2f, P1111: %0.2f, P1212: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, P_n[0][0], P_n[0][1], P_n[0][2], P_n[0][3], P_n[0][4], P_n[0][5], P_n[0][6], P_n[0][7], P_n[0][8], P_n[0][9], P_n[0][10], P_n[0][11], dt);
      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | z_k_dot_z_n: %0.2f, P00: %0.2f, P_n11: %0.2f, P_n22: %0.2f, P_n33: %0.2f, P_n44: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, z_k_dot_z_n, getIndex(model.W_n, 0, 0), getIndex(model.W_n, 1, 1), getIndex(model.W_n, 2, 2), getIndex(model.W_n, 3, 3), getIndex(model.W_n, 4, 4), dt);

      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | z_k_dot_z_n: %0.2f, z_n0: %0.2f, z_n1: %0.2f, z_n2: %0.2f, z_n3: %0.2f, z_n4: %0.2f, z_n5: %0.2f, z_n6: %0.2f, z_n7: %0.2f, z_n8: %0.2f, z_n9: %0.2f, z_n10: %0.2f, z_n11: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, z_k_dot_z_n, z_n[0], z_n[1], z_n[2], z_n[3], z_n[4], z_n[5], z_n[6], z_n[7], z_n[8], z_n[9], z_n[10], z_n[11], dt);

      // sprintf(MSG,
      // "yaw: %0.2f, roll: %0.2f, rollv: %0.2f, pitch: %0.2f, pitchv: %0.2f, | aX1: %0.2f, aY1: %0.2f, aZ1: %0.2f, | aX2: %0.2f, aY2: %0.2f, aZ2: %0.2f, | encB: %d, encT: %d, dt: %0.6f\r\n",
      // model.imu1.yaw, model.imu1.roll, model.imu1.roll_velocity, model.imu1.pitch, model.imu1.pitch_velocity, model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, TIM3->CNT, TIM4->CNT, dt);
      // sprintf(MSG, "Bottom: current: %d, curVel: %0.2f, enc: %d, angle: %0.2f, velocity: %0.2f, acceleration: %0.2f, | Top: current: %d, curvel: %0.2f, enc: %d, angle: %0.2f, velocity: %0.2f, acceleration: %0.2f, dt: %0.6f\r\n",
      //   model.rollingCurrentSensor.current0, model.rollingCurrentSensor.currentVelocity, TIM4->CNT, model.rollingEncoder.angle, model.rollingEncoder.velocity, model.rollingEncoder.acceleration,
      //   model.reactionCurrentSensor.current0, model.reactionCurrentSensor.currentVelocity, TIM3->CNT, model.reactionEncoder.angle, model.reactionEncoder.velocity, model.reactionEncoder.acceleration, dt);

      // sprintf(MSG,
      //   "ax2: %d, ay2: %d, az2: %d, | gx2: %d, gy2: %d, gz2: %d, dt: %0.6f\r\n",
      //   model.imu2.rawAccX, model.imu2.rawAccY, model.imu2.rawAccZ, model.imu2.rawGyrX, model.imu2.rawGyrY, model.imu2.rawGyrZ, dt);
      // sprintf(MSG,
      //   "ax2: %0.2f, ay2: %0.2f, az2: %0.2f, | gx2: %0.2f, gy2: %0.2f, gz2: %0.2f, dt: %0.6f\r\n",
      //   model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.imu2.gyrX, model.imu2.gyrY, model.imu2.gyrZ, dt);

      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);
    }
    t2 = DWT->CYCCNT;
    diff = t2 - t1;
    dt = (float)diff / CPU_CLOCK;
    model.dt = dt;
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
  huart1.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC2 PC5 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_8;
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
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
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
