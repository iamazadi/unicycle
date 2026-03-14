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
#define TRANSMIT_LENGTH 512
#define RECEIVE_FRAME_LENGTH 32
#define TRANSMIT_FRAME_LENGTH 5
#define N 12
#define M 3
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

TIM_HandleTypeDef htim1;
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
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[RECEIVE_FRAME_LENGTH] = {0};
uint8_t UART1_txBuffer[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x03, 0x08, 0x1B, 0xCA};
uint8_t UART1_txBuffer_cfg[TRANSMIT_FRAME_LENGTH] = {0xA4, 0x06, 0x01, 0x06, 0xB1};
// response: a4030812f93dfbcf002a000100010001ddb416bafffa48
int uart_receive_ok = 0;
uint8_t transferRequest = MASTER_REQ_ACC_X_H;
uint8_t raw_data[14] = {0};
uint16_t AD_RES = 0;
uint32_t AD_RES_BUFFER[3];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_receive_ok == 0 && huart->Instance == USART1)
  {
    uart_receive_ok = 1;
  }
}

void setServoAngle(uint32_t angle)
{
  if (angle > 180)
  {
    angle = 180;
  }

  uint32_t minPulseWidth = 1000; // 1ms pulse width at a 1MHz clock
  uint32_t maxPulseWidth = 2000; // 2ms pulse width
  uint32_t pulse = ((angle * (maxPulseWidth - minPulseWidth)) / 180) + minPulseWidth;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse); // Changed to TIM2, Channel 1
}

typedef struct
{
  float row0[M + N];
  float row1[M + N];
  float row2[M + N];
  float row3[M + N];
  float row4[M + N];
  float row5[M + N];
  float row6[M + N];
  float row7[M + N];
  float row8[M + N];
  float row9[M + N];
  float row10[M + N];
  float row11[M + N];
  float row12[M + N];
  float row13[M + N];
  float row14[M + N];
} Mat15;

float getIndexMat15(Mat15 matrix, int i, int j)
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
  case 12:
    return matrix.row12[j];
    break;
  case 13:
    return matrix.row13[j];
    break;
  case 14:
    return matrix.row14[j];
    break;
  default:
    break;
  }
  return 0.0;
}

void setIndexMat15(Mat15 *matrix, int i, int j, float value)
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
  case 12:
    matrix->row12[j] = value;
    break;
  case 13:
    matrix->row13[j] = value;
    break;
  case 14:
    matrix->row14[j] = value;
    break;
  default:
    break;
  }
  return;
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
  float x0;
  float x1;
} Vec2;

float getIndexVec2(Vec2 vector, int i)
{
  switch (i)
  {
  case 0:
    return vector.x0;
    break;
  case 1:
    return vector.x1;
    break;

  default:
    break;
  }
  return 0.0;
}

void setIndexVec2(Vec2 *vector, int i, float value)
{
  switch (i)
  {
  case 0:
    vector->x0 = value;
    break;
  case 1:
    vector->x1 = value;
    break;

  default:
    break;
  }
}

typedef struct
{
  float x0;
  float x1;
  float x2;
} Vec3;

float getIndexVec3(Vec3 vector, int i)
{
  switch (i)
  {
  case 0:
    return vector.x0;
    break;
  case 1:
    return vector.x1;
    break;
  case 2:
    return vector.x2;
    break;

  default:
    break;
  }
  return 0.0;
}

void setIndexVec3(Vec3 *vector, int i, float value)
{
  switch (i)
  {
  case 0:
    vector->x0 = value;
    break;
  case 1:
    vector->x1 = value;
    break;
  case 2:
    vector->x2 = value;
    break;

  default:
    break;
  }
}

typedef struct
{
  float row0[2];
  float row1[2];
} Mat2;

float getIndexMat2(Mat2 matrix, int i, int j)
{
  switch (i)
  {
  case 0:
    return matrix.row0[j];
    break;
  case 1:
    return matrix.row1[j];
    break;

  default:
    break;
  }
  return 0.0;
}

void setIndexMat2(Mat2 *matrix, int i, int j, float value)
{
  switch (i)
  {
  case 0:
    matrix->row0[j] = value;
    break;
  case 1:
    matrix->row1[j] = value;
    break;

  default:
    break;
  }
}

typedef struct
{
  float row0[3];
  float row1[3];
  float row2[3];
} Mat3;

float getIndexMat3(Mat3 matrix, int i, int j)
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

  default:
    break;
  }
  return 0.0;
}

void setIndexMat3(Mat3 *matrix, int i, int j, float value)
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

  default:
    break;
  }
}

typedef struct
{
  float row0[4];
  float row1[4];
  float row2[4];
} Mat34;

float getIndexMat34(Mat34 matrix, int i, int j)
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

  default:
    break;
  }
  return 0.0;
}

void setIndexMat34(Mat34 *matrix, int i, int j, float value)
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

  default:
    break;
  }
}

typedef struct
{
  float row0[4];
  float row1[4];
} Mat24;

float getIndexMat24(Mat24 matrix, int i, int j)
{
  switch (i)
  {
  case 0:
    return matrix.row0[j];
    break;
  case 1:
    return matrix.row1[j];
    break;

  default:
    break;
  }
  return 0.0;
}

void setIndexMat24(Mat24 *matrix, int i, int j, float value)
{
  switch (i)
  {
  case 0:
    matrix->row0[j] = value;
    break;
  case 1:
    matrix->row1[j] = value;
    break;

  default:
    break;
  }
}

typedef struct
{
  float row0[2];
  float row1[2];
  float row2[2];
} Mat32;

float getIndexMat32(Mat32 matrix, int i, int j)
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

  default:
    break;
  }
  return 0.0;
}

void setIndexMat32(Mat32 *matrix, int i, int j, float value)
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

  default:
    break;
  }
}

typedef struct
{
  int16_t accXOffset;
  int16_t accYOffset;
  int16_t accZOffset;
  float accXScale;
  float accYScale;
  float accZScale;
  int16_t gyrXOffset;
  int16_t gyrYOffset;
  int16_t gyrZOffset;
  float gyrXScale;
  float gyrYScale;
  float gyrZScale;
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
  Mat3 B_A_R; // The rotation of the local frame of the sensor i to the robot frame B̂
  Vec3 R;     // accelerometer sensor measurements in the local frame of the sensors
  Vec3 _R;    // accelerometer sensor measurements in the robot body frame
  Vec3 G;     // gyro sensor measurements in the local frame of the sensors
  Vec3 _G;    // gyro sensor measurements in the robot body frame
} IMU;

void encodeWheel(Encoder *encoder, int newValue)
{
  int difference = newValue - encoder->value;
  if (abs(difference) > 30000)
  {
    if (newValue > 30000)
    {
      difference = (newValue - 65535) - encoder->value;
    }
    else
    {
      difference = newValue - (encoder->value - 65535);
    }
  }
  encoder->value = newValue;
  float angle = encoder->radianAngle + (float)difference / (float)encoder->pulse_per_revolution * 2.0 * M_PI;
  // encoder->radianAngle = (float)(encoder->value % encoder->pulse_per_revolution) / (float)encoder->pulse_per_revolution * 2.0 * M_PI;
  // float angle = sin(encoder->radianAngle);
  float velocity = angle - encoder->radianAngle;
  float acceleration = velocity - encoder->velocity;
  encoder->radianAngle = angle;
  encoder->velocity = velocity;
  encoder->acceleration = acceleration;
  return;
}

void senseCurrent(CurrentSensor *reactionCurrentSensor, CurrentSensor *rollingCurrentSensor, CurrentSensor *pendulumCurrentSensor)
{
  // Start ADC Conversion in DMA Mode (Periodically Every 1ms)
  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 3);
  reactionCurrentSensor->current1 = reactionCurrentSensor->current0;
  rollingCurrentSensor->current1 = rollingCurrentSensor->current0;
  pendulumCurrentSensor->current1 = pendulumCurrentSensor->current0;
  reactionCurrentSensor->current0 = (AD_RES_BUFFER[0] << 4);
  rollingCurrentSensor->current0 = (AD_RES_BUFFER[1] << 4);
  pendulumCurrentSensor->current0 = (AD_RES_BUFFER[2] << 4);
  reactionCurrentSensor->currentVelocity = (float)(reactionCurrentSensor->current0 - reactionCurrentSensor->current1) / reactionCurrentSensor->currentScale;
  rollingCurrentSensor->currentVelocity = (float)(rollingCurrentSensor->current0 - rollingCurrentSensor->current1) / rollingCurrentSensor->currentScale;
  pendulumCurrentSensor->currentVelocity = (float)(pendulumCurrentSensor->current0 - pendulumCurrentSensor->current1) / pendulumCurrentSensor->currentScale;
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
    sensor->accX = sensor->accXScale * (sensor->rawAccX - sensor->accXOffset);
    sensor->accY = sensor->accYScale * (sensor->rawAccY - sensor->accYOffset);
    sensor->accZ = sensor->accZScale * (sensor->rawAccZ - sensor->accZOffset);
    sensor->gyrX = sensor->gyrXScale * (sensor->rawGyrX - sensor->gyrXOffset);
    sensor->gyrY = sensor->gyrYScale * (sensor->rawGyrY - sensor->gyrYOffset);
    sensor->gyrZ = sensor->gyrZScale * (sensor->rawGyrZ - sensor->gyrZOffset);
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
      sensor->accX = sensor->accXScale * (sensor->rawAccX - sensor->accXOffset);
      sensor->accY = sensor->accYScale * (sensor->rawAccY - sensor->accYOffset);
      sensor->accZ = sensor->accZScale * (sensor->rawAccZ - sensor->accZOffset);
      sensor->gyrX = sensor->gyrXScale * (sensor->rawGyrX - sensor->gyrXOffset);
      sensor->gyrY = sensor->gyrYScale * (sensor->rawGyrY - sensor->gyrYOffset);
      sensor->gyrZ = sensor->gyrZScale * (sensor->rawGyrZ - sensor->gyrZOffset);
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
} Vec15;

float getIndexVec15(Vec15 vector, int i)
{
  switch (i)
  {
  case 0:
    return vector.x0;
    break;

  case 1:
    return vector.x1;
    break;

  case 2:
    return vector.x2;
    break;

  case 3:
    return vector.x3;
    break;

  case 4:
    return vector.x4;
    break;

  case 5:
    return vector.x5;
    break;

  case 6:
    return vector.x6;
    break;

  case 7:
    return vector.x7;
    break;

  case 8:
    return vector.x8;
    break;

  case 9:
    return vector.x9;
    break;

  case 10:
    return vector.x10;
    break;

  case 11:
    return vector.x11;
    break;
  case 12:
    return vector.x12;
    break;
  case 13:
    return vector.x13;
    break;
  case 14:
    return vector.x14;
    break;
  default:
    break;
  }
  return 0.0;
}

void setIndexVec15(Vec15 *vector, int i, float value)
{
  switch (i)
  {
  case 0:
    vector->x0 = value;
    break;
  case 1:
    vector->x1 = value;
    break;
  case 2:
    vector->x2 = value;
    break;
  case 3:
    vector->x3 = value;
    break;
  case 4:
    vector->x4 = value;
    break;
  case 5:
    vector->x5 = value;
    break;
  case 6:
    vector->x6 = value;
    break;
  case 7:
    vector->x7 = value;
    break;
  case 8:
    vector->x8 = value;
    break;
  case 9:
    vector->x9 = value;
    break;
  case 10:
    vector->x10 = value;
    break;
  case 11:
    vector->x11 = value;
    break;
  case 12:
    vector->x12 = value;
    break;
  case 13:
    vector->x13 = value;
    break;
  case 14:
    vector->x14 = value;
    break;
  default:
    break;
  }
}

typedef struct
{
  float row0[N];
  float row1[N];
  float row2[N];
} Mat312;

float getIndexMat312(Mat312 matrix, int i, int j)
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
  default:
    break;
  }
  return 0.0;
}

void setIndexMat312(Mat312 *matrix, int i, int j, float value)
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
  default:
    break;
  }
}

// Represents a Linear Quadratic Regulator (LQR) model.
typedef struct
{
  Mat15 W_n;       // filter matrix
  Mat15 P_n;       // inverse autocorrelation matrix
  Mat312 K_j;      // feedback policy
  Vec15 dataset;   // (xₖ, uₖ)
  Vec15 z_n;       // z_n in RLS
  Vec15 g_n;       // g_n in RLS
  Vec15 alpha_n;   // alpha_n in RLS
  Mat3 Suu;        // The input-input kernel
  Mat3 SuuInverse; // the inverse of the input-input kernel
  Mat312 Sux;      // the input-state kernel
  Vec3 u_k;
  float x_n_dot_z_n;                   // the inner product of the x_n (dataset) and z_n
  int j;                               // step number
  int k;                               // time k
  int n;                               // xₖ ∈ ℝⁿ
  int m;                               // uₖ ∈ ℝᵐ
  float lambda;                        // exponential wighting factor
  float delta;                         // value used to intialize P(0)
  int active;                          // is the model controller active
  float CPUClock;                      // the CPU clock
  float dt;                            // period in seconds
  float reactionDutyCycle;             // reaction wheel's motor PWM duty cycle
  float rollingDutyCycle;              // rolling wheel's motor PWM duty cycle
  float servoAngle;                    // the angle of the laterla pendulum servo motor
  float reactionDutyCycleChange;       // the maximum incremental change in the reaction motor's duty cycle
  float rollingDutyCycleChange;        // the maximum incremental change in the rolling motor's duty cycle
  float servoAngleChange;              // the incremental changes to the servo angle
  float decayValue;                    // the decay value for any of the P matrix elements at which decaying is applied
  float decayFactor;                   // the coefficient by which the P matrix elements are rescaled through scalar multiplication
  float rollSafetyAngle;               // the roll angle in radian beyond which the controller must become deactive for safety
  float pitchSafetyAngle;              // the pitch angle in radian beyond which the controller must become deactive for safety
  float kappa1;                        // tuning parameters to minimize estimate variance (the ratio between the accelerometer and the gyroscope in sensor fusion)
  float kappa2;                        // tuning parameters to minimize estimate variance (the ratio between the accelerometer and the gyroscope in sensor fusion)
  int maxEpisodeLength;                // the maximum number of interactions with the nevironment before the model becomes deactive for safety
  int logPeriod;                       // the period between printing two log messages in terms of control cycles
  int logCounter;                      // the number of control cycles elpased since the last log message printing
  int maxOutOfBounds;                  // the maximum number of consecutive cycles where states are out of the safety bounds
  int outOfBoundsCounter;              // the number of consecutive times when either of safety angles have been detected out of bounds
  float beta;                          // y-Euler angle (pitch)
  float gamma;                         // x-Euler angle (roll)
  float fusedBeta;                     // y-Euler angle (pitch) as the result of fusing the accelerometer sensor measurements with the gyroscope sensor measurements
  float fusedGamma;                    // x-Euler angle (roll) as the result of fusing the accelerometer sensor measurements with the gyroscope sensor measurements
  int noiseQuotient;                   // the quotient of the random number for generating the probing noise
  float noiseScale;                    // the scale of by which the remainder of the probing noise is to be divided
  float time;                          // the time that has elapsed since the start up of the microcontroller in seconds
  float changes;                       // the magnitude of the changes to the filter coefficients after one step forward
  float convergenceThreshold;          // the threshold value of the changes to filter coefficients below which the RLS is assumed to be converged
  int convergenceCounter;              // the number of consecutive times that the changes to filter coefficients are less than the convergence threshold
  int convergenceMaxCount;             // the maximum number of consecutive times for the changes below the threshold to determine convergence
  Mat34 Q;                             // The matrix of unknown parameters
  Vec3 r;                              // the average of the body angular rate from rate gyro
  Vec3 rDot;                           // the average of the body angular rate in Euler angles
  Mat3 E;                              // a matrix transfom from body rates to Euler angular rates
  Mat24 X;                             // The optimal fusion matrix
  Mat32 Matrix;                        // all sensor measurements combined
  Vec3 g;                              // The gravity vector                          // the input vector
  IMU imu1;                            // the first inertial measurement unit
  IMU imu2;                            // the second inertial measurement unit
  Encoder reactionEncoder;             // the reaction wheel encoder
  Encoder rollingEncoder;              // the rolling wheel encoder
  CurrentSensor reactionCurrentSensor; // the reaction wheel's motor current sensor
  CurrentSensor rollingCurrentSensor;  // the rolling wheel's motor current sensor
  CurrentSensor pendulumCurrentSensor; // the lateral pendulum current sensor
  Vec3 position;                       // the position of the body relative to the inertial frame origin
} LinearQuadraticRegulator;

void updateIMU(LinearQuadraticRegulator *model)
{
  // updateIMU1(&(model->imu1));
  updateIMU2(&(model->imu2));
  // setIndexVec3(&(model->imu1.R), 0, model->imu1.accX);
  // setIndexVec3(&(model->imu1.R), 1, model->imu1.accY);
  // setIndexVec3(&(model->imu1.R), 2, model->imu1.accZ);
  setIndexVec3(&(model->imu2.R), 0, model->imu2.accX);
  setIndexVec3(&(model->imu2.R), 1, model->imu2.accY);
  setIndexVec3(&(model->imu2.R), 2, model->imu2.accZ);

  // for (int i = 0; i < 3; i++)
  // {
  //   setIndexVec3(&(model->imu1._R), i, 0.0);
  //   setIndexVec3(&(model->imu2._R), i, 0.0);
  // }

  // for (int i = 0; i < 3; i++)
  // {
  //   for (int j = 0; j < 3; j++)
  //   {
  //     setIndexVec3(&(model->imu1._R), i, getIndexVec3(model->imu1._R, i) + getIndexMat3(model->imu1.B_A_R, i, j) * getIndexVec3(model->imu1.R, j));
  //     setIndexVec3(&(model->imu2._R), i, getIndexVec3(model->imu2._R, i) + getIndexMat3(model->imu2.B_A_R, i, j) * getIndexVec3(model->imu2.R, j));
  //   }
  // }

  // for (int i = 0; i < 3; i++)
  // {
  //   setIndexMat32(&(model->Matrix), i, 0, getIndexVec3(model->imu1._R, i));
  //   setIndexMat32(&(model->Matrix), i, 1, getIndexVec3(model->imu2._R, i));
  // }

  // for (int i = 0; i < 3; i++)
  // {
  //   for (int j = 0; j < 4; j++)
  //   {
  //     setIndexMat34(&(model->Q), i, j, 0.0);
  //     for (int k = 0; k < 2; k++)
  //     {
  //       setIndexMat34(&(model->Q), i, j, getIndexMat34(model->Q, i, j) + getIndexMat32(model->Matrix, i, k) * getIndexMat24(model->X, k, j));
  //     }
  //   }
  // }
  // setIndexVec3(&(model->g), 0, getIndexMat34(model->Q, 0, 0));
  // setIndexVec3(&(model->g), 1, getIndexMat34(model->Q, 1, 0));
  // setIndexVec3(&(model->g), 2, getIndexMat34(model->Q, 2, 0));
  setIndexVec3(&(model->g), 0, getIndexVec3(model->imu2.R, 0));
  setIndexVec3(&(model->g), 1, getIndexVec3(model->imu2.R, 1));
  setIndexVec3(&(model->g), 2, getIndexVec3(model->imu2.R, 2));
  model->beta = atan2(-getIndexVec3(model->g, 0), sqrt(pow(getIndexVec3(model->g, 1), 2) + pow(getIndexVec3(model->g, 2), 2)));
  model->gamma = atan2(getIndexVec3(model->g, 1), getIndexVec3(model->g, 2));

  // setIndexVec3(&(model->imu1.G), 0, model->imu1.gyrX);
  // setIndexVec3(&(model->imu1.G), 1, model->imu1.gyrY);
  // setIndexVec3(&(model->imu1.G), 2, model->imu1.gyrZ);
  setIndexVec3(&(model->imu2.G), 0, model->imu2.gyrX);
  setIndexVec3(&(model->imu2.G), 1, model->imu2.gyrY);
  setIndexVec3(&(model->imu2.G), 2, model->imu2.gyrZ);

  for (int i = 0; i < 3; i++)
  {
    // setIndexVec3(&(model->imu1._G), i, 0.0);
    setIndexVec3(&(model->imu2._G), i, 0.0);
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      // setIndexVec3(&(model->imu1._G), i, getIndexVec3(model->imu1._G, i) + getIndexMat3(model->imu1.B_A_R, i, j) * getIndexVec3(model->imu1.G, j));
      setIndexVec3(&(model->imu2._G), i, getIndexVec3(model->imu2._G, i) + getIndexMat3(model->imu2.B_A_R, i, j) * getIndexVec3(model->imu2.G, j));
    }
  }
  for (int i = 0; i < 3; i++)
  {
    // setIndexVec3(&(model->r), i, (getIndexVec3(model->imu1._G, i) + getIndexVec3(model->imu2._G, i)) / 2.0);
    setIndexVec3(&(model->r), i, getIndexVec3(model->imu2._G, i));
  }

  setIndexMat3(&(model->E), 0, 0, 0.0);
  setIndexMat3(&(model->E), 0, 1, sin(model->gamma) / cos(model->beta));
  setIndexMat3(&(model->E), 0, 2, cos(model->gamma) / cos(model->beta));
  setIndexMat3(&(model->E), 1, 0, 0.0);
  setIndexMat3(&(model->E), 1, 1, cos(model->gamma));
  setIndexMat3(&(model->E), 1, 2, -sin(model->gamma));
  setIndexMat3(&(model->E), 2, 0, 1.0);
  setIndexMat3(&(model->E), 2, 1, sin(model->gamma) * tan(model->beta));
  setIndexMat3(&(model->E), 2, 2, cos(model->gamma) * tan(model->beta));

  for (int i = 0; i < 3; i++)
  {
    setIndexVec3(&(model->rDot), i, 0.0);
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      setIndexVec3(&(model->rDot), i, getIndexVec3(model->rDot, i) + getIndexMat3(model->E, i, j) * getIndexVec3(model->r, j));
    }
  }

  float _roll_velocity = getIndexVec3(model->rDot, 1) / 180.0 * M_PI;
  float _pitch_velocity = getIndexVec3(model->rDot, 2) / 180.0 * M_PI;
  float _yaw_velocity = getIndexVec3(model->rDot, 0) / 180.0 * M_PI;
  model->fusedBeta = model->kappa1 * model->beta + (1.0 - model->kappa1) * (model->fusedBeta + model->dt * (getIndexVec3(model->rDot, 1) / 180.0 * M_PI));
  model->fusedGamma = model->kappa2 * model->gamma + (1.0 - model->kappa2) * (model->fusedGamma + model->dt * (getIndexVec3(model->rDot, 2) / 180.0 * M_PI));
  model->imu2.yaw += model->dt * _yaw_velocity;

  float _roll = model->fusedBeta;
  float _pitch = -model->fusedGamma;
  _roll_velocity = ((getIndexVec3(model->rDot, 1) / 180.0 * M_PI) + (_roll - model->imu2.roll) / model->dt) / 2.0;
  _pitch_velocity = ((-getIndexVec3(model->rDot, 2) / 180.0 * M_PI) + (_pitch - model->imu2.pitch) / model->dt) / 2.0;
  
  model->imu2.roll_acceleration = _roll_velocity - model->imu2.roll_velocity;
  model->imu2.pitch_acceleration = _pitch_velocity - model->imu2.pitch_velocity;
  model->imu2.roll_velocity = _roll_velocity;
  model->imu2.pitch_velocity = _pitch_velocity;
  model->imu2.yaw_velocity = _yaw_velocity;
  model->imu2.roll = _roll;
  model->imu2.pitch = _pitch;
}

void calculatePosition(LinearQuadraticRegulator *model)
{
  float delta_distance = -model->rollingEncoder.velocity * 0.075;
  float angle = model->imu2.yaw;
  Vec3 delta_position;
  Mat3 rotation;
  setIndexMat3(&rotation, 0, 0, cos(angle));
  setIndexMat3(&rotation, 0, 1, -sin(angle));
  setIndexMat3(&rotation, 0, 2, 0.0);
  setIndexMat3(&rotation, 1, 0, sin(angle));
  setIndexMat3(&rotation, 1, 1, cos(angle));
  setIndexMat3(&rotation, 1, 2, 0.0);
  setIndexMat3(&rotation, 2, 0, 0.0);
  setIndexMat3(&rotation, 2, 1, 0.0);
  setIndexMat3(&rotation, 2, 2, 1.0);

  for (int i = 0; i < 3; i++)
  {
    setIndexVec3(&delta_position, i, 0.0);
  }
  setIndexVec3(&delta_position, 0, delta_distance);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      setIndexVec3(&(delta_position), i, getIndexVec3(delta_position, i) + getIndexMat3(rotation, i, j) * getIndexVec3(delta_position, j));
    }
  }
  setIndexVec3(&(model->position), 0, getIndexVec3(model->position, 0) + getIndexVec3(delta_position, 0));
  setIndexVec3(&(model->position), 1, getIndexVec3(model->position, 1) + getIndexVec3(delta_position, 1));
  setIndexVec3(&(model->position), 2, getIndexVec3(model->position, 2) + getIndexVec3(delta_position, 2));
}

void updateSensors(LinearQuadraticRegulator *model)
{
  updateIMU(model);
  encodeWheel(&(model->reactionEncoder), TIM3->CNT);
  encodeWheel(&(model->rollingEncoder), TIM4->CNT);
  senseCurrent(&(model->reactionCurrentSensor), &(model->rollingCurrentSensor), &(model->pendulumCurrentSensor));
  // dataset = (xₖ, uₖ)
  // setIndexVec15(&(model->dataset), 0, model->imu2.roll);
  setIndexVec15(&(model->dataset), 0, model->reactionEncoder.acceleration);
  setIndexVec15(&(model->dataset), 1, model->imu2.roll_velocity);
  setIndexVec15(&(model->dataset), 2, model->imu2.roll_acceleration);
  // setIndexVec15(&(model->dataset), 2, 0.0);
  // setIndexVec15(&(model->dataset), 3, model->imu2.pitch);
  setIndexVec15(&(model->dataset), 3, model->rollingEncoder.acceleration);
  setIndexVec15(&(model->dataset), 4, model->imu2.pitch_velocity);
  setIndexVec15(&(model->dataset), 5, model->imu2.pitch_acceleration);
  // setIndexVec15(&(model->dataset), 5, 0.0);
  setIndexVec15(&(model->dataset), 6, model->reactionEncoder.velocity);
  setIndexVec15(&(model->dataset), 7, model->rollingEncoder.velocity);
  setIndexVec15(&(model->dataset), 8, model->imu2.yaw_velocity);
  setIndexVec15(&(model->dataset), 9, model->reactionCurrentSensor.currentVelocity);
  setIndexVec15(&(model->dataset), 10, model->rollingCurrentSensor.currentVelocity);
  setIndexVec15(&(model->dataset), 11, model->pendulumCurrentSensor.currentVelocity);
  calculatePosition(model);
  return;
}

void computeFeedbackPolicy(LinearQuadraticRegulator *model)
{
  setIndexVec3(&(model->u_k), 0, 0.0);
  setIndexVec3(&(model->u_k), 1, 0.0);
  setIndexVec3(&(model->u_k), 2, 0.0);
  // feeback policy
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      setIndexVec3(&(model->u_k), i, getIndexVec3(model->u_k, i) + -getIndexMat312(model->K_j, i, j) * getIndexVec15(model->dataset, j));
    }
  }
  // dataset = (xₖ, uₖ)
  setIndexVec15(&(model->dataset), 12, getIndexVec3(model->u_k, 0));
  setIndexVec15(&(model->dataset), 13, getIndexVec3(model->u_k, 1));
  setIndexVec15(&(model->dataset), 14, getIndexVec3(model->u_k, 2));
  return;
}

void applyFeedbackPolicy(LinearQuadraticRegulator *model)
{
  // Add probing noise to the input for persistent excitation
  int seed = DWT->CYCCNT;
  srand(seed);
  float input0 = model->dataset.x12 + (float)(rand() % model->noiseQuotient - model->noiseQuotient / 2) / model->noiseScale;
  model->dataset.x12 = input0;
  seed = DWT->CYCCNT;
  srand(seed);
  float input1 = model->dataset.x13 + (float)(rand() % model->noiseQuotient - model->noiseQuotient / 2) / model->noiseScale;
  model->dataset.x13 = input1;
  seed = DWT->CYCCNT;
  srand(seed);
  float input2 = model->dataset.x14 + (float)(rand() % model->noiseQuotient - model->noiseQuotient / 2) / model->noiseScale;
  model->dataset.x14 = input2;
  model->reactionDutyCycle += model->reactionDutyCycleChange * input0;
  model->rollingDutyCycle += model->rollingDutyCycleChange * input1;
  model->servoAngle += model->servoAngleChange * input2;
  model->reactionDutyCycle = fmin(255.0 * 255.0, model->reactionDutyCycle);
  model->reactionDutyCycle = fmax(-255.0 * 255.0, model->reactionDutyCycle);
  model->rollingDutyCycle = fmin(255.0 * 255.0, model->rollingDutyCycle);
  model->rollingDutyCycle = fmax(-255.0 * 255.0, model->rollingDutyCycle);
  model->servoAngle = fmin(80.0, model->servoAngle);
  model->servoAngle = fmax(-80.0, model->servoAngle);
  TIM2->CCR1 = (int)fabs(model->rollingDutyCycle);
  TIM2->CCR2 = (int)fabs(model->reactionDutyCycle);
  if (model->reactionDutyCycle < 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  }
  if (model->rollingDutyCycle < 0)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  }
  setServoAngle(90.0 + model->servoAngle);
}

void resetActuators(LinearQuadraticRegulator *model)
{
  model->reactionDutyCycle = 0.0;
  model->rollingDutyCycle = 0.0;
  model->servoAngle = 0.0;
  setServoAngle(90.0);
  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
}

float calculateChanges(Mat15 W_1, Mat15 W_2)
{
  float changes = 0.0;
  for (int i = 0; i < N + M; i++)
  {
    for (int j = 0; j < N + M; j++)
    {
      changes += fabs(getIndexMat15(W_2, i, j) - getIndexMat15(W_1, i, j));
    }
  }
  return changes;
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
  model->n = N;
  model->m = M;
  model->lambda = 0.85;
  model->delta = 0.01;
  model->active = 0;
  model->CPUClock = 84000000.0;
  model->dt = 0.0;
  model->reactionDutyCycleChange = 255.0 * 84.0;
  model->rollingDutyCycleChange = 255.0 * 32.0;
  model->servoAngleChange = 1.5;
  model->changes = 0.0;
  model->convergenceThreshold = 1.0;
  model->convergenceCounter = 0;
  model->convergenceMaxCount = 3;
  model->servoAngle = 0.0;
  model->reactionDutyCycle = 0.0;
  model->rollingDutyCycle = 0.0;
  model->decayValue = 100.0;
  model->decayFactor = 0.90;
  model->rollSafetyAngle = 0.11;
  model->pitchSafetyAngle = 0.11;
  model->maxEpisodeLength = 50000;
  model->logPeriod = 2;
  model->logCounter = 0;
  model->maxOutOfBounds = 10;
  model->outOfBoundsCounter = 0;
  model->kappa1 = 0.01;
  model->kappa2 = 0.01;
  model->noiseQuotient = 100;
  model->noiseScale = 100000.0;
  model->time = 0.0;

  Mat15 P_n;
  Mat15 W_n;
  Mat312 K_j;
  Vec15 dataset;
  Vec3 u_k;
  Vec15 z_n;
  Vec15 g_n;
  Vec15 alpha_n;
  Mat3 Suu;
  Mat3 SuuInverse;
  Mat312 Sux;

  Mat3 B_A1_R;
  Mat3 B_A2_R;
  Mat34 Q;
  Mat24 X;
  Vec3 R1;
  Vec3 R2;
  Vec3 _R1;
  Vec3 _R2;
  Mat32 Matrix;
  Vec3 g;
  Vec3 r;
  Vec3 rDot;
  Vec3 G1;
  Vec3 G2;
  Vec3 _G1;
  Vec3 _G2;
  Mat3 E;

  for (int i = 0; i < M; i++)
  {
    for (int j = 0; j < M; j++)
    {
      setIndexMat3(&Suu, i, j, 0.0);
      setIndexMat3(&SuuInverse, i, j, 0.0);
    }
    for (int j = 0; j < N; j++)
    {
      setIndexMat312(&Sux, i, j, 0.0);
    }
  }

  int seed = DWT->CYCCNT;
  srand(seed);

  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      seed = DWT->CYCCNT;
      srand(seed);
      setIndexMat15(&W_n, i, j, (float)(rand() % 100) / 100.0);
      if (i == j)
      {
        setIndexMat15(&P_n, i, j, 1.0);
      }
      else
      {
        setIndexMat15(&P_n, i, j, 0.0);
      }
      setIndexMat312(&K_j, i, j, (float)(rand() % 100) / 100.0);
    }
    setIndexVec15(&dataset, i, 0.0);
  }

  for (int i = 0; i < model->n + model->m; i++)
  {
    setIndexVec15(&z_n, i, 0.0);
    setIndexVec15(&g_n, i, 0.0);
    setIndexVec15(&alpha_n, i, 0.0);
  }

  setIndexVec3(&u_k, 0, 0.0);
  setIndexVec3(&u_k, 1, 0.0);
  setIndexVec3(&u_k, 2, 0.0);

  for (int i = 0; i < 3; i++)
  {
    setIndexVec3(&R1, i, 0.0);
    setIndexVec3(&R2, i, 0.0);
    setIndexVec3(&_R1, i, 0.0);
    setIndexVec3(&_R2, i, 0.0);
    setIndexVec3(&g, i, 0.0);
    setIndexVec3(&r, i, 0.0);
    setIndexVec3(&rDot, i, 0.0);
    setIndexVec3(&G1, i, 0.0);
    setIndexVec3(&G2, i, 0.0);
    setIndexVec3(&_G1, i, 0.0);
    setIndexVec3(&_G2, i, 0.0);
    for (int j = 0; j < 3; j++)
    {
      if (i == j)
      {
        setIndexMat3(&B_A1_R, i, j, 1.0);
        // setIndexMat3(&B_A2_R, i, j, 1.0);
      }
      else
      {
        setIndexMat3(&B_A1_R, i, j, 0.0);
        // setIndexMat3(&B_A2_R, i, j, 0.0);
      }
      setIndexMat3(&E, i, j, 0.0);
    }
  }

  setIndexMat3(&B_A2_R, 0, 0, 0.0);
  setIndexMat3(&B_A2_R, 1, 0, 1.0);
  setIndexMat3(&B_A2_R, 2, 0, 0.0);

  setIndexMat3(&B_A2_R, 0, 1, -1.0);
  setIndexMat3(&B_A2_R, 1, 1, 0.0);
  setIndexMat3(&B_A2_R, 2, 1, 0.0);

  setIndexMat3(&B_A2_R, 0, 2, 0.0);
  setIndexMat3(&B_A2_R, 1, 2, 0.0);
  setIndexMat3(&B_A2_R, 2, 2, 1.0);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      setIndexMat32(&Matrix, i, j, 0.0);
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      setIndexMat34(&Q, i, j, 0.0);
    }
  }

  setIndexMat24(&X, 0, 0, 1.568);
  setIndexMat24(&X, 0, 1, 15.28);
  setIndexMat24(&X, 0, 2, 20.80);
  setIndexMat24(&X, 0, 3, 8.028);
  setIndexMat24(&X, 1, 0, -0.639);
  setIndexMat24(&X, 1, 1, -13.65);
  setIndexMat24(&X, 1, 2, 36.04);
  setIndexMat24(&X, 1, 3, -2.785);

  model->P_n = P_n;
  model->W_n = W_n;
  model->K_j = K_j;
  model->dataset = dataset;
  model->Suu = Suu;
  model->Suu = SuuInverse;
  model->Sux = Sux;
  model->Q = Q;
  model->E = E;
  model->X = X;
  model->Matrix = Matrix;
  model->r = r;
  model->rDot = rDot;
  model->g = g;
  model->u_k = u_k;
  model->z_n = z_n;
  model->g_n = g_n;
  model->alpha_n = alpha_n;
  model->beta = 0.0;
  model->fusedBeta = 0.0;
  model->gamma = 0.0;
  model->fusedGamma = 0.0;

  IMU imu1;
  IMU imu2;
  imu1.accXOffset = -12;
  imu1.accYOffset = -90;
  imu1.accZOffset = 30;
  imu1.accXScale = 0.000488281; // scale : 1 / 2048
  imu1.accYScale = 0.000488281;
  imu1.accZScale = 0.000488281;
  imu1.gyrXOffset = 0;
  imu1.gyrYOffset = 0;
  imu1.gyrZOffset = 0;
  imu1.gyrXScale = 0.017444444;
  imu1.gyrYScale = 0.017444444;
  imu1.gyrZScale = 0.017444444;
  imu1.B_A_R = B_A1_R;
  imu1.R = R1;
  imu1._R = _R1;
  imu1.G = G1;
  imu1._G = _G1;
  imu2.accXOffset = 0;
  imu2.accYOffset = 0;
  imu2.accZOffset = 0;
  imu2.accXScale = 0.000488281;
  imu2.accYScale = 0.000488281;
  imu2.accZScale = 0.000488281;
  imu2.gyrXOffset = 0;
  imu2.gyrYOffset = 0;
  imu2.gyrZOffset = 0;
  imu2.gyrXScale = 0.017444444;
  imu2.gyrYScale = 0.017444444;
  imu2.gyrZScale = 0.017444444;
  imu2.B_A_R = B_A2_R;
  imu2.R = R2;
  imu2._R = _R2;
  imu2.G = G2;
  imu2._G = _G2;
  Encoder reactionEncoder = {1736, 0, 0, 0, 0, 0};
  Encoder rollingEncoder = {3020, 0, 0, 0, 0, 0};
  CurrentSensor reactionCurrentSensor = {31500.0, 0, 0, 0};
  CurrentSensor rollingCurrentSensor = {31500.0, 0, 0, 0};
  CurrentSensor pendulumCurrentSensor = {31500.0, 0, 0, 0};
  model->imu1 = imu1;
  model->imu2 = imu2;
  model->reactionEncoder = reactionEncoder;
  model->rollingEncoder = rollingEncoder;
  model->reactionCurrentSensor = reactionCurrentSensor;
  model->rollingCurrentSensor = rollingCurrentSensor;
  model->pendulumCurrentSensor = pendulumCurrentSensor;
  model->reactionDutyCycle = 0.0;
  model->rollingDutyCycle = 0.0;
  Vec3 position;
  setIndexVec3(&position, 0, 0.0);
  setIndexVec3(&position, 1, 0.0);
  setIndexVec3(&position, 2, 0.0);
  model->position = position;
  return;
}
/*
Identify the Q function using RLS with the given pointer to the `model`.
The algorithm is finished when there are no further updates
to the Q function or the control policy at each step.
*/
void stepForward(LinearQuadraticRegulator *model)
{
  for (int i = 0; i < (model->n + model->m); i++)
  {
    setIndexVec15(&(model->z_n), i, 0.0);
  }
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      setIndexVec15(&(model->z_n), i, getIndexVec15(model->z_n, i) + getIndexMat15(model->P_n, i, j) * getIndexVec15(model->dataset, j));
    }
  }
  model->x_n_dot_z_n = 0.0;
  float buffer = 0.0;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    buffer = getIndexVec15(model->dataset, i) * getIndexVec15(model->z_n, i);
    if (isnanf(buffer) == 0)
    {
      model->x_n_dot_z_n += buffer;
    }
  }
  if (fabs(model->lambda + model->x_n_dot_z_n) > 0)
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      setIndexVec15(&(model->g_n), i, (1.0 / (model->lambda + model->x_n_dot_z_n)) * getIndexVec15(model->z_n, i));
    }
  }
  else
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      setIndexVec15(&(model->g_n), i, (1.0 / model->lambda) * getIndexVec15(model->z_n, i));
    }
  }
  for (int i = 0; i < (model->n + model->m); i++)
  {
    setIndexVec15(&(model->alpha_n), i, 0.0);
  }
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      setIndexVec15(&(model->alpha_n), i, getIndexVec15(model->alpha_n, i) + 0.0 - getIndexMat15(model->W_n, i, j) * getIndexVec15(model->dataset, j));
    }
  }
  // a backup of old filter coefficients before updating for calculating the magnitude of changes
  Mat15 W_1;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      setIndexMat15(&W_1, i, j, getIndexMat15(model->W_n, i, j));
      buffer = getIndexMat15(model->W_n, i, j) + getIndexVec15(model->alpha_n, i) * getIndexVec15(model->g_n, j);
      if (isnanf(buffer) == 0)
      {
        setIndexMat15(&(model->W_n), i, j, buffer);
      }
    }
  }
  float _changes = calculateChanges(W_1, model->W_n);
  if (_changes <= model->convergenceThreshold)
  {
    model->convergenceCounter = model->convergenceCounter + 1;
  }
  else
  {
    model->convergenceCounter = 0;
  }
  model->changes = _changes;
  int scaleFlag = 0;
  for (int i = 0; i < (model->n + model->m); i++)
  {
    for (int j = 0; j < (model->n + model->m); j++)
    {
      buffer = (1.0 / model->lambda) * (getIndexMat15(model->P_n, i, j) - getIndexVec15(model->g_n, i) * getIndexVec15(model->z_n, j));
      if (isnanf(buffer) == 0)
      {
        if (fabs(buffer) > model->decayValue)
        {
          scaleFlag = 1;
        }
        setIndexMat15(&(model->P_n), i, j, buffer);
      }
    }
  }
  if (scaleFlag == 1)
  {
    for (int i = 0; i < (model->n + model->m); i++)
    {
      for (int j = 0; j < (model->n + model->m); j++)
      {
        setIndexMat15(&(model->P_n), i, j, model->decayFactor * getIndexMat15(model->P_n, i, j));
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

  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->n; j++)
    {
      setIndexMat312(&(model->Sux), i, j, getIndexMat15(model->W_n, model->n + i, j));
    }
  }
  for (int i = 0; i < model->m; i++)
  {
    for (int j = 0; j < model->m; j++)
    {
      setIndexMat3(&(model->Suu), i, j, getIndexMat15(model->W_n, model->n + i, model->n + j));
    }
  }

  // Perform the control update using (S24), which is uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  // uₖ = -S⁻¹ᵤᵤ * Sᵤₓ * xₖ
  // float determinant = getIndexMat3(model->Suu, 1, 1) * getIndexMat3(model->Suu, 2, 2) - getIndexMat3(model->Suu, 1, 2) * getIndexMat3(model->Suu, 2, 1);
  float determinant = getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 1, 1) * getIndexMat3(model->Suu, 2, 2) +
                      getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 1, 2) * getIndexMat3(model->Suu, 2, 0) +
                      getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 1, 0) * getIndexMat3(model->Suu, 2, 1) -
                      getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 1, 2) * getIndexMat3(model->Suu, 2, 1) -
                      getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 1, 0) * getIndexMat3(model->Suu, 2, 2) -
                      getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 1, 1) * getIndexMat3(model->Suu, 2, 0);
  // check the rank of S_uu to see if it's equal to 2 (invertible matrix)
  if (fabs(determinant) > 0.01) // greater than zero
  {
    model->k = 1;
    model->j = model->j + 1;
    // setIndexMat3(&(model->SuuInverse), 0, 0, getIndexMat3(model->Suu, 1, 1) / determinant);
    // setIndexMat3(&(model->SuuInverse), 0, 1, -getIndexMat3(model->Suu, 0, 1) / determinant);
    // setIndexMat3(&(model->SuuInverse), 1, 0, -getIndexMat3(model->Suu, 1, 0) / determinant);
    // setIndexMat3(&(model->SuuInverse), 1, 1, getIndexMat3(model->Suu, 0, 0) / determinant);
    Mat3 cofactor;
    setIndexMat3(&cofactor, 0, 0, getIndexMat3(model->Suu, 1, 1) * getIndexMat3(model->Suu, 2, 2) - getIndexMat3(model->Suu, 1, 2) * getIndexMat3(model->Suu, 2, 1));
    setIndexMat3(&cofactor, 0, 1, -(getIndexMat3(model->Suu, 1, 0) * getIndexMat3(model->Suu, 2, 2) - getIndexMat3(model->Suu, 1, 2) * getIndexMat3(model->Suu, 2, 0)));
    setIndexMat3(&cofactor, 0, 2, getIndexMat3(model->Suu, 1, 0) * getIndexMat3(model->Suu, 2, 1) - getIndexMat3(model->Suu, 1, 1) * getIndexMat3(model->Suu, 2, 0));
    setIndexMat3(&cofactor, 1, 0, -(getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 2, 2) - getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 2, 1)));
    setIndexMat3(&cofactor, 1, 1, getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 2, 2) - getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 2, 0));
    setIndexMat3(&cofactor, 1, 2, -(getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 2, 1) - getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 2, 0)));
    setIndexMat3(&cofactor, 2, 0, getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 1, 2) - getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 1, 1));
    setIndexMat3(&cofactor, 2, 1, -(getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 1, 2) - getIndexMat3(model->Suu, 0, 2) * getIndexMat3(model->Suu, 1, 0)));
    setIndexMat3(&cofactor, 2, 2, getIndexMat3(model->Suu, 0, 0) * getIndexMat3(model->Suu, 1, 1) - getIndexMat3(model->Suu, 0, 1) * getIndexMat3(model->Suu, 1, 0));
    // transpose of the cofactor matrix
    for (int i = 0; i < model->m; i++)
    {
      for (int j = 0; j < model->m; j++)
      {
        setIndexMat3(&(model->SuuInverse), i, j, getIndexMat3(cofactor, j, i) / determinant);
      }
    }
    // initialize the gain matrix
    for (int i = 0; i < model->m; i++)
    {
      for (int j = 0; j < model->n; j++)
      {
        setIndexMat312(&(model->K_j), i, j, 0.0);
      }
    }
    for (int i = 0; i < model->m; i++)
    {
      for (int j = 0; j < model->n; j++)
      {
        for (int k = 0; k < model->m; k++)
        {
          setIndexMat312(&(model->K_j), i, j, getIndexMat312(model->K_j, i, j) + getIndexMat3(model->SuuInverse, i, k) * getIndexMat312(model->Sux, k, j));
        }
      }
    }
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
  unsigned long elapsedTime1 = 0;
  unsigned long elapsedTime2 = 0;
  unsigned long elapsedTime = 0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, RECEIVE_FRAME_LENGTH);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_Delay(10);

  // initialize the Encoder and IMU
  HAL_Delay(10);
  updateSensors(&model);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    elapsedTime1 = DWT->CYCCNT;

    // if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0)
    // {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      model.active = 1;
    }
    // }
    // else
    // {
    // model.active = 0;
    // resetActuators(&model);
    // }

    if (fabs(model.imu2.roll) > model.rollSafetyAngle || fabs(model.imu2.pitch) > model.pitchSafetyAngle || model.j > model.maxEpisodeLength)
    {
      model.outOfBoundsCounter = model.outOfBoundsCounter + 1;
    }
    else
    {
      model.outOfBoundsCounter = fmax(0, model.outOfBoundsCounter - 1);
    }

    if (model.outOfBoundsCounter > model.maxOutOfBounds)
    {
      model.active = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (model.active == 1)
    {
      t1 = DWT->CYCCNT;
      updateSensors(&model);
      computeFeedbackPolicy(&model);
      applyFeedbackPolicy(&model);
      stepForward(&model);
      if (model.convergenceCounter >= model.convergenceMaxCount)
      {
        updateControlPolicy(&model);
      }
      model.logCounter = model.logCounter + 1;
      t2 = DWT->CYCCNT;
      diff = t2 - t1;
      model.dt = (float)diff / model.CPUClock;
    }
    else
    {
      t1 = DWT->CYCCNT;
      resetActuators(&model);
      updateSensors(&model);
      computeFeedbackPolicy(&model);
      model.logCounter = model.logCounter + 1;
      t2 = DWT->CYCCNT;
      diff = t2 - t1;
      model.dt = (float)diff / model.CPUClock;
    }

    // if (model.logCounter > model.logPeriod && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
    if (model.logCounter > model.logPeriod)
    {
      transmit = 1;
    }
    if (transmit == 1)
    {
      t1 = DWT->CYCCNT;
      transmit = 0;
      model.logCounter = 0;

      sprintf(MSG,
              "changes: %0.2f, x: %0.2f, y: %0.2f, z: %0.2f, active: %0.1f | AX2: %0.2f, AY2: %0.2f, AZ2: %0.2f, | roll: %0.2f, pitch: %0.2f, yaw: %0.2f, | encT: %0.2f, encB: %0.2f, encS: %0.2f, | j: %0.1f, k: %0.1f, | P0: %0.2f, P1: %0.2f, P2: %0.2f, P3: %0.2f, P4: %0.2f, P5: %0.2f, P6: %0.2f, P7: %0.2f, P8: %0.2f, P9: %0.2f, P10: %0.2f, P11: %0.2f, P12: %0.2f, P13: %0.2f, P14: %0.2f, time: %0.2f, dt: %0.6f\r\n",
              model.changes, getIndexVec3(model.position, 0), getIndexVec3(model.position, 1), getIndexVec3(model.position, 2), (float)model.active, model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.imu2.roll, model.imu2.pitch, model.imu2.yaw, model.reactionEncoder.radianAngle, model.rollingEncoder.radianAngle, model.servoAngle, (float)model.j, (float)model.k, getIndexMat15(model.P_n, 0, 0), getIndexMat15(model.P_n, 1, 1), getIndexMat15(model.P_n, 2, 2), getIndexMat15(model.P_n, 3, 3), getIndexMat15(model.P_n, 4, 4), getIndexMat15(model.P_n, 5, 5), getIndexMat15(model.P_n, 6, 6), getIndexMat15(model.P_n, 7, 7), getIndexMat15(model.P_n, 8, 8), getIndexMat15(model.P_n, 9, 9), getIndexMat15(model.P_n, 10, 10), getIndexMat15(model.P_n, 11, 11), getIndexMat15(model.P_n, 12, 12), getIndexMat15(model.P_n, 13, 13), getIndexMat15(model.P_n, 14, 14), model.time, model.dt);

      // sprintf(MSG,
      //         "active: %0.1f, changes: %0.2f, AX1: %0.2f, AY1: %0.2f, AZ1: %0.2f, | AX2: %0.2f, AY2: %0.2f, AZ2: %0.2f, | roll: %0.2f, pitch: %0.2f, | encT: %0.2f, encB: %0.2f, | j: %0.1f, k: %0.1f, | P0: %0.2f, P1: %0.2f, P2: %0.2f, P3: %0.2f, P4: %0.2f, P5: %0.2f, P6: %0.2f, P7: %0.2f, P8: %0.2f, P9: %0.2f, P10: %0.2f, P11: %0.2f, time: %0.2f, dt: %0.6f\r\n",
      //         (float)model.active, model.changes, model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.imu1.roll, model.imu1.pitch, model.reactionEncoder.radianAngle, model.rollingEncoder.radianAngle, (float)model.j, (float)model.k, getIndexMat15(model.P_n, 0, 0), getIndexMat15(model.P_n, 1, 1), getIndexMat15(model.P_n, 2, 2), getIndexMat15(model.P_n, 3, 3), getIndexMat15(model.P_n, 4, 4), getIndexMat15(model.P_n, 5, 5), getIndexMat15(model.P_n, 6, 6), getIndexMat15(model.P_n, 7, 7), getIndexMat15(model.P_n, 8, 8), getIndexMat15(model.P_n, 9, 9), getIndexMat15(model.P_n, 10, 10), getIndexMat15(model.P_n, 11, 11), model.time, model.dt);

      // sprintf(MSG,
      //         "x0: %0.2f, x1: %0.2f, x2: %0.2f, x3: %0.2f, x4: %0.2f, x5: %0.2f, x6: %0.2f, x7: %0.2f, x8: %0.2f, x9: %0.2f, dt: %0.6f\r\n",
      //         model.dataset.x0, model.dataset.x1, model.dataset.x2, model.dataset.x3, model.dataset.x4, model.dataset.x5, model.dataset.x6, model.dataset.x7, model.dataset.x8, model.dataset.x9, dt);

      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | P11: %0.2f, P22: %0.2f, P33: %0.2f, P44: %0.2f, P55: %0.2f, P66: %0.2f, P77: %0.2f, P88: %0.2f, P99: %0.2f, P1010: %0.2f, P1111: %0.2f, P1212: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, P_n[0][0], P_n[0][1], P_n[0][2], P_n[0][3], P_n[0][4], P_n[0][5], P_n[0][6], P_n[0][7], P_n[0][8], P_n[0][9], P_n[0][10], P_n[0][11], dt);
      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | x_n_dot_z_n: %0.2f, P00: %0.2f, P_n11: %0.2f, P_n22: %0.2f, P_n33: %0.2f, P_n44: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, x_n_dot_z_n, getIndexMat15(model.W_n, 0, 0), getIndexMat15(model.W_n, 1, 1), getIndexMat15(model.W_n, 2, 2), getIndexMat15(model.W_n, 3, 3), getIndexMat15(model.W_n, 4, 4), dt);

      // sprintf(MSG,
      //         "roll: %0.2f, pitch: %0.2f, | x_n_dot_z_n: %0.2f, z_n0: %0.2f, z_n1: %0.2f, z_n2: %0.2f, z_n3: %0.2f, z_n4: %0.2f, z_n5: %0.2f, z_n6: %0.2f, z_n7: %0.2f, z_n8: %0.2f, z_n9: %0.2f, z_n10: %0.2f, z_n11: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.roll, model.imu1.pitch, x_n_dot_z_n, z_n[0], z_n[1], z_n[2], z_n[3], z_n[4], z_n[5], z_n[6], z_n[7], z_n[8], z_n[9], z_n[10], z_n[11], dt);

      // sprintf(MSG,
      // "yaw: %0.2f, roll: %0.2f, rollv: %0.2f, pitch: %0.2f, pitchv: %0.2f, | aX1: %0.2f, aY1: %0.2f, aZ1: %0.2f, | aX2: %0.2f, aY2: %0.2f, aZ2: %0.2f, | encB: %d, encT: %d, dt: %0.6f\r\n",
      // model.imu1.yaw, model.imu1.roll, model.imu1.roll_velocity, model.imu1.pitch, model.imu1.pitch_velocity, model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, TIM3->CNT, TIM4->CNT, dt);
      // sprintf(MSG, "Bottom: current: %d, curVel: %0.2f, enc: %d, angle: %0.2f, velocity: %0.2f, acceleration: %0.2f, | Top: current: %d, curvel: %0.2f, enc: %d, angle: %0.2f, velocity: %0.2f, acceleration: %0.2f, dt: %0.6f\r\n",
      //   model.rollingCurrentSensor.current0, model.rollingCurrentSensor.currentVelocity, TIM4->CNT, model.rollingEncoder.angle, model.rollingEncoder.velocity, model.rollingEncoder.acceleration,
      //   model.reactionCurrentSensor.current0, model.reactionCurrentSensor.currentVelocity, TIM3->CNT, model.reactionEncoder.angle, model.reactionEncoder.velocity, model.reactionEncoder.acceleration, dt);

      // sprintf(MSG, "rawAx1: %d, rawAy1: %d, rawAz1: %d, | rawAx2: %d, rawAy2: %d, rawAz2: %d, dt: %0.6f\r\n",
      //         model.imu1.rawAccX, model.imu1.rawAccY, model.imu1.rawAccZ, model.imu2.rawAccX, model.imu2.rawAccY, model.imu2.rawAccZ, model.dt);
      // sprintf(MSG, "ax1: %0.2f, ay1: %0.2f, az1: %0.2f, | ax2: %0.2f, ay2: %0.2f, az2: %0.2f, dt: %0.6f\r\n",
      //         model.imu1.accX, model.imu1.accY, model.imu1.accZ, model.imu2.accX, model.imu2.accY, model.imu2.accZ, model.dt);

      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);
      t2 = DWT->CYCCNT;
      diff = t2 - t1;
      model.dt += (float)diff / model.CPUClock;
    }

    // Rinse and repeat :)
    elapsedTime2 = DWT->CYCCNT;
    elapsedTime = elapsedTime2 - elapsedTime1;
    model.time += (float)elapsedTime / model.CPUClock;
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
  hadc1.Init.NbrOfConversion = 3;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
