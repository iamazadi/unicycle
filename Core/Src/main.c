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
#define MPU6050_ADDR 0xD0
#define MPU6050_ADDR2 0xD2
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define TRANSMIT_LENGTH 220
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
  float Ax_raw;
  float Ay_raw;
  float Az_raw;
  float Gx_raw;
  float Gy_raw;
  float Gz_raw;
  float Ax;
  float Ay;
  float Az;
  float Gx;
  float Gy;
  float Gz;
  float _Gx;
  float _Gy;
  float _Gz;
  float Ax_scale;
  float Ay_scale;
  float Az_scale;
  float Gx_scale;
  float Gy_scale;
  float Gz_scale;
  float Ax_offset;
  float Ay_offset;
  float Az_offset;
  float Gx_offset;
  float Gy_offset;
  float Gz_offset;
  float phi;
  float theta;
  float psi;
} Imu;

void initialize(Imu *imu) {
  imu->_Gx = 0.0;
  imu->_Gy = 0.0;
  imu->_Gz = 0.0;
  imu->Ax_scale = 1.0;
  imu->Ay_scale = 1.0;
  imu->Az_scale = 1.0;
  imu->Gx_scale = 1.0;
  imu->Gy_scale = 1.0;
  imu->Gz_scale = 1.0;
  imu->Ax_offset = 0.0;
  imu->Ay_offset = 0.0;
  imu->Az_offset = 0.0;
  imu->Gx_offset = 0.0;
  imu->Gy_offset = 0.0;
  imu->Gz_offset = 0.0;
}

int MPU6050_Init (I2C_HandleTypeDef hi2c, int address)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c, address, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c, address, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c, address, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c, address, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c, address, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    return 1;
	} else {
    return 0;
  }
}

void MPU6050_Read_Accel (I2C_HandleTypeDef hi2c, int address, Imu *imu)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, address, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	imu->Ax = imu->Ax_raw / 16384.0;
	imu->Ay = imu->Ay_raw / 16384.0;
	imu->Az = imu->Az_raw / 16384.0;
}

void MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c, int address, Imu *imu)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, address, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu->Gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu->Gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu->Gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

  imu->_Gx = imu->Gx;
  imu->_Gy = imu->Gy;
  imu->_Gz = imu->Gz;
	imu->Gx = imu->Gx_raw / 131.0;
	imu->Gy = imu->Gy_raw / 131.0;
	imu->Gz = imu->Gz_raw / 131.0;
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
  unsigned long t1 = 0;
  unsigned long t2 = 0;
  unsigned long diff = 0;
  // sampling time
  float dt = 0.0;
  const float CPU_CLOCK = 84000000.0;
  const float rolling_wheel_pulse = 17.0;
  const float reaction_wheel_pulse = 26.0;
  int rolling_wheel_previous_count = 0;
  int rolling_wheel_count = 0;
  float rolling_wheel_frequency = 0;
  float rolling_wheel_velocity = 0.0;
  int reaction_wheel_previous_count = 0;
  int reaction_wheel_count = 0;
  int reaction_wheel_frequency = 0;
  float reaction_wheel_acceleration = 0.0;
  float reaction_wheel_velocity = 0.0;
  int initialization_delay = 10;
  int sensor_delay = 1;
  Imu imu1, imu2, imu3, imu4;
  initialize(&imu1);
  initialize(&imu2);
  initialize(&imu3);
  initialize(&imu4);
  // Accelerometers calibration (removing bias)
  imu1.Ax_offset = -0.02;
  imu1.Ay_offset = 0.0;
  imu1.Az_offset = 0.13;
  imu2.Ax_offset = 0.0;
  imu2.Ay_offset = -0.01;
  imu2.Az_offset = 0.08;
  imu3.Ax_offset = -0.06;
  imu3.Ay_offset = 0.0;
  imu3.Az_offset = -0.11;
  imu4.Ax_offset = 0.0;
  imu4.Ay_offset = 0.0;
  imu4.Az_offset = -0.09;
  // Gyroscopes calibration
  imu1.Gx_offset = 5.5;
  imu1.Gy_offset = -1.3;
  imu1.Gz_offset = 0.0;
  imu2.Gx_offset = 0.3;
  imu2.Gy_offset = -1.0;
  imu2.Gz_offset = -0.7;
  imu3.Gx_offset = 7.0;
  imu3.Gy_offset = -0.1;
  imu3.Gz_offset = 0.01;
  imu4.Gx_offset = 2.0;
  imu4.Gy_offset = 1.1;
  imu4.Gz_offset = 0.6;
  int init1, init2, init3, init4;
  int transmit = 0;
  int log_counter = 0;
  const int LOG_CYCLE = 135;

  // the pivot point B̂ in the inertial frame Ô
  float pivot[3] = {-0.097, -0.1, -0.032};
  // the position of sensors mounted on the body in the body frame of reference
  float p1[3] = {-0.035, -0.19, -0.04};
  float p2[3] = {0.025, -0.144, -0.07};
  float p3[3] = {-0.11, -0.01, 0.13};
  float p4[3] = {-0.11, -0.19, 0.13};
  // the vectors of the standard basis for the input space ℝ³
  float e1[3] = {1.0, 0.0, 0.0};
  float e2[3] = {0.0, 1.0, 0.0};
  float e3[3] = {0.0, 0.0, 1.0};
  // The rotation of the inertial frame Ô to the body frame B̂
  float O_B_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  float B_O_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  // The rotation of the local frame of the sensor i to the robot frame B̂
  float A1_B_R[3][3] = {{0.0, -1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}; // [-ê[2] -ê[1] -ê[3]]
  float A2_B_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // [ê[1] ê[2] ê[3]]
  float A3_B_R[3][3] = {{0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0}}; // [-ê[3] ê[1] -ê[2]]
  float A4_B_R[3][3] = {{0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}}; // [-ê[3] -ê[1] ê[2]]
  float B_A1_R[3][3] = {{0.0, -1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}; // LinearAlgebra.inv(A1_B_R)
  float B_A2_R[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // LinearAlgebra.inv(A2_B_R)
  float B_A3_R[3][3] = {{0.0, 0.0, -1.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}}; // LinearAlgebra.inv(A3_B_R)
  float B_A4_R[3][3] = {{0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}; // LinearAlgebra.inv(A4_B_R)

  // The matrix of unknown parameters
  float Q[3][4] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
  // The matrix of sensor locations (known parameters)
  float P[4][4] = {{1.0, 1.0, 1.0, 1.0}, {0.062, 0.122, -0.013, -0.013}, {-0.09, -0.044, 0.09, -0.09}, {-0.008, -0.038, 0.162, 0.162}}; // [[1.0; vec(p1 - pivot)] [1.0; vec(p2 - pivot)] [1.0; vec(p3 - pivot)] [1.0; vec(p4 - pivot)]]
  // The optimal fusion matrix
  float X[4][4] = {{2.4239, -25.1572, 0.0, -16.9811}, {-1.25031, 21.3836, 0.0, 9.43396}, {0.819525, -5.46471, 5.55556, -2.4109}, {-0.99311, 9.2383, -5.55556, 9.95807}}; // transpose(P) * LinearAlgebra.inv(P * transpose(P))

  // accelerometer sensor measurements in the local frame of the sensors
  float R1[3] = {0.0, 0.0, 0.0};
  float R2[3] = {0.0, 0.0, 0.0};
  float R3[3] = {0.0, 0.0, 0.0};
  float R4[3] = {0.0, 0.0, 0.0};
  // accelerometer sensor measurements in the robot body frame
  float _R1[3] = {0.0, 0.0, 0.0};
  float _R2[3] = {0.0, 0.0, 0.0};
  float _R3[3] = {0.0, 0.0, 0.0};
  float _R4[3] = {0.0, 0.0, 0.0};
  // all sensor measurements combined
  float M[3][4] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
  // The gravity vector
  float g[3] = {0.0, 0.0, 0.0};
  // y-Euler angle (pitch)
  float beta = 0.0;
  float fused_beta = 0.0;
  // x-Euler angle (roll)
  float gamma = 0.0;
  float fused_gamma = 0.0;
  // tuning parameters to minimize estimate variance
  float kappa1 = 0.01;
  float kappa2 = 0.01;
  // the average of the body angular rate from rate gyro
  float r[3] = {0.0, 0.0, 0.0};
  // the average of the body angular rate in Euler angles
  float r_dot[3] = {0.0, 0.0, 0.0};
  // gyro sensor measurements in the local frame of the sensors
  float G1[3] = {0.0, 0.0, 0.0};
  float G2[3] = {0.0, 0.0, 0.0};
  float G3[3] = {0.0, 0.0, 0.0};
  float G4[3] = {0.0, 0.0, 0.0};
  // gyro sensor measurements in the robot body frame
  float _G1[3] = {0.0, 0.0, 0.0};
  float _G2[3] = {0.0, 0.0, 0.0};
  float _G3[3] = {0.0, 0.0, 0.0};
  float _G4[3] = {0.0, 0.0, 0.0};
  // a matrix transfom from body rates to Euler angular rates
  float E[3][3] = {{0.0, sin(gamma) / cos(beta), cos(gamma) / cos(beta)},
                   {0.0, cos(gamma), -sin(gamma)},
                   {1.0, sin(gamma) * tan(beta), cos(gamma) * tan(beta)}};

  int initialized = 0;
  float rolling_wheel_speed = 0.0;
  float reaction_wheel_speed = 0.0;
  float safety_angle = 10.0 / 180.0 * 3.14;
  float pitch_average = 0.0;
  float roll_average = 0.0;
  float pitch = 0.0;
  float roll = 0.0;
  float gyro_pitch = 0.0;
  float gyro_roll = 0.0;
  float yaw = 0.0;
  float reaction_wheel_ki = 1000.0;
  float k1 = 3000.0;
  float k2 = 10000.0;
  float k3 = 2500.0;
  float k4 = 2000.0;
  float k5 = 10000.0;
  float kp = 6000.0;
  float ki = 50.0;
  float kd = 30.0;
  float smooth = 1.0;
  float windup = 32.0;
  float reaction_wheel_windup = 64.0;
  float rolling_wheel_integrator = 0.0;
  float reaction_wheel_integrator = 0.0;
  float pitch_velocity = 0.0;
  float roll_velocity = 0.0;
  float roll_acceleration = 0.0;
  float q1 = 0.0; // roll angle
  float q2 = 0.0; // pitch angle
  float q3 = 0.0; // yaw angle
  float q4 = 0.0; // rolling wheel angle
  float q5 = 0.0; // reaction wheel angle
  float q1_dotdot = 0.0;
  float q1_dot = 0.0;
  float q3_dot = 0.0;
  float q3_dotdot = 0.0;
  float r_w = 75.0; // +-0.1mm
  float B_g[3] = {0.0, 0.0, 0.0}; // gravitational acceleration
  float O_pwc[3] = {0.0, 0.0, 0.0}; // acceleration from contact point to the rolling wheel center in inertial frame O
  float B_pwc[3] = {0.0, 0.0, 0.0}; // acceleration from contact point to the rolling wheel center in the body frame B
  // the rotation matrix that takes vectors from the inertial frame to the body frame through a matrix-vector multiplication
  float R_x_T_R_y_T[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  // full state estmation with encoders
  float encoder_beta = 0.0;
  float encoder_gamma = 0.0;
  float pitch_target_angle = 1.5 / 180.0 * 3.14;
  float roll_target_angle = -4.0 / 180.0 * 3.14;
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
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(3000);
  HAL_Delay(initialization_delay);
  init1 = MPU6050_Init(hi2c1, MPU6050_ADDR);
  HAL_Delay(initialization_delay);
  init2 = MPU6050_Init(hi2c1, MPU6050_ADDR2);
  HAL_Delay(initialization_delay);
  init3 = MPU6050_Init(hi2c3, MPU6050_ADDR);
  HAL_Delay(initialization_delay);
  init4 = MPU6050_Init(hi2c3, MPU6050_ADDR2);
  HAL_Delay(initialization_delay);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    t1 = DWT->CYCCNT;

    MPU6050_Read_Accel(hi2c1, MPU6050_ADDR, &imu1);
    MPU6050_Read_Gyro(hi2c1, MPU6050_ADDR, &imu1);
    // HAL_Delay(sensor_delay);
    MPU6050_Read_Accel(hi2c1, MPU6050_ADDR2, &imu2);
    MPU6050_Read_Gyro(hi2c1, MPU6050_ADDR2, &imu2);
    // HAL_Delay(sensor_delay);
    MPU6050_Read_Accel(hi2c3, MPU6050_ADDR, &imu3);
    MPU6050_Read_Gyro(hi2c3, MPU6050_ADDR, &imu3);
    MPU6050_Read_Accel(hi2c3, MPU6050_ADDR2, &imu4);
    MPU6050_Read_Gyro(hi2c3, MPU6050_ADDR2, &imu4);

    R1[0] = imu1.Ax + imu1.Ax_offset;
    R1[1] = imu1.Ay + imu1.Ay_offset;
    R1[2] = imu1.Az + imu1.Az_offset;
    R2[0] = imu2.Ax + imu2.Ax_offset;
    R2[1] = imu2.Ay + imu2.Ay_offset;
    R2[2] = imu2.Az + imu2.Az_offset;
    R3[0] = imu3.Ax + imu3.Ax_offset;
    R3[1] = imu3.Ay + imu3.Ay_offset;
    R3[2] = imu3.Az + imu3.Az_offset;
    R4[0] = imu4.Ax + imu4.Ax_offset;
    R4[1] = imu4.Ay + imu4.Ay_offset;
    R4[2] = imu4.Az + imu4.Az_offset;

    _R1[0] = 0.0;
    _R1[1] = 0.0;
    _R1[2] = 0.0;
    _R2[0] = 0.0;
    _R2[1] = 0.0;
    _R2[2] = 0.0;
    _R3[0] = 0.0;
    _R3[1] = 0.0;
    _R3[2] = 0.0;
    _R4[0] = 0.0;
    _R4[1] = 0.0;
    _R4[2] = 0.0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        _R1[i] += B_A1_R[i][j] * R1[j];
        _R2[i] += B_A2_R[i][j] * R2[j];
        _R3[i] += B_A3_R[i][j] * R3[j];
        _R4[i] += B_A4_R[i][j] * R4[j];
      }
    }

    for (int i = 0; i < 3; i++) {
      M[i][0] = _R1[i];
      M[i][1] = _R2[i];
      M[i][2] = _R3[i];
      M[i][3] = _R4[i];
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        Q[i][j] = 0.0;
        for (int k = 0; k < 4; k++) {
          Q[i][j] += M[i][k] * X[k][j];
        }
      }
    }
    g[0] = Q[0][0];
    g[1] = Q[1][0];
    g[2] = Q[2][0];
    beta = atan2(-g[0], sqrt(pow(g[1], 2) + pow(g[2], 2)));
    gamma = atan2(g[1], g[2]);

    G1[0] = (imu1.Gx + imu1.Gx_offset) / 180.0 * 3.14;
    G1[1] = (imu1.Gy + imu1.Gy_offset) / 180.0 * 3.14;
    G1[2] = (imu1.Gz + imu1.Gz_offset) / 180.0 * 3.14;
    G2[0] = (imu2.Gx + imu2.Gx_offset) / 180.0 * 3.14;
    G2[1] = (imu2.Gy + imu2.Gy_offset) / 180.0 * 3.14;
    G2[2] = (imu2.Gz + imu2.Gz_offset) / 180.0 * 3.14;
    G3[0] = (imu3.Gx + imu3.Gx_offset) / 180.0 * 3.14;
    G3[1] = (imu3.Gy + imu3.Gy_offset) / 180.0 * 3.14;
    G3[2] = (imu3.Gz + imu3.Gz_offset) / 180.0 * 3.14;
    G4[0] = (imu4.Gx + imu4.Gx_offset) / 180.0 * 3.14;
    G4[1] = (imu4.Gy + imu4.Gy_offset) / 180.0 * 3.14;
    G4[2] = (imu4.Gz + imu4.Gz_offset) / 180.0 * 3.14;

    _G1[0] = 0.0;
    _G1[1] = 0.0;
    _G1[2] = 0.0;
    _G2[0] = 0.0;
    _G2[1] = 0.0;
    _G2[2] = 0.0;
    _G3[0] = 0.0;
    _G3[1] = 0.0;
    _G3[2] = 0.0;
    _G4[0] = 0.0;
    _G4[1] = 0.0;
    _G4[2] = 0.0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        _G1[i] += B_A1_R[i][j] * G1[j];
        _G2[i] += B_A2_R[i][j] * G2[j];
        _G3[i] += B_A3_R[i][j] * G3[j];
        _G4[i] += B_A4_R[i][j] * G4[j];
      }
    }
    for (int i = 0; i < 3; i++) {
      r[i] = (_G1[i] + _G2[i] + _G3[i] + _G4[i]) / 4.0;
    }

    E[0][0] = 0.0;
    E[0][1] = sin(fused_gamma) / cos(fused_beta);
    E[0][2] = cos(fused_gamma) / cos(fused_beta);
    E[1][0] = 0.0;
    E[1][1] = cos(fused_gamma);
    E[1][2] = -sin(fused_gamma);
    E[2][0] = 1.0;
    E[2][1] = sin(fused_gamma) * tan(fused_beta);
    E[2][2] = cos(fused_gamma) * tan(fused_beta);

    r_dot[0] = 0.0;
    r_dot[1] = 0.0;
    r_dot[2] = 0.0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        r_dot[i] += E[i][j] * r[j];
      }
    }

    if (initialized == 1) {
      gyro_pitch += dt * (0.8 * r_dot[1] + 0.2 * pitch_velocity);
      gyro_roll += dt * (0.8 * r_dot[2] + 0.2 * roll_velocity);
    } else {
      gyro_pitch = beta;
      gyro_roll = gamma;
      initialized = 1;
    }
    
    fused_beta = kappa1 * beta + (1.0 - kappa1) * gyro_pitch;
    fused_gamma = kappa2 * gamma + (1.0 - kappa2) * gyro_roll;
    // fused_beta = kappa1 * beta + (1.0 - kappa1) * (fused_beta + dt * r_dot[1]);
    // fused_gamma = kappa2 * gamma + (1.0 - kappa2) * (fused_gamma + dt * r_dot[2]);

    roll_acceleration = (fused_gamma - roll) - roll_velocity;
    roll_velocity = fused_gamma - roll;
    roll = fused_gamma;
    pitch_velocity = fused_beta - pitch;
    pitch = fused_beta;

    pitch_average = 0.99 * pitch_average + 0.01 * pitch;
    roll_average = 0.99 * roll_average + 0.01 * roll;

    rolling_wheel_previous_count = rolling_wheel_count;
    reaction_wheel_previous_count = reaction_wheel_count;
    rolling_wheel_count = (TIM2->CNT);
    reaction_wheel_count = (TIM3->CNT);
    
    rolling_wheel_frequency = (float) (rolling_wheel_count - rolling_wheel_previous_count) / 1750.0 * 2.0 * 3.14 * dt;
    reaction_wheel_acceleration = rolling_wheel_frequency - reaction_wheel_velocity;
    reaction_wheel_velocity = rolling_wheel_frequency ; // in radian per second

    // q1_dotdot = (roll - q1) - q1_dot;
    // q1_dot = roll - q1;
    // q1 = atan2(g[1], sqrt(pow(g[0], 2) + pow(g[2], 2)));
    // q2 = atan2(-g[1], g[2]);
    // q3_dotdot = (yaw - q3) - q3_dot;
    // q3_dot = yaw - q3;
    // q3 = dt * r_dot[0];
    // q4 = (float)rolling_wheel_count / 300.0 * 2.0 * 3.14;
    // q5 = (float)reaction_wheel_count / 1800.0 * 2.0 * 3.14;

    // B_g[0] = -cos(q1) * sin(q2);
    // B_g[1] = sin(q1);
    // B_g[2] = cos(q1) * cos(q2);

    // O_pwc[0] = 2.0 * r_w * cos(q1) * q1_dot * q3_dot + r_w * sin(q1) * q3_dotdot;
    // O_pwc[1] = r_w * sin(q1) * (pow(q1_dot, 2) + pow(q3_dot, 2)) - r_w * cos(q1) * q1_dotdot;
    // O_pwc[2] = -r_w * cos(q1) * pow(q1_dot, 2) - r_w * sin(q1) * q1_dotdot;
    // R_x_T_R_y_T[0][0] = cos(beta);
    // R_x_T_R_y_T[0][1] = 0.0;
    // R_x_T_R_y_T[0][2] = -sin(beta);
    // R_x_T_R_y_T[1][0] = sin(beta) * sin(gamma);
    // R_x_T_R_y_T[1][1] = cos(gamma);
    // R_x_T_R_y_T[1][2] = sin(gamma) * cos(beta);
    // R_x_T_R_y_T[2][0] = cos(gamma) * sin(beta);
    // R_x_T_R_y_T[2][1] = -sin(gamma);
    // R_x_T_R_y_T[2][2] = cos(gamma) * cos(beta);
    // B_pwc[0] = 0.0;
    // B_pwc[1] = 0.0;
    // B_pwc[2] = 0.0;
    // for (int i = 0; i < 3; i++) {
    //   for (int j = 0; j < 3; j++) {
    //     B_pwc[i] += R_x_T_R_y_T[i][j] * O_pwc[j];
    //   }
    // }

    // encoder_beta = atan2(-B_pwc[0], sqrt(pow(B_pwc[1], 2) + pow(B_pwc[2], 2)));
    // encoder_gamma = atan2(B_pwc[1], B_pwc[2]);


    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }

    reaction_wheel_integrator = reaction_wheel_integrator + (roll - roll_target_angle) * reaction_wheel_ki;
    if (reaction_wheel_integrator > reaction_wheel_windup) {
      reaction_wheel_integrator = reaction_wheel_windup;
    }
    if (reaction_wheel_integrator < -reaction_wheel_windup) {
      reaction_wheel_integrator = -reaction_wheel_windup;
    }

    reaction_wheel_speed = (int)(smooth * (fabs(k1 * (roll - roll_target_angle) + k2 * roll_velocity + k3 * roll_acceleration + reaction_wheel_integrator + k4 * fabs(reaction_wheel_velocity) + k5 * fabs(reaction_wheel_acceleration))) + (1.0 - smooth) * (float)reaction_wheel_speed);
    if (reaction_wheel_speed > 255) {
      reaction_wheel_speed = 255;
    }


    rolling_wheel_integrator = rolling_wheel_integrator + (pitch - pitch_target_angle) * ki;
    if (rolling_wheel_integrator > windup) {
      rolling_wheel_integrator = windup;
    }
    if (rolling_wheel_integrator < -windup) {
      rolling_wheel_integrator = -windup;
    }

    rolling_wheel_speed = (int)(smooth * fabs(kp * (pitch - pitch_target_angle) + kd * pitch_velocity + rolling_wheel_integrator) + (1.0 - smooth) * (float)rolling_wheel_speed); //PID function
    if (rolling_wheel_speed > 255) {
      rolling_wheel_speed = 255;
    }
    if ((pitch_average > safety_angle) || (pitch_average < -safety_angle) || (roll_average > safety_angle) || (roll_average < -safety_angle)) {
      rolling_wheel_speed = 0; // the controller is active in -10~+10 deg range
      reaction_wheel_speed = 0;
      initialized = 0;
    }
    if (pitch > pitch_target_angle) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 255 * rolling_wheel_speed);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255 * rolling_wheel_speed);
    }
    if (roll > roll_target_angle) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 255 * reaction_wheel_speed);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 255 * reaction_wheel_speed);
    }

    // Wait for 1 ms
    // HAL_Delay(1);
    
    
    log_counter++;
    if (log_counter > LOG_CYCLE) {
      transmit = 1;
    }
    if (transmit == 1) {
      transmit = 0;
      log_counter = 0;
      // Toggle the LED
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      // use these log templates for software feature design
      // sprintf(MSG, "beta: %0.2f, gamma: %0.2f, k1*roll: %0.2f, k2*droll: %0.2f, k3*ddroll: %0.2f, k4*f2: %0.2f, speed: %0.2f, init: %d%d%d%d, dt: %0.6f\r\n",
      //         fused_beta, fused_gamma, k1 * roll, k2 * roll_velocity, k3 * roll_acceleration, k4 * fabs((float)reaction_wheel_frequency),
      //         fabs(k1 * roll + k2 * roll_velocity + k3 * roll_acceleration + k4 * fabs((float)reaction_wheel_frequency)), init1, init2, init3, init4, dt);
      // sprintf(MSG, "beta: %0.2f, gamma: %0.2f, rolv: %0.2f, reav: %0.2f, f1: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         fused_beta, fused_gamma, rolling_wheel_speed, reaction_wheel_speed, reaction_wheel_frequency, init1, init2, init3, init4, dt);
      sprintf(MSG, "beta: %0.2f, gamma: %0.2f, A1: %0.2f, %0.2f, %0.2f, A2: %0.2f, %0.2f, %0.2f, A3: %0.2f, %0.2f, %0.2f, A4: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
              fused_beta, fused_gamma, imu1.Ax, imu1.Ay, imu1.Az, imu2.Ax, imu2.Ay, imu2.Az, imu3.Ax, imu3.Ay, imu3.Az, imu4.Ax, imu4.Ay, imu4.Az,
              rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      // sprintf(MSG, "Pitch: %0.2f, Roll: %0.2f, R1: %0.2f, %0.2f, %0.2f, R2: %0.2f, %0.2f, %0.2f, R3: %0.2f, %0.2f, %0.2f, R4: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         beta, gamma, R1[0], R1[1], R1[2], R2[0], R2[1], R2[2], R3[0], R3[1], R3[2], R4[0], R4[1], R4[2],
      //         rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      // sprintf(MSG, "Pitch: %0.2f, Roll: %0.2f, G1: %0.2f, %0.2f, %0.2f, G2: %0.2f, %0.2f, %0.2f, G3: %0.2f, %0.2f, %0.2f, G4: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         beta, gamma, G1[0], G1[1], G1[2], G2[0], G2[1], G2[2], G3[0], G3[1], G3[2], G4[0], G4[1], G4[2],
      //         rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      // sprintf(MSG, "Pitch: %0.2f, Roll: %0.2f, r: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         beta, gamma, r[0], r[1], r[2],
      // sprintf(MSG, "beta: %0.2f, fused beta: %0.2f, gamma: %0.2f, fused_gamma: %0.2f, r: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         beta, fused_beta, gamma, fused_gamma, r[0], r[1], r[2],
      //         rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), 1000);
    }
    
    t2 = DWT->CYCCNT;
    diff = t2 - t1;
    dt = (float) diff / CPU_CLOCK;

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
