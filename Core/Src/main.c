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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define MPU6050_ADDR2 0xD2

// matrix dimensions so that we dont have to pass them as
// parametersmat1[R1][C1] and mat2[R2][C2]
#define R1 3 // number of rows in Matrix-1
#define C1 3 // number of columns in Matrix-1
#define R2 3 // number of rows in Matrix-2
#define C2 3 // number of columns in Matrix-2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t UART1_rxBuffer[12] = {0};
uint8_t UART1_txBuffer[12] = {0};

int samples = 500;
float dt = 0.0;
float log_dt = 0.0;
char buf[4];
uint8_t MSG[160] = {'\0'};
int speed1 = 0;
int speed2 = 0;
const float W = 0.3;
const float PI = 3.14;
const int LOG_CYCLES = 100;
const int signals = 11; // encoder motor end 11 signals
const float interval = 0.01;
const float K = 0.0175;
int previous_count = 0;
int log_counter;
int transmit_logs = 0;
float moving_average = 0.0;
float hue = 0.0;
float R_est_x = 0.0;
float R_est_y = 0.0;
float R_est_z = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
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

typedef struct {
  float R_est_x;
  float R_est_y;
  float R_est_z;
  float _R_est_x;
  float _R_est_y;
  float _R_est_z;
  float R_acc_x;
  float R_acc_y;
  float R_acc_z;
  float R_gyro_x;
  float R_gyro_y;
  float R_gyro_z;
  float magnitude;
  float phi;
  float theta;
  float psi;
} Acceleration;

typedef struct {
  float r;       // a fraction between 0 and 1
  float g;       // a fraction between 0 and 1
  float b;       // a fraction between 0 and 1
} Rgb;

typedef struct {
  float h;       // angle in degrees
  float s;       // a fraction between 0 and 1
  float v;       // a fraction between 0 and 1
} Hsv;

//static Orientation getOrientation(Orientation in, MPU9250 mpu, SimpleKalmanFilter phiFilter, SimpleKalmanFilter thetaFilter, double ratio);
static Rgb HSVtoRGB(Hsv in);

Rgb HSVtoRGB(Hsv input) {
  float H = input.h;
  float S = input.s;
  float V = input.v;
  Rgb output;
  output.r = 0;
  output.g = 0;
  output.b = 0;

  if (H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0) {
    // cout<<"The givem HSV values are not in valid range"<<endl;
    return output;
  }
  float s = S / 100;
  float v = V / 100;
  float C = s * v;
  float mod = fmod(H / 60.0, 2.0);
  // float absolute = abs(mod - 1);
  float absolute = mod - 1;
  absolute = (absolute >= 0.0) ? absolute : -absolute;
  float X = C * (1 - absolute);
  float m = v - C;
  float r, g, b;
  if (H >= 0 && H < 60) {
    r = C, g = X, b = 0;
  }
  else if (H >= 60 && H < 120) {
    r = X, g = C, b = 0;
  }
  else if (H >= 120 && H < 180) {
    r = 0, g = C, b = X;
  }
  else if (H >= 180 && H < 240) {
    r = 0, g = X, b = C;
  }
  else if (H >= 240 && H < 300) {
    r = X, g = 0, b = C;
  }
  else {
    r = C, g = 0, b = X;
  }
  output.r = (r + m) * 255;
  output.g = (g + m) * 255;
  output.b = (b + m) * 255;
  return output;
}

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

void MPU6050_Init (I2C_HandleTypeDef hi2c, int address)
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
	}

}


void MPU6050_Read_Accel (Imu *imu1, Imu *imu2, Imu *imu3)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu1->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu1->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu1->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	imu1->Ax = imu1->Ax_raw / 16384.0;
	imu1->Ay = imu1->Ay_raw / 16384.0;
	imu1->Az = imu1->Az_raw / 16384.0;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu2->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu2->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu2->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	imu2->Ax = imu2->Ax_raw / 16384.0;
	imu2->Ay = imu2->Ay_raw / 16384.0;
	imu2->Az = imu2->Az_raw / 16384.0;
        
        // Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR2, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu3->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu3->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu3->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	imu3->Ax = imu3->Ax_raw / 16384.0;
	imu3->Ay = imu3->Ay_raw / 16384.0;
	imu3->Az = imu3->Az_raw / 16384.0;
}


void MPU6050_Read_Gyro (Imu *imu1, Imu *imu2, Imu *imu3)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu1->Gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu1->Gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu1->Gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

        imu1->_Gx = imu1->Gx;
        imu1->_Gy = imu1->Gy;
        imu1->_Gz = imu1->Gz;
	imu1->Gx = imu1->Gx_raw / 131.0;
	imu1->Gy = imu1->Gy_raw / 131.0;
	imu1->Gz = imu1->Gz_raw / 131.0;
        
        // Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu2->Gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu2->Gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu2->Gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

        imu2->_Gx = imu2->Gx;
        imu2->_Gy = imu2->Gy;
        imu2->_Gz = imu2->Gz;
	imu2->Gx = imu2->Gx_raw / 131.0;
	imu2->Gy = imu2->Gy_raw / 131.0;
	imu2->Gz = imu2->Gz_raw / 131.0;
        
        // Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR2, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	imu3->Gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	imu3->Gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	imu3->Gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

        imu3->_Gx = imu3->Gx;
        imu3->_Gy = imu3->Gy;
        imu3->_Gz = imu3->Gz;
	imu3->Gx = imu3->Gx_raw / 131.0;
	imu3->Gy = imu3->Gy_raw / 131.0;
	imu3->Gz = imu3->Gz_raw / 131.0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  unsigned long t1, t2, diff;
  float timer = 0.0;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  log_counter = LOG_CYCLES;
  float roll_target_angle = 0.0;
  float pitch_target_angle = 0.0;
  float main_wheel_integrator = 0.0;
  float alpha = 0.1; // the exponential smoothing factor for controller output
  float beta = 0.1; // the exponential smoothing factor for controller output
  float kp = 100.0 * 10.0;
  float ki = 5.0 * 10.0;
  float kd = 10.0 * 10.0;
  float windup = 10.0;
  float k1 = 165.0 * 10.0;
  float k2 = 70.0 * 10.0;
  float k3 = 0.11;
  float k4 = 120.0;
  float safety_angle = 45.0;
  float wheel_velocity = 0.0;
  int frequency = 0; // the number of hall effect sensor pulses over one second
  int count = 0;
  Hsv hsvColor;
  Rgb rgbColor;
  hsvColor.h = hue;
  hsvColor.s = 100.0;
  hsvColor.v = 100.0;
  Imu imu1, imu2, imu3;
  Acceleration r_est1, r_est2, r_est3;
  float roll = 0.0;
  float pitch = 0.0;;
  float _roll = 0.0;
  float _pitch = 0.0;
  float roll_velocity = 0.0;
  float roll_acceleration = 0.0;
  float pitch_velocity = 0.0;
  float magnitude;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_Delay (100);  // wait for a while
  MPU6050_Init(hi2c1, MPU6050_ADDR);
  MPU6050_Init(hi2c1, MPU6050_ADDR2);
  MPU6050_Init(hi2c2, MPU6050_ADDR);
  HAL_Delay (3000);  // wait for a while
  
  initialize(&imu1);
  initialize(&imu2);
  initialize(&imu3);
  hue = 0.0;
//  for (int i = 0; i < samples; i++) {
//    MPU6050_Read_Accel(&imu1, &imu2, &imu3);
//    MPU6050_Read_Gyro(&imu1, &imu2, &imu3);
//    imu1.Ax_offset += imu1.Ax;
//    imu1.Ay_offset += imu1.Ay;
//    imu1.Az_offset += imu1.Az;
//    imu1.Gx_offset += imu1.Gx;
//    imu1.Gy_offset += imu1.Gy;
//    imu1.Gz_offset += imu1.Gz;
//    imu2.Ax_offset += imu2.Ax;
//    imu2.Ay_offset += imu2.Ay;
//    imu2.Az_offset += imu2.Az;
//    imu2.Gx_offset += imu2.Gx;
//    imu2.Gy_offset += imu2.Gy;
//    imu2.Gz_offset += imu2.Gz;
//    imu3.Ax_offset += imu3.Ax;
//    imu3.Ay_offset += imu3.Ay;
//    imu3.Az_offset += imu3.Az;
//    imu3.Gx_offset += imu3.Gx;
//    imu3.Gy_offset += imu3.Gy;
//    imu3.Gz_offset += imu3.Gz;
//    HAL_Delay (5);  // wait for a while
//    hue += 360.0 / (float)samples;
//    hsvColor.h = hue;
//    rgbColor = HSVtoRGB(hsvColor);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int)rgbColor.b);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (int)rgbColor.r);
//    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (int)rgbColor.g * 255);
//  }
//  imu1.Ax_offset /= (float) samples;
//  imu1.Ay_offset /= (float) samples;
//  imu1.Az_offset /= (float) samples;
//  imu1.Gx_offset /= (float) samples;
//  imu1.Gy_offset /= (float) samples;
//  imu1.Gz_offset /= (float) samples;
//  imu2.Ax_offset /= (float) samples;
//  imu2.Ay_offset /= (float) samples;
//  imu2.Az_offset /= (float) samples;
//  imu2.Gx_offset /= (float) samples;
//  imu2.Gy_offset /= (float) samples;
//  imu2.Gz_offset /= (float) samples;
//  imu3.Ax_offset /= (float) samples;
//  imu3.Ay_offset /= (float) samples;
//  imu3.Az_offset /= (float) samples;
//  imu3.Gx_offset /= (float) samples;
//  imu3.Gy_offset /= (float) samples;
//  imu3.Gz_offset /= (float) samples;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MPU6050_Read_Accel(&imu1, &imu2, &imu3);
  r_est1._R_est_x = imu1.Ax_scale * (imu1.Ax - imu1.Ax_offset);
  r_est1._R_est_y = imu1.Ay_scale * (imu1.Ay - imu1.Ay_offset);
  r_est1._R_est_z = imu1.Az_scale * (imu1.Az - imu1.Az_offset);
  r_est2._R_est_x = imu2.Ax_scale * (imu2.Ax - imu2.Ax_offset);
  r_est2._R_est_y = imu2.Ay_scale * (imu2.Ay - imu2.Ay_offset);
  r_est2._R_est_z = imu2.Az_scale * (imu2.Az - imu2.Az_offset);
  r_est3._R_est_x = imu3.Ax_scale * (imu3.Ax - imu3.Ax_offset);
  r_est3._R_est_y = imu3.Ay_scale * (imu3.Ay - imu3.Ay_offset);
  r_est3._R_est_z = imu3.Az_scale * (imu3.Az - imu3.Az_offset);
  MPU6050_Read_Accel(&imu1, &imu2, &imu3);
  r_est1.R_est_x = imu1.Ax_scale * (imu1.Ax - imu1.Ax_offset);
  r_est1.R_est_y = imu1.Ay_scale * (imu1.Ay - imu1.Ay_offset);
  r_est1.R_est_z = imu1.Az_scale * (imu1.Az - imu1.Az_offset);
  r_est2.R_est_x = imu2.Ax_scale * (imu2.Ax - imu2.Ax_offset);
  r_est2.R_est_y = imu2.Ay_scale * (imu2.Ay - imu2.Ay_offset);
  r_est2.R_est_z = imu2.Az_scale * (imu2.Az - imu2.Az_offset);
  r_est3.R_est_x = imu3.Ax_scale * (imu3.Ax - imu3.Ax_offset);
  r_est3.R_est_y = imu3.Ay_scale * (imu3.Ay - imu3.Ay_offset);
  r_est3.R_est_z = imu3.Az_scale * (imu3.Az - imu3.Az_offset);
  r_est1.phi = 0.0;
  r_est1.theta = 0.0;
  r_est1.psi = 0.0;
  r_est2.phi = 0.0;
  r_est2.theta = 0.0;
  r_est2.psi = 0.0;
  r_est3.phi = 0.0;
  r_est3.theta = 0.0;
  r_est3.psi = 0.0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    t1 = DWT->CYCCNT;
    
    // read the Accelerometer and Gyro values
    MPU6050_Read_Accel(&imu1, &imu2, &imu3);
    MPU6050_Read_Gyro(&imu1, &imu2, &imu3);
    
    r_est1.R_acc_x = imu1.Ax_scale * (imu1.Ax - imu1.Ax_offset);
    r_est1.R_acc_y = imu1.Ay_scale * (imu1.Ay - imu1.Ay_offset);
    r_est1.R_acc_z = imu1.Az_scale * (imu1.Az - imu1.Az_offset);
    r_est2.R_acc_x = imu2.Ax_scale * (imu2.Ax - imu2.Ax_offset);
    r_est2.R_acc_y = imu2.Ay_scale * (imu2.Ay - imu2.Ay_offset);
    r_est2.R_acc_z = imu2.Az_scale * (imu2.Az - imu2.Az_offset);
    r_est3.R_acc_x = imu3.Ax_scale * (imu3.Ax - imu3.Ax_offset);
    r_est3.R_acc_y = imu3.Ay_scale * (imu3.Ay - imu3.Ay_offset);
    r_est3.R_acc_z = imu3.Az_scale * (imu3.Az - imu3.Az_offset);
    
    r_est1.magnitude = sqrt(imu1.Ax * imu1.Ax + imu1.Ay * imu1.Ay + imu1.Az * imu1.Az);
    if (r_est1.magnitude > 0.0001 || r_est1.magnitude < -0.0001) {
      r_est1.R_acc_x /= r_est1.magnitude;
      r_est1.R_acc_y /= r_est1.magnitude;
      r_est1.R_acc_z /= r_est1.magnitude;
    }
    r_est2.magnitude = sqrt(imu2.Ax * imu2.Ax + imu2.Ay * imu2.Ay + imu2.Az * imu2.Az);
    if (r_est2.magnitude > 0.0001 || r_est2.magnitude < -0.0001) {
      r_est2.R_acc_x /= r_est2.magnitude;
      r_est2.R_acc_y /= r_est2.magnitude;
      r_est2.R_acc_z /= r_est2.magnitude;
    }
    r_est3.magnitude = sqrt(imu3.Ax * imu3.Ax + imu3.Ay * imu3.Ay + imu3.Az * imu3.Az);
    if (r_est3.magnitude > 0.0001 || r_est3.magnitude < -0.0001) {
      r_est3.R_acc_x /= r_est3.magnitude;
      r_est3.R_acc_y /= r_est3.magnitude;
      r_est3.R_acc_z /= r_est3.magnitude;
    }
    
    r_est1.phi = K * dt * (imu1.Gx_scale * (imu1._Gx - imu1.Gx_offset) + imu1.Gx_scale * (imu1.Gx - imu1.Gx_offset)) / 2.0;
    r_est1.theta = K * dt * (imu1.Gy_scale * (imu1._Gy - imu1.Gy_offset) + imu1.Gy_scale * (imu1.Gy - imu1.Gy_offset)) / 2.0;
    r_est1.psi = K * dt * (imu1.Gz_scale * (imu1._Gz - imu1.Gz_offset) + imu1.Gz_scale * (imu1.Gz - imu1.Gz_offset)) / 2.0;
    
    r_est2.phi = K * dt * (imu2.Gx_scale * (imu2._Gx - imu2.Gx_offset) + imu2.Gx_scale * (imu2.Gx - imu2.Gx_offset)) / 2.0;
    r_est2.theta = K * dt * (imu2.Gy_scale * (imu2._Gy - imu2.Gy_offset) + imu2.Gy_scale * (imu2.Gy - imu2.Gy_offset)) / 2.0;
    r_est2.psi = K * dt * (imu2.Gz_scale * (imu2._Gz - imu2.Gz_offset) + imu2.Gz_scale * (imu2.Gz - imu2.Gz_offset)) / 2.0;
    
    r_est3.phi = K * dt * (imu3.Gx_scale * (imu3._Gx - imu3.Gx_offset) + imu3.Gx_scale * (imu3.Gx - imu3.Gx_offset)) / 2.0;
    r_est3.theta = K * dt * (imu3.Gy_scale * (imu3._Gy - imu3.Gy_offset) + imu3.Gy_scale * (imu3.Gy - imu3.Gy_offset)) / 2.0;
    r_est3.psi = K * dt * (imu3.Gz_scale * (imu3._Gz - imu3.Gz_offset) + imu3.Gz_scale * (imu3.Gz - imu3.Gz_offset)) / 2.0;
    
    r_est1.R_gyro_x = cos(r_est1.psi) * cos(r_est1.theta) * r_est1._R_est_x +
      cos(r_est1.psi) * sin(r_est1.theta) * sin(r_est1.phi) * r_est1._R_est_y -
        sin(r_est1.psi) * cos(r_est1.phi) * r_est1._R_est_y +
          cos(r_est1.psi) * sin(r_est1.theta) * cos(r_est1.phi) * r_est1._R_est_z +
            sin(r_est1.psi) * sin(r_est1.phi) * r_est1._R_est_z;
   
    r_est1.R_gyro_y = sin(r_est1.psi) * cos(r_est1.theta) * r_est1._R_est_x +
      sin(r_est1.psi) * sin(r_est1.theta) * sin(r_est1.phi) * r_est1._R_est_y +
        cos(r_est1.psi) * cos(r_est1.phi) * r_est1._R_est_y +
          sin(r_est1.psi) * sin(r_est1.theta) * cos(r_est1.phi) * r_est1._R_est_z -
            cos(r_est1.psi) * sin(r_est1.phi) * r_est1._R_est_z;
    
    r_est1.R_gyro_z = -sin(r_est1.theta) * r_est1._R_est_x + 
      cos(r_est1.theta) * sin(r_est1.phi) * r_est1._R_est_y + 
        cos(r_est1.theta) * cos(r_est1.phi) * r_est1._R_est_z;
    
    r_est2.R_gyro_x = cos(r_est2.psi) * cos(r_est2.theta) * r_est2._R_est_x +
      cos(r_est2.psi) * sin(r_est2.theta) * sin(r_est2.phi) * r_est2._R_est_y -
        sin(r_est2.psi) * cos(r_est2.phi) * r_est2._R_est_y +
          cos(r_est2.psi) * sin(r_est2.theta) * cos(r_est2.phi) * r_est2._R_est_z +
            sin(r_est2.psi) * sin(r_est2.phi) * r_est2._R_est_z;
   
    r_est2.R_gyro_y = sin(r_est2.psi) * cos(r_est2.theta) * r_est2._R_est_x +
      sin(r_est2.psi) * sin(r_est2.theta) * sin(r_est2.phi) * r_est2._R_est_y +
        cos(r_est2.psi) * cos(r_est2.phi) * r_est2._R_est_y +
          sin(r_est2.psi) * sin(r_est2.theta) * cos(r_est2.phi) * r_est2._R_est_z -
            cos(r_est2.psi) * sin(r_est2.phi) * r_est2._R_est_z;
    
    r_est2.R_gyro_z = -sin(r_est2.theta) * r_est2._R_est_x + 
      cos(r_est2.theta) * sin(r_est2.phi) * r_est2._R_est_y + 
        cos(r_est2.theta) * cos(r_est2.phi) * r_est2._R_est_z;
    
    r_est3.R_gyro_x = cos(r_est3.psi) * cos(r_est3.theta) * r_est3._R_est_x +
      cos(r_est3.psi) * sin(r_est3.theta) * sin(r_est3.phi) * r_est3._R_est_y -
        sin(r_est3.psi) * cos(r_est3.phi) * r_est3._R_est_y +
          cos(r_est3.psi) * sin(r_est3.theta) * cos(r_est3.phi) * r_est3._R_est_z +
            sin(r_est3.psi) * sin(r_est3.phi) * r_est3._R_est_z;
   
    r_est3.R_gyro_y = sin(r_est3.psi) * cos(r_est3.theta) * r_est3._R_est_x +
      sin(r_est3.psi) * sin(r_est3.theta) * sin(r_est3.phi) * r_est3._R_est_y +
        cos(r_est3.psi) * cos(r_est3.phi) * r_est3._R_est_y +
          sin(r_est3.psi) * sin(r_est3.theta) * cos(r_est3.phi) * r_est3._R_est_z -
            cos(r_est3.psi) * sin(r_est3.phi) * r_est3._R_est_z;
    
    r_est3.R_gyro_z = -sin(r_est3.theta) * r_est3._R_est_x + 
      cos(r_est3.theta) * sin(r_est3.phi) * r_est3._R_est_y + 
        cos(r_est3.theta) * cos(r_est3.phi) * r_est3._R_est_z;
        
//    r_est1.R_gyro_x = r_est1._R_est_x +
//      r_est1.theta * r_est1.phi * r_est1._R_est_y -
//        r_est1.psi * r_est1._R_est_y +
//          r_est1.theta * r_est1._R_est_z +
//            r_est1.psi * r_est1.phi * r_est1._R_est_z;
//   
//    r_est1.R_gyro_y = r_est1.psi * r_est1._R_est_x +
//      r_est1.psi * r_est1.theta * r_est1.phi * r_est1._R_est_y +
//        r_est1._R_est_y +
//          r_est1.psi * r_est1.theta * r_est1._R_est_z -
//            r_est1.phi * r_est1._R_est_z;
//    
//    r_est1.R_gyro_z = -r_est1.theta * r_est1._R_est_x + 
//      r_est1.phi * r_est1._R_est_y + 
//        r_est1._R_est_z;
//    
//    r_est2.R_gyro_x = r_est2._R_est_x +
//      r_est2.theta * r_est2.phi * r_est2._R_est_y -
//        r_est2.psi * r_est2._R_est_y +
//          r_est2.theta * r_est2._R_est_z +
//            r_est2.psi * r_est2.phi * r_est2._R_est_z;
//   
//    r_est2.R_gyro_y = r_est2.psi * r_est2._R_est_x +
//      r_est2.psi * r_est2.theta * r_est2.phi * r_est2._R_est_y +
//        r_est2._R_est_y +
//          r_est2.psi * r_est2.theta * r_est2._R_est_z -
//            r_est2.phi * r_est2._R_est_z;
//    
//    r_est2.R_gyro_z = -r_est2.theta * r_est2._R_est_x + 
//      r_est2.phi * r_est2._R_est_y + 
//        r_est2._R_est_z;
//    
//    r_est3.R_gyro_x = r_est3._R_est_x +
//      r_est3.theta * r_est3.phi * r_est3._R_est_y -
//        r_est3.psi * r_est3._R_est_y +
//          r_est3.theta * r_est3._R_est_z +
//            r_est3.psi * r_est3.phi * r_est3._R_est_z;
//   
//    r_est3.R_gyro_y = r_est3.psi * r_est3._R_est_x +
//      r_est3.psi * r_est3.theta * r_est3.phi * r_est3._R_est_y +
//        r_est3._R_est_y +
//          r_est3.psi * r_est3.theta * r_est3._R_est_z -
//            r_est3.phi * r_est3._R_est_z;
//    
//    r_est3.R_gyro_z = -r_est3.theta * r_est3._R_est_x + 
//      r_est3.phi * r_est3._R_est_y + 
//        r_est3._R_est_z;
    
    r_est1._R_est_x = r_est1.R_est_x;
    r_est1._R_est_y = r_est1.R_est_y;
    r_est1._R_est_z = r_est1.R_est_z;
    r_est2._R_est_x = r_est2.R_est_x;
    r_est2._R_est_y = r_est2.R_est_y;
    r_est2._R_est_z = r_est2.R_est_z;
    r_est3._R_est_x = r_est3.R_est_x;
    r_est3._R_est_y = r_est3.R_est_y;
    r_est3._R_est_z = r_est3.R_est_z;
    
    r_est1.R_est_x = W * r_est1.R_acc_x + (1.0 - W) * r_est1.R_gyro_x;
    r_est1.R_est_y = W * r_est1.R_acc_y + (1.0 - W) * r_est1.R_gyro_y;
    r_est1.R_est_z = W * r_est1.R_acc_z + (1.0 - W) * r_est1.R_gyro_z;
    r_est2.R_est_x = W * r_est2.R_acc_x + (1.0 - W) * r_est2.R_gyro_x;
    r_est2.R_est_y = W * r_est2.R_acc_y + (1.0 - W) * r_est2.R_gyro_y;
    r_est2.R_est_z = W * r_est2.R_acc_z + (1.0 - W) * r_est2.R_gyro_z;
    r_est3.R_est_x = W * r_est3.R_acc_x + (1.0 - W) * r_est3.R_gyro_x;
    r_est3.R_est_y = W * r_est3.R_acc_y + (1.0 - W) * r_est3.R_gyro_y;
    r_est3.R_est_z = W * r_est3.R_acc_z + (1.0 - W) * r_est3.R_gyro_z;
    
    r_est1.magnitude = sqrt(r_est1.R_est_x * r_est1.R_est_x + r_est1.R_est_y * r_est1.R_est_y + r_est1.R_est_z * r_est1.R_est_z);
    if (r_est1.magnitude > 0.0001 || r_est1.magnitude < -0.0001) {
      r_est1.R_est_x /= r_est1.magnitude;
      r_est1.R_est_y /= r_est1.magnitude;
      r_est1.R_est_z /= r_est1.magnitude;
    }
    r_est2.magnitude = sqrt(r_est2.R_est_x * r_est2.R_est_x + r_est2.R_est_y * r_est2.R_est_y + r_est2.R_est_z * r_est2.R_est_z);
    if (r_est2.magnitude > 0.0001 || r_est2.magnitude < -0.0001) {
      r_est2.R_est_x /= r_est2.magnitude;
      r_est2.R_est_y /= r_est2.magnitude;
      r_est2.R_est_z /= r_est2.magnitude;
    }
    r_est3.magnitude = sqrt(r_est3.R_est_x * r_est3.R_est_x + r_est3.R_est_y * r_est3.R_est_y + r_est3.R_est_z * r_est3.R_est_z);
    if (r_est3.magnitude > 0.0001 || r_est3.magnitude < -0.0001) {
      r_est3.R_est_x /= r_est3.magnitude;
      r_est3.R_est_y /= r_est3.magnitude;
      r_est3.R_est_z /= r_est3.magnitude;
    }
    
    R_est_x = (r_est1.R_est_x - r_est2.R_est_x + r_est3.R_est_x) / 3.0;
    R_est_y = (r_est1.R_est_z + r_est2.R_est_z - r_est3.R_est_y) / 3.0;
    R_est_z = (r_est1.R_est_y - r_est2.R_est_y + r_est3.R_est_z) / 3.0;
    magnitude = sqrt(R_est_x * R_est_x + R_est_y * R_est_y + R_est_z * R_est_z);
    if (magnitude > 0.0001 || magnitude < -0.0001) {
      R_est_x /= magnitude;
      R_est_y /= magnitude;
      R_est_z /= magnitude;
    }
    
    _roll = roll;
    _pitch = pitch;
    roll = atan2(R_est_x, R_est_z);
    pitch = atan2(R_est_y, R_est_z);
    roll_acceleration = (roll - _roll) - roll_velocity;
    roll_velocity = roll - _roll;
    pitch_velocity = pitch - _pitch;
    
    speed1 = (int)(beta * fabs(k1 * roll + k2 * roll_velocity + k3 * fabs(wheel_velocity) + k4 * roll_acceleration) + (1.0 - beta) * (float)speed1);
    if (speed1 > 255) {
      speed1 = 255;
    }
    if (speed1 < 0) {
      speed1 = 0;
    }
    
    main_wheel_integrator = main_wheel_integrator + pitch * ki;
    if (main_wheel_integrator > windup) {
      main_wheel_integrator = windup;
    }
    if (main_wheel_integrator < -windup) {
      main_wheel_integrator = -windup;
    }

    speed2 = (int)(alpha * fabs(kp * pitch + kd * pitch_velocity + main_wheel_integrator) + (1.0 - alpha) * (float)speed2); //PID function
    if (speed2 > 255) {
      speed2 = 255;
    }
    if (speed2 < 0) {
      speed2 = 0;
    }
    if ((pitch > safety_angle) || (pitch < -safety_angle) || (roll > safety_angle) || (roll < -safety_angle)) {
      speed1 = 0;
      speed2 = 0; // the controller is active in -10~+10 deg range
    }
    // Enable/Disable the reaction motor for the calibration of the main motor
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
      if (roll > roll_target_angle) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed1);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed1);
      }
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    if (pitch > pitch_target_angle) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed2);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed2);
    }
    
    // transmit the Acceleration and Gyro values via the serial connection
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) {
      transmit_logs = 1;
    }
    
    log_counter--;
    
    if (log_counter > 0) {
      t2 = DWT->CYCCNT;
      diff = t2 - t1;
      dt = (float) diff / 72000000.0;
      timer += dt;
      if (timer > interval) {
        previous_count = count;
        count = (TIM3->CNT);
        frequency = count - previous_count;
        hue += frequency;
        if (hue > 360.0) {
          hue = 0.0;
        }
        if (hue < 0.0) {
          hue = 360.0;
        }
        wheel_velocity = (float)abs(frequency) * 60.0 / (float)signals;
        timer = 0.0;
        hsvColor.h = hue;
        rgbColor = HSVtoRGB(hsvColor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int)rgbColor.b); // blue
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (int)rgbColor.r); // red
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (int)rgbColor.g * 255); // green
      }
    } else {
      if (transmit_logs == 1) {
        // sprintf(MSG, "Acc1: %0.2f, %0.2f, %0.2f, Acc2: %0.2f, %0.2f, %0.2f, Gyr1: %0.2f, %0.2f, %0.2f, Gyr2: %0.2f, %0.2f, %0.2f\r\n", acc_roll1, acc_pitch1, acc_yaw1, acc_roll2, acc_pitch2, acc_yaw2, gyr_roll1, gyr_pitch1, gyr_yaw1, gyr_roll2, gyr_pitch2, gyr_yaw2);
        //sprintf(MSG, "AccRol: %0.2f, AccPit: %0.2f, GyrRol: %0.2f, GyrPit: %0.2f\r\n", acc_roll, acc_pitch, gyr_roll, gyr_pitch);
        //sprintf(MSG, "k1: %0.2f, k2: %0.2f, k3: %0.2f\r\n", k1 * roll, k2 * roll_velocity, k3 * wheel_velocity);
        //sprintf(MSG, "AR: %0.2f, AP: %0.2f, GR: %0.2f, GP: %0.2f, v1: %d, v2: %d\r\n", acc_roll, acc_pitch, gyr_roll, gyr_pitch, frequency, speed2);
        // sprintf(MSG, "AR: %0.2f, %0.2f, %0.2f, AP: %0.2f, %0.2f, %0.2f, AY: %0.2f, %0.2f, %0.2f\r\n", acc_roll1, acc_roll2, acc_roll3, acc_pitch1, acc_pitch2, acc_pitch3, acc_yaw1, acc_yaw2, acc_yaw3);
        sprintf(MSG, "R1: %0.2f, %0.2f, %0.2f, R2: %0.2f, %0.2f, %0.2f, R3: %0.2f, %0.2f, %0.2f, R: %0.2f, %0.2f, %0.2f, roll: %0.2f, pitch: %0.2f, dt = %f, log_dt = %f\r\n",
                r_est1.R_est_x, r_est1.R_est_y, r_est1.R_est_z,
                r_est2.R_est_x, r_est2.R_est_y, r_est2.R_est_z,
                r_est3.R_est_x, r_est3.R_est_y, r_est3.R_est_z,
                R_est_x, R_est_y, R_est_z,
                roll, pitch,
                dt, log_dt);
        // sprintf(MSG, "GR: %0.2f, %0.2f, %0.2f, GP: %0.2f, %0.2f, %0.2f, GY: %0.2f, %0.2f, %0.2f\r\n", gyr_roll1, gyr_roll2, gyr_roll3, gyr_pitch1, gyr_pitch2, gyr_pitch3, gyr_yaw1, gyr_yaw2, gyr_yaw3);
        
        HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 160);
      }
      log_counter = LOG_CYCLES;
    
      t2 = DWT->CYCCNT;
      diff = t2 - t1;
      log_dt = (float) diff / 72000000.0;
      timer += log_dt;
    }
    // dt = 0.0018, ldt = 0.1330

    //HAL_Delay(1); // wait for a while
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sConfig.IC2Filter = 0;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
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
