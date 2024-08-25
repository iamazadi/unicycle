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

#define TRANSMIT_LENGTH 180
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

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
  float dt = 0.0;
  const float CPU_CLOCK = 84000000.0;
  const float rolling_wheel_pulse = 17.0;
  const float reaction_wheel_pulse = 26.0;
  int rolling_wheel_previous_count = 0;
  int rolling_wheel_count = 0;
  int rolling_wheel_frequency = 0;
  float rolling_wheel_velocity = 0.0;
  int reaction_wheel_previous_count = 0;
  int reaction_wheel_count = 0;
  int reaction_wheel_frequency = 0;
  float reaction_wheel_velocity = 0.0;
  int initialization_delay = 10;
  int sensor_delay = 1;
  Imu imu1, imu2, imu3, imu4;
  initialize(&imu1);
  initialize(&imu2);
  initialize(&imu3);
  initialize(&imu4);
  int init1, init2, init3, init4;
  int transmit = 0;
  int log_counter = 0;
  const int LOG_CYCLE = 5;

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

  // sensor measurements in the local frame of the sensors
  float R1[3] = {0.0, 0.0, 0.0};
  float R2[3] = {0.0, 0.0, 0.0};
  float R3[3] = {0.0, 0.0, 0.0};
  float R4[3] = {0.0, 0.0, 0.0};
  // sensor measurements in the robot body frame
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
  // x-Euler angle (roll)
  float gamma = 0.0;

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(initialization_delay);
  init1 = MPU6050_Init(hi2c1, MPU6050_ADDR);
  HAL_Delay(initialization_delay);
  init2 = MPU6050_Init(hi2c1, MPU6050_ADDR2);
  HAL_Delay(initialization_delay);
  init3 = MPU6050_Init(hi2c3, MPU6050_ADDR);
  HAL_Delay(initialization_delay);
  init4 = MPU6050_Init(hi2c3, MPU6050_ADDR2);
  HAL_Delay(initialization_delay);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    t1 = DWT->CYCCNT;

    MPU6050_Read_Accel(hi2c1, MPU6050_ADDR, &imu1);
    HAL_Delay(sensor_delay);
    MPU6050_Read_Accel(hi2c1, MPU6050_ADDR2, &imu2);
    HAL_Delay(sensor_delay);
    MPU6050_Read_Accel(hi2c3, MPU6050_ADDR, &imu3);
    MPU6050_Read_Accel(hi2c3, MPU6050_ADDR2, &imu4);

    R1[0] = imu1.Ax;
    R1[1] = imu1.Ay;
    R1[2] = imu1.Az;
    R2[0] = imu2.Ax;
    R2[1] = imu2.Ay;
    R2[2] = imu2.Az;
    R3[0] = imu3.Ax;
    R3[1] = imu3.Ay;
    R3[2] = imu3.Az;
    R4[0] = imu4.Ax;
    R4[1] = imu4.Ay;
    R4[2] = imu4.Az;

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

    beta = beta / 3.14 * 180.0;
    gamma = gamma / 3.14 * 180.0;

    // Toggle the LED
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    
    /* Output a message on Hyperterminal using printf function */
    // printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");

    // Wait for 1 ms
    // HAL_Delay(1);
    rolling_wheel_count = (TIM2->CNT);
    reaction_wheel_count = (TIM3->CNT);
    
    log_counter++;
    if (log_counter > LOG_CYCLE) {
      transmit = 1;
    }
    if (transmit == 1) {
      transmit = 0;
      log_counter = 0;
      // sprintf(MSG, "Pitch: %0.2f, Roll: %0.2f, A1: %0.2f, %0.2f, %0.2f, A2: %0.2f, %0.2f, %0.2f, A3: %0.2f, %0.2f, %0.2f, A4: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
      //         beta, gamma, imu1.Ax, imu1.Ay, imu1.Az, imu2.Ax, imu2.Ay, imu2.Az, imu3.Ax, imu3.Ay, imu3.Az, imu4.Ax, imu4.Ay, imu4.Az,
      //         rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      sprintf(MSG, "Pitch: %0.2f, Roll: %0.2f, A1: %0.2f, %0.2f, %0.2f, A2: %0.2f, %0.2f, %0.2f, A3: %0.2f, %0.2f, %0.2f, A4: %0.2f, %0.2f, %0.2f, c1: %d, c2: %d, init: %d%d%d%d, dt: %0.6f\r\n",
              beta, gamma, imu1.Ax, imu1.Ay, imu1.Az, imu2.Ax, imu2.Ay, imu2.Az, imu3.Ax, imu3.Ay, imu3.Az, imu4.Ax, imu4.Ay, imu4.Az,
              rolling_wheel_count, reaction_wheel_count, init1, init2, init3, init4, dt);
      HAL_UART_Transmit(&huart6, MSG, sizeof(MSG), TRANSMIT_LENGTH);
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
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
