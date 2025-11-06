/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bno055.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char uart_buf[250];
BNO055_Vector_t accel_data = {0};
BNO055_Vector_t gyro_data = {0};
BNO055_Vector_t mag_data = {0};
BNO055_Vector_t euler_data = {0};
BNO055_CalibStatus_t calib_status = {0};
uint32_t i2c_errors = 0;
uint32_t i2c_success = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  // I2C Scanner - Scan for connected devices
  sprintf(uart_buf, "\r\n=== I2C Scanner ===\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  sprintf(uart_buf, "Scanning I2C bus...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  
  int devices_found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    // Try to communicate with device at address
    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 10);
    if (result == HAL_OK) {
      sprintf(uart_buf, "Device found at 0x%02X\r\n", addr);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      devices_found++;
    }
  }
  
  if (devices_found == 0) {
    sprintf(uart_buf, "No I2C devices found!\r\n");
  } else {
    sprintf(uart_buf, "Scan complete. Found %d device(s).\r\n", devices_found);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  sprintf(uart_buf, "==================\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  
  HAL_Delay(1000);
  
  // Initialize BNO055 sensor
  sprintf(uart_buf, "BNO055 Initializing...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  
  if (BNO055_Init(&hi2c1) == HAL_OK) {
    sprintf(uart_buf, "BNO055 Initialization OK!\r\n");
  } else {
    sprintf(uart_buf, "BNO055 Initialization FAILED!\r\n");
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    HAL_StatusTypeDef status;
    uint8_t read_success = 1;  // Flag to track if all reads succeed
    
    // Read calibration status
    status = BNO055_GetCalibration(&hi2c1, &calib_status);
    if (status == HAL_OK) {
      i2c_success++;
      sprintf(uart_buf, "\r\n=== BNO055 Data (OK:%lu ERR:%lu) ===\r\n", i2c_success, i2c_errors);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      
      sprintf(uart_buf, "Calib: Sys=%d Gyro=%d Accel=%d Mag=%d\r\n",
              calib_status.sys, calib_status.gyro, 
              calib_status.accel, calib_status.mag);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      i2c_errors++;
      read_success = 0;
      sprintf(uart_buf, "[ERROR] Calib read failed: %d (Errors:%lu)\r\n", status, i2c_errors);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
    
    // Read Euler angles (orientation)
    status = BNO055_ReadEuler(&hi2c1, &euler_data);
    if (status == HAL_OK) {
      int16_t heading = euler_data.x / 16;
      int16_t roll = euler_data.y / 16;
      int16_t pitch = euler_data.z / 16;
      int16_t h_dec = abs((euler_data.x % 16) * 100 / 16);
      int16_t r_dec = abs((euler_data.y % 16) * 100 / 16);
      int16_t p_dec = abs((euler_data.z % 16) * 100 / 16);
      
      sprintf(uart_buf, "Euler - H:%d.%02d R:%d.%02d P:%d.%02d deg\r\n", 
              heading, h_dec, roll, r_dec, pitch, p_dec);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      i2c_errors++;
      read_success = 0;
      sprintf(uart_buf, "[ERROR] Euler read failed: %d\r\n", status);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
    
    HAL_Delay(10);  // Wait for sensor data to be ready
    
    // Read accelerometer data
    status = BNO055_ReadAccel(&hi2c1, &accel_data);
    if (status == HAL_OK) {
      int16_t ax = accel_data.x / 100;
      int16_t ay = accel_data.y / 100;
      int16_t az = accel_data.z / 100;
      int16_t ax_dec = abs((accel_data.x % 100));
      int16_t ay_dec = abs((accel_data.y % 100));
      int16_t az_dec = abs((accel_data.z % 100));
      
      sprintf(uart_buf, "Accel - X:%d.%02d Y:%d.%02d Z:%d.%02d m/s2\r\n", 
              ax, ax_dec, ay, ay_dec, az, az_dec);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      i2c_errors++;
      read_success = 0;
      sprintf(uart_buf, "[ERROR] Accel read failed: %d\r\n", status);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
    
    HAL_Delay(10);  // Wait for sensor data to be ready
    
    // Read gyroscope data
    status = BNO055_ReadGyro(&hi2c1, &gyro_data);
    if (status == HAL_OK) {
      int16_t gx = gyro_data.x / 16;
      int16_t gy = gyro_data.y / 16;
      int16_t gz = gyro_data.z / 16;
      int16_t gx_dec = abs((gyro_data.x % 16) * 100 / 16);
      int16_t gy_dec = abs((gyro_data.y % 16) * 100 / 16);
      int16_t gz_dec = abs((gyro_data.z % 16) * 100 / 16);
      
      sprintf(uart_buf, "Gyro  - X:%d.%02d Y:%d.%02d Z:%d.%02d deg/s\r\n", 
              gx, gx_dec, gy, gy_dec, gz, gz_dec);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      i2c_errors++;
      read_success = 0;
      sprintf(uart_buf, "[ERROR] Gyro read failed: %d\r\n", status);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
    
    HAL_Delay(10);  // Wait for sensor data to be ready
    
    // Read magnetometer data
    status = BNO055_ReadMag(&hi2c1, &mag_data);
    if (status == HAL_OK) {
      int16_t mx = mag_data.x / 16;
      int16_t my = mag_data.y / 16;
      int16_t mz = mag_data.z / 16;
      int16_t mx_dec = abs((mag_data.x % 16) * 100 / 16);
      int16_t my_dec = abs((mag_data.y % 16) * 100 / 16);
      int16_t mz_dec = abs((mag_data.z % 16) * 100 / 16);
      
      sprintf(uart_buf, "Mag   - X:%d.%02d Y:%d.%02d Z:%d.%02d uT\r\n", 
              mx, mx_dec, my, my_dec, mz, mz_dec);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      i2c_errors++;
      read_success = 0;
      sprintf(uart_buf, "[ERROR] Mag read failed: %d\r\n", status);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
    
    // If there were errors, wait a bit before retrying
    if (!read_success) {
      HAL_Delay(100);
    }
    
    HAL_Delay(100);  // Read data every 100ms (10Hz update rate)
    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
