/*
 * @Author: Master ou 2630483757@qq.com
 * @Date: 2026-04-28 08:53:05
 * @LastEditors: Master ou 2630483757@qq.com
 * @LastEditTime: 2026-05-03 10:50:03
 * @FilePath: \Kalman_mpu6050\Core\Src\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: Master ou 2630483757@qq.com
 * @Date: 2026-04-28 08:53:05
 * @LastEditors: Master ou 2630483757@qq.com
 * @LastEditTime: 2026-05-02 16:20:03
 * @FilePath: \Kalman_mpu6050\Core\Src\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "mpu6050.h"
#include <stdint.h>
#include <stdio.h>
#include "qmc5883p.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
Mpu6050_t Mpu6050;
Kalman_t Kalman;
QMC5883P_Data_t QMC5883P_Data;
float heading;
// 定义偏差值
#define QMC5883P_OFFSET_X -134
#define QMC5883P_OFFSET_Y 347
#define QMC5883P_OFFSET_Z -118.5
float calibrated_x = 0;
float calibrated_y = 0;
float calibrated_z = 0;
float filter_yaw_sin = 0;
float filter_yaw_cos = 1;
float sin_h=0;
float cos_h=1;
float yaw=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float low_pass_filter(float *input, float *last, float alpha)
{
   return alpha * *input + (1 - alpha) * *last;
}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  while(MPU6050_Init(&hi2c1) == 1); 
  Kalman_Init();
  MPU6050_Calibrate_Gyro_offset();

  /* 强制识别QMC5883P地址 */
  // printf("Start I2C Scan...\r\n");
  // printf("Start I2C Scan...\r\n");
  // for(uint8_t i = 1; i < 128; i++) {
  //     // 强制发送不同地址探活
  //     if(HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 3, 100) == HAL_OK) {
  //         printf(">>> Found I2C Device! 7-bit Addr: 0x%02X, 8-bit Addr: 0x%02X\r\n", i, i << 1);
  //     }
  // }
  // printf("I2C Scan Complete.\r\n");

  QMC5883P_Init(&hi2c2);
  // 在main.c中添加
  // printf("Testing I2C2...\r\n");
  // uint8_t test = 0;
  // if(HAL_I2C_Master_Transmit(&hi2c2, 0x2c<<1, &test, 0, 100) == HAL_OK) {
  //     printf("I2C2 OK\r\n");
  // } else {
  //     printf("I2C2 Error\r\n");
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 读取 MPU6050，更新 dt 和陀螺仪数据 */
    MPU6050_Read_All();
    if(QMC5883P_Ready(&hi2c2))
    {
      if(QMC5883P_ReadData(&hi2c2, &QMC5883P_Data) == 0)
      {
     
        // printf("%d,%d,%d\r\n", QMC5883P_Data.x, QMC5883P_Data.y, QMC5883P_Data.z);
        calibrated_x = QMC5883P_Data.x - QMC5883P_OFFSET_X;
        calibrated_y = QMC5883P_Data.y - QMC5883P_OFFSET_Y;
        heading = atan2f(-calibrated_x, -calibrated_y);
        /*通过sin/cos滤波减少误差*/
        sin_h = sinf(heading);
        cos_h = cosf(heading);
        filter_yaw_sin = low_pass_filter(&sin_h, &filter_yaw_sin, 0.2f);
        filter_yaw_cos = low_pass_filter(&cos_h, &filter_yaw_cos, 0.2f);
        yaw = atan2f(filter_yaw_sin, filter_yaw_cos);
        yaw = yaw * 180.0f / M_PI;
        /*将角度转换为0-360度，归一化*/
        if(yaw<0) yaw += 360.0f;
        else if (yaw>=360) yaw-=360.0f;
        
        printf("Heading: %d\r\n", (int16_t)yaw);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
