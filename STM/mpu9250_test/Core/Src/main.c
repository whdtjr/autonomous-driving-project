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
#include <stdio.h>
#include <math.h>
#include "MPU9250.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU9250_t mpu;              // MPU9250 핸들
static float mag_off_x = 0; // 간이 하드아이언 오프셋(초기 0)
static float mag_off_y = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float heading_deg_from_magnet(float mx, float my)
{
  float hdg = atan2f(my, mx) * 180.0f / 3.1415926f; // atan2f 대용 함수 쓰면 atan2f로 교체
  if (hdg < 0) hdg += 360.0f;
  return hdg;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (MPU9250_Init(&mpu, MPU9250_Device_0,
                   ACCEL_SCALE_2G,
                   GYRO_SCALE_250dps,
                   MAG_SCALE_16bit) != MPU9250_RESULT_OK)
  {
    printf("MPU9250 init FAILED\r\n");
    while (1) { HAL_Delay(500); }
  }
  printf("MPU9250 init OK\r\n");

  // ---- AK8963 / BYPASS 확인용 빠른 진단 ----
  uint8_t mpu_who=0, int_cfg=0, ak_wia=0, ak_cntl1=0;

  HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x75, I2C_MEMADD_SIZE_8BIT, &mpu_who, 1, 100);  // MPU WHO_AM_I
  HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x37, I2C_MEMADD_SIZE_8BIT, &int_cfg,  1, 100); // INT_PIN_CFG
  HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x00, I2C_MEMADD_SIZE_8BIT, &ak_wia,   1, 100); // AK8963 WIA
  HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x0A, I2C_MEMADD_SIZE_8BIT, &ak_cntl1, 1, 100); // AK8963 CNTL1

  printf("MPU WHO_AM_I=0x%02X (expect 0x71)\r\n", mpu_who);
  printf("INT_PIN_CFG =0x%02X (BYPASS_EN bit1 should be 1)\r\n", int_cfg);
  printf("AK8963 WIA  =0x%02X (expect 0x48)\r\n", ak_wia);
  printf("AK8963 CNTL1=0x%02X (0x12=16bit cont1, 0x16=16bit cont2)\r\n", ak_cntl1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 1
		MPU9250_Result_t r = MPU9250_ReadMag(&mpu);
		if (r == MPU9250_RESULT_OK) {
			float mx = (float)mpu.mag_raw[0] * mpu.magMult;
			float my = (float)mpu.mag_raw[1] * mpu.magMult;
			float mz = (float)mpu.mag_raw[2] * mpu.magMult;

			// |B| 계산
			float B_mag = sqrtf(mx*mx + my*my + mz*mz);

			printf("Mag(uT) X:%.2f Y:%.2f Z:%.2f \r\n", mx, my, mz);
			printf("|B| = %.1f (uT) \r\n", B_mag);
		}
		HAL_Delay(50);
	}
#endif
#if 0
  // MPU9250_ReadMag 함수 사용 안했을 때
	  // 1) AK8963 ST1에서 DRDY 체크 후 강제 Read (DataReady() 없이)
	  uint8_t st1=0, raw[7]={0};
	  HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x02, I2C_MEMADD_SIZE_8BIT, &st1, 1, 10); // ST1

	  if (st1 & 0x01) {
	    // HXL~HZH(6byte) + ST2(1byte)를 한 번에
	    if (HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x03, I2C_MEMADD_SIZE_8BIT, raw, 7, 50) == HAL_OK) {
	      int16_t mx_raw = (int16_t)((raw[1]<<8) | raw[0]);
	      int16_t my_raw = (int16_t)((raw[3]<<8) | raw[2]);
	      int16_t mz_raw = (int16_t)((raw[5]<<8) | raw[4]);

	      // 센서코드값을 물리단위로 변환 : raw -> uT 변환은 드라이버의 mpu.magMult 사용(초기화 시 세팅됨)
	      float mx = (float)mx_raw * mpu.magMult;
	      float my = (float)my_raw * mpu.magMult;
	      float mz = (float)mz_raw * mpu.magMult;

	      // |B| 계산
	      float B_mag = sqrtf(mx*mx + my*my + mz*mz);

//	      // 간이 heading
//	      float hdg = atan2f(my, mx) * 180.0f / 3.1415926f;
//	      if (hdg < 0) hdg += 360.0f;

	      printf("Mag(uT) X:%.2f Y:%.2f Z:%.2f", mx, my, mz);
	      printf("|B| = %.1f (uT) \r\n", B_mag);
	    }
	  } else {
	    // DRDY가 아직 아니면 점 하나 찍어서 루프가 도는지 확인
//	    printf(".");  // 필요 시 주석 해제
	  }

	  HAL_Delay(50); // 약 20Hz
  }
#endif

#if 0
	  if (MPU9250_DataReady(&mpu) == MPU9250_RESULT_OK)
	  {
		// 필요하면 가속/자이로도 읽기
		// MPU9250_ReadAccel(&mpu);
		// MPU9250_ReadGyro(&mpu);

		// --- 자기장 읽기 ---
		MPU9250_ReadMag(&mpu);

		// 드라이버가 mag[]에 각도 변환을 섞어둘 수 있으니 raw * scale 사용하는 편이 안전
		float mx = (float)mpu.mag_raw[0] * mpu.magMult;
		float my = (float)mpu.mag_raw[1] * mpu.magMult;
		float mz = (float)mpu.mag_raw[2] * mpu.magMult;

		// 간이 오프셋 보정(초기엔 0, 극값 측정 후 갱신)
		float hdg = heading_deg_from_magnet(mx - mag_off_x, my - mag_off_y);

		printf("Mag(uT) X:%.2f Y:%.2f Z:%.2f | Heading: %.1f deg\r\n",
			   mx, my, mz, hdg);
	  }
	  HAL_Delay(50); // 출력 주기 약 20Hz
	}
#endif

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART6 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

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
