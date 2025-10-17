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
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
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

#define TOF_CAN_THRESH_MM   145
#define TOF_MODE CONTINUOUS

#define MAG_CAN_THRESH_uT   1000   // 트립 임계값
#define MAG_CAN_HYST_uT      900   // 히스테리시스(재무장 임계값)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */




/* can 통신 변수 */
CAN_FilterTypeDef canFilter1;
CAN_RxHeaderTypeDef canRxHeader;
CAN_TxHeaderTypeDef canTxHeader;
uint8_t can1Rx0Data[8];
uint32_t TxMailBox;
uint8_t can1Tx0Data[8];
volatile int button_flag = 0;
volatile int can1_rx0_flag = 0;
static VL53L0X_Dev_t tof;
uint32_t refSpadCount = 0;
uint8_t  isAperture   = 0;




/* TOF 센서 변수 */
#ifdef TOF_MODE
	static volatile uint16_t g_last_dist_mm = 0; // 연속 모드 변수
#else
	static VL53L0X_RangingMeasurementData_t meas; // 싱글모드 변수
#endif
static volatile int16_t g_tof_offset_mm = 45; // 거리 측정 센서 오차


/* mpu센서 변수 */
MPU9250_t mpu;              // MPU9250 핸들
static float mag_off_x = 0; // 간이 하드아이언 오프셋(초기 0)
static float mag_off_y = 0;
static uint8_t g_mag_sent = 0;     // 0: 대기, 1: 이미 보냄(히스테리시스 내릴 때까지 잠금)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

#ifdef TOF_MODE // 연속 모드
	void TOF_CONTINUOUS_Init(void);
	static inline void TOF_ReadOnce(void);
#else // 싱글 모드
	void TOF_SINGLE_Init(void);
#endif


static inline void CAN_SendMag(uint16_t b_uT);
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* 필터 쌍이 2개임. 의미는 없어서 같게 설정함. */
//  canFilter1.FilterMaskIdHigh = 0x7F3 << 5;
//  canFilter1.FilterIdHigh = 0x106 << 5; // 시프트 이유는 레지스터를 참고
//  canFilter1.FilterMaskIdLow = 0x7F3 << 5;
//  canFilter1.FilterIdLow = 0x106 << 5;
//
//  canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
//  canFilter1.FilterScale = CAN_FILTERSCALE_16BIT;
//  canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//
//  canFilter1.SlaveStartFilterBank = 0; // 필터를 통과한 ID가 저장될 곳.
//
//  HAL_CAN_ConfigFilter(&hcan1, &canFilter1); // can id filter setting
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 메시지가 들어오면 인터럽트 발생.
//
//  HAL_CAN_Start(&hcan1);

  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;

  // 표준 ID는 High 레지스터에 << 5 해서 넣음
  f.FilterIdHigh     = (0x000A << 5);    // 수신 ID = 0x0A
  f.FilterIdLow      = 0x0000;           // IDE=0, RTR=0
  f.FilterMaskIdHigh = (0x07FF << 5);    // 11비트 모두 비교
  f.FilterMaskIdLow  = 0x0000;

  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;           // (듀얼 CAN이면 경계값, 단일이면 무시)
  HAL_CAN_ConfigFilter(&hcan1, &f);

  HAL_CAN_Start(&hcan1);
  // 수신 FIFO0 메시지 도착 인터럽트만 활성화 (에러/경고 알림은 잠깐 비활성)
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* 거리 측정 센서 초기화 */
#ifdef TOF_MODE
  TOF_CONTINUOUS_Init(); /* 연속 모드 초기화 */
#else
  TOF_SINGLE_Init(); /* 싱글 모드 초기화 */
#endif


//  /* MPU 센서 초기화 */
  if (MPU9250_Init(&mpu, MPU9250_Device_0, ACCEL_SCALE_2G, GYRO_SCALE_250dps, MAG_SCALE_16bit) != MPU9250_RESULT_OK)
  {
    printf("MPU9250 init FAILED\r\n");
  }
  else
  {
	  printf("MPU9250 init OK\r\n");
  }

  printf("main start()!!\r\n");

  /* 테스트 코드 */
#if 0// ---- AK8963 / BYPASS 확인용 빠른 진단 ----
  uint8_t mpu_who=0, int_cfg=0, ak_wia=0, ak_cntl1=0;

  HAL_I2C_Mem_Read(&hi2c2, (0x68<<1), 0x75, I2C_MEMADD_SIZE_8BIT, &mpu_who, 1, 100);  // MPU WHO_AM_I
  HAL_I2C_Mem_Read(&hi2c2, (0x68<<1), 0x37, I2C_MEMADD_SIZE_8BIT, &int_cfg,  1, 100); // INT_PIN_CFG
  HAL_I2C_Mem_Read(&hi2c2, (0x0C<<1), 0x00, I2C_MEMADD_SIZE_8BIT, &ak_wia,   1, 100); // AK8963 WIA
  HAL_I2C_Mem_Read(&hi2c2, (0x0C<<1), 0x0A, I2C_MEMADD_SIZE_8BIT, &ak_cntl1, 1, 100); // AK8963 CNTL1

  printf("MPU WHO_AM_I=0x%02X (expect 0x71)\r\n", mpu_who);
  printf("INT_PIN_CFG =0x%02X (BYPASS_EN bit1 should be 1)\r\n", int_cfg);
  printf("AK8963 WIA  =0x%02X (expect 0x48)\r\n", ak_wia);
  printf("AK8963 CNTL1=0x%02X (0x12=16bit cont1, 0x16=16bit cont2)\r\n", ak_cntl1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 0	  /* 싱글모드 연속 동작 코드 */
//	  static uint32_t t0;
//	  if (HAL_GetTick() - t0 >= 250)
//	  {
//	      t0 = HAL_GetTick();
//	      VL53L0X_RangingMeasurementData_t m;
//	      VL53L0X_Error e = VL53L0X_PerformSingleRangingMeasurement(&tof, &m);
//	      if (!e)
//	      {
//	          printf("dist=%4u mm status=%u\r\n", (unsigned)m.RangeMilliMeter, (unsigned)m.RangeStatus);
//	      }
//	      else
//	      {
//	          printf("ranging error=%d\r\n", e);
//	      }
//	  }
#endif
	  uint8_t ready = 0;
	  if (VL53L0X_GetMeasurementDataReady(&tof, &ready) == VL53L0X_ERROR_NONE && ready) {
	      TOF_ReadOnce();              // 여기서 ClearInterruptMask() 해줌
	  }
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

			uint16_t B_uT_u16 = (B_mag <= 0) ? 0 : (B_mag > 65535.0f ? 65535 : (uint16_t)(B_mag + 0.5f));

			if (!g_mag_sent && B_uT_u16 >= MAG_CAN_THRESH_uT) {
			    CAN_SendMag(B_uT_u16);   // 0x103 송신
			    g_mag_sent = 1;          // 락
			} else if (g_mag_sent && B_uT_u16 <= MAG_CAN_HYST_uT) {
			    g_mag_sent = 0;          // 재무장
			}
		}
		HAL_Delay(50);
#endif
	  if(button_flag)
	  {
		  printf("push button!!\r\n");
		  button_flag = 0;

		  canTxHeader.StdId = 0x102;
		  canTxHeader.RTR = CAN_RTR_DATA;
		  canTxHeader.IDE = CAN_ID_STD;
		  canTxHeader.DLC = 8;

		  for(int i=0; i<8; i++)
		  {
			  can1Tx0Data[i]++;
		  }

		  TxMailBox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1); // 비어있는 메일 박스 번호를 리턴
		  HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, &can1Tx0Data, &TxMailBox); // 비어있는 메일 박스에 데이터를 채움.
	  }
	  if(can1_rx0_flag)
	  {
		  can1_rx0_flag = 0;
		  printf("receive\r\n");
		  for(int i=0; i<8; i++)
		  {
			  printf("%s ",can1Rx0Data[i]);
		  }
		  printf("\r\n");
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : blue_button_Pin */
  GPIO_InitStruct.Pin = blue_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(blue_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_Ready_Flag_Pin */
  GPIO_InitStruct.Pin = TOF_Ready_Flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_Ready_Flag_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef TOF_MODE
	void TOF_CONTINUOUS_Init(void)
	{
		uint8_t id=0;
		VL53L0X_Error px = VL53L0X_RdByte(&tof, 0xC0, &id);  // module ID 근처
		printf("Ping=%d id=0x%02X\r\n", px, id);

		VL53L0X_Error st;
		uint8_t vhv = 0, phase = 0;

		HAL_Delay(10);
		tof.I2cDevAddr      = 0x29;   // 7-bit
		tof.comms_type      = 1;      // I2C
		tof.comms_speed_khz = 400;    // 정보용(실제 속도는 MX_I2C1_Init 설정)

		st = VL53L0X_WaitDeviceBooted(&tof);                       printf("Boot=%d\r\n", st);
		st = VL53L0X_DataInit(&tof);                               printf("DataInit=%d\r\n", st);
		st = VL53L0X_StaticInit(&tof);                             printf("StaticInit=%d\r\n", st);

		st = VL53L0X_PerformRefSpadManagement(&tof, &refSpadCount, &isAperture);
																   printf("SPAD=%d count=%lu aperture=%u\r\n",
																		  st, (unsigned long)refSpadCount, isAperture);
		st = VL53L0X_PerformRefCalibration(&tof, &vhv, &phase);    printf("RefCal=%d (vhv=%u,phase=%u)\r\n",
																		  st, vhv, phase);

		st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof, 200000);
																   printf("Budget=%d\r\n", st);
		st = VL53L0X_SetInterMeasurementPeriodMilliSeconds(&tof, 250);
																   printf("InterMeas=%d\r\n", st);

		st = VL53L0X_SetGpioConfig(&tof, 0,
				VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
				VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
				VL53L0X_INTERRUPTPOLARITY_LOW);                    printf("GpioCfg=%d\r\n", st);

		st = VL53L0X_ClearInterruptMask(&tof, 0);                  printf("ClearInt=%d\r\n", st);

		st = VL53L0X_SetDeviceMode(&tof, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
																   printf("DevMode=%d\r\n", st);
		st = VL53L0X_StartMeasurement(&tof);                       printf("Start=%d\r\n", st);
	}
#else
	void TOF_SINGLE_Init(void)
	{
		  HAL_Delay(10);   // 전원/리셋 후 부팅 대기
		  tof.I2cDevAddr = 0x29; // 7-bit
		  tof.comms_type = 1; // I2C
		  tof.comms_speed_khz = 400;

		  VL53L0X_Error st;
		  st = VL53L0X_WaitDeviceBooted(&tof);
		  printf("Boot=%d\r\n", st);
		  st = VL53L0X_DataInit(&tof);
		  printf("DataInit=%d\r\n", st);
		  st = VL53L0X_StaticInit(&tof);
		  printf("StaticInit=%d\r\n", st);

		  // 캘리브레이션(권장)
		  st = VL53L0X_PerformRefSpadManagement(&tof, &refSpadCount, &isAperture);
		  printf("SPAD=%d count=%lu aperture=%u\r\n", st, (unsigned long)refSpadCount, isAperture);
		  uint8_t vhv=0, phase=0;
		  st = VL53L0X_PerformRefCalibration(&tof, &vhv, &phase);
		  printf("RefCal=%d (vhv=%u,phase=%u)\r\n", st, vhv, phase);

		  // 타이밍 버짓
		  st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof, 200000);
		  printf("Budget=%d\r\n", st);
		  if (VL53L0X_PerformSingleRangingMeasurement(&tof, &meas) == VL53L0X_ERROR_NONE)
		  {
		      printf("dist=%u mm, status=%u\r\n", (unsigned)meas.RangeMilliMeter, (unsigned)meas.RangeStatus);
		  }
		  else
		  {
		      printf("single ranging failed\r\n");
		  }
	}
#endif

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART6 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rxh;
    uint8_t d[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, d) == HAL_OK) {
            // 표준/확장 구분
            if (rxh.IDE == CAN_ID_STD) {
                printf("RX std 0x%03lX DLC=%d : ", rxh.StdId, rxh.DLC);
            } else {
                printf("RX ext 0x%08lX DLC=%d : ", rxh.ExtId, rxh.DLC);
            }
            // DLC 만큼만 HEX로
            for (uint32_t i=0; i<rxh.DLC && i<8; ++i) printf("%02X ", d[i]);
            printf("\r\n");
        } else {
            // 읽기 실패 시 탈출(무한루프 방지)
            break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		button_flag = 1;
	}
	else if (GPIO_Pin == GPIO_PIN_14)
	{
		TOF_ReadOnce();
	}

}

// --- 추가: VL53L0X 거리(mm) 2바이트(빅엔디언)로 CAN 송신 ---
static inline void CAN_SendVL53(uint16_t dist_mm)
{
    CAN_TxHeaderTypeDef txh = {0};
    txh.StdId  = 0x102;               // 필요 시 0x100 등으로 바꿔도 됨(수신측과 맞추기)
    txh.IDE    = CAN_ID_STD;
    txh.RTR    = CAN_RTR_DATA;
    txh.DLC    = 2;

    uint8_t data[2];
    data[0] = (uint8_t)(dist_mm >> 8);    // MSB
    data[1] = (uint8_t)(dist_mm & 0xFF);  // LSB

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        uint32_t mailbox;
        (void)HAL_CAN_AddTxMessage(&hcan1, &txh, data, &mailbox);
    }
}

static inline void CAN_SendMag(uint16_t b_uT)
{
    CAN_TxHeaderTypeDef txh = {0};
    txh.StdId  = 0x103;            // 요구사항: 자력 이벤트는 0x103으로 송신
    txh.IDE    = CAN_ID_STD;
    txh.RTR    = CAN_RTR_DATA;
    txh.DLC    = 2;                // |B| (uT) 2바이트로 전송

    uint8_t data[2];
    if (b_uT > 65535) b_uT = 65535;
    data[0] = (uint8_t)(b_uT >> 8);    // MSB
    data[1] = (uint8_t)(b_uT & 0xFF);  // LSB

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        uint32_t mailbox;
        (void)HAL_CAN_AddTxMessage(&hcan1, &txh, data, &mailbox);
    }
}

static inline void TOF_ReadOnce(void)
{
    VL53L0X_RangingMeasurementData_t m;
    if (VL53L0X_GetRangingMeasurementData(&tof, &m) == VL53L0X_ERROR_NONE) {

        g_last_dist_mm = (uint16_t)m.RangeMilliMeter;

        // 유효 측정(status==0) && 임계값 이하면 CAN 송신
        if (m.RangeStatus == 0 && g_last_dist_mm <= TOF_CAN_THRESH_MM) {
            CAN_SendVL53(g_last_dist_mm - g_tof_offset_mm);
        }

        // (선택) 디버그
        printf("dist=%u mm (status=%u)\r\n",
               (unsigned)g_last_dist_mm - g_tof_offset_mm , (unsigned)m.RangeStatus);

        // 다음 측정을 위해 반드시 클리어
        VL53L0X_ClearInterruptMask(&tof, 0);
    }
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
