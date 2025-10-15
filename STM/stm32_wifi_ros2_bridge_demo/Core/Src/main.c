#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* micro-ROS 앱 엔트리 */
void microros_app_run(void);

/* CubeMX가 생성한 외부 선언들 */
UART_HandleTypeDef huart6;   /* ESP8266 AT (USART6 on PC6/PC7) */
UART_HandleTypeDef huart2;   /* Debug (선택) */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();

  /* 여기서부터 micro-ROS 애플리케이션 */
  microros_app_run();

  while (1) { /* NOTREACHED */ }
}

/* ====== 이하 CubeMX 기본 템플릿에 맞춰 작성 ====== */
static void SystemClock_Config(void)
{
  /* 네가 쓰던 설정 유지(CubeMX 생성값 사용). 필요 시 기존 코드 그대로 둠 */
}

static void MX_GPIO_Init(void)
{
  /* GPIO 초기화: CubeMX 생성값 */
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
