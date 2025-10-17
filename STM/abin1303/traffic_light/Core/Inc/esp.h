//서울기술교육센터 AIOT & Embedded System
//2024-04-16 By KSH


#ifndef __ESP_H__
#define __ESP_H__

#include "stm32f4xx_hal.h"

#define MAX_ESP_RX_BUFFER      1024
#define MAX_ESP_COMMAND_LEN    64
#define MAX_ESP_CLIENT_NUM     10

#define SSID "embB"
#define PASS "embB1234"
#define LOGID "JAB_STM"
#define PASSWD "PASSWD"
//#define DST_IP "10.10.141.86"
//#define DST_IP "192.168.0.99"
#define DST_IP "10.10.16.80"
#define DST_PORT 9000

typedef struct _cb_data_t
{
    uint8_t buf[MAX_ESP_RX_BUFFER];
    uint16_t length;
}cb_data_t;

int drv_esp_init(void);
int drv_esp_test_command(void);
void AiotClient_Init(void);
int esp_client_conn(void);
void esp_send_data(char *data);
int esp_get_status(void);

//==================uart3=========================
#define MAX_UART_RX_BUFFER      512
#define MAX_UART_COMMAND_LEN    64

int drv_uart_init(void);
int drv_uart_rx_buffer(uint8_t *buf, uint16_t size);
int drv_uart_tx_buffer(uint8_t *buf, uint16_t size);
//==================uart3=========================

#endif
