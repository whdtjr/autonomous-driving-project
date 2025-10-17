#include "esp.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart6;   /* ESP8266 AT */
extern UART_HandleTypeDef huart2;   /* Debug UART */

static inline void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

static void uart_send(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len)
{
    (void)HAL_UART_Transmit(huart, (uint8_t*)data, len, 1000);
}

static int uart_recv_some(UART_HandleTypeDef* huart, uint8_t* buf, size_t maxread, uint32_t timeout_ms)
{
    size_t got = 0;
    uint32_t t0 = HAL_GetTick();
    while (got < maxread)
    {
        uint32_t slice = (timeout_ms == 0) ? 10U : ((timeout_ms > 10U) ? 10U : timeout_ms);
        if (HAL_UART_Receive(huart, &buf[got], 1, slice) == HAL_OK)
        {
            got++;
        }
        else
        {
            if (timeout_ms == 0U)
            {
                break;
            }
            if ((HAL_GetTick() - t0) >= timeout_ms)
            {
                break;
            }
        }
    }
    return (int)got;
}

void dbg(const char* s)
{
    if (!s)
    {
        return;
    }
    uart_send(&huart2, (const uint8_t*)s, (uint16_t)strlen(s));
}

void dbgf(const char* fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    dbg(buf);
}

static bool wait_for_any_token(uint32_t timeout_ms, const char* const* expects, size_t count)
{
    static uint8_t rx[1024];
    size_t wr = 0;
    uint32_t t0 = HAL_GetTick();

    bool has_expect = false;
    for (size_t i = 0; expects && i < count; ++i)
    {
        if (expects[i] && *expects[i])
        {
            has_expect = true;
            break;
        }
    }

    const char* err1 = "ERROR";
    const char* err2 = "FAIL";
    const char* err3 = "CLOSED";
    const char* err4 = "link is not valid";

    while ((HAL_GetTick() - t0) < timeout_ms && (wr + 1) < sizeof(rx))
    {
        int n = uart_recv_some(&huart6, rx + wr, sizeof(rx) - 1U - wr, 20U);
        if (n > 0)
        {
            wr += (size_t)n;
            rx[wr] = 0;

#ifdef ESP_DEBUG_ECHO
            /* ?òÏã†???êÎ¨∏???îÎ≤ÑÍ∑?UARTÎ°?Í∑∏Î?Î°??êÏΩî */
            uart_send(&huart2, rx + (wr - (size_t)n), (uint16_t)n);
#endif

            if (strstr((char*)rx, err1) || strstr((char*)rx, err2) ||
                strstr((char*)rx, err3) || strstr((char*)rx, err4))
            {
                return false;
            }

            if (has_expect)
            {
                for (size_t i = 0; i < count; ++i)
                {
                    const char* token = expects[i];
                    if (token && *token && strstr((char*)rx, token) != NULL)
                    {
                        return true;
                    }
                }
            }
        }
    }

    if (!has_expect)
    {
        return true;
    }
    return false;
}

static bool esp_cmd_expect_any(const char* cmd,
                               const char* const* expects,
                               size_t expect_count,
                               uint32_t timeout_ms)
{
    if (cmd && *cmd)
    {
        uint8_t ch;
        while (HAL_UART_Receive(&huart6, &ch, 1, 2) == HAL_OK)
        {
            /* flush */
        }
        uart_send(&huart6, (const uint8_t*)cmd, (uint16_t)strlen(cmd));
    }

    bool ok = wait_for_any_token(timeout_ms, expects, expect_count);
#ifdef ESP_DEBUG_ECHO
    if (cmd)
    {
        dbgf("[ESP]> %s", cmd);
    }
    dbgf("[ESP]=> %s\r\n", ok ? "OK" : "FAIL");
#endif
    return ok;
}

bool esp_cmd(const char* cmd, const char* expect, uint32_t timeout_ms)
{
    if (expect && *expect)
    {
        const char* expects[] = { expect };
        return esp_cmd_expect_any(cmd, expects, 1, timeout_ms);
    }
    return esp_cmd_expect_any(cmd, NULL, 0, timeout_ms);
}

static void esp_escape_transparent(void)
{
    HAL_Delay(1100);
    const uint8_t pluses[3] = { '+', '+', '+' };
    HAL_UART_Transmit(&huart6, (uint8_t*)pluses, 3, 1000);
    HAL_Delay(1100);
    uint8_t ch;
    while (HAL_UART_Receive(&huart6, &ch, 1, 10) == HAL_OK)
    {
    }
}

#ifndef ESP_HAS_GPIO_RESET
#define ESP_HAS_GPIO_RESET 0
#endif

static void esp_hw_reset(void)
{
#if ESP_HAS_GPIO_RESET
    // HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(120);
    // HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
#endif
    HAL_Delay(1500);
    uint8_t ch;
    while (HAL_UART_Receive(&huart6, &ch, 1, 5) == HAL_OK)
    {
    }
}

static bool set_uart1_baud(uint32_t baud)
{
    huart6.Init.BaudRate = baud;
    if (HAL_UART_DeInit(&huart6) != HAL_OK)
    {
        return false;
    }
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        return false;
    }
    return true;
}

static bool esp_autobaud(void)
{
    /* Try default 115200 first */
    (void)set_uart1_baud(115200);
    for (int i = 0; i < 3; ++i)
    {
        if (esp_cmd("AT\r\n", "OK", 300))
        {
            return true;
        }
        HAL_Delay(80);
    }

    /* Try common fallback 9600 */
    if (!set_uart1_baud(9600))
    {
        return false;
    }
    for (int i = 0; i < 5; ++i)
    {
        if (esp_cmd("AT\r\n", "OK", 400))
        {
            /* Switch module to 115200 and revert UART1 back */
            (void)esp_cmd("AT+UART_CUR=115200,8,1,0,0\r\n", "OK", 800);
            HAL_Delay(50);
            (void)set_uart1_baud(115200);
            /* confirm */
            (void)esp_cmd("AT\r\n", "OK", 400);
            return true;
        }
        HAL_Delay(100);
    }
    return false;
}

bool esp_is_ready(void)
{
    return esp_cmd("AT\r\n", "OK", 500);
}

void esp_flush_rx(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart6, &ch, 1, 5) == HAL_OK)
    {
    }
}

bool esp_init(void)
{
    dbgf("[ESP] init...\r\n");

    esp_escape_transparent();
    esp_hw_reset();

#if ESP_ENABLE_AUTOBAUD
    if (!esp_autobaud())
    {
        dbgf("[ESP] no AT response (after RST)\r\n");
        return false;
    }
    goto ok_comm;
#else
    for (int i = 0; i < 3; ++i)
    {
        (void)esp_cmd("ATE0\r\n", "OK", 300);
        if (esp_cmd("AT\r\n", "OK", 400))
        {
            break;
        }
        HAL_Delay(100);
    }

    for (int i = 0; i < 5; ++i)
    {
        if (esp_cmd("AT\r\n", "OK", 400))
        {
            goto ok_comm;
        }
        HAL_Delay(120);
    }
    dbgf("[ESP] no AT response (after RST)\r\n");
    return false;
#endif

ok_comm:
    (void)esp_cmd("ATE0\r\n", "OK", 300);
    if (!esp_cmd("AT+CWMODE=1\r\n", "OK", 1000))
    {
        HAL_Delay(100);
        if (!esp_cmd("AT+CWMODE=1\r\n", "OK", 1000))
        {
            dbgf("[ESP] CWMODE fail\r\n");
            return false;
        }
    }

    if (!esp_cmd("AT+CIPMUX=0\r\n", "OK", 1000))
    {
        HAL_Delay(100);
        if (!esp_cmd("AT+CIPMUX=0\r\n", "OK", 1000))
        {
            dbgf("[ESP] CIPMUX fail\r\n");
            return false;
        }
    }

    if (!esp_cmd("AT+CIPMODE=0\r\n", "OK", 1000))
    {
        HAL_Delay(100);
        if (!esp_cmd("AT+CIPMODE=0\r\n", "OK", 1000))
        {
            dbgf("[ESP] CIPMODE fail\r\n");
            return false;
        }
    }
    return true;
}

bool esp_join_ap(const char* ssid, const char* pass)
{
    if (!esp_init())
    {
        return false;
    }

    char cmd[200];
    dbgf("[ESP] join \"%s\"...\r\n", ssid);

    /* DHCP ?§ÌÖå?¥ÏÖò Î™®Îìú ?úÏÑ± (Î≤†Ïä§???êÌè¨?? */
    (void)esp_cmd("AT+CWDHCP=1,1\r\n", "OK", 500);
    /* ?¥Ï†Ñ ?∞Í≤∞ ?ïÎ¶¨ Î∞??êÎèô?∞Í≤∞ Î∞©Ï? */
    (void)esp_cmd("AT+CWQAP\r\n", "OK", 1000);
    (void)esp_cmd("AT+CWAUTOCONN=0\r\n", "OK", 500);

    /* Í∏∞Î≥∏ Ï°∞Ïù∏ 2???¨Ïãú??(?ºÎ? AP?êÏÑú ?úÍ∞Ñ????Í±∏Î¶º) */
    bool joined = false;
    for (int attempt = 1; attempt <= 2 && !joined; ++attempt)
    {
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
        if (esp_cmd(cmd, "OK", 30000))
        {
            joined = true;
            break;
        }
        dbgf("[ESP] CWJAP try %d fail\r\n", attempt);
    }
    if (!joined)
    {
        dbgf("[ESP] CWJAP fail\r\n");
        (void)esp_cmd("AT+CWJAP?\r\n", "OK", 600);
        (void)esp_cmd("AT+CIPSTATUS\r\n", "STATUS", 600);

#ifdef WIFI_BSSID
        /* ?§Ï∫î ?ÜÏù¥ BSSID/Ï±ÑÎÑê Í≥†Ï†ï ?¨Ïãú??(?¨Ïö©?êÍ? esp.h???ïÏùò??Í∞??¨Ïö©) */
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\",\"%s\",%u\r\n",
                 ssid, pass, WIFI_BSSID, (unsigned)WIFI_CHANNEL);
        dbgf("[ESP] retry with BSSID %s ch%u...\r\n", WIFI_BSSID, (unsigned)WIFI_CHANNEL);
        if (esp_cmd(cmd, "OK", 25000))
        {
            dbgf("[ESP] AP OK\r\n");
            return true;
        }
#endif
        /* ?êÎèô ?§Ï∫î Í∏∞Î∞ò BSSID/Ï±ÑÎÑê Í≥†Ï†ï 1???¨Ïãú??*/
        {
            uint8_t buf[1024]; size_t wr = 0; uint32_t t0 = HAL_GetTick();
            /* flush */ uint8_t ch; while (HAL_UART_Receive(&huart6, &ch, 1, 5) == HAL_OK) {}
            snprintf((char*)buf, sizeof(buf), "AT+CWLAP=\"%s\"\r\n", ssid);
            uart_send(&huart6, buf, (uint16_t)strlen((char*)buf));
            /* read for up to 6s */
            while ((HAL_GetTick() - t0) < 6000 && (wr + 1) < sizeof(buf))
            {
                int n = uart_recv_some(&huart6, buf + wr, sizeof(buf) - 1U - wr, 50);
                if (n > 0) { wr += (size_t)n; buf[wr] = 0; }
                if (strstr((char*)buf, "OK") || strstr((char*)buf, "ERROR")) break;
            }
            buf[wr] = 0;
            /* ?åÏã±: +CWLAP:(ecn,"ssid",rssi,"bssid",channel,...) */
            const char* p = (const char*)buf;
            const char* best = NULL; long best_rssi = -1000; char best_bssid[20] = {0}; int best_ch = 0;
            while ((p = strstr(p, "+CWLAP:(")) != NULL)
            {
                const char* q1 = strchr(p, '"'); if (!q1) { p += 8; continue; }
                const char* q2 = strchr(q1 + 1, '"'); if (!q2) { p += 8; continue; }
                size_t sslen = (size_t)(q2 - q1 - 1);
                if (sslen == strlen(ssid) && strncmp(q1 + 1, ssid, sslen) == 0)
                {
                    /* find RSSI after closing quote and comma */
                    const char* prssi = strchr(q2 + 1, ','); if (!prssi) { p = q2 + 1; continue; }
                    long rssi = strtol(prssi + 1, NULL, 10);
                    /* find bssid in quotes next */
                    const char* qb1 = strchr(q2 + 1, '"'); if (!qb1) { p = q2 + 1; continue; }
                    const char* qb2 = strchr(qb1 + 1, '"'); if (!qb2) { p = q2 + 1; continue; }
                    size_t blen = (size_t)(qb2 - qb1 - 1);
                    char bssid[20] = {0}; if (blen < sizeof(bssid)) { memcpy(bssid, qb1 + 1, blen); bssid[blen] = 0; }
                    /* channel after bssid closing quote comma */
                    const char* pch = strchr(qb2 + 1, ','); if (!pch) { p = qb2 + 1; continue; }
                    int chn = atoi(pch + 1);
                    if (rssi > best_rssi && bssid[0] != 0 && chn > 0)
                    {
                        best_rssi = rssi; best = p; strncpy(best_bssid, bssid, sizeof(best_bssid) - 1); best_ch = chn;
                    }
                }
                p = q2 + 1;
            }
            if (best)
            {
                dbgf("[ESP] retry scan BSSID %s ch%d...\r\n", best_bssid, best_ch);
                snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\",\"%s\",%d\r\n", ssid, pass, best_bssid, best_ch);
                if (esp_cmd(cmd, "OK", 25000))
                {
                    dbgf("[ESP] AP OK\r\n");
                    return true;
                }
            }
        }
        return false;
    }

    dbgf("[ESP] AP OK\r\n");
    return true;
}

bool esp_tcp_connect(const char* ip, uint16_t port)
{
    (void)esp_cmd("AT+CIPMUX=0\r\n", "OK", 500);
    (void)esp_cmd("AT+CIPCLOSE\r\n", "OK", 500);
    (void)esp_cmd("AT+CIPSTATUS\r\n", "STATUS", 500);

    char ping_cmd[64];
    snprintf(ping_cmd, sizeof(ping_cmd), "AT+PING=\"%s\"\r\n", ip);
    if (esp_cmd(ping_cmd, "OK", 4000))
    {
        dbgf("[ESP] PING %s OK\r\n", ip);
    }
    else
    {
        dbgf("[ESP] PING %s FAIL\r\n", ip);
    }
    (void)esp_cmd(ping_cmd, "OK", 2000);

    bool connected = false;
    for (int attempt = 1; attempt <= 3 && !connected; ++attempt)
    {
        dbgf("[ESP] TCP connect %s:%u (try %d)...\r\n", ip, (unsigned)port, attempt);

        char cmd[96];
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", ip, (unsigned)port);

        const char* ok_tokens[] = { "CONNECT", "ALREADY CONNECTED", "Linked", "OK" };
        if (esp_cmd_expect_any(cmd, ok_tokens, sizeof(ok_tokens) / sizeof(ok_tokens[0]), 10000))
        {
            (void)esp_cmd(NULL, "OK", 300);
            connected = true;
            break;
        }

        dbgf("[ESP] CIPSTART not CONNECT; try close & retry\r\n");
        (void)esp_cmd("AT+CIPCLOSE\r\n", "OK", 500);
        delay_ms(300);
    }

    if (!connected)
    {
        dbgf("[ESP] CIPSTART fail\r\n");
        return false;
    }

    if (!esp_cmd("AT+CIPSEND\r\n", ">", 2000))
    {
        dbgf("[ESP] CIPSEND prompt fail\r\n");
        (void)esp_cmd("AT+CIPCLOSE\r\n", "OK", 500);
        return false;
    }

    dbgf("[ESP] TCP OK\r\n");
    return true;
}

bool esp_tcp_close(void)
{
    esp_escape_transparent();

    if (!esp_cmd("AT+CIPCLOSE\r\n", "OK", 2000))
    {
        dbgf("[ESP] CIPCLOSE fail (ignored)\r\n");
    }
    return true;
}

bool esp_send_raw(const uint8_t* buf, size_t len)
{
    if (!buf || len == 0)
    {
        return true;
    }
    uart_send(&huart6, buf, (uint16_t)len);
    return true;
}

int esp_recv_raw(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    if (!buf || len == 0)
    {
        return 0;
    }
    return uart_recv_some(&huart6, buf, len, timeout_ms);
}

