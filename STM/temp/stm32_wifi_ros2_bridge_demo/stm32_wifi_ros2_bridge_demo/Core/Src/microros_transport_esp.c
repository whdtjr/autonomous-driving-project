// microros_transport_esp.c (UDP transport)
#include "microros_transport_esp.h"
#include "esp.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;   // ESP8266 AT
extern UART_HandleTypeDef huart2;   // Debug (?†ÌÉù)

// ===== ?†Ìã∏ =====
static void uart_send_str(UART_HandleTypeDef* huart, const char* s)
{
    HAL_UART_Transmit(huart, (uint8_t*)s, (uint16_t)strlen(s), 1000);
}

static void uart_send_data(UART_HandleTypeDef* huart, const uint8_t* data, size_t len)
{
    if (len) HAL_UART_Transmit(huart, (uint8_t*)data, (uint16_t)len, 1000);
}

static void flush_esp(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t ch;
    while ((HAL_GetTick() - t0) < ms)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 5) != HAL_OK) break;
    }
}

static bool wait_for_token_any(uint32_t timeout_ms, const char* tok1, const char* tok2)
{
    char window[32] = {0};
    size_t wlen = 0;
    uint32_t t0 = HAL_GetTick();
    uint8_t ch;

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK)
        {
            if (wlen < sizeof(window) - 1) window[wlen++] = (char)ch;
            else
            {
                memmove(window, window + 1, sizeof(window) - 2);
                window[sizeof(window) - 2] = (char)ch;
            }
            window[wlen] = '\0';

            if ((tok1 && strstr(window, tok1)) || (tok2 && strstr(window, tok2)))
                return true;

            if (strstr(window, "ERROR") || strstr(window, "FAIL") || strstr(window, "CLOSED"))
                return false;
        }
    }
    return false;
}

static bool wait_for_token(const char* token, uint32_t timeout_ms)
{
    return wait_for_token_any(timeout_ms, token, NULL);
}

static bool at_ok(const char* cmd, uint32_t timeout_ms)
{
    flush_esp(10);
    uart_send_str(&huart1, cmd);
    uart_send_str(&huart1, "\r\n");
    return wait_for_token("OK", timeout_ms);
}

// +IPD,<len>: ?åÏÑú ???îÏ≤≠??Í∏∏Ïù¥ÍπåÏ? ?¥Ïñ¥Î∂ôÏûÑ
static int recv_ipd(uint8_t* out, size_t want_len, uint32_t overall_timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    size_t total = 0;
    const char* head = "+IPD,";
    size_t hlen = strlen(head);

    while (total < want_len && (HAL_GetTick() - t0) < overall_timeout_ms)
    {
        // "+IPD," ?ôÍ∏∞
        size_t matched = 0;
        uint8_t ch = 0;
        while (matched < hlen && (HAL_GetTick() - t0) < overall_timeout_ms)
        {
            if (HAL_UART_Receive(&huart1, &ch, 1, 10) != HAL_OK) continue;
            matched = (ch == head[matched]) ? matched + 1 : (ch == head[0] ? 1 : 0);
        }
        if (matched < hlen) break; // timeout

        // Í∏∏Ïù¥ ?ΩÍ∏∞
        char numbuf[8] = {0};
        size_t ni = 0;
        while (ni < sizeof(numbuf) - 1 && (HAL_GetTick() - t0) < overall_timeout_ms)
        {
            if (HAL_UART_Receive(&huart1, &ch, 1, 50) != HAL_OK) continue;
            if (ch == ':') break;
            if (ch >= '0' && ch <= '9') numbuf[ni++] = (char)ch;
        }
        int ipd_len = atoi(numbuf);
        if (ipd_len <= 0) continue;

        // ?∞Ïù¥???òÏã†
        while (ipd_len > 0 && (HAL_GetTick() - t0) < overall_timeout_ms)
        {
            if (HAL_UART_Receive(&huart1, &ch, 1, 50) != HAL_OK) continue;
            if (total < want_len) out[total++] = ch;
            ipd_len--;
        }
    }
    return (int)total;
}

// ===== Ïª§Ïä§?Ä ?∏Îûú?§Ìè¨??ÏΩúÎ∞± (UDP) =====
bool transport_esp_open(uxrCustomTransport* transport)
{
    (void)transport;

    // 1) AP Ï°∞Ïù∏ (Í∏∞Ï°¥ esp.c ?úÏö©) ???¥Î??êÏÑú ATE0/CWMODE ??Ï≤òÎ¶¨
    dbgf("[ESP] open: join AP \"%s\"...\r\n", WIFI_SSID);
    if (!esp_join_ap(WIFI_SSID, WIFI_PASS))
    {
        dbgf("[ESP] join AP fail\r\n");
        return false;
    }
    dbgf("[ESP] AP OK\r\n");


    // --- diag: «ˆ¿Á ø¨∞· ªÛ≈¬/¡÷º“ √‚∑¬ ---
    (void)at_ok("AT+CWJAP?", 1000);
    (void)at_ok("AT+CWSTATE?", 1000);
    (void)at_ok("AT+CIPSTA?", 1000);

    // 2) UDP Í∏∞Î≥∏ Î™®Îìú ?§Ï†ï
    //    (esp_init?êÏÑú CIPMODE=1??Ïº∞ÏùÑ ???àÏúº??Î∞òÎìú???ÑÍ∏∞)
    if (!at_ok("AT+CIPMUX=0", 1000))   return false;
    if (!at_ok("AT+CIPDINFO=0", 1000)) return false;  // +IPD,<len>:
    if (!at_ok("AT+CIPMODE=0", 1000))  return false;  // ?ºÎ∞ò Î™®Îìú

    // ¿Ã¿¸ ∏µ≈© ¡§∏Æ (¿÷¿ª ºˆ ¿÷¿∏π«∑Œ ¥›∞Ì ¡¯«‡)
    (void)at_ok("AT+CIPCLOSE", 500);

    // 3) UDP ?∏ÏÖò ?§Ìîà
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",%u", AGENT_IP, (unsigned)AGENT_PORT);
    if (!at_ok(cmd, 8000))
    {
        dbgf("[ESP] CIPSTART UDP fail\r\n");
        return false;
    }

    dbgf("[ESP] UDP OK (%s:%u)\r\n", AGENT_IP, (unsigned)AGENT_PORT);
    dbgf("[ESP] UDP OK (%s:%u)\r\n", AGENT_IP, (unsigned)AGENT_PORT);
    HAL_Delay(300);
    (void)at_ok("AT+CIPSTA?", 800);

    // ?êÏù¥?ÑÌä∏ ?ÑÎã¨???êÍ?: ICMP PING ?úÎèÑ (AT+PING)
    char ping_cmd[64];
    snprintf(ping_cmd, sizeof(ping_cmd), "AT+PING=\"%s\"", AGENT_IP);
    if (at_ok(ping_cmd, 4000))
    {
        dbgf("[ESP] PING %s OK\r\n", AGENT_IP);
    }
    else
    {
        dbgf("[ESP] PING %s FAIL\r\n", AGENT_IP);
        // ?§Ìä∏?åÌÅ¨ Í≤ΩÎ°úÍ∞Ä ÎßâÌûå Í≤ΩÏö∞ ?∏ÏÖò ?ùÏÑ± ?ÑÏóê ?§Ìå® Î∞òÌôò
        // continue despite ICMP ping failure
    }
    return true;
}

// ===== ƒøΩ∫≈“ TCP ø¿«¬ (¥Î√º ∞Ê∑Œ) =====
bool transport_esp_open_tcp(uxrCustomTransport* transport)
{
    (void)transport;
    dbgf("[ESP] open(TCP): join AP \"%s\"...\r\n", WIFI_SSID);
    if (!esp_join_ap(WIFI_SSID, WIFI_PASS))
    {
        dbgf("[ESP] join AP fail\r\n");
        return false;
    }
    dbgf("[ESP] AP OK\r\n");

    if (!at_ok("AT+CIPMUX=0", 1000))   return false;
    if (!at_ok("AT+CIPDINFO=0", 1000)) return false;
    if (!at_ok("AT+CIPMODE=0", 1000))  return false;

    (void)at_ok("AT+CIPCLOSE", 500);

    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", AGENT_IP, (unsigned)AGENT_PORT);
    if (!at_ok(cmd, 10000))
    {
        dbgf("[ESP] CIPSTART TCP fail\r\n");
        return false;
    }

    dbgf("[ESP] TCP OK (%s:%u)\r\n", AGENT_IP, (unsigned)AGENT_PORT);
    HAL_Delay(300);
    (void)at_ok("AT+CIPSTATUS", 800);
    return true;
}


bool transport_esp_close(uxrCustomTransport* transport)
{
    (void)transport;
    (void)at_ok("AT+CIPCLOSE", 2000);
    dbgf("[ESP] UDP close\r\n");
    return true;
}

size_t transport_esp_write(uxrCustomTransport* transport,
                           const uint8_t* buf, size_t len, uint8_t* errcode)
{
    (void)transport;
    if (errcode) *errcode = 0;
    if (!buf || len == 0) return 0;

    // CIPSEND=len ??'>' ?ÑÎ°¨?ÑÌä∏ ?ÄÍ∏???payload ??"SEND OK"
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", (unsigned)len);
    flush_esp(5);
    uart_send_str(&huart1, cmd);
    uart_send_str(&huart1, "\r\n");

        if (!wait_for_token_any(2000, ">", "OK")) {
        if (errcode) *errcode = 1;
        return 0;
    }

    uart_send_data(&huart1, buf, len);

    if (!wait_for_token("SEND OK", 3000))
    {
        if (errcode) *errcode = 2;
        return 0;
    }

#ifdef ESP_DEBUG_ECHO
    dbgf("[ESP][UDP] TX %u byte(s)\r\n", (unsigned)len);
#endif
    return len;
}

size_t transport_esp_read(uxrCustomTransport* transport,
                          uint8_t* buf, size_t len, int timeout_ms, uint8_t* errcode)
{
    (void)transport;
    if (errcode) *errcode = 0;
    if (!buf || len == 0) return 0;

    uint32_t to = (timeout_ms > 0) ? (uint32_t)timeout_ms : 1000U;
    int got = recv_ipd(buf, len, to);
    if (got <= 0)
    {
        if (errcode) *errcode = 1;
        return 0;
    }

#ifdef ESP_DEBUG_ECHO
    dbgf("[ESP][UDP] RX %d byte(s)\r\n", got);
#endif
    return (size_t)got;
}

// uxr Ï¥àÍ∏∞???òÌçº (?ÑÎ†à?¥Î∞ç=false ?†Ï?)

bool uxr_custom_transport_init_tcp(uxrCustomTransport* tr)
{
    memset(tr, 0, sizeof(*tr));
    uxr_set_custom_transport_callbacks(
        tr,
        false,
        transport_esp_open_tcp,
        transport_esp_close,
        transport_esp_write,
        transport_esp_read);

    if (!uxr_init_custom_transport(tr, NULL))
    {
        dbgf("[uXRCE] custom transport (TCP) init failed\r\n");
        return false;
    }
    return true;
}

bool uxr_custom_transport_init_compat(uxrCustomTransport* tr)
{
    memset(tr, 0, sizeof(*tr));
    uxr_set_custom_transport_callbacks(
        tr,
        false, // framing off (TCP/UDP?êÏÑú???ÑÎ†à?¥Î∞ç Î∂àÌïÑ??
        transport_esp_open,
        transport_esp_close,
        transport_esp_write,
        transport_esp_read);

    if (!uxr_init_custom_transport(tr, NULL))
    {
        dbgf("[uXRCE] custom transport init failed\r\n");
        return false;
    }
    return true;
}

