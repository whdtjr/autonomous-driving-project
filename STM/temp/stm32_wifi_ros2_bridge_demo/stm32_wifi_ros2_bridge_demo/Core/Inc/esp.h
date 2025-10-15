#ifndef ESP_H
#define ESP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef ESP_DEBUG_ECHO
#define ESP_DEBUG_ECHO 1
#endif

/* 자동 보레이트(autobaud) 사용: 115200 실패 시 9600 탐지 후 115200으로 복원 */
#ifndef ESP_ENABLE_AUTOBAUD
#define ESP_ENABLE_AUTOBAUD 1
#endif
/* ===== 기본 Wi-Fi / ?�이?�트 ?�정 ===== */
#ifndef WIFI_SSID
#  define WIFI_SSID  "embA"
#endif
#ifndef WIFI_PASS
#  define WIFI_PASS  "embA1234"
#endif
#ifndef AGENT_IP
#  define AGENT_IP   "10.10.16.235"
#endif
#ifndef AGENT_PORT
#  define AGENT_PORT 8888
#endif

/* ===== ?�택: ?�정 AP BSSID/채널�?고정 조인 =====
 * ?�경???�라 같�? SSID??AP가 ?�럿?�거???�책???��? ???�습?�다.
 * ?�래 매크로�? ?�의?�면 ?�반 조인 ?�패 ??BSSID+채널�??�시?�합?�다.
 * ??
 *   #define WIFI_BSSID   "70:5d:cc:d4:4a:e8"
 *   #define WIFI_CHANNEL 2
 */
#ifdef WIFI_BSSID
#  ifndef WIFI_CHANNEL
#    define WIFI_CHANNEL 0
#  endif
#endif

/* ===== ?�버�?출력 (esp.c?�서 구현) ===== */
void dbg (const char* s);
void dbgf(const char* fmt, ...);

/* ===== ESP8266 AT 고수준 API ===== */
bool    esp_init(void);
bool    esp_join_ap(const char* ssid, const char* pass);
bool    esp_tcp_connect(const char* ip, uint16_t port);
bool    esp_tcp_close(void);
bool    esp_send_raw(const uint8_t* buf, size_t len);
int     esp_recv_raw(uint8_t* buf, size_t maxlen, uint32_t timeout_ms);

#endif /* ESP_H */


