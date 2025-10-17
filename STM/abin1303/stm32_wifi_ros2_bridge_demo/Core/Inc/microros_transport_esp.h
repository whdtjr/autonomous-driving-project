#ifndef MICROROS_TRANSPORT_ESP_H
#define MICROROS_TRANSPORT_ESP_H

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "esp.h"

/* uxr_custom_transport_init_compat:
 * 마이크로XRCE의 uxr_init_custom_transport 시그니처 변화에
 * 대응하기 위한 초기화 래퍼 */
bool uxr_custom_transport_init_compat(uxrCustomTransport* tr);
// TCP 대체 경로 초기화 (UDP가 실패할 때 사용)
bool uxr_custom_transport_init_tcp(uxrCustomTransport* tr);

/* 콜백 프로토타입 */
bool   transport_esp_open (uxrCustomTransport* transport);
bool   transport_esp_close(uxrCustomTransport* transport);
size_t transport_esp_write(uxrCustomTransport* transport,
                           const uint8_t* buf, size_t len, uint8_t* errcode);
size_t transport_esp_read (uxrCustomTransport* transport,
                           uint8_t* buf, size_t len, int timeout_ms, uint8_t* errcode);

#endif
