// microros_app.c
#include "uxr/client/client.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"
#include "uxr/client/util/ping.h"

#include "microros_transport_esp.h"
#include "esp.h"
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifndef STM32_UID_BASE
#define STM32_UID_BASE  (0x1FFF7A10UL) // STM32F4(F411 ?? Í∏∞Ï?. MCU ?ºÏù∏ ?§Î•¥Î©??∞Ïù¥?∞Ïãú??Ï£ºÏÜåÎ°?Î≥ÄÍ≤?
#endif

static uint32_t make_client_key_from_uid(void)
{
    volatile const uint32_t* UID = (const uint32_t*)STM32_UID_BASE;
    uint32_t a = UID[0], b = UID[1], c = UID[2];
    uint32_t key = (a ^ (b << 1) ^ (c << 2)) ^ 0xA5A5A5A5u;
    if (key == 0) key = 0x5A5A5A5Au; // ?îÎ≤ÑÍπ??∏ÏùòÎ•??ÑÌïú 0 ?åÌîº
    return key;
}

void microros_app_run(void)
{
    dbgf("[uXRCE] app start\r\n");

    // ¥Ÿºˆ ∫∏µÂ µøΩ√ ¡¢º” Ω√ »•¿‚ øœ»≠øÎ ¡ˆ≈Õ ¥Î±‚
    uint32_t jitter_key = make_client_key_from_uid();
    HAL_Delay((jitter_key & 0x7FFu)); // 0~2047ms ?úÎç§ ÏßÄ??
    uxrCustomTransport transport = {0};
    if (!uxr_custom_transport_init_compat(&transport)) {
        dbgf("[uXRCE] custom transport init failed\r\n");
        return;
    }
    dbgf("[uXRCE] custom transport init OK (UDP %s:%u)\r\n", AGENT_IP, (unsigned)AGENT_PORT);

    HAL_Delay(300); // settle before ping/session
    bool agent_alive = uxr_ping_agent_attempts(&transport.comm, 1000, 5);
    dbgf("[uXRCE] agent ping: %s\r\n", agent_alive ? "OK" : "FAIL");
    // agent ping may fail; proceed to try creating session

    static uint8_t out_buf[1024];
    static uint8_t in_buf [1024];

    uxrSession session;
    uint32_t client_key = make_client_key_from_uid();
    dbgf("[uXRCE] CLIENT_KEY=0x%08lX\r\n", (unsigned long)client_key);
    uxr_init_session(&session, &transport.comm, client_key);

    bool created = false;
    for (int attempt = 1; attempt <= 5 && !created; ++attempt)
    {
        dbgf("[uXRCE] create_session try %d...\r\n", attempt);
        if (uxr_create_session(&session))
        {
            created = true;
            break;
        }
        HAL_Delay(400);
    }
    if (!created) {
        dbgf("[uXRCE] create_session FAILED (UDP)\r\n");
        (void)uxr_close_custom_transport(&transport);
        dbgf("[uXRCE] fallback to TCP transport (run: micro-ros-agent tcp4 --port %u)\r\n", (unsigned)AGENT_PORT);
        if (!uxr_custom_transport_init_tcp(&transport)) {
            dbgf("[uXRCE] TCP transport init failed\r\n");
            return;
        }
        created = false;
        for (int attempt = 1; attempt <= 5 && !created; ++attempt) {
            dbgf("[uXRCE][TCP] create_session try %d...\r\n", attempt);
            if (uxr_create_session(&session)) { created = true; break; }
            HAL_Delay(500);
        }
        if (!created) {
            dbgf("[uXRCE] create_session FAILED (TCP)\r\n");
            (void)uxr_close_custom_transport(&transport);
            return;
        }
        dbgf("[uXRCE] create_session OK (TCP)\r\n");
    }

    for (;;) {
        uxr_run_session_time(&session, 50);
    }
}

