// microros_app.c

#include "uxr/client/client.h"

#include "uxr/client/profile/transport/custom/custom_transport.h"

#include "uxr/client/util/ping.h"

#include "ucdr/microcdr.h"



#include "microros_transport_esp.h"

#include "esp.h"

#include <stdbool.h>

#include <stdint.h>

#include <stdio.h>

#include <string.h>

#include "stm32f4xx_hal.h"



#ifndef STM32_UID_BASE

#define STM32_UID_BASE  (0x1FFF7A10UL) // STM32F4(F411 xx). MCU Unique ID base address

#endif



#define TL_TOPIC_NAME       "traffic_light/state"

#define TL_DATATYPE_NAME    "std_msgs::msg::dds_::String_"

#define TL_PUBLISH_PERIOD_MS    1000u



static const uxrObjectId TL_PARTICIPANT_ID = { .id = 0x01, .type = UXR_PARTICIPANT_ID };
static const uxrObjectId TL_TOPIC_ID       = { .id = 0x02, .type = UXR_TOPIC_ID };
static const uxrObjectId TL_PUBLISHER_ID   = { .id = 0x03, .type = UXR_PUBLISHER_ID };
static const uxrObjectId TL_DATAWRITER_ID  = { .id = 0x04, .type = UXR_DATAWRITER_ID };


static const char participant_xml[] =
    "<dds>"
      "<participant>"
        "<rtps>"
          "<name>traffic_light_bridge</name>"
        "</rtps>"
      "</participant>"
    "</dds>";

static const char topic_xml[] =
    "<dds>"
      "<topic>"
        "<name>" TL_TOPIC_NAME "</name>"
        "<dataType>" TL_DATATYPE_NAME "</dataType>"
      "</topic>"
    "</dds>";

static const char publisher_xml[] =
    "<dds>"
      "<publisher/>"
    "</dds>";

static const char datawriter_xml[] =
    "<dds>"
      "<data_writer>"
        "<topic>"
          "<kind>NO_KEY</kind>"
          "<name>" TL_TOPIC_NAME "</name>"
          "<dataType>" TL_DATATYPE_NAME "</dataType>"
        "</topic>"
      "</data_writer>"
    "</dds>";

static bool create_entities(uxrSession* session, uxrStreamId reliable_out)
{
    const int timeout_ms = 5000;
    uint16_t requests[4];
    uint8_t statuses[4];
    size_t idx = 0;

    requests[idx++] = uxr_buffer_create_participant_xml(
        session, reliable_out, TL_PARTICIPANT_ID, 0, participant_xml, UXR_REPLACE);
    requests[idx++] = uxr_buffer_create_topic_xml(
        session, reliable_out, TL_TOPIC_ID, TL_PARTICIPANT_ID, topic_xml, UXR_REPLACE);
    requests[idx++] = uxr_buffer_create_publisher_xml(
        session, reliable_out, TL_PUBLISHER_ID, TL_PARTICIPANT_ID, publisher_xml, UXR_REPLACE);
    requests[idx++] = uxr_buffer_create_datawriter_xml(
        session, reliable_out, TL_DATAWRITER_ID, TL_PUBLISHER_ID, datawriter_xml, UXR_REPLACE);

    uxr_flash_output_streams(session);

    if (!uxr_run_session_until_all_status(session, timeout_ms, requests, statuses, idx))
    {
        dbgf("[uXRCE] create entities timeout\r\n");
        return false;
    }

    for (size_t i = 0; i < idx; ++i)
    {
        if (statuses[i] != UXR_STATUS_OK && statuses[i] != UXR_STATUS_OK_MATCHED)
        {
            dbgf("[uXRCE] entity creation failed (req=%u status=%u)\r\n",
                 (unsigned)requests[i], (unsigned)statuses[i]);
            return false;
        }
    }

    return true;
}

static bool publish_dummy_state(uxrSession* session, uxrStreamId reliable_out)

{

    static uint8_t sample_idx = 0;

    static const char* colors[] = { "GREEN", "YELLOW", "RED" };

    static const uint8_t durations[] = { 6u, 2u, 6u };



    const char* color = colors[sample_idx];

    uint8_t remaining = durations[sample_idx];

    sample_idx = (uint8_t)((sample_idx + 1u) % (sizeof(colors) / sizeof(colors[0])));



    char payload[32];

    int written = snprintf(payload, sizeof(payload), "%s,%u", color, (unsigned)remaining);

    if (written <= 0 || (size_t)written >= sizeof(payload))

    {

        dbgf("[uXRCE] payload build error\r\n");

        return false;

    }



    size_t serialized_size = sizeof(uint32_t) + (size_t)written + 1u;



    ucdrBuffer ub;

    (void)uxr_prepare_output_stream(session, reliable_out, TL_DATAWRITER_ID, &ub, (uint32_t)serialized_size);



    if (!ucdr_serialize_string(&ub, payload))

    {

        dbgf("[uXRCE] serialize string failed\r\n");

        return false;

    }



    uxr_flash_output_streams(session);

    dbgf("[uXRCE] publish \"%s\"\r\n", payload);

    return true;

}



static uint32_t make_client_key_from_uid(void)

{

    volatile const uint32_t* UID = (const uint32_t*)STM32_UID_BASE;

    uint32_t a = UID[0], b = UID[1], c = UID[2];

    uint32_t key = (a ^ (b << 1) ^ (c << 2)) ^ 0xA5A5A5A5u;

    if (key == 0u)

    {

        key = 0x5A5A5A5Au;

    }

    return key;

}



void microros_app_run(void)

{

    dbgf("[uXRCE] app start\r\n");



    uint32_t jitter_key = make_client_key_from_uid();

    HAL_Delay(jitter_key & 0x7FFu);



    uxrCustomTransport transport = {0};

    if (!uxr_custom_transport_init_compat(&transport))

    {

        dbgf("[uXRCE] custom transport init failed\r\n");

        return;

    }

    dbgf("[uXRCE] custom transport init OK (UDP %s:%u)\r\n", AGENT_IP, (unsigned)AGENT_PORT);



    HAL_Delay(300);

    bool agent_alive = uxr_ping_agent_attempts(&transport.comm, 1000, 5);

    dbgf("[uXRCE] agent ping: %s\r\n", agent_alive ? "OK" : "FAIL");



    static uint8_t out_buf[1024];

    static uint8_t in_buf[1024];



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



    if (!created)

    {

        dbgf("[uXRCE] create_session FAILED (UDP)\r\n");

        (void)uxr_close_custom_transport(&transport);

        dbgf("[uXRCE] fallback to TCP transport (run: micro-ros-agent tcp4 --port %u)\r\n", (unsigned)AGENT_PORT);

        if (!uxr_custom_transport_init_tcp(&transport))

        {

            dbgf("[uXRCE] TCP transport init failed\r\n");

            return;

        }



        created = false;

        for (int attempt = 1; attempt <= 5 && !created; ++attempt)

        {

            dbgf("[uXRCE][TCP] create_session try %d...\r\n", attempt);

            if (uxr_create_session(&session))

            {

                created = true;

                break;

            }

            HAL_Delay(500);

        }



        if (!created)

        {

            dbgf("[uXRCE] create_session FAILED (TCP)\r\n");

            (void)uxr_close_custom_transport(&transport);

            return;

        }

        dbgf("[uXRCE] create_session OK (TCP)\r\n");

    }



    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, out_buf, sizeof(out_buf), 1);

    uxrStreamId reliable_in  = uxr_create_input_reliable_stream(&session, in_buf, sizeof(in_buf), 1);

    (void)reliable_in;



    if (!create_entities(&session, reliable_out))

    {

        dbgf("[uXRCE] entity setup failed\r\n");

        (void)uxr_close_custom_transport(&transport);

        return;

    }

    dbgf("[uXRCE] entities ready\r\n");



    uint32_t next_publish_ms = HAL_GetTick();



    for (;;)

    {

        uxr_run_session_time(&session, 50);



        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - next_publish_ms) >= 0)

        {

            (void)publish_dummy_state(&session, reliable_out);

            next_publish_ms = now + TL_PUBLISH_PERIOD_MS;

        }

    }

}

