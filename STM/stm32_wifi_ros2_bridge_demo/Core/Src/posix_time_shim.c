// Core/Src/posix_time_shim.c
#include "stm32f4xx_hal.h"
#include <time.h>
#include <stdint.h>

/* 일부 툴체인에서는 CLOCK_REALTIME 매크로가 없을 수 있으니 안전하게 정의 */
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

/* STM32 HAL tick(HAL_GetTick) 기반 clock_gettime 대체 구현
   - HAL tick은 1ms 단위 (SysTick) */
int clock_gettime(int clk_id, struct timespec* ts)
{
    (void)clk_id;
    uint32_t ms = HAL_GetTick();
    ts->tv_sec  = ms / 1000U;
    ts->tv_nsec = (long)((ms % 1000U) * 1000000UL);
    return 0;
}
