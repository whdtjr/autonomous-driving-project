/* vl53l0x_i2c_platform.c  —  STM32 HAL port (VL53L0X)
/*
 * Drop-in replacement for the PC/Windows version shipped in STSW-IMG005.
 * Target: STM32F4 (tested conceptually on STM32F446, HAL drivers)
 *
 * What you may need to change:
 *   - I2C handle name  (VL53L0X_I2C_HANDLE)
 *   - XSHUT/INT GPIOs  (optional; see macros below)
 *
 * Notes:
 *   - VL53L0X uses 7-bit I2C address 0x29 (HAL expects address<<1).
 *   - Register index is 8-bit.
 *   - Word/DWord accesses are big-endian on the bus.
 */

#include "stm32f4xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

/* ----------------- User configuration section ------------------ */

/* I2C instance used for VL53L0X */
extern I2C_HandleTypeDef hi2c1;
#define VL53L0X_I2C_HANDLE     hi2c1

/* HAL timeouts (ms) */
#ifndef VL53L0X_I2C_TIMEOUT_MS
#define VL53L0X_I2C_TIMEOUT_MS  100
#endif

/* Optional XSHUT/INT pins (comment out if not used) */
#if 0
#define VL53L0X_XSHUT_GPIO_Port   GPIOB
#define VL53L0X_XSHUT_Pin         GPIO_PIN_0

#define VL53L0X_INT_GPIO_Port     GPIOB
#define VL53L0X_INT_Pin           GPIO_PIN_1
#endif

/* --------------------------------------------------------------- */

#define STATUS_OK     (0)
#define STATUS_FAIL   (1)

/* HAL uses 8-bit address field => (7-bit << 1) */
static inline uint16_t devaddr_to_hal(uint8_t addr7)
{
    return (uint16_t)((uint16_t)addr7 << 1);
}

/* --------- Minimal µs delay using DWT (if available) ----------- */
static inline void delay_us_blocking(uint32_t us)
{
#if (__CORTEX_M == 4U)
    /* Use DWT cycle counter if enabled */
    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)
    {
        /* enable if needed */
        if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
        {
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = (SystemCoreClock / 1000000UL) * us;
        while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
        return;
    }
#endif
    /* Fallback: rough loop (less accurate) */
    uint32_t cycles = (SystemCoreClock / 3000000UL) * us; /* ~3 cycles/loop */
    while (cycles--) { __NOP(); }
}

/* ----------------- COMMS (init/close) -------------------------- */

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
    /* Nothing special required with HAL:
       - I2C peripheral should already be init’d by CubeMX (hi2cX.Init.Timing/ClockSpeed).
       - Optionally, you can reconfigure speed at runtime if needed.
    */
    (void)comms_type;
    (void)comms_speed_khz;
    return STATUS_OK;
}

int32_t VL53L0X_comms_close(void)
{
    /* Nothing to close for HAL I2C static instance. */
    return STATUS_OK;
}

/* ----------------- Low-level I2C primitives -------------------- */

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if (pdata == NULL || count <= 0) return STATUS_FAIL;

    if (HAL_I2C_Mem_Write(&VL53L0X_I2C_HANDLE,
                          devaddr_to_hal(address),
                          (uint16_t)index,
                          I2C_MEMADD_SIZE_8BIT,
                          pdata,
                          (uint16_t)count,
                          VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
    {
        return STATUS_OK;
    }
    return STATUS_FAIL;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if (pdata == NULL || count <= 0) return STATUS_FAIL;

    if (HAL_I2C_Mem_Read(&VL53L0X_I2C_HANDLE,
                         devaddr_to_hal(address),
                         (uint16_t)index,
                         I2C_MEMADD_SIZE_8BIT,
                         pdata,
                         (uint16_t)count,
                         VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
    {
        return STATUS_OK;
    }
    return STATUS_FAIL;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    return VL53L0X_write_multi(address, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);     /* MSB first */
    buf[1] = (uint8_t)(data & 0xFF);   /* LSB */
    return VL53L0X_write_multi(address, index, buf, 2);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)(data >> 16);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data);
    return VL53L0X_write_multi(address, index, buf, 4);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_read_multi(address, index, pdata, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    if (pdata == NULL) return STATUS_FAIL;
    uint8_t buf[2];
    if (VL53L0X_read_multi(address, index, buf, 2) != STATUS_OK) return STATUS_FAIL;
    *pdata = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1]; /* MSB first */
    return STATUS_OK;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    if (pdata == NULL) return STATUS_FAIL;
    uint8_t buf[4];
    if (VL53L0X_read_multi(address, index, buf, 4) != STATUS_OK) return STATUS_FAIL;
    *pdata = ((uint32_t)buf[0] << 24) |
             ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8)  |
             ((uint32_t)buf[3]);
    return STATUS_OK;
}

/* --------------- Platform delay / timers ----------------------- */

int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    if (wait_us <= 0) return STATUS_OK;
    delay_us_blocking((uint32_t)wait_us);
    return STATUS_OK;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    if (wait_ms <= 0) return STATUS_OK;
    HAL_Delay((uint32_t)wait_ms);
    return STATUS_OK;
}

/* --------------- Optional GPIO helpers (XSHUT/INT) ------------- */

int32_t VL53L0X_set_gpio(uint8_t level)
{
#if defined(VL53L0X_XSHUT_GPIO_Port) && defined(VL53L0X_XSHUT_Pin)
    HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port,
                      VL53L0X_XSHUT_Pin,
                      (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
    return STATUS_OK;
#else
    (void)level;
    return STATUS_OK; /* No-op if not wired */
#endif
}

int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
#if defined(VL53L0X_INT_GPIO_Port) && defined(VL53L0X_INT_Pin)
    if (!plevel) return STATUS_FAIL;
    *plevel = (uint8_t)HAL_GPIO_ReadPin(VL53L0X_INT_GPIO_Port, VL53L0X_INT_Pin);
    return STATUS_OK;
#else
    if (plevel) *plevel = 0;
    return STATUS_OK; /* No-op */
#endif
}

int32_t VL53L0X_release_gpio(void)
{
    /* If you temporarily forced lines, release here. Not needed on HAL */
    return STATUS_OK;
}

int32_t VL53L0X_cycle_power(void)
{
#if defined(VL53L0X_XSHUT_GPIO_Port) && defined(VL53L0X_XSHUT_Pin)
    /* Toggle XSHUT low→high with small delay */
    HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    return STATUS_OK;
#else
    return STATUS_OK; /* No-op if not wired */
#endif
}

/* Optional — map to HAL tick if needed */
int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
    if (!ptimer_freq_hz) return STATUS_FAIL;
    *ptimer_freq_hz = 1000; /* HAL tick is 1 kHz */
    return STATUS_OK;
}

int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
    if (!ptimer_count) return STATUS_FAIL;
    *ptimer_count = (int32_t)HAL_GetTick(); /* ms tick */
    return STATUS_OK;
}

/* ----------------- ST API hook (PollingDelay) ------------------ */
/* The ST API calls this between polls; keep ~1 ms granularity. */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}
