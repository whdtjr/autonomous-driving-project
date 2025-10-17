// Core/Src/vl53l0x_platform.c
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

#include "stm32f4xx_hal.h"
#include "main.h"   // hi2c1 선언이 여기에 있음(또는 아래 extern으로 보강)

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 사용자 환경에 맞게 조정 ---- */
#ifndef VL53L0X_I2C_TIMEOUT_MS
#define VL53L0X_I2C_TIMEOUT_MS   50U
#endif

/* 프로젝트 I2C 핸들 (필요하면 hi2c2/3 등으로 변경) */
extern I2C_HandleTypeDef hi2c1;
#define VL53_HAL_I2C   (hi2c1)

/* HAL은 8-bit 주소를 기대하므로 7-bit 주소 << 1 */
#define I2C_ADDR(dev)  ((uint16_t)((dev)->I2cDevAddr << 1))

/* ---------------- Lock / Unlock ---------------- */
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

/* ---------------- I2C Multi ---------------- */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Write(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                          pdata, (uint16_t)count, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Read(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                         pdata, (uint16_t)count, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

/* ---------------- I2C Byte ---------------- */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    if (HAL_I2C_Mem_Write(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    if (HAL_I2C_Mem_Read(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                         data, 1, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

/* ---------------- I2C Word (MSB first) ---------------- */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);
    if (HAL_I2C_Mem_Write(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                          buf, 2, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                         buf, 2, VL53L0X_I2C_TIMEOUT_MS) != HAL_OK)
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return VL53L0X_ERROR_NONE;
}

/* ---------------- I2C DWord (MSB first) ---------------- */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)(data >> 16);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFF);
    if (HAL_I2C_Mem_Write(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                          buf, 4, VL53L0X_I2C_TIMEOUT_MS) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buf[4];
    if (HAL_I2C_Mem_Read(&VL53_HAL_I2C, I2C_ADDR(Dev), index, I2C_MEMADD_SIZE_8BIT,
                         buf, 4, VL53L0X_I2C_TIMEOUT_MS) != HAL_OK)
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    *data = ((uint32_t)buf[0] << 24) |
            ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8)  |
            (uint32_t)buf[3];
    return VL53L0X_ERROR_NONE;
}

/* ---------------- I2C UpdateByte ---------------- */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t data;
    VL53L0X_Error status;

    status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE) return status;

    data = (data & AndData) | OrData;
    return VL53L0X_WrByte(Dev, index, data);
}

#ifdef __cplusplus
}
#endif
