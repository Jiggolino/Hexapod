/**
 * @file  vl53l1_platform.h
 * @brief Platform I2C abstraction for VL53L1X driver
 *
 * The VL53L1X API calls these functions.
 * dev = 8-bit I2C address (0x52 for VL53L1X).
 */

#ifndef VL53L1_PLATFORM_H
#define VL53L1_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* The HAL I2C handle used by the ToF sensor — set before calling VL53L1X_SensorInit */
extern I2C_HandleTypeDef *vl53l1x_hi2c;

int8_t VL53L1_WrByte     (uint16_t dev, uint16_t reg, uint8_t  data);
int8_t VL53L1_WrWord     (uint16_t dev, uint16_t reg, uint16_t data);
int8_t VL53L1_WrDWord    (uint16_t dev, uint16_t reg, uint32_t data);
int8_t VL53L1_RdByte     (uint16_t dev, uint16_t reg, uint8_t  *data);
int8_t VL53L1_RdWord     (uint16_t dev, uint16_t reg, uint16_t *data);
int8_t VL53L1_RdDWord    (uint16_t dev, uint16_t reg, uint32_t *data);
int8_t VL53L1_ReadMulti  (uint16_t dev, uint16_t reg, uint8_t  *data, uint32_t count);
int8_t VL53L1_WriteMulti (uint16_t dev, uint16_t reg, uint8_t  *data, uint32_t count);
int8_t VL53L1_WaitMs     (uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1_PLATFORM_H */
