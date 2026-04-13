/**
 * @file  tof.h
 * @brief Thin wrapper around the VL53L1X ULD driver
 *
 * I2C address: 0x52 (8-bit)
 * XSHUT pin:   GPIOD, pin 15  (active-low, defined in main.h)
 * Distance mode: Long (up to ~4 m)
 * Timing budget: 50 ms
 */

#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "VL53L1X_api.h"

#define VL53L1X_DEV_ADDR  0x52   /* 8-bit I2C address */

typedef struct {
    uint16_t distance_mm;
    uint8_t  range_status;    /* 0 = valid range */
} TOF_Data_t;

/**
 * @brief  Initialise the VL53L1X.
 *         Toggles XSHUT, loads default config, sets 50 ms budget.
 * @param  hi2c  HAL I2C handle
 * @return 0 on success
 */
int8_t TOF_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read the latest measurement (non-blocking poll).
 *         Returns 1 if new data was ready, 0 if not ready yet, <0 on error.
 */
int8_t TOF_Read(TOF_Data_t *out);

/** @brief  Print ToF data over UART. */
void TOF_Print(const TOF_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
