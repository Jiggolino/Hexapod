/**
 * @file  tof.c
 * @brief VL53L1X wrapper implementation
 */

#include "tof.h"
#include "vl53l1_platform.h"
#include "main.h"    /* XSHUT_Pin / XSHUT_GPIO_Port */
#include <stdio.h>

/* ── Public functions ───────────────────────────────────────────────────── */

int8_t TOF_Init(I2C_HandleTypeDef *hi2c)
{
    /* Store HAL handle for the platform layer */
    vl53l1x_hi2c = hi2c;

    /* Hardware reset via XSHUT (active-low) */
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    /* Load default 135-byte configuration and boot the sensor */
    if (VL53L1X_SensorInit(VL53L1X_DEV_ADDR) != VL53L1X_ERROR_NONE)
        return -1;

    /* Distance mode: 1 = Short (<1.3 m, better ambient rejection)
     *                2 = Long  (up to ~4 m)                         */
    if (VL53L1X_SetDistanceMode(VL53L1X_DEV_ADDR, 2) != VL53L1X_ERROR_NONE)
        return -2;

    /* Timing budget: 50 ms — good balance of speed vs. accuracy */
    if (VL53L1X_SetTimingBudgetInMs(VL53L1X_DEV_ADDR, 50) != VL53L1X_ERROR_NONE)
        return -3;

    /* Inter-measurement period must be ≥ timing budget */
    if (VL53L1X_SetInterMeasurementInMs(VL53L1X_DEV_ADDR, 55) != VL53L1X_ERROR_NONE)
        return -4;

    /* Start continuous ranging */
    if (VL53L1X_StartRanging(VL53L1X_DEV_ADDR) != VL53L1X_ERROR_NONE)
        return -5;

    return 0;
}

int8_t TOF_Read(TOF_Data_t *out)
{
    uint8_t ready = 0;

    VL53L1X_CheckForDataReady(VL53L1X_DEV_ADDR, &ready);
    if (!ready) return 0;   /* no new data yet */

    VL53L1X_Result_t result;
    if (VL53L1X_GetResult(VL53L1X_DEV_ADDR, &result) != VL53L1X_ERROR_NONE)
        return -1;

    VL53L1X_ClearInterrupt(VL53L1X_DEV_ADDR);

    out->distance_mm  = result.Distance;
    out->range_status = result.Status;
    return 1;   /* fresh data */
}

void TOF_Print(const TOF_Data_t *data)
{
    if (data->range_status == 0)
        printf("TOF | %u mm\r\n", data->distance_mm);
    else
        printf("TOF | err status=%u\r\n", data->range_status);
}
