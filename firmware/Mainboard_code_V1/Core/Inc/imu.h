/**
 * @file  imu.h
 * @brief Thin wrapper around the ST LSM6DSO driver (lsm6dso_reg)
 *
 * I2C address: 0xD4 (8-bit, SA0=1)
 * ODR:  104 Hz accelerometer & gyroscope
 * Range: ±2 g  /  ±250 dps
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "lsm6dso_reg.h"

#define LSM6DSO_I2C_ADDR  0xD4   /* 8-bit address, SA0 = 1 */

/** Scaled sensor output */
typedef struct {
    float accel_x_mg;   /* acceleration X in milli-g  */
    float accel_y_mg;
    float accel_z_mg;
    float gyro_x_mdps;  /* angular rate X in milli-dps */
    float gyro_y_mdps;
    float gyro_z_mdps;
} IMU_Data_t;

/**
 * @brief  Initialise the LSM6DSO.
 * @param  ctx   ST driver context (write_reg / read_reg / handle filled here)
 * @param  hi2c  HAL I2C handle
 * @return 0 on success, non-zero on error
 */
int32_t IMU_Init(stmdev_ctx_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read accelerometer and gyroscope, apply sensitivity.
 * @param  ctx  ST driver context
 * @param  out  Destination struct
 * @return 0 on success
 */
int32_t IMU_Read(stmdev_ctx_t *ctx, IMU_Data_t *out);

/**
 * @brief  Print IMU data over UART (uses printf → _write retarget).
 */
void IMU_Print(const IMU_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
