/**
 * @file  imu.c
 * @brief LSM6DSO wrapper implementation
 */

#include "imu.h"
#include <stdio.h>

/* ── Platform callbacks required by ST driver ───────────────────────────── */

static int32_t platform_write(void *handle, uint8_t reg,
                               const uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Write((I2C_HandleTypeDef *)handle,
                      LSM6DSO_I2C_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)buf, len, 20);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)handle,
                     LSM6DSO_I2C_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT,
                     buf, len, 20);
    return 0;
}

/* ── Public functions ───────────────────────────────────────────────────── */

int32_t IMU_Init(stmdev_ctx_t *ctx, I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;

    ctx->write_reg = platform_write;
    ctx->read_reg  = platform_read;
    ctx->handle    = hi2c;

    /* Verify device identity */
    lsm6dso_device_id_get(ctx, &whoami);
    if (whoami != LSM6DSO_ID) return -1;

    /* Reset device */
    lsm6dso_reset_set(ctx, PROPERTY_ENABLE);
    uint8_t rst = 1;
    while (rst) lsm6dso_reset_get(ctx, &rst);

    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE);

    /* Accelerometer: 104 Hz, ±2 g */
    lsm6dso_xl_data_rate_set(ctx, LSM6DSO_XL_ODR_104Hz);
    lsm6dso_xl_full_scale_set(ctx, LSM6DSO_2g);

    /* Gyroscope: 104 Hz, ±250 dps */
    lsm6dso_gy_data_rate_set(ctx, LSM6DSO_GY_ODR_104Hz);
    lsm6dso_gy_full_scale_set(ctx, LSM6DSO_250dps);

    return 0;
}

int32_t IMU_Read(stmdev_ctx_t *ctx, IMU_Data_t *out)
{
    int16_t raw[3];

    /* Accelerometer */
    lsm6dso_acceleration_raw_get(ctx, raw);
    /* Sensitivity: 0.061 mg/LSB at ±2 g */
    out->accel_x_mg = lsm6dso_from_fs2_to_mg(raw[0]);
    out->accel_y_mg = lsm6dso_from_fs2_to_mg(raw[1]);
    out->accel_z_mg = lsm6dso_from_fs2_to_mg(raw[2]);

    /* Gyroscope */
    lsm6dso_angular_rate_raw_get(ctx, raw);
    /* Sensitivity: 8.75 mdps/LSB at ±250 dps */
    out->gyro_x_mdps = lsm6dso_from_fs250_to_mdps(raw[0]);
    out->gyro_y_mdps = lsm6dso_from_fs250_to_mdps(raw[1]);
    out->gyro_z_mdps = lsm6dso_from_fs250_to_mdps(raw[2]);

    return 0;
}

void IMU_Print(const IMU_Data_t *data)
{
    printf("IMU | A: x=%.1f y=%.1f z=%.1f mg | G: x=%.1f y=%.1f z=%.1f mdps\r\n",
           data->accel_x_mg, data->accel_y_mg, data->accel_z_mg,
           data->gyro_x_mdps, data->gyro_y_mdps, data->gyro_z_mdps);
}
