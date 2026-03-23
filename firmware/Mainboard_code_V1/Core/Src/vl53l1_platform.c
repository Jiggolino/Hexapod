/**
 * @file  vl53l1_platform.c
 * @brief HAL I2C platform layer for the VL53L1X ULD driver
 *
 * The VL53L1X register map uses 16-bit register addresses.
 * All multi-byte values are big-endian on the wire.
 */

#include "vl53l1_platform.h"

/* Shared I2C handle — set by TOF_Init() before any VL53L1X call */
I2C_HandleTypeDef *vl53l1x_hi2c = NULL;

/* ── Helper: write N bytes to a 16-bit register address ────────────────── */
/* Maximum single write: 135 bytes (SensorInit default config) + 2 addr bytes */
#define WR_BUF_MAX 140

static int8_t wr_bytes(uint16_t dev, uint16_t reg,
                        const uint8_t *data, uint16_t len)
{
    uint8_t buf[WR_BUF_MAX];
    if (len + 2u > WR_BUF_MAX) return -1;

    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (uint16_t i = 0; i < len; i++) buf[2 + i] = data[i];

    HAL_StatusTypeDef s =
        HAL_I2C_Master_Transmit(vl53l1x_hi2c, (uint16_t)dev,
                                buf, (uint16_t)(len + 2u), 20);
    return (s == HAL_OK) ? 0 : -1;
}

/* ── Helper: read N bytes from a 16-bit register address ───────────────── */
static int8_t rd_bytes(uint16_t dev, uint16_t reg,
                        uint8_t *data, uint16_t len)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

    HAL_StatusTypeDef s =
        HAL_I2C_Master_Transmit(vl53l1x_hi2c, (uint16_t)dev,
                                addr, 2, 20);
    if (s != HAL_OK) return -1;

    s = HAL_I2C_Master_Receive(vl53l1x_hi2c, (uint16_t)dev,
                               data, len, 20);
    return (s == HAL_OK) ? 0 : -1;
}

/* ── Public platform functions called by VL53L1X_api.c ─────────────────── */

int8_t VL53L1_WrByte(uint16_t dev, uint16_t reg, uint8_t data)
{
    return wr_bytes(dev, reg, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t reg, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return wr_bytes(dev, reg, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t reg, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >>  8), (uint8_t)(data & 0xFF)
    };
    return wr_bytes(dev, reg, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t reg, uint8_t *data)
{
    return rd_bytes(dev, reg, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t reg, uint16_t *data)
{
    uint8_t buf[2];
    int8_t s = rd_bytes(dev, reg, buf, 2);
    if (s == 0) *data = ((uint16_t)buf[0] << 8) | buf[1];
    return s;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t reg, uint32_t *data)
{
    uint8_t buf[4];
    int8_t s = rd_bytes(dev, reg, buf, 4);
    if (s == 0)
        *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    return s;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count)
{
    return rd_bytes(dev, reg, data, (uint16_t)count);
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count)
{
    return wr_bytes(dev, reg, data, (uint16_t)count);
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    HAL_Delay((uint32_t)wait_ms);
    return 0;
}
