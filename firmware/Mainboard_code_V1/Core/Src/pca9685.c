/**
 * @file  pca9685.c
 * @brief PCA9685 16-channel PWM driver implementation
 *
 * Internal oscillator: 25 MHz (typical)
 * PWM resolution:      12 bit → 4096 ticks per period
 *
 * Prescale formula:  round(25 000 000 / (4096 × freq_hz)) − 1
 *
 * Pulse-to-tick:     tick = pulse_us × 4096 × freq_hz / 1 000 000
 */

#include "pca9685.h"
#include <math.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef write_reg(PCA9685_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, buf, 2, 10);
}

static HAL_StatusTypeDef read_reg(PCA9685_t *dev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, &reg, 1, 10);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->addr, val, 1, 10);
}

/* ── Public functions ───────────────────────────────────────────────────── */

HAL_StatusTypeDef PCA9685_Init(PCA9685_t *dev)
{
    HAL_StatusTypeDef s;

    /* Default pulse limits if not set by caller */
    if (dev->min_us == 0) dev->min_us = PCA9685_SERVO_MIN_US;
    if (dev->max_us == 0) dev->max_us = PCA9685_SERVO_MAX_US;

    /* Software reset — write MODE1 SLEEP to this device only */
    write_reg(dev, PCA9685_MODE1, PCA9685_MODE1_SLEEP);
    HAL_Delay(10);

    /* Enable auto-increment, clear sleep bit */
    s = write_reg(dev, PCA9685_MODE1, PCA9685_MODE1_AI);
    if (s != HAL_OK) return s;

    /* MODE2: output change on STOP command, totem-pole outputs */
    s = write_reg(dev, PCA9685_MODE2, 0x04);
    if (s != HAL_OK) return s;

    /* Set frequency (defaults to 50 Hz for servos) */
    float freq = (dev->freq_hz > 0.0f) ? dev->freq_hz : 50.0f;
    return PCA9685_SetFrequency(dev, freq);
}

HAL_StatusTypeDef PCA9685_SetFrequency(PCA9685_t *dev, float freq_hz)
{
    HAL_StatusTypeDef s;
    uint8_t old_mode, prescale;

    /* prescale = round(25 MHz / (4096 × freq)) − 1 */
    prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * freq_hz)) - 1.0f);

    /* Read current MODE1 so we can restore non-sleep bits */
    s = read_reg(dev, PCA9685_MODE1, &old_mode);
    if (s != HAL_OK) return s;

    /* Enter sleep (oscillator off) to allow prescale write */
    s = write_reg(dev, PCA9685_MODE1, (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    s = write_reg(dev, PCA9685_PRESCALE, prescale);
    if (s != HAL_OK) return s;

    /* Restore mode (clear sleep) */
    s = write_reg(dev, PCA9685_MODE1, old_mode & ~PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    HAL_Delay(5); /* oscillator stabilise */

    /* Set RESTART bit to resume PWM */
    s = write_reg(dev, PCA9685_MODE1,
                  (old_mode & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);

    dev->freq_hz = freq_hz;
    return s;
}

HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_t *dev, uint8_t ch,
                                   uint16_t on, uint16_t off)
{
    uint8_t buf[5];
    buf[0] = PCA9685_LED0_ON_L + (4u * ch);
    buf[1] = (uint8_t)(on  & 0xFF);
    buf[2] = (uint8_t)(on  >> 8);
    buf[3] = (uint8_t)(off & 0xFF);
    buf[4] = (uint8_t)(off >> 8);
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, buf, 5, 10);
}

HAL_StatusTypeDef PCA9685_SetServoPulse(PCA9685_t *dev, uint8_t ch,
                                          uint16_t pulse_us)
{
    /* Clamp to configured limits */
    if (pulse_us < dev->min_us) pulse_us = dev->min_us;
    if (pulse_us > dev->max_us) pulse_us = dev->max_us;

    /* Convert µs → 12-bit tick count */
    uint16_t off = (uint16_t)((pulse_us * 4096.0f * dev->freq_hz) / 1000000.0f);

    return PCA9685_SetPWM(dev, ch, 0, off);
}

HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *dev, uint8_t ch,
                                          float angle_deg)
{

//	if (dev->addr == PCA9685_ADDR_LEFT) {
//	    if      (ch == 1 || ch == 5 || ch == 7) angle_deg = 90.0f - angle_deg;
//	    else if (ch == 2 || ch == 4 || ch == 8) angle_deg = 147.0f - angle_deg;
//	    else if (ch == 3 || ch == 6 || ch == 9) angle_deg = 144.0f - angle_deg;
//	    else return HAL_ERROR;
//	}
//	else if (dev->addr == PCA9685_ADDR_RIGHT) {
//	    if      (ch == 1 || ch == 5 || ch == 7) angle_deg = 90.0f - angle_deg;
//	    else if (ch == 2 || ch == 4 || ch == 8) angle_deg = 93.0f - angle_deg;
//	    else if (ch == 3 || ch == 6 || ch == 9) angle_deg = 36.0f - angle_deg;
//	    else return HAL_ERROR;
//	}
//	else return HAL_ERROR;

	if (angle_deg < 0.0f)   angle_deg = 0.0f;
	if (angle_deg > 180.0f) angle_deg = 180.0f;

    uint16_t pulse_us = (uint16_t)(dev->min_us + (angle_deg / 180.0f) * (dev->max_us - dev->min_us));

    return PCA9685_SetServoPulse(dev, ch, pulse_us);
}

void PCA9685_Sleep(PCA9685_t *dev)
{
    uint8_t mode;
    if (read_reg(dev, PCA9685_MODE1, &mode) == HAL_OK)
        write_reg(dev, PCA9685_MODE1, mode | PCA9685_MODE1_SLEEP);
}

void PCA9685_Wake(PCA9685_t *dev)
{
    uint8_t mode;
    if (read_reg(dev, PCA9685_MODE1, &mode) == HAL_OK) {
        write_reg(dev, PCA9685_MODE1, mode & ~PCA9685_MODE1_SLEEP);
        HAL_Delay(5);
    }
}
