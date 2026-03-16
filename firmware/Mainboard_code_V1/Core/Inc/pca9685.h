/**
 * @file  pca9685.h
 * @brief PCA9685 16-channel PWM driver — servo-focused, C only
 *
 * Two boards are used:
 *   Right: I2C 8-bit address 0xC0
 *   Left:  I2C 8-bit address 0x80
 *
 * Command API mirrors the Arduino Servo library concept but in plain C.
 * Angles are in degrees (0–180). Pulse widths are in microseconds.
 */

#ifndef PCA9685_H
#define PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* ── I2C addresses (8-bit, already shifted) ────────────────────────────── */
#define PCA9685_ADDR_RIGHT   0xC0
#define PCA9685_ADDR_LEFT    0x80
#define PCA9685_ADDR_ALL     0xE0   /* All-Call: writes to both boards */

/* ── Servo pulse width limits (µs) ─────────────────────────────────────── */
#define PCA9685_SERVO_MIN_US  500u
#define PCA9685_SERVO_MAX_US  2500u

/* ── Internal register map ──────────────────────────────────────────────── */
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_PRESCALE      0xFE
#define PCA9685_LED0_ON_L     0x06

/* MODE1 bits */
#define PCA9685_MODE1_SLEEP   (1u << 4)
#define PCA9685_MODE1_AI      (1u << 5)
#define PCA9685_MODE1_RESTART (1u << 7)

/* ── Device handle ──────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            addr;        /* 8-bit I2C address */
    float              freq_hz;     /* actual PWM frequency after init */
    uint16_t           min_us;      /* servo minimum pulse (µs) */
    uint16_t           max_us;      /* servo maximum pulse (µs) */
} PCA9685_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the PCA9685.  Sets 50 Hz and default pulse limits.
 * @param  dev   Pointer to handle (hi2c and addr must be filled before call).
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef PCA9685_Init(PCA9685_t *dev);

/**
 * @brief  Set PWM frequency.  Device is put to sleep during prescale write.
 */
HAL_StatusTypeDef PCA9685_SetFrequency(PCA9685_t *dev, float freq_hz);

/**
 * @brief  Set raw 12-bit ON/OFF counts for one channel (0-15).
 */
HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_t *dev, uint8_t ch,
                                  uint16_t on, uint16_t off);

/**
 * @brief  Set servo angle in degrees (0–180).
 *         Maps linearly to dev->min_us … dev->max_us.
 */
HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *dev, uint8_t ch,
                                         float angle_deg);

/**
 * @brief  Set servo pulse width directly in microseconds.
 */
HAL_StatusTypeDef PCA9685_SetServoPulse(PCA9685_t *dev, uint8_t ch,
                                         uint16_t pulse_us);

/** @brief  Put device into low-power sleep mode. */
void PCA9685_Sleep(PCA9685_t *dev);

/** @brief  Wake device from sleep. */
void PCA9685_Wake(PCA9685_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_H */
