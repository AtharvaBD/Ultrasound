#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// One reading in physical units, so callers never touch registers or raw counts
typedef struct {
    float ax, ay, az;      /* g   */
    float gx, gy, gz;      /* dps */
    float mx, my, mz;      /* uT  (only meaningful if mag_valid) */
    bool  mag_valid;
} imu_sample_t;

// Sets up the I2C bus, verifies the chip, and configures mode/ranges/events
esp_err_t imu_init(void);

// Reads accel, gyro and (if present) mag, scaled to physical units
esp_err_t imu_read_sample(imu_sample_t *out);

// True if the attached model has a magnetometer (9DOF / 10DOF)
bool imu_has_mag(void);

// Reads the on-chip event registers (step count, tap type, orientation)
esp_err_t imu_read_events(uint32_t *steps, uint16_t *tap, uint16_t *orient);

#ifdef __cplusplus
}
#endif