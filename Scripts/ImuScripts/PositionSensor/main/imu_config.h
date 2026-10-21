#pragma once

/* Board wiring and tuning. Edit these, not the driver code. */

// ---- Board / wiring ----
#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_SDA_IO          21
#define IMU_SCL_IO          22
#define IMU_I2C_FREQ_HZ     400000

// The sensor's address is switch-selectable, so we scan this range
#define IMU_ADDR_MIN        0x4A
#define IMU_ADDR_MAX        0x4D

// ---- Timing ----
#define IMU_PROBE_TIMEOUT_MS  50    /* per-address probe */
#define IMU_XFER_TIMEOUT_MS   100   /* per I2C transfer */
#define IMU_LATCH_DELAY_MS    12    /* firmware needs ~10 ms to latch data */
#define IMU_SETTLE_DELAY_MS   20    /* pause after each config write */

// ---- Ranges (selector values only; full-scale is looked up in imu.c) ----
#define IMU_ACC_SEL         2       /* 0=2g 1=4g 2=8g 3=16g */
#define IMU_GYR_SEL         4       /* 0=125 1=250 2=500 3=1000 4=2000 dps */

// ---- Motion events ----
// 0=off 1=data ready 2=any motion 3=no motion 4=significant motion
// 5=step counter 6=flat 7=orientation 8=tap 9=tilt
#define IMU_ENABLE_EVENTS   1
#define IMU_INT1_EVENT      5
#define IMU_INT2_EVENT      8

// ---- Gyro calibration ----
// Samples to average. Real rate is below 100 Hz (each read blocks ~12-24 ms),
// so 500 samples takes roughly 6-12 seconds
#define CAL_GYRO_SAMPLES          500
// Wait after the keypress so the board stops moving
#define CAL_GYRO_SETTLE_MS        500
// If any axis's standard deviation exceeds this, the board moved. Tune on your bench
#define CAL_GYRO_MAX_STDDEV_DPS   0.5
// Give up if this many reads fail in one run
#define CAL_MAX_READ_FAILS        20

// ---- Accel calibration ----
// Samples averaged per pose
#define CAL_ACCEL_SAMPLES          200
// Wait after the keypress so the board stops moving
#define CAL_ACCEL_SETTLE_MS        500
// If any axis's standard deviation exceeds this, the board moved. Tune on your bench
#define CAL_ACCEL_MAX_STDDEV_G     0.02
// The axis that should point up/down must read at least this much (in g)
#define CAL_ACCEL_ORIENT_MIN_G     0.7
// The other two axes must read within this much of 0 (in g)
#define CAL_ACCEL_ORIENT_MAX_OFF_G 0.3
// Reject results outside these limits: they mean a bad pose or a faulty sensor
#define CAL_ACCEL_MAX_BIAS_G       0.3
#define CAL_ACCEL_SCALE_MIN        0.8
#define CAL_ACCEL_SCALE_MAX        1.2

// ---- Mahony filter ----
// How hard the accel pulls the estimate toward gravity. Higher = faster, but more vibration-sensitive
#define MAHONY_KP              1.0f
// Learns leftover gyro bias. Start at 0, then try 0.005 to 0.05 once Kp works
#define MAHONY_KI              0.0f
// Large Kp for the first moments so the filter snaps to gravity, then it drops to MAHONY_KP
#define MAHONY_KP_STARTUP      10.0f
#define MAHONY_STARTUP_MS      2000
// Only trust the accel as "gravity" when its magnitude is near 1 g
#define MAHONY_ACCEL_GATE_MIN_G 0.8f
#define MAHONY_ACCEL_GATE_MAX_G 1.2f
// Ignore updates with a dt longer than this (stall, or first iteration after a calibration)
#define MAHONY_MAX_DT_S        0.1f
// Print one line every N updates, so slow float formatting doesn't distort dt
#define MAHONY_PRINT_EVERY     10