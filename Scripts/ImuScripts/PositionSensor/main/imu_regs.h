#pragma once

/* Chip facts from the datasheet. These only change if you swap sensors. */

// Expected value of the vendor ID register
#define IMU_VID_EXPECTED  0x3343

// DFRobot I2C-mode register map (16-bit word registers)
#define R_VID        0x0000   /* expect 0x3343 */
#define R_PID        0x0001   /* 0x0006=6DOF, 0x0009=9DOF, 0x0010=10DOF */
#define R_FW         0x0002
#define R_MODE       0x0005   /* 0 sleep,1 low power,2 normal,3 high perf */
#define R_ACC_RANGE  0x0006   /* 0=2g 1=4g 2=8g 3=16g */
#define R_GYR_RANGE  0x0007   /* 0=125 1=250 2=500 3=1000 4=2000 dps */
#define R_INT1       0x000C
#define R_INT2       0x000D
#define R_ACC_X      0x0010   /* 12 bytes: accXYZ, gyrXYZ (int16 LE) */
#define R_MAG_X_LO   0x0016   /* 12 bytes: magXYZ (int32 LE, 0.01 uT), 9/10DOF only */
#define R_STEP_LO    0x001E   /* 4 bytes: step count (uint32 LE) */
#define R_TAP        0x0020   /* 1=single 2=double 3=triple */
#define R_ORIENT     0x0021   /* high byte: orientation, low byte: face up/down */

// PID values that tell us which model is attached
#define IMU_PID_9DOF  0x0009  /* anything >= this has a magnetometer */
#define IMU_PID_10DOF 0x0010

// The sensor's mode register value for normal operation
#define IMU_MODE_NORMAL 2