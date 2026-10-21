#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "imu.h"     /* for imu_sample_t */

#ifdef __cplusplus
extern "C" {
#endif

#define CAL_MAGIC       0xCA11B8A7u
#define CAL_VERSION     1u

#define CAL_FLAG_GYRO   (1u << 0)
#define CAL_FLAG_ACCEL  (1u << 1)
#define CAL_FLAG_MAG    (1u << 2)

//data
typedef struct {
    uint32_t magic; // a fixed 32-bit constant. It tells data blob not garbage
    uint16_t version; // 16-bit number to bump when struct layout changes
    uint16_t flags; // a bitmask with one bit each for gyro, accel and mag calibrated. Allows for partial calibration
    float gyro_bias[3]; // Payload bias for gyro in all 3 DOF
    float accel_bias[3]; // Payload bias for accel in all 3 DOF
    float accel_scale[3]; // Payload scale for accel in all 3 DOF
    float mag_bias[3]; // Payload bias for hard/soft iron bias of magnetometers
    float mag_scale[3]; // Payload scale for hard/soft iron bias of magnetometers
    uint32_t crc;          /* Cyclic Redundancy Check: short number computed from a block of data, used to detect whether the data has been accidentally changed or corrupted. must stay last: covers every byte before it */
} cal_data_t;


typedef enum {
    CAL_LOADED = 0,
    CAL_NOT_FOUND,         /* first boot / never calibrated */
    CAL_BAD_SIZE,          /* old layout */
    CAL_BAD_HEADER,        /* wrong magic or version */
    CAL_BAD_CRC,           /* corrupted */
    CAL_ERR                /* unexpected NVS error */
} cal_status_t;


cal_status_t cal_load(void);        /* call after nvs_flash_init() */
esp_err_t    cal_save(void);        /* call when a routine finishes */
cal_data_t  *cal_get(void);         /* live struct, for routines and cal_apply */
void         cal_set_defaults(void);
esp_err_t cal_run_gyro(void);       /* interactive: board must be still */
esp_err_t cal_run_accel(void);      /* interactive: six poses, board still in each */
// Corrects a raw sample using the live calibration. `in` and `out` may be the same pointer
void cal_apply(const imu_sample_t *in, imu_sample_t *out);


#ifdef __cplusplus
}
#endif