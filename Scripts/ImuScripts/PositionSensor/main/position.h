#pragma once

#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

// All state lives here, so there are no hidden globals
typedef struct {
    float v[3];          /* velocity in the world frame (m/s)  */
    float p[3];          /* position in the world frame (m)    */
    int   still_count;   /* consecutive quiet samples so far   */
    bool  stationary;    /* true while the zero-velocity update is holding v at 0 */
} position_t;

// Zeroes position, velocity and the stationary counter
void position_init(position_t *pos);

// Same as init: call it to set "here" as the new origin
void position_reset(position_t *pos);

// One step. q is the body->world quaternion (w,x,y,z) from the Mahony filter.
// Accel in g and gyro in dps, both already calibrated. dt in seconds
void position_update(position_t *pos, const float q[4],
                     float ax, float ay, float az,
                     float gx, float gy, float gz, float dt);

#ifdef __cplusplus
}
#endif