#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// All filter state lives here, so there are no hidden globals
typedef struct {
    float q0, q1, q2, q3;               /* quaternion (w, x, y, z), body -> world */
    float ex_int, ey_int, ez_int;       /* integral of the error: learned gyro bias (rad/s) */
    float kp, ki;                       /* gains */
} mahony_t;

// Resets the quaternion to identity (no rotation) and sets the gains
void mahony_init(mahony_t *f, float kp, float ki);

// One filter step. Gyro in rad/s, accel in g (any unit works, it gets normalized), dt in seconds
void mahony_update(mahony_t *f, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt);

// Copies the quaternion out as (w, x, y, z)
void mahony_get_quaternion(const mahony_t *f, float q[4]);

// Converts to roll/pitch/yaw in degrees. For printing and debugging only
void mahony_get_euler_deg(const mahony_t *f, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif