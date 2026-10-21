#include <math.h>
#include <stddef.h>

#include "position.h"
#include "imu_config.h"

void position_init(position_t *pos)
{
    if (pos == NULL) return;
    for (int i = 0; i < 3; i++) {
        pos->v[i] = 0.0f;
        pos->p[i] = 0.0f;
    }
    pos->still_count = 0;
    pos->stationary  = false;
}

void position_reset(position_t *pos)
{
    position_init(pos);
}

void position_update(position_t *pos, const float q[4],
                     float ax, float ay, float az,
                     float gx, float gy, float gz, float dt)
{
    // A zero or negative dt would make the integration meaningless
    if (pos == NULL || q == NULL || dt <= 0.0f) return;

    // ---- Step 1: stationary detection (body frame) ----
    // At rest the accel reads exactly gravity (1 g) and the gyro reads about 0
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
    bool quiet = (fabsf(amag - 1.0f) < POS_STILL_ACCEL_BAND_G) &&
                 (gmag < POS_STILL_GYRO_DPS);

    // Count consecutive quiet samples; any noisy sample resets the count
    if (quiet) {
        if (pos->still_count < POS_STILL_SAMPLES) pos->still_count++;
    } else {
        pos->still_count = 0;
    }
    pos->stationary = (pos->still_count >= POS_STILL_SAMPLES);

    // Zero-velocity update (ZUPT): while still, force velocity to zero and skip
    // integration. This stops velocity error from piling up into position drift
    if (pos->stationary) {
        pos->v[0] = pos->v[1] = pos->v[2] = 0.0f;
        return;
    }

    // ---- Step 2: rotate accel from the body frame into the world frame ----
    // This is the body->world rotation matrix built from the quaternion.
    // Its third row matches the gravity prediction inside mahony_update()
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float wx = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ax
             + (2.0f * (q1 * q2 - q0 * q3))        * ay
             + (2.0f * (q1 * q3 + q0 * q2))        * az;
    float wy = (2.0f * (q1 * q2 + q0 * q3))        * ax
             + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * ay
             + (2.0f * (q2 * q3 - q0 * q1))        * az;
    float wz = (2.0f * (q1 * q3 - q0 * q2))        * ax
             + (2.0f * (q2 * q3 + q0 * q1))        * ay
             + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * az;

    // ---- Step 3: remove gravity ----
    // In the world frame gravity is a constant +1 g on Z. What's left is the
    // board's own (linear) acceleration
    wz -= 1.0f;

    // ---- Step 4: convert g to m/s^2 ----
    float a[3] = { wx * POS_GRAVITY_MS2,
                   wy * POS_GRAVITY_MS2,
                   wz * POS_GRAVITY_MS2 };

    // ---- Step 5: integrate twice ----
    // Velocity first, then position using the NEW velocity
    for (int i = 0; i < 3; i++) {
        pos->v[i] += a[i] * dt;
        pos->p[i] += pos->v[i] * dt;
    }
}   