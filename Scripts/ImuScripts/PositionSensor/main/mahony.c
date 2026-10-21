#include <math.h>
#include <stddef.h>

#include "mahony.h"
#include "imu_config.h"

void mahony_init(mahony_t *f, float kp, float ki)
{
    if (f == NULL) return;
    // Identity quaternion: the board starts "level, facing zero yaw"
    f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
    f->ex_int = f->ey_int = f->ez_int = 0.0f;
    f->kp = kp;
    f->ki = ki;
}

void mahony_update(mahony_t *f, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    if (f == NULL || dt <= 0.0f) return;

    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // Accel magnitude, in g. Used both to normalize and to decide if we trust it
    float anorm = sqrtf(ax * ax + ay * ay + az * az);

    // Trust the accel as "gravity" only if its length is near 1 g. During motion it also
    // measures linear acceleration, which would corrupt roll and pitch. A zero length
    // (bad read) would divide by zero, and the gate range excludes it too
    if (anorm >= MAHONY_ACCEL_GATE_MIN_G && anorm <= MAHONY_ACCEL_GATE_MAX_G) {
        // Normalize to a unit vector pointing "up" in the body frame
        float inv = 1.0f / anorm;
        ax *= inv; ay *= inv; az *= inv;

        // Gravity direction the CURRENT quaternion predicts, in the body frame.
        // Flat and level this is (0, 0, 1), matching +1 g on Z
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error = measured x predicted (cross product). It points along the axis we
        // need to rotate about, and grows with the angle between the two vectors
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        if (f->ki > 0.0f) {
            // Integral term: slowly absorbs a constant gyro bias
            f->ex_int += ex * f->ki * dt;
            f->ey_int += ey * f->ki * dt;
            f->ez_int += ez * f->ki * dt;
        } else {
            // Ki off: make sure no stale bias estimate lingers
            f->ex_int = f->ey_int = f->ez_int = 0.0f;
        }

        // The heart of the filter: the accel error nudges the gyro rate
        gx += f->kp * ex + f->ex_int;
        gy += f->kp * ey + f->ey_int;
        gz += f->kp * ez + f->ez_int;
    }

    // Integrate the quaternion: q_dot = 0.5 * q (x) (0, gx, gy, gz).
    // Fold the 0.5 and dt into the rates first
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    // Save the old components, since each line below needs the pre-update values
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // Renormalize: floating-point error slowly pushes q off unit length
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n > 0.0f) {
        float inv = 1.0f / n;
        f->q0 = q0 * inv;
        f->q1 = q1 * inv;
        f->q2 = q2 * inv;
        f->q3 = q3 * inv;
    }
}

void mahony_get_quaternion(const mahony_t *f, float q[4])
{
    q[0] = f->q0; q[1] = f->q1; q[2] = f->q2; q[3] = f->q3;
}

void mahony_get_euler_deg(const mahony_t *f, float *roll, float *pitch, float *yaw)
{
    const float RAD2DEG = 57.29577951f;

    // Roll: rotation about X
    *roll  = atan2f(2.0f * (f->q0 * f->q1 + f->q2 * f->q3),
                    1.0f - 2.0f * (f->q1 * f->q1 + f->q2 * f->q2)) * RAD2DEG;

    // Pitch: rotation about Y. asinf needs an input in [-1, 1], and rounding can
    // push it slightly outside, so clamp
    float s = 2.0f * (f->q0 * f->q2 - f->q3 * f->q1);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    *pitch = asinf(s) * RAD2DEG;

    // Yaw: rotation about Z. On a 6DOF board nothing corrects this, so it drifts
    *yaw   = atan2f(2.0f * (f->q0 * f->q3 + f->q1 * f->q2),
                    1.0f - 2.0f * (f->q2 * f->q2 + f->q3 * f->q3)) * RAD2DEG;
}