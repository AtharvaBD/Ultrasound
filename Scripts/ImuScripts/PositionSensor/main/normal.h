#pragma once
#include <math.h>

// Gravity ("up") direction expressed in the sensor/body frame, from the Mahony quaternion (w,x,y,z).
static inline void gravity_from_quat(const float q[4], float g[3])
{
    g[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    g[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

// Angle between two unit vectors, in degrees.
// atan2(|a x b|, a.b) stays accurate near 0 and 180 deg, unlike acosf(dot).
static inline float angle_between_deg(const float a[3], const float b[3])
{
    float cx = a[1] * b[2] - a[2] * b[1];
    float cy = a[2] * b[0] - a[0] * b[2];
    float cz = a[0] * b[1] - a[1] * b[0];
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return atan2f(sqrtf(cx * cx + cy * cy + cz * cz), dot) * 57.29577951f;
}