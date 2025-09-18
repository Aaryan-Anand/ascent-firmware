// orientation.c
#include "orientation.h"
#include <math.h>
#include <float.h>
#include "interface_bno055.h"
#ifdef ESP_PLATFORM
#include "esp_timer.h"
#endif


static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float wrap180(float a) {
    while (a <= -180.0f) a += 360.0f;
    while (a >   180.0f) a -= 360.0f;
    return a;
}
static inline void normalize_quat(float* qw, float* qx, float* qy, float* qz) {
    const float n = sqrtf((*qw)*(*qw) + (*qx)*(*qx) + (*qy)*(*qy) + (*qz)*(*qz));
    if (n > 0.0f) { *qw /= n; *qx /= n; *qy /= n; *qz /= n; }
    else { *qw = 1.0f; *qx = *qy = *qz = 0.0f; }
}

static void euler_from_quat_excel(float qw, float qx, float qy, float qz,
                                  float* roll_deg, float* pitch_deg, float* yaw_deg)
{
    normalize_quat(&qw, &qx, &qy, &qz);

    const float sinr_cosp = 2.0f * (qw*qx + qy*qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (qw*qy - qz*qx);
    float pitch = asinf(clampf(sinp, -1.0f, 1.0f));

    const float siny_cosp = 2.0f * (qw*qz + qx*qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    roll  = rad2deg(roll)  - 90.0f;
    pitch = rad2deg(pitch);
    yaw   = rad2deg(yaw)   - 90.0f;

    *roll_deg  = wrap180(roll);
    *pitch_deg = clampf(pitch, -90.0f, 90.0f);
    *yaw_deg   = wrap180(yaw);
}

static void integrate_quat_from_local_gyr(float* qw, float* qx, float* qy, float* qz,
                                          const imu_local_3d_t* g,
                                          float dt_sec)
{
    const float wx = deg2rad(g->x);
    const float wy = deg2rad(g->y);
    const float wz = deg2rad(g->z);

    const float qdot_w = 0.5f * ( -(*qx)*wx - (*qy)*wy - (*qz)*wz );
    const float qdot_x = 0.5f * (  (*qw)*wx + (*qy)*wz - (*qz)*wy );
    const float qdot_y = 0.5f * (  (*qw)*wy + (*qz)*wx - (*qx)*wz );
    const float qdot_z = 0.5f * (  (*qw)*wz + (*qx)*wy - (*qy)*wx );

    float qw_pre = (*qw) + qdot_w * dt_sec;
    float qx_pre = (*qx) + qdot_x * dt_sec;
    float qy_pre = (*qy) + qdot_y * dt_sec;
    float qz_pre = (*qz) + qdot_z * dt_sec;

    normalize_quat(&qw_pre, &qx_pre, &qy_pre, &qz_pre);

    *qw = qw_pre; *qx = qx_pre; *qy = qy_pre; *qz = qz_pre;
}


void orientation_make_quat_from_euler(float roll_deg, float pitch_deg, float yaw_deg,
                                      float* qw, float* qx, float* qy, float* qz)
{
    const float cr = cosf(deg2rad(roll_deg)  * 0.5f);
    const float sr = sinf(deg2rad(roll_deg)  * 0.5f);
    const float cp = cosf(deg2rad(pitch_deg) * 0.5f);
    const float sp = sinf(deg2rad(pitch_deg) * 0.5f);
    const float cy = cosf(deg2rad(yaw_deg)   * 0.5f);
    const float sy = sinf(deg2rad(yaw_deg)   * 0.5f);

    *qw = cr*cp*cy + sr*sp*sy;
    *qx = sr*cp*cy - cr*sp*sy;
    *qy = cr*sp*cy + sr*cp*sy;
    *qz = cr*cp*sy - sr*sp*cy;
    normalize_quat(qw, qx, qy, qz);
}

void orientation_init_euler(orientation_t* s, float roll_deg, float pitch_deg, float yaw_deg) {
    if (!s) return;
    s->roll = roll_deg; s->pitch = pitch_deg; s->yaw = yaw_deg;
    orientation_make_quat_from_euler(roll_deg, pitch_deg, yaw_deg, &s->qw, &s->qx, &s->qy, &s->qz);
    euler_from_quat_excel(s->qw, s->qx, s->qy, s->qz, &s->roll, &s->pitch, &s->yaw);
}

void orientation_init_quat(orientation_t* s, float qw, float qx, float qy, float qz) {
    if (!s) return;
    s->qw = qw; s->qx = qx; s->qy = qy; s->qz = qz;
    normalize_quat(&s->qw, &s->qx, &s->qy, &s->qz);
    euler_from_quat_excel(s->qw, s->qx, s->qy, s->qz, &s->roll, &s->pitch, &s->yaw);
}


void orientation_update_from_euler_rates_dt(orientation_t* s,
                                            const imu_local_3d_t* local_gyr,
                                            float dt_sec)
{
    if (!s || !local_gyr || dt_sec <= 0.0f) return;

    integrate_quat_from_local_gyr(&s->qw, &s->qx, &s->qy, &s->qz, local_gyr, dt_sec);
    euler_from_quat_excel(s->qw, s->qx, s->qy, s->qz, &s->roll, &s->pitch, &s->yaw);
}

void orientation_update_from_euler_rates(orientation_t* s,
                                         const imu_local_3d_t* local_gyr)
{
    if (!s || !local_gyr) return;

#ifndef ESP_PLATFORM
    return;
#else
    static int64_t last_us = 0;
    const int64_t now_us = esp_timer_get_time();
    if (last_us == 0) { last_us = now_us; return; }
    const float dt_sec = (now_us - last_us) / 1e6f;
    last_us = now_us;
    if (dt_sec <= 0.0f) return;
    integrate_quat_from_local_gyr(&s->qw, &s->qx, &s->qy, &s->qz, local_gyr, dt_sec);
    euler_from_quat_excel(s->qw, s->qx, s->qy, s->qz, &s->roll, &s->pitch, &s->yaw);
#endif
}


void orientation_sync_euler_from_quat(orientation_t* s) {
    if (!s) return;
    euler_from_quat_excel(s->qw, s->qx, s->qy, s->qz, &s->roll, &s->pitch, &s->yaw);
    s->roll = wrap180(s->roll + 90.0f);
    s->yaw  = wrap180(s->yaw  + 90.0f);
}

void orientation_init_from_gravity(orientation_t* s, bool use_filtered)
{
    if (!s) return;

    imu_local_3d_t acc, gyr, mag;
    bno055_get_local(&acc, &gyr, &mag, use_filtered);

    float ax = acc.x, ay = acc.y, az = acc.z;
    float n2 = ax*ax + ay*ay + az*az;
    if (n2 <= FLT_EPSILON) {
        orientation_init_quat(s, 1.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    float invn = 1.0f / sqrtf(n2);
    ax *= invn; ay *= invn; az *= invn;

    float pitch_deg = (180.0f / (float)M_PI) * atan2f(-az, -ax);

    float qw, qx, qy, qz;
    orientation_make_quat_from_euler(0.0f, pitch_deg, 0.0f, &qw, &qx, &qy, &qz);

    orientation_init_quat(s, qw, qx, qy, qz);
}