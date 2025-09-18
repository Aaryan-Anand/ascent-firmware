// orientation.h
#pragma once
#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>
#include "interface_bno055.h"   // for imu_local_3d_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Euler angles in degrees (ZYX: yaw, pitch, roll)
    float roll;   // about +X
    float pitch;  // about +Y
    float yaw;    // about +Z

    float qw;
    float qx;
    float qy;
    float qz;
} orientation_t;


void orientation_init_euler(orientation_t* s, float roll_deg, float pitch_deg, float yaw_deg);
void orientation_init_quat(orientation_t* s, float qw, float qx, float qy, float qz);

void orientation_update_from_euler_rates(orientation_t* s,
                                         const imu_local_3d_t* local_gyr);

void orientation_update_from_euler_rates_dt(orientation_t* s,
                                            const imu_local_3d_t* local_gyr,
                                            float dt_sec);

void orientation_sync_euler_from_quat(orientation_t* s);

void orientation_make_quat_from_euler(float roll_deg, float pitch_deg, float yaw_deg,
                                      float* qw, float* qx, float* qy, float* qz);

#ifdef __cplusplus
}
#endif
#endif // ORIENTATION_H
