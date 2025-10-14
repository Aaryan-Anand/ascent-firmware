// sensor_filtering.h
#pragma once
#ifndef SENSOR_FILTERING_H
#define SENSOR_FILTERING_H

#include <stdint.h>
#include <stdbool.h>

#include "interface_bno055.h"    // imu_local_3d_t for acc/gyr/mag
#include "interface_h3lis331dl.h"   // imu_float_3d_t for high-g accel

#ifdef __cplusplus
extern "C" {
#endif

// Profiles for per-sensor constants (extend as we add filters)
typedef enum {
    SENSOR_PROFILE_LOW_G_ACC = 0,
    SENSOR_PROFILE_GYRO      = 1,
    SENSOR_PROFILE_MAG       = 2,
    SENSOR_PROFILE_HIGH_G_ACC= 3,
    SENSOR_PROFILE_COUNT
} sensor_filter_profile_t;

// Override the Hampel-like parameters per profile (optional).
// Defaults: LOW_G_ACC = { beta_h=0.03, k=5.0, eps=1e-6 }
// Others default to same values for now (harmless).
void sensor_filter_set_profile_params(sensor_filter_profile_t profile,
                                      float beta_h, float k_clamp, float epsilon_mad);

// Biquad params (Direct Form I):
// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
void sensor_filter_set_biquad_params(sensor_filter_profile_t profile,
                                     float b0, float b1, float b2,
                                     float a1, float a2);

/* ---------- Filters ---------- */
void sensor_filter_reset(void);  // resets all internal states

// In-place filters (modify the structs you pass in).
// `flight_state` is used to re-seed states when it changes.
void sensor_filter_acc(imu_local_3d_t* local_acc, uint8_t flight_state, bool clamp);
void sensor_filter_gyr(imu_local_3d_t* local_gyr, uint8_t flight_state);
void sensor_filter_mag(imu_local_3d_t* local_mag, uint8_t flight_state);
void sensor_filter_high_g_acc(imu_float_3d_t* high_g_acc, uint8_t flight_state);

#ifdef __cplusplus
}
#endif
#endif // SENSOR_FILTERING_H
