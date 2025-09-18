#include "sensor_filtering.h"
#include "interface_bno055.h"
#include "interface_H3LIS331DL.h"

static bool s_gyr_initialized = false;
static float s_gyr_x_prev = 0.0f;
static float s_gyr_y_prev = 0.0f;
static float s_gyr_z_prev = 0.0f;

static inline float iir_1st_ema(float x, float y_prev) {
    return 0.9f * x + 0.1f * y_prev;
}

void sensor_filter_reset(void) {
    s_gyr_initialized = false;
    s_gyr_x_prev = s_gyr_y_prev = s_gyr_z_prev = 0.0f;
}

void sensor_filter_acc(imu_local_3d_t* local_acc, uint8_t flight_state) {
    (void)flight_state;
    if (!local_acc) return;
}

void sensor_filter_mag(imu_local_3d_t* local_mag, uint8_t flight_state) {
    (void)flight_state;
    if (!local_mag) return;
}

void sensor_filter_high_g_acc(imu_float_3d_t* high_g_acc, uint8_t flight_state) {
    (void)flight_state;
    if (!high_g_acc) return;
}

void sensor_filter_gyr(imu_local_3d_t* local_gyr, uint8_t flight_state) {
    (void)flight_state;
    if (!local_gyr) return;

    if (!s_gyr_initialized) {
        s_gyr_x_prev = local_gyr->x;
        s_gyr_y_prev = local_gyr->y;
        s_gyr_z_prev = local_gyr->z;
        s_gyr_initialized = true;
        return;
    }

    float x_f = iir_1st_ema(local_gyr->x, s_gyr_x_prev);
    float y_f = iir_1st_ema(local_gyr->y, s_gyr_y_prev);
    float z_f = iir_1st_ema(local_gyr->z, s_gyr_z_prev);

    local_gyr->x = x_f;
    local_gyr->y = y_f;
    local_gyr->z = z_f;

    s_gyr_x_prev = x_f;
    s_gyr_y_prev = y_f;
    s_gyr_z_prev = z_f;
}
