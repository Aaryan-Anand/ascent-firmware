// sensor_filtering.c
#include "sensor_filtering.h"
#include <math.h>

// ======================
// Parameter LUT (per sensor type)
// ======================
typedef struct {
    float beta_h;       // step size toward sample (median & mad updates)
    float k_clamp;      // clamp multiplier on MAD
    float epsilon_mad;  // floor for MAD
} robust_params_t;

static robust_params_t s_params[SENSOR_PROFILE_COUNT] = {
    [SENSOR_PROFILE_LOW_G_ACC] = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_GYRO]      = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_MAG]       = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_HIGH_G_ACC]= { 0.05f, 6.0f, 1e-6f },
};

void sensor_filter_set_profile_params(sensor_filter_profile_t profile,
                                      float beta_h, float k_clamp, float epsilon_mad)
{
    if (profile < 0 || profile >= SENSOR_PROFILE_COUNT) return;
    s_params[profile].beta_h      = beta_h;
    s_params[profile].k_clamp     = k_clamp;
    s_params[profile].epsilon_mad = epsilon_mad;
}

// ======================
// Gyro EMA (from earlier stub)
// ======================
static bool  s_gyr_initialized = false;
static float s_gyr_x_prev = 0.0f;
static float s_gyr_y_prev = 0.0f;
static float s_gyr_z_prev = 0.0f;

static inline float ema_1st(float x, float y_prev) {
    // 0.9 new, 0.1 previous filtered
    return 0.9f * x + 0.1f * y_prev;
}

// ======================
// Low-g accelerometer Hampel-like clamp state
// ======================
static bool    s_acc_initialized = false;
static uint8_t s_acc_prev_state  = 0;

// Per-axis cumulative "median" (EMA toward sample) and MAD
static float s_acc_med_x = 0.0f, s_acc_med_y = 0.0f, s_acc_med_z = 0.0f;
static float s_acc_mad_x = 0.0f, s_acc_mad_y = 0.0f, s_acc_mad_z = 0.0f;

static inline float hampel_update_median(float prev_med, float sample, float beta_h) {
    // med_new = med_prev + beta_h * (sample - med_prev)
    return prev_med + beta_h * (sample - prev_med);
}

static inline float hampel_update_mad(float prev_mad, float prev_med,
                                      float sample, float beta_h, float eps)
{
    // mad_new = max(eps, prev_mad + beta_h * ( |sample - prev_med| - prev_mad ))
    float dev = fabsf(sample - prev_med);
    float mad = prev_mad + beta_h * (dev - prev_mad);
    return (mad < eps) ? eps : mad;
}

static inline float hampel_clamp(float sample, float med_new, float mad_new, float k) {
    float lower = med_new - k * mad_new;
    float upper = med_new + k * mad_new;
    if (sample < lower) sample = lower;
    if (sample > upper) sample = upper;
    return sample;
}

// ======================
// Public API
// ======================

void sensor_filter_reset(void) {
    // gyro
    s_gyr_initialized = false;
    s_gyr_x_prev = s_gyr_y_prev = s_gyr_z_prev = 0.0f;

    // low-g acc
    s_acc_initialized = false;
    s_acc_prev_state  = 0;
    s_acc_med_x = s_acc_med_y = s_acc_med_z = 0.0f;
    s_acc_mad_x = s_acc_mad_y = s_acc_mad_z = 0.0f;
}

void sensor_filter_acc(imu_local_3d_t* local_acc, uint8_t flight_state) {
    if (!local_acc) return;

    const robust_params_t p = s_params[SENSOR_PROFILE_LOW_G_ACC];

    // Reseed when flight state changes or on first use
    if (!s_acc_initialized || flight_state != s_acc_prev_state) {
        s_acc_prev_state  = flight_state;
        s_acc_initialized = true;

        // Seed medians with the current sample, MAD with epsilon
        s_acc_med_x = local_acc->x;
        s_acc_med_y = local_acc->y;
        s_acc_med_z = local_acc->z;

        s_acc_mad_x = p.epsilon_mad;
        s_acc_mad_y = p.epsilon_mad;
        s_acc_mad_z = p.epsilon_mad;

        // First sample after reseed: keep as-is
        return;
    }

    // --- X axis ---
    float med_x_new = hampel_update_median(s_acc_med_x, local_acc->x, p.beta_h);
    float mad_x_new = hampel_update_mad(s_acc_mad_x, s_acc_med_x, local_acc->x, p.beta_h, p.epsilon_mad);
    float x_clamped = hampel_clamp(local_acc->x, med_x_new, mad_x_new, p.k_clamp);

    // --- Y axis ---
    float med_y_new = hampel_update_median(s_acc_med_y, local_acc->y, p.beta_h);
    float mad_y_new = hampel_update_mad(s_acc_mad_y, s_acc_med_y, local_acc->y, p.beta_h, p.epsilon_mad);
    float y_clamped = hampel_clamp(local_acc->y, med_y_new, mad_y_new, p.k_clamp);

    // --- Z axis ---
    float med_z_new = hampel_update_median(s_acc_med_z, local_acc->z, p.beta_h);
    float mad_z_new = hampel_update_mad(s_acc_mad_z, s_acc_med_z, local_acc->z, p.beta_h, p.epsilon_mad);
    float z_clamped = hampel_clamp(local_acc->z, med_z_new, mad_z_new, p.k_clamp);

    // Write back in-place
    local_acc->x = x_clamped;
    local_acc->y = y_clamped;
    local_acc->z = z_clamped;

    // Commit state
    s_acc_med_x = med_x_new;  s_acc_mad_x = mad_x_new;
    s_acc_med_y = med_y_new;  s_acc_mad_y = mad_y_new;
    s_acc_med_z = med_z_new;  s_acc_mad_z = mad_z_new;
}

void sensor_filter_gyr(imu_local_3d_t* local_gyr, uint8_t flight_state) {
    (void)flight_state;
    if (!local_gyr) return;

    if (!s_gyr_initialized) {
        s_gyr_x_prev = local_gyr->x;
        s_gyr_y_prev = local_gyr->y;
        s_gyr_z_prev = local_gyr->z;
        s_gyr_initialized = true;
        return; // first sample unmodified
    }

    float xf = ema_1st(local_gyr->x, s_gyr_x_prev);
    float yf = ema_1st(local_gyr->y, s_gyr_y_prev);
    float zf = ema_1st(local_gyr->z, s_gyr_z_prev);

    local_gyr->x = xf;
    local_gyr->y = yf;
    local_gyr->z = zf;

    s_gyr_x_prev = xf;
    s_gyr_y_prev = yf;
    s_gyr_z_prev = zf;
}

void sensor_filter_mag(imu_local_3d_t* local_mag, uint8_t flight_state) {
    (void)flight_state;
    if (!local_mag) return;
    // pass-through for now
}

void sensor_filter_high_g_acc(imu_float_3d_t* high_g_acc, uint8_t flight_state) {
    (void)flight_state;
    if (!high_g_acc) return;
    // pass-through for now
}
