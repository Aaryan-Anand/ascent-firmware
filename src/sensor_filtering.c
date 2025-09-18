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

typedef struct {
    float b0, b1, b2;
    float a1, a2;       // note the minus signs are applied in the DF1 eq
} biquad_params_t;

static robust_params_t s_params[SENSOR_PROFILE_COUNT] = {
    [SENSOR_PROFILE_LOW_G_ACC] = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_GYRO]      = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_MAG]       = { 0.03f, 5.0f, 1e-6f },
    [SENSOR_PROFILE_HIGH_G_ACC]= { 0.05f, 6.0f, 1e-6f },
};

// Defaults: only low-g ACC has a biquad configured initially (your constants)
static biquad_params_t s_biquad[SENSOR_PROFILE_COUNT] = {
    [SENSOR_PROFILE_LOW_G_ACC] = { // band-pass biquad you provided
        .b0 = 0.145323884f,
        .b1 = 0.290647768f,
        .b2 = 0.145323884f,
        .a1 = -0.671029091f,
        .a2 = 0.252324626f
    },
    [SENSOR_PROFILE_GYRO]       = {0,0,0,0,0},
    [SENSOR_PROFILE_MAG]        = {0,0,0,0,0},
    [SENSOR_PROFILE_HIGH_G_ACC] = { // default to same as LOW_G_ACC
        .b0 = 0.391335773f,
        .b1 = 0.782671545f,
        .b2 = 0.391335773f,
        .a1 = 0.369527377f,
        .a2 = 0.195815713f
    },
};

void sensor_filter_set_profile_params(sensor_filter_profile_t profile,
                                      float beta_h, float k_clamp, float epsilon_mad)
{
    if (profile < 0 || profile >= SENSOR_PROFILE_COUNT) return;
    s_params[profile].beta_h      = beta_h;
    s_params[profile].k_clamp     = k_clamp;
    s_params[profile].epsilon_mad = epsilon_mad;
}

void sensor_filter_set_biquad_params(sensor_filter_profile_t profile,
                                     float b0, float b1, float b2,
                                     float a1, float a2)
{
    if (profile < 0 || profile >= SENSOR_PROFILE_COUNT) return;
    s_biquad[profile].b0 = b0;
    s_biquad[profile].b1 = b1;
    s_biquad[profile].b2 = b2;
    s_biquad[profile].a1 = a1;
    s_biquad[profile].a2 = a2;
}

/* ======================
   Gyro EMA state
   ====================== */
static bool  s_gyr_initialized = false;
static float s_gyr_x_prev = 0.0f, s_gyr_y_prev = 0.0f, s_gyr_z_prev = 0.0f;
static inline float ema_1st(float x, float y_prev) { return 0.9f*x + 0.1f*y_prev; }

/* ======================
   Low-g ACC Hampel state
   ====================== */
static bool    s_acc_initialized = false;
static uint8_t s_acc_prev_state  = 0;

// Per-axis cumulative "median" (EMA toward sample) and MAD
static float s_acc_med_x = 0.0f, s_acc_med_y = 0.0f, s_acc_med_z = 0.0f;
static float s_acc_mad_x = 0.0f, s_acc_mad_y = 0.0f, s_acc_mad_z = 0.0f;

static inline float hampel_update_median(float prev_med, float sample, float beta_h) {
    return prev_med + beta_h * (sample - prev_med);
}
static inline float hampel_update_mad(float prev_mad, float prev_med, float sample, float beta_h, float eps) {
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

// (X, Y, Z axes each need their own history)
typedef struct { float x1,x2,y1,y2; } biquad_hist_t;
static biquad_hist_t s_hist_acc_x = {0}, s_hist_acc_y = {0}, s_hist_acc_z = {0};

static inline float biquad_df1(const biquad_params_t* p, biquad_hist_t* h, float x0) {
    // y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2
    float y0 = p->b0 * x0 + p->b1 * h->x1 + p->b2 * h->x2
             - p->a1 * h->y1 - p->a2 * h->y2;
    // shift history
    h->x2 = h->x1; h->x1 = x0;
    h->y2 = h->y1; h->y1 = y0;
    return y0;
}

/* ======================
   High-g ACC Hampel state
   ====================== */
static bool    s_hg_initialized = false;
static uint8_t s_hg_prev_state  = 0;

// Per-axis cumulative "median" (EMA toward sample) and MAD
static float s_hg_med_x = 0.0f, s_hg_med_y = 0.0f, s_hg_med_z = 0.0f;
static float s_hg_mad_x = 0.0f, s_hg_mad_y = 0.0f, s_hg_mad_z = 0.0f;

// Biquad histories for high-g
static biquad_hist_t s_hist_hg_x = {0}, s_hist_hg_y = {0}, s_hist_hg_z = {0};

/* ======================
   Public API
   ====================== */
void sensor_filter_reset(void) {
    // gyro
    s_gyr_initialized = false;
    s_gyr_x_prev = s_gyr_y_prev = s_gyr_z_prev = 0.0f;

    // low-g acc (clamp)
    s_acc_initialized = false;
    s_acc_prev_state  = 0;
    s_acc_med_x = s_acc_med_y = s_acc_med_z = 0.0f;
    s_acc_mad_x = s_acc_mad_y = s_acc_mad_z = 0.0f;

    // low-g acc (biquad)
    s_hist_acc_x = (biquad_hist_t){0};
    s_hist_acc_y = (biquad_hist_t){0};
    s_hist_acc_z = (biquad_hist_t){0};

    // high-g acc (clamp)
    s_hg_initialized = false;
    s_hg_prev_state  = 0;
    s_hg_med_x = s_hg_med_y = s_hg_med_z = 0.0f;
    s_hg_mad_x = s_hg_mad_y = s_hg_mad_z = 0.0f;

    // high-g acc (biquad)
    s_hist_hg_x = (biquad_hist_t){0};
    s_hist_hg_y = (biquad_hist_t){0};
    s_hist_hg_z = (biquad_hist_t){0};
}

void sensor_filter_acc(imu_local_3d_t* local_acc, uint8_t flight_state) {
    if (!local_acc) return;

    const robust_params_t rp = s_params[SENSOR_PROFILE_LOW_G_ACC];
    const biquad_params_t bp = s_biquad[SENSOR_PROFILE_LOW_G_ACC];

    // Re-seed on flight-state change or first run
    if (!s_acc_initialized || flight_state != s_acc_prev_state) {
        s_acc_prev_state  = flight_state;
        s_acc_initialized = true;

        // Seed Hampel (median = current, MAD = eps)
        s_acc_med_x = local_acc->x;  s_acc_mad_x = rp.epsilon_mad;
        s_acc_med_y = local_acc->y;  s_acc_mad_y = rp.epsilon_mad;
        s_acc_med_z = local_acc->z;  s_acc_mad_z = rp.epsilon_mad;

        // Seed biquad histories to current (avoid startup pop); pass-through this frame
        s_hist_acc_x = (biquad_hist_t){ .x1=local_acc->x, .x2=local_acc->x,
                                        .y1=local_acc->x, .y2=local_acc->x };
        s_hist_acc_y = (biquad_hist_t){ .x1=local_acc->y, .x2=local_acc->y,
                                        .y1=local_acc->y, .y2=local_acc->y };
        s_hist_acc_z = (biquad_hist_t){ .x1=local_acc->z, .x2=local_acc->z,
                                        .y1=local_acc->z, .y2=local_acc->z };
        return;
    }

    /* ---- 1) Hampel-style clamp per axis (in-place) ---- */
    float med_x_new = hampel_update_median(s_acc_med_x, local_acc->x, rp.beta_h);
    float mad_x_new = hampel_update_mad(s_acc_mad_x, s_acc_med_x, local_acc->x, rp.beta_h, rp.epsilon_mad);
    float x_clamped = hampel_clamp(local_acc->x, med_x_new, mad_x_new, rp.k_clamp);

    float med_y_new = hampel_update_median(s_acc_med_y, local_acc->y, rp.beta_h);
    float mad_y_new = hampel_update_mad(s_acc_mad_y, s_acc_med_y, local_acc->y, rp.beta_h, rp.epsilon_mad);
    float y_clamped = hampel_clamp(local_acc->y, med_y_new, mad_y_new, rp.k_clamp);

    float med_z_new = hampel_update_median(s_acc_med_z, local_acc->z, rp.beta_h);
    float mad_z_new = hampel_update_mad(s_acc_mad_z, s_acc_med_z, local_acc->z, rp.beta_h, rp.epsilon_mad);
    float z_clamped = hampel_clamp(local_acc->z, med_z_new, mad_z_new, rp.k_clamp);

    // commit clamp state
    s_acc_med_x = med_x_new; s_acc_mad_x = mad_x_new;
    s_acc_med_y = med_y_new; s_acc_mad_y = mad_y_new;
    s_acc_med_z = med_z_new; s_acc_mad_z = mad_z_new;

    /* ---- 2) Biquad (DF1) on clamped values; write back in-place ---- */
    float x_f = biquad_df1(&bp, &s_hist_acc_x, x_clamped);
    float y_f = biquad_df1(&bp, &s_hist_acc_y, y_clamped);
    float z_f = biquad_df1(&bp, &s_hist_acc_z, z_clamped);

    local_acc->x = x_f;
    local_acc->y = y_f;
    local_acc->z = z_f;
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

    float xf = ema_1st(local_gyr->x, s_gyr_x_prev);
    float yf = ema_1st(local_gyr->y, s_gyr_y_prev);
    float zf = ema_1st(local_gyr->z, s_gyr_z_prev);

    local_gyr->x = xf; local_gyr->y = yf; local_gyr->z = zf;
    s_gyr_x_prev = xf; s_gyr_y_prev = yf; s_gyr_z_prev = zf;
}

void sensor_filter_mag(imu_local_3d_t* local_mag, uint8_t flight_state) {
    (void)flight_state; (void)local_mag;
    // pass-through for now
}

void sensor_filter_high_g_acc(imu_float_3d_t* high_g_acc, uint8_t flight_state) {
    if (!high_g_acc) return;

    const robust_params_t rp = s_params[SENSOR_PROFILE_HIGH_G_ACC];
    const biquad_params_t bp = s_biquad[SENSOR_PROFILE_HIGH_G_ACC];

    // Convert incoming double samples to float for internal processing
    float x_in = (float)high_g_acc->x;
    float y_in = (float)high_g_acc->y;
    float z_in = (float)high_g_acc->z;

    // Re-seed on flight-state change or first run
    if (!s_hg_initialized || flight_state != s_hg_prev_state) {
        s_hg_prev_state  = flight_state;
        s_hg_initialized = true;

        // Seed Hampel (median = current, MAD = eps)
        s_hg_med_x = x_in;  s_hg_mad_x = rp.epsilon_mad;
        s_hg_med_y = y_in;  s_hg_mad_y = rp.epsilon_mad;
        s_hg_med_z = z_in;  s_hg_mad_z = rp.epsilon_mad;

        // Seed biquad histories to current (avoid startup pop); pass-through this frame
        s_hist_hg_x = (biquad_hist_t){ .x1=x_in, .x2=x_in, .y1=x_in, .y2=x_in };
        s_hist_hg_y = (biquad_hist_t){ .x1=y_in, .x2=y_in, .y1=y_in, .y2=y_in };
        s_hist_hg_z = (biquad_hist_t){ .x1=z_in, .x2=z_in, .y1=z_in, .y2=z_in };

        return;
    }

    /* ---- 1) Hampel-style clamp per axis (in-place) ---- */
    float med_x_new = hampel_update_median(s_hg_med_x, x_in, rp.beta_h);
    float mad_x_new = hampel_update_mad(s_hg_mad_x, s_hg_med_x, x_in, rp.beta_h, rp.epsilon_mad);
    float x_clamped = hampel_clamp(x_in, med_x_new, mad_x_new, rp.k_clamp);

    float med_y_new = hampel_update_median(s_hg_med_y, y_in, rp.beta_h);
    float mad_y_new = hampel_update_mad(s_hg_mad_y, s_hg_med_y, y_in, rp.beta_h, rp.epsilon_mad);
    float y_clamped = hampel_clamp(y_in, med_y_new, mad_y_new, rp.k_clamp);

    float med_z_new = hampel_update_median(s_hg_med_z, z_in, rp.beta_h);
    float mad_z_new = hampel_update_mad(s_hg_mad_z, s_hg_med_z, z_in, rp.beta_h, rp.epsilon_mad);
    float z_clamped = hampel_clamp(z_in, med_z_new, mad_z_new, rp.k_clamp);

    // commit clamp state
    s_hg_med_x = med_x_new; s_hg_mad_x = mad_x_new;
    s_hg_med_y = med_y_new; s_hg_mad_y = mad_y_new;
    s_hg_med_z = med_z_new; s_hg_mad_z = mad_z_new;

    /* ---- 2) Biquad (DF1) on clamped values; write back in-place ---- */
    float x_f = biquad_df1(&bp, &s_hist_hg_x, x_clamped);
    float y_f = biquad_df1(&bp, &s_hist_hg_y, y_clamped);
    float z_f = biquad_df1(&bp, &s_hist_hg_z, z_clamped);

    high_g_acc->x = (double)x_f;
    high_g_acc->y = (double)y_f;
    high_g_acc->z = (double)z_f;
}
