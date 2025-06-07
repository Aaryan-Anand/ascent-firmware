#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "interface_bno055.h"

#define SCALE_MAG_VECTORS

// Scale magnetometer vectors to normalized range
#ifdef SCALE_MAG_VECTORS
#define SCALE_MAG_VECTORS(mag) do { \
    (mag)->x = mapf((mag)->x, -59.0f, -130.0f, -10.0f, 10.0f); \
    (mag)->y = mapf((mag)->y, -80.0f, 10.0f, -10.0f, 10.0f); \
    (mag)->z = mapf((mag)->z, 35.0f, 117.0f, -10.0f, 10.0f); \
} while(0)
#else
#define SCALE_MAG_VECTORS(mag) ((void)0)
#endif

typedef enum {
    DCS_TYPE_NULL = 0,
    DCS_TYPE_ACC,
    DCS_TYPE_MAG
} dcs_type_t;

typedef struct {
    float x;
    float y;
    float z;
} dcs_3d_t;

typedef struct {
    dcs_3d_t gravity;
    dcs_3d_t magnetic;
} reference_dcs_t;

typedef struct {
    float roll;   // around X-axis (longitudinal)
    float pitch;  // around Y-axis (lateral)
    float yaw;    // around Z-axis (vertical at launch)
} orientation_t;

// Convert IMU vector to direction cosines
void vector_to_dcs(const imu_local_3d_t* vector, dcs_3d_t* dcs);

// Convert direction cosines to angles relative to X-axis (1,0,0)
// Uses dcs_type to determine angle calculation and normalization
void dcs_to_degrees(const dcs_3d_t* dcs, orientation_t* orientation, dcs_type_t dcs_type);

// Get orientation from accelerometer data
void get_acc_orientation(const imu_local_3d_t* acc, orientation_t* orientation);

// Lazy-initialized function to provide initial reference direction cosines
const reference_dcs_t* get_initial_vectors();

// Get orientation from magnetometer data
void get_mag_orientation(imu_local_3d_t* mag, dcs_3d_t* body_relative_dcs, orientation_t* orientation);

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#endif