#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "interface_bno055.h"

typedef struct {
    imu_raw_3d_t acc;
    imu_raw_3d_t mag;
} initial_vectors_t;


// Lazy-initialized function to provide initial reference vectors
const initial_vectors_t* get_initial_vectors();


typedef struct {
    float roll;   // around X-axis (longitudinal)
    float pitch;  // around Y-axis (lateral)
    float yaw;    // around Z-axis (vertical at launch)
} Orientation;

Orientation get_mag_orientation_with_reference(float mag_x, float mag_y, float mag_z);


#endif