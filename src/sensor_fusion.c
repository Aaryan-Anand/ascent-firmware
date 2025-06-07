#include "sensor_fusion.h"

#include "math.h"
#include "beep.h"
#include "stdio.h"

void vector_to_dcs(const imu_local_3d_t* vector, dcs_3d_t* dcs) {
    // Calculate vector magnitude
    float norm = sqrtf(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
    
    if (norm > 0.0f) {
        // Normalize to get direction cosines
        dcs->x = vector->x / norm;
        dcs->y = vector->y / norm;
        dcs->z = vector->z / norm;
    } else {
        // Handle zero vector case
        dcs->x = 0.0f;
        dcs->y = 0.0f;
        dcs->z = 0.0f;
    }
}

void dcs_to_degrees(const dcs_3d_t* dcs, orientation_t* orientation, dcs_type_t dcs_type) {
    // Calculate angles relative to X-axis (1,0,0)
    // Roll is rotation around X axis (longitudinal)
    orientation->roll = (dcs_type == DCS_TYPE_ACC) ? 0.0f : atan2f(dcs->y, dcs->z) * 180.0f / M_PI;
    
    // Pitch is rotation around Y axis (lateral)
    orientation->pitch = atan2f(-dcs->x, 
                               sqrtf(dcs->y * dcs->y + dcs->z * dcs->z)) * 180.0f / M_PI;
    
    // Yaw is rotation around Z axis (vertical at launch)
    orientation->yaw = atan2f(dcs->y, dcs->x) * 180.0f / M_PI;

    // Apply dcs-specific adjustments
    switch (dcs_type) {
        case DCS_TYPE_ACC:
            // For accelerometer: adjust pitch and normalize yaw to -180 to +180
            orientation->pitch += 90.0f;
            if (orientation->yaw > 180.0f) {
                orientation->yaw -= 360.0f;
            }
            break;
            
        case DCS_TYPE_MAG:
            // For magnetometer: normalize yaw to 0 to 360
            if (orientation->yaw < 0) {
                orientation->yaw += 360.0f;
            }
            break;
            
        case DCS_TYPE_NULL:
        default:
            // No special handling for NULL type
            break;
    }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Lazy-initialized function to provide initial reference direction cosines
const reference_dcs_t* get_initial_vectors() {
    static reference_dcs_t init_reference;
    static bool initialized = false;

    if (!initialized) {
        imu_local_3d_t acc, gyr, mag;
        float acc_sum;
        uint8_t acc_wait_count = 0;

        // Wait for stable gravity vector (~1g)
        while (1) {
            bno055_get_local(&acc, &gyr, &mag, false);
            acc_sum = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

            if (fabsf(acc_sum - 9.81f) <= 0.1f) {
                printf("Accelerometer validated: %.1f mg\n", acc_sum);
                break;
            }

            error_beep();
            if (++acc_wait_count > 40) {
                printf("Warning: Accelerometer not stable, vector sum: %.1f mg\n", acc_sum);
                break;
            }
        }

        // Final reading with calibration ON
        bno055_get_local(&acc, &gyr, &mag, true);
        SCALE_MAG_VECTORS(&mag);
        // Convert accelerometer vector to direction cosines
        vector_to_dcs(&acc, &init_reference.gravity);
        
        // Convert magnetic vector to direction cosines
        vector_to_dcs(&mag, &init_reference.magnetic);
        
        initialized = true;

        printf("Initial reference direction cosines set.\n");
        printf("Gravity direction cosines: [%.3f, %.3f, %.3f]\n", 
               init_reference.gravity.x, init_reference.gravity.y, init_reference.gravity.z);
        printf("Magnetic direction cosines: [%.3f, %.3f, %.3f]\n", 
               init_reference.magnetic.x, init_reference.magnetic.y, init_reference.magnetic.z);
    }

    return &init_reference;
}

void get_mag_orientation(imu_local_3d_t* mag, dcs_3d_t* body_relative_dcs, orientation_t* orientation) {
    dcs_3d_t current_mag_dcs;
    const reference_dcs_t* ref = get_initial_vectors();

    // Scale magnetometer vectors if enabled
    SCALE_MAG_VECTORS(mag);

    // Convert current magnetic vector to direction cosines
    vector_to_dcs(mag, &current_mag_dcs);

    // Calculate relative magnetic direction cosines by subtracting reference
    body_relative_dcs->x = current_mag_dcs.x - ref->magnetic.x;
    body_relative_dcs->y = current_mag_dcs.y - ref->magnetic.y;
    body_relative_dcs->z = current_mag_dcs.z - ref->magnetic.z;

    // Add gravity reference to get final orientation
    body_relative_dcs->x -= ref->gravity.x;
    body_relative_dcs->y -= ref->gravity.y;
    body_relative_dcs->z -= ref->gravity.z;


    // Convert direction cosines to orientation angles (using magnetometer mode)
    dcs_to_degrees(body_relative_dcs, orientation, DCS_TYPE_MAG);
}

void get_acc_orientation(const imu_local_3d_t* acc, orientation_t* orientation) {
    dcs_3d_t acc_dcs;
    
    // Convert accelerometer vector to direction cosines
    vector_to_dcs(acc, &acc_dcs);
    
    // Convert direction cosines to orientation angles (using accelerometer mode)
    dcs_to_degrees(&acc_dcs, orientation, DCS_TYPE_ACC);
}