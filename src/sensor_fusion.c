#include "sensor_fusion.h"

#include "math.h"
#include "beep.h"
#include "stdio.h"
#include "esp_timer.h"

// ==== Static Orientation State ====
static quat_t orientation_quat = {1.0f, 0.0f, 0.0f, 0.0f};

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
    orientation->pitch = atan2f(-dcs->x, dcs->z) * 180.0f / M_PI;
    
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
            if (orientation->pitch > 180.0f) {
                orientation->pitch -= 360.0f;
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
                printf("Accelerometer validated: %.1f m/s^2\n", acc_sum);
                break;
            }

            error_beep();
            if (++acc_wait_count > 40) {
                printf("Warning: Accelerometer not stable, vector sum: %.1f m/s^2\n", acc_sum);
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

// ==== Helper Functions ====

// Converts Euler angles (deg) to quaternion
void euler_to_quaternion(const orientation_t* euler, quat_t* quat) {
    float roll_rad  = euler->roll  * M_PI / 180.0f;
    float pitch_rad = euler->pitch * M_PI / 180.0f;
    float yaw_rad   = euler->yaw   * M_PI / 180.0f;

    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;

    float norm = sqrtf(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    if (norm > 0.0f) {
        quat->w /= norm;
        quat->x /= norm;
        quat->y /= norm;
        quat->z /= norm;
    }
}

// Converts quaternion to Euler angles (deg), aerospace ZYX (yaw-pitch-roll)
void quaternion_to_euler(const quat_t* quat, orientation_t* euler) {
    float norm = sqrtf(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    float qw = quat->w / norm;
    float qx = quat->x / norm;
    float qy = quat->y / norm;
    float qz = quat->z / norm;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(90.0f, sinp);
    } else {
        euler->pitch = asinf(sinp) * 180.0f / M_PI;
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

    // Normalize ranges
    if (euler->roll > 180.0f) euler->roll -= 360.0f;
    if (euler->roll < -180.0f) euler->roll += 360.0f;
    if (euler->yaw < 0.0f)    euler->yaw += 360.0f;
}

// ==== Main Integration Function ====

// Call this at fixed intervals or in your sensor update callback
// Pass in latest gyro values in deg/s
void update_orientation_from_gyro(const imu_local_3d_t* gyr) {
    static int64_t last_gyr_timestamp = 0;
    static bool first_call = true;

    // Platform-specific: replace with your timer function as needed
    int64_t current_time = esp_timer_get_time(); // microseconds

    if (first_call) {
        last_gyr_timestamp = current_time;
        first_call = false;
        return;
    }

    float dt = (current_time - last_gyr_timestamp) / 1000000.0f; // seconds
    last_gyr_timestamp = current_time;

    // Convert rates to rad/s
    float wx = gyr->x * M_PI / 180.0f;
    float wy = gyr->y * M_PI / 180.0f;
    float wz = gyr->z * M_PI / 180.0f;

    // Omega quaternion [0, wx, wy, wz]
    quat_t q = orientation_quat;
    quat_t q_dot;
    q_dot.w = -0.5f * (q.x * wx + q.y * wy + q.z * wz);
    q_dot.x =  0.5f * (q.w * wx + q.y * wz - q.z * wy);
    q_dot.y =  0.5f * (q.w * wy + q.z * wx - q.x * wz);
    q_dot.z =  0.5f * (q.w * wz + q.x * wy - q.y * wx);

    // Integrate
    orientation_quat.w += q_dot.w * dt;
    orientation_quat.x += q_dot.x * dt;
    orientation_quat.y += q_dot.y * dt;
    orientation_quat.z += q_dot.z * dt;

    // Normalize
    float norm = sqrtf(orientation_quat.w * orientation_quat.w +
                       orientation_quat.x * orientation_quat.x +
                       orientation_quat.y * orientation_quat.y +
                       orientation_quat.z * orientation_quat.z);
    if (norm > 0.0f) {
        orientation_quat.w /= norm;
        orientation_quat.x /= norm;
        orientation_quat.y /= norm;
        orientation_quat.z /= norm;
    }
}

// ==== Output/Reset Functions ====

// Use this to read current orientation in Euler angles (deg)
void get_orientation_euler(orientation_t* euler) {
    quaternion_to_euler(&orientation_quat, euler);
    euler->yaw = euler->yaw-180;
}

// Use this to manually set orientation from Euler (deg), e.g., at startup or after a reset
void set_orientation_euler(const orientation_t* euler) {
    euler_to_quaternion(euler, &orientation_quat);
}

void euler_rates_to_quaternion_derivative(const quat_t* current_quat, 
                                        const imu_local_3d_t* rates, 
                                        quat_t* quat_derivative) {
    // Convert angular rates from degrees/s to radians/s
    float wx = rates->x * M_PI / 180.0f;
    float wy = rates->y * M_PI / 180.0f;
    float wz = rates->z * M_PI / 180.0f;

    // Calculate quaternion derivative using the quaternion kinematic equation:
    // q̇ = 0.5 * q ⊗ [0, ωx, ωy, ωz]
    quat_derivative->w = -current_quat->x * wx - current_quat->y * wy - current_quat->z * wz;
    quat_derivative->x =  current_quat->w * wx + current_quat->y * wz - current_quat->z * wy;
    quat_derivative->y =  current_quat->w * wy + current_quat->z * wx - current_quat->x * wz;
    quat_derivative->z =  current_quat->w * wz + current_quat->x * wy - current_quat->y * wx;

    // Then scale by 0.5
    quat_derivative->w *= 0.5f;
    quat_derivative->x *= 0.5f;
    quat_derivative->y *= 0.5f;
    quat_derivative->z *= 0.5f;
}
