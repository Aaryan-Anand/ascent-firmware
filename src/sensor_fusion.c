#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "math.h"
#include <stdbool.h>

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "globals.h"

#include "sensor_manager.h"
#include "beep.h"
#include "flight.h"
#include "sensor_fusion.h"

// Static variables for origin and zenith state
static origin_vector_t origin_vectors = {0};
static direction_cosine_t current_zenith = {0};
static bool origin_set = false;
static direction_cosine_t previous_orientation = {0};  // Store previous orientation estimate
static bool has_previous_orientation = false;  // Track if we have a valid previous estimate

// Mathematical constants
#define PI 3.14159265358979323846
#define DEG_TO_RAD_FACTOR (PI / 180.0)
#define RAD_TO_DEG_FACTOR (180.0 / PI)

extern void fake_gps(float *lat, float *lon, uint32_t *alt, uint8_t *num_sat);

double deg_to_rad(double deg) {
    return deg * DEG_TO_RAD_FACTOR;
}

double rad_to_deg(double rad) {
    return rad * RAD_TO_DEG_FACTOR;
}

bool is_origin_set(void) {
    return origin_set;
}

void update_zenith_direction(const direction_cosine_t* current_mag) {
    if (!origin_set) return;
    
    estimate_zenith_from_mag(
        &origin_vectors.magnetic_vector,
        current_mag,
        &origin_vectors.gravity_vector,
        &current_zenith
    );
}

void get_current_zenith(direction_cosine_t* zenith_out) {
    if (!origin_set) {
        // Return zero vector if origin not set
        zenith_out->x = 0.0;
        zenith_out->y = 0.0;
        zenith_out->z = 0.0;
        return;
    }
    
    // Copy current zenith
    zenith_out->x = current_zenith.x;
    zenith_out->y = current_zenith.y;
    zenith_out->z = current_zenith.z;
}

bool set_origin_state_vectors(origin_vector_t* origin) {
    // If NULL is passed, use internal static variable
    origin_vector_t* target = (origin == NULL) ? &origin_vectors : origin;
    
    // Local variables for sensor data
    imu_raw_3d_t acc, gyr, mag;
    baro_double_t baro;
    float lat = 0.0f, lon = 0.0f;
    uint32_t gps_alt = 0;
    uint8_t num_sat = 0;
    float acc_sum;
    
    // Validate accelerometer data - check if vector sum is within 0.1g of 1g
    uint8_t acc_wait_count = 0;
    while (1) {
        // Get fresh accelerometer data
        bno055_get_local(&acc, &gyr, &mag, false);
        
        // Calculate vector sum (in mg)
        acc_sum = sqrt((float)(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z));
        
        // Check if within 0.1g (100mg) of 1g (1000mg)
        if (fabs(acc_sum - 1000.0f) <= 100.0f) {
            printf("Accelerometer validated: %.1f mg\n", acc_sum);
            break;
        }
        
        error_beep();
        acc_wait_count++;
        if (acc_wait_count > 40) {
            printf("Warning: Accelerometer not stable, vector sum: %.1f mg\n", acc_sum);
            break;  // Continue anyway after timeout
        }
    }

    // Get one final reading after stability is confirmed
    bno055_get_local(&acc, &gyr, &mag, false);
    
    // Calculate gravity direction cosine (unit vector pointing down)
    double acc_magnitude = sqrt((double)(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z));
    target->gravity_vector.x = (double)acc.x / acc_magnitude;  // Normalize to unit vector
    target->gravity_vector.y = (double)acc.y / acc_magnitude;
    target->gravity_vector.z = (double)acc.z / acc_magnitude;
    
    // Calculate magnetic field direction cosine (unit vector pointing north)
    double mag_magnitude = sqrt((double)(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z));
    target->magnetic_vector.x = (double)mag.x / mag_magnitude;  // Normalize to unit vector
    target->magnetic_vector.y = (double)mag.y / mag_magnitude;
    target->magnetic_vector.z = (double)mag.z / mag_magnitude;
    
    printf("Gravity vector: [%.3f, %.3f, %.3f]\n", 
           target->gravity_vector.x, 
           target->gravity_vector.y, 
           target->gravity_vector.z);
    printf("Magnetic vector: [%.3f, %.3f, %.3f]\n", 
           target->magnetic_vector.x, 
           target->magnetic_vector.y, 
           target->magnetic_vector.z);

    // Check GPS satellites
    uint8_t wait_count = 0;
    while (num_sat < 6) {
        error_beep();
        fake_gps(&lat, &lon, &gps_alt, &num_sat);
        wait_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (wait_count > 1) {
            printf("Failed to acquire GPS fix, flying blind!\n");
            break;
        }
    }

    // Get barometric altitude
    bmp390_get_local(&baro);

    if(num_sat >= 6) {
        printf("GPS fix acquired, setting origin!\n");
        // Store GPS coordinates
        target->x_pos_coord = lon;  // Longitude maps to x coordinate
        target->y_pos_coord = lat;  // Latitude maps to y coordinate
        target->z_pos_alt = (double)gps_alt;  // GPS altitude
    } else {
        printf("Using barometric altitude as fallback\n");
        target->x_pos_coord = 0.0f;
        target->y_pos_coord = 0.0f;
        target->z_pos_alt = baro.alt;  // Barometric altitude
    }
    
    // Store orientation from magnetometer (in geographic/magnetic frame)
    // These angles represent the initial orientation of the rocket
    // relative to the geographic/magnetic frame
    target->x_ori_geo_mag = atan2(mag.y, mag.x);  // Yaw (around Z)
    target->y_ori_geo_mag = atan2(-mag.x, sqrt(mag.y * mag.y + mag.z * mag.z));  // Pitch (around Y)
    target->z_ori_geo_mag = atan2(mag.z, sqrt(mag.x * mag.x + mag.y * mag.y));  // Roll (around X)

    if (origin == NULL) {
        origin_set = true;  // Only set the flag if we're using internal storage
    }
    
    return true;  // Successfully set origin vectors
}

void estimate_zenith_from_mag(const direction_cosine_t* m0,
                              const direction_cosine_t* m,
                              const direction_cosine_t* g0,
                              direction_cosine_t* zenith_out) {
    // Compute cross product v = m0 x m
    double vx = m0->y * m->z - m0->z * m->y;
    double vy = m0->z * m->x - m0->x * m->z;
    double vz = m0->x * m->y - m0->y * m->x;

    // Compute dot product c = m0 • m
    double c = m0->x * m->x + m0->y * m->y + m0->z * m->z;

    // Norm of cross product = sin(theta)
    double s = sqrt(vx * vx + vy * vy + vz * vz);

    // Normalize rotation axis v
    if (s < 1e-6) {
        // m0 and m are nearly aligned, no rotation needed
        zenith_out->x = -g0->x;
        zenith_out->y = -g0->y;
        zenith_out->z = -g0->z;
        return;
    }

    vx /= s; vy /= s; vz /= s;

    // Rodrigues' rotation formula
    double kx = vx, ky = vy, kz = vz;
    double K[3][3] = {
        { 0, -kz, ky },
        { kz, 0, -kx },
        { -ky, kx, 0 }
    };

    double I[3][3] = {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    };

    double sin_theta = s;
    double cos_theta = c;

    // Compute R = I + sinθ * K + (1 - cosθ) * K²
    double K2[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            K2[i][j] = K[i][0] * K[0][j] + K[i][1] * K[1][j] + K[i][2] * K[2][j];

    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = I[i][j] + sin_theta * K[i][j] + (1 - cos_theta) * K2[i][j];

    // Rotate g0 using R
    double gx = g0->x, gy = g0->y, gz = g0->z;
    zenith_out->x = -(R[0][0] * gx + R[0][1] * gy + R[0][2] * gz);
    zenith_out->y = -(R[1][0] * gx + R[1][1] * gy + R[1][2] * gz);
    zenith_out->z = -(R[2][0] * gx + R[2][1] * gy + R[2][2] * gz);
}

void update_age(imu_raw_3d_t acc,imu_raw_3d_t gyr, imu_raw_3d_t mag, imu_float_3d_t hgacc, baro_double_t baro, float lat, float lon, uint32_t gps_alt, uint16_t sat,sensor_data_age_t age){
    static imu_raw_3d_t acc1, gyr1, mag1;
    static imu_float_3d_t hgacc1;
    baro_double_t baro1;
    float lat1, lon1;
    uint32_t gps_alt1;
    uint16_t sat1;

    if(acc.x == acc1.x && acc.y == acc1.y && acc.z == acc1.z){age->acc_age = age.acc_age + 1;}
    else{age->acc_age = 0;}
    if(gyr.x == gyr1.x && gyr.y == gyr1.y && gyr.z == gyr1.z){age->gyr_age = age.gyr_age + 1;}
    else{age->gyr_age = 0;}
    if(mag.x == mag1.x && mag.y == mag1.y && mag.z == mag1.z){age->mag_age = age.mag_age + 1;}
    else{age->mag_age = 0;}
    if(hgacc.x == hgacc1.x && hgacc.y == hgacc1.y && hgacc.z == hgacc1.z){age->hgacc_age = age.hgacc_age + 1;}
    else{age->hgacc_age = 0;}
    if(baro.alt == baro1.alt){age->baro_age = age.baro_age + 1;}
    else{age->baro_age = 0;}
    if(lat == lat1 && lon == lon1 && gps_alt == gps_alt1 && sat == sat1){age->gps_age = age.gps_age + 1;}
    else{age->gps_age = 0;}   
}

void estimate_orientation(
    const direction_cosine_t* current_mag,
    const imu_raw_3d_t* current_gyr,
    double dt,
    uint32_t mag_age,
    direction_cosine_t* orientation_out
) {
    // Calculate first adaptive gain based on magnetic sensor age
    const uint32_t MAX_MAG_AGE = 10;  // Maximum age threshold
    double alpha1 = 1/mag_age;
    
    // Calculate second adaptive gain based on magnetic field vector sum deviation
    // Expected magnitude is approximately 1.0 (unit vector)
    double mag_sum = sqrt(
        current_mag->x * current_mag->x +
        current_mag->y * current_mag->y +
        current_mag->z * current_mag->z
    );
    double mag_deviation = fabs(mag_sum - 48.0); //absolute deviation from expected magnitude
    const double MAX_MAG_DEVIATION = 0.2;  // Maximum allowed deviation
    double alpha2 = (mag_deviation >= MAX_MAG_DEVIATION) ? 0.0 : 1.0 / (1.0 + mag_deviation);
    
    // Combine both gains
    double alpha = alpha1 * alpha2;
    
    // First get the zenith direction using magnetic field data
    direction_cosine_t zenith;
    estimate_zenith_from_mag(
        &origin_vectors.magnetic_vector,  // Initial magnetic vector
        current_mag,                      // Current magnetic vector
        &origin_vectors.gravity_vector,   // Initial gravity vector
        &zenith                          // Output zenith direction
    );

    // Convert gyroscope readings to radians per second
    double wx = deg_to_rad(current_gyr->x);
    double wy = deg_to_rad(current_gyr->y);
    double wz = deg_to_rad(current_gyr->z);

    // Create rotation matrix from gyroscope data using small angle approximation
    double R[3][3] = {
        {1.0, -wz*dt,  wy*dt},
        {wz*dt,  1.0, -wx*dt},
        {-wy*dt, wx*dt,  1.0}
    };

    // Get gyro-based orientation by rotating either previous orientation or zenith
    direction_cosine_t gyro_orientation;
    if (has_previous_orientation) {
        // Use previous orientation as base for gyro integration
        gyro_orientation.x = R[0][0] * previous_orientation.x + R[0][1] * previous_orientation.y + R[0][2] * previous_orientation.z;
        gyro_orientation.y = R[1][0] * previous_orientation.x + R[1][1] * previous_orientation.y + R[1][2] * previous_orientation.z;
        gyro_orientation.z = R[2][0] * previous_orientation.x + R[2][1] * previous_orientation.y + R[2][2] * previous_orientation.z;
    } else {
        // First time through, use zenith as base
        gyro_orientation.x = R[0][0] * zenith.x + R[0][1] * zenith.y + R[0][2] * zenith.z;
        gyro_orientation.y = R[1][0] * zenith.x + R[1][1] * zenith.y + R[1][2] * zenith.z;
        gyro_orientation.z = R[2][0] * zenith.x + R[2][1] * zenith.y + R[2][2] * zenith.z;
    }

    // Normalize gyro orientation
    double gyro_magnitude = sqrt(
        gyro_orientation.x * gyro_orientation.x +
        gyro_orientation.y * gyro_orientation.y +
        gyro_orientation.z * gyro_orientation.z
    );
    gyro_orientation.x /= gyro_magnitude;
    gyro_orientation.y /= gyro_magnitude;
    gyro_orientation.z /= gyro_magnitude;

    // Blend between magnetic and gyro-based orientation using combined adaptive gain
    orientation_out->x = alpha * zenith.x + (1.0 - alpha) * gyro_orientation.x;
    orientation_out->y = alpha * zenith.y + (1.0 - alpha) * gyro_orientation.y;
    orientation_out->z = alpha * zenith.z + (1.0 - alpha) * gyro_orientation.z;

    // Normalize final orientation vector
    double magnitude = sqrt(
        orientation_out->x * orientation_out->x +
        orientation_out->y * orientation_out->y +
        orientation_out->z * orientation_out->z
    );

    orientation_out->x /= magnitude;
    orientation_out->y /= magnitude;
    orientation_out->z /= magnitude;

    // Store current orientation for next iteration
    previous_orientation = *orientation_out;
    has_previous_orientation = true;
}
