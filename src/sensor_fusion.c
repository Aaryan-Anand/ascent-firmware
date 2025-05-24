#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "math.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "globals.h"

#include "sensor_manager.h"
#include "beep.h"
#include "flight.h"
#include "sensor_fusion.h"

// Mathematical constants
#define PI 3.14159265358979323846
#define DEG_TO_RAD_FACTOR (PI / 180.0)
#define RAD_TO_DEG_FACTOR (180.0 / PI)

double deg_to_rad(double deg) {
    return deg * DEG_TO_RAD_FACTOR;
}

double rad_to_deg(double rad) {
    return rad * RAD_TO_DEG_FACTOR;
}

bool set_origin_state_vectors(origin_vector_t* origin) {
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
    origin->gravity_vector.x = (double)acc.x / acc_magnitude;  // Normalize to unit vector
    origin->gravity_vector.y = (double)acc.y / acc_magnitude;
    origin->gravity_vector.z = (double)acc.z / acc_magnitude;
    
    // Calculate magnetic field direction cosine (unit vector pointing north)
    double mag_magnitude = sqrt((double)(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z));
    origin->magnetic_vector.x = (double)mag.x / mag_magnitude;  // Normalize to unit vector
    origin->magnetic_vector.y = (double)mag.y / mag_magnitude;
    origin->magnetic_vector.z = (double)mag.z / mag_magnitude;
    
    printf("Gravity vector: [%.3f, %.3f, %.3f]\n", 
           origin->gravity_vector.x, 
           origin->gravity_vector.y, 
           origin->gravity_vector.z);
    printf("Magnetic vector: [%.3f, %.3f, %.3f]\n", 
           origin->magnetic_vector.x, 
           origin->magnetic_vector.y, 
           origin->magnetic_vector.z);

    // Check GPS satellites
    uint8_t wait_count = 0;
    while (num_sat < 6) {
        error_beep();
        // GPS_read(NULL,NULL,NULL,NULL,NULL,NULL,NULL); // AARYAN MAKE THIS PULL FROM POINTERS IN MAIN LOOP PLEASE - abdul
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
        origin->x_pos_coord = lon;  // Longitude maps to x coordinate
        origin->y_pos_coord = lat;  // Latitude maps to y coordinate
        origin->z_pos_alt = (double)gps_alt;  // GPS altitude
    } else {
        printf("Using barometric altitude as fallback\n");
        origin->x_pos_coord = 0.0f;
        origin->y_pos_coord = 0.0f;
        origin->z_pos_alt = baro.alt;  // Barometric altitude
    }
    
    // Store orientation from magnetometer (in geographic/magnetic frame)
    // These angles represent the initial orientation of the rocket
    // relative to the geographic/magnetic frame
    origin->x_ori_geo_mag = atan2(mag.y, mag.x);  // Yaw (around Z)
    origin->y_ori_geo_mag = atan2(-mag.x, sqrt(mag.y * mag.y + mag.z * mag.z));  // Pitch (around Y)
    origin->z_ori_geo_mag = atan2(mag.z, sqrt(mag.x * mag.x + mag.y * mag.y));  // Roll (around X)

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
