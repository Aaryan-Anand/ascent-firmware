#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>  // For uint32_t

/**
 * @brief State vector containing the full 6DOF state of the rocket
 * 
 * Contains measurements in both body frame and inertial frame (ECI - Earth Centered Inertial)
 * Units:
 * - Position: meters
 * - Velocity: meters/second
 * - Acceleration: meters/second^2
 * - Orientation: radians (roll, pitch, yaw in ECI frame)
 */
typedef struct {
    // Position in body frame (m)
    double x_pos_body;
    double y_pos_body;
    double z_pos_body;

    // Position in inertial frame (ECI) (m)
    double x_pos_eci;
    double y_pos_eci;
    double z_pos_eci;

    // Velocity in body frame (m/s)
    double x_vel_body;
    double y_vel_body;
    double z_vel_body;

    // Velocity in inertial frame (ECI) (m/s)
    double x_vel_eci;
    double y_vel_eci;
    double z_vel_eci;

    // Acceleration in body frame (m/s^2)
    double x_acc_body;
    double y_acc_body;
    double z_acc_body;

    // Acceleration in inertial frame (ECI) (m/s^2)
    double x_acc_eci;
    double y_acc_eci;
    double z_acc_eci;

    // Orientation in inertial frame (ECI) (rad)
    // These angles represent the rocket's orientation relative to the ECI frame
    // x_ori: roll (rotation around x-axis)
    // y_ori: pitch (rotation around y-axis)
    // z_ori: yaw (rotation around z-axis)
    double x_ori;
    double y_ori;
    double z_ori;
} state_vector_t;

/**
 * @brief Direction cosine vectors (unit vectors) for reference frames
 */
typedef struct {
    double x;  // X component of unit vector
    double y;  // Y component of unit vector
    double z;  // Z component of unit vector
} direction_cosine_t;

/**
 * @brief Origin position vectors for coordinate system reference points
 * 
 * Stores the position of various reference points/origins in both
 * inertial (ECI) and body frames. Useful for coordinate transformations
 * and relative position calculations.
 */
typedef struct {
    double x_pos_coord;
    double y_pos_coord;
    double z_pos_alt;
    double x_ori_geo_mag;
    double y_ori_geo_mag;
    double z_ori_geo_mag;
    
    // Direction cosine vectors for reference frames
    direction_cosine_t gravity_vector;    // Unit vector pointing down in body frame
    direction_cosine_t magnetic_vector;   // Unit vector pointing north in body frame
} origin_vector_t;

/**
 * @brief Tracks the age of sensor data in milliseconds
 * 
 * Stores cycles since each sensor's data was last updated.
 * Used to determine gain values for each sensor.
 * All ages are in fusion cycle counts - number grater than zero represents stale data.
 */
typedef struct {
    uint32_t gps_age;        // Age of GPS data
    uint32_t mag_age;        // Age of magnetometer data
    uint32_t acc_age;        // Age of accelerometer data
    uint32_t high_g_acc_age; // Age of high-G accelerometer data
    uint32_t gyr_age;        // Age of gyroscope data
    uint32_t baro_age;       // Age of barometer data
} sensor_data_age_t;

/**
 * @brief Convert degrees to radians
 * @param deg Angle in degrees
 * @return Angle in radians
 */
double deg_to_rad(double deg);

/**
 * @brief Convert radians to degrees
 * @param rad Angle in radians
 * @return Angle in degrees
 */
double rad_to_deg(double rad);

/**
 * @brief Sets the origin state vectors based on sensor data
 * 
 * Function internally reads from all required sensors (BNO055, GPS, BMP390)
 * and validates the data before setting the origin vectors.
 * 
 * @param origin Pointer to origin_vector_t struct to store the data
 * @return true if origin was successfully set with valid sensor data, false otherwise
 */
bool set_origin_state_vectors(origin_vector_t* origin);

/**
 * @brief Estimates the zenith direction (up vector) using magnetic field measurements
 * 
 * Uses Rodrigues' rotation formula to compute the rotation between initial and current
 * magnetic field vectors, then applies this rotation to the initial gravity vector to
 * estimate the current zenith direction.
 * 
 * @param m0 Initial magnetic field direction cosine (unit vector)
 * @param m Current magnetic field direction cosine (unit vector)
 * @param g0 Initial gravity direction cosine (unit vector)
 * @param zenith_out Pointer to store the estimated zenith direction cosine
 */
void estimate_zenith_from_mag(
    const direction_cosine_t* m0,
    const direction_cosine_t* m,
    const direction_cosine_t* g0,
    direction_cosine_t* zenith_out
);

#endif // SENSOR_FUSION_H
