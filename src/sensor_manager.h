#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// === Structs ===

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_raw_3d_t;

typedef struct {
    double x;
    double y;
    double z;
} imu_float_3d_t;

// === Sensor Initialization Functions ===
/**
 * @brief Initialize I2C
 */
void i2c_init(void);

/**
 * @brief Initialize BMP390 and calibrate for ground pressure.
 */
void bmp_flight_init(void);

/**
 * @brief Initialize BNO055 IMU with high-G interrupt for liftoff detection.
 */
void bno_flight_init(void);

/**
 * @brief Initialize H3LIS331DL high-G accelerometer with liftoff interrupt.
 */
void lis331_flight_init(void);

// === Sensor Data Fetch Functions ===

/**
 * @brief Read acceleration, gyroscope, and magnetometer from BNO055.
 * 
 * @param acc Pointer to store accelerometer data (raw 16-bit)
 * @param gyr Pointer to store gyroscope data (raw 16-bit)
 * @param mag Pointer to store magnetometer data (raw 16-bit)
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t bno_get(imu_raw_3d_t* acc, imu_raw_3d_t* gyr, imu_raw_3d_t* mag);

/**
 * @brief Read high-G acceleration data (in g) from H3LIS331DL.
 * 
 * @param acc Pointer to store acceleration in g
 */
void lis331_get(imu_float_3d_t* acc);

/**
 * @brief Read BMP
 * 
 * @param acc Pointer to store agl in m
 */
void bmp_get(float* bmp_agl);

// Add these declarations to sensor_manager.h
esp_err_t bno_calib(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out);
void lis331_calib(imu_float_3d_t* acc_out);

// Add these declarations
esp_err_t bno_local(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out, bool local_up_flipped);
void lis331_local(imu_float_3d_t* acc_out, bool local_up_flipped);

#endif // SENSOR_MANAGER_H
