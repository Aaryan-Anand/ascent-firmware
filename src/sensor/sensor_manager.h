#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "interface_bno055.h"
#include "interface_h3lis331dl.h"
#include "interface_bmp390l.h"

/**
 * @brief Initialize I2C
 */
void i2c_init(void);

/**
 * @brief Initalize the sensor manager and load calib matricies
 */
void sensor_manager_init();

/**
 * @brief Update the ground alt for the barometer
 */
void bmp_aquire_ground();

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
 * @brief Read BMP390 barometer data
 * 
 * @param baro Pointer to store barometer data
 */
void bmp_get(baro_double_t* baro);

// Add these declarations to sensor_manager.h
esp_err_t bno_calib(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out);
void lis331_calib(imu_float_3d_t* acc_out);

// Add these declarations
esp_err_t bno_local(imu_local_3d_t* acc_out, imu_local_3d_t* gyr_out, imu_local_3d_t* mag_out, bool local_up_flipped);
void lis331_local(imu_float_3d_t* acc_out, bool local_up_flipped);

/**
 * @brief Get calibrated barometer data
 * 
 * @param baro_out Pointer to store calibrated barometer data
 */
void bmp_calib(baro_double_t* baro_out);

/**
 * @brief Get altitude above ground level
 * 
 * @param baro_out Pointer to store altitude data in local reference frame
 */
void bmp_local(baro_double_t* baro_out);

void baro_update(const baro_double_t * const baro, float *agl, float *vel, float *avg_vel);

void calibrate_gyr_bias_5s(void);

#endif // SENSOR_MANAGER_H
