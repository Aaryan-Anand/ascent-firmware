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

// === Optional: ISR Setup for BNO055 Interrupt Handling ===

/**
 * @brief GPIO interrupt service routine for BNO055 interrupt pin.
 * 
 * @param arg Pointer passed from gpio_isr_handler_add (GPIO number as void* cast)
 */
void gpio_isr_handler(void* arg);

/**
 * @brief Task that waits on GPIO interrupt queue and handles BNO055 interrupt events.
 * 
 * @param arg FreeRTOS task parameter (unused)
 */
void bno_interrupt_task(void* arg);

// Add near the top with other declarations
extern QueueHandle_t gpio_evt_queue;

#endif // SENSOR_MANAGER_H
