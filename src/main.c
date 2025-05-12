#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include <inttypes.h>

#include <rom/ets_sys.h>

#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_w25qxx.h"

#include "esp_wifi.h"
#include "esp_cpu.h"
#include "xtensa/core-macros.h"

#include "spi_manager.h"

#include "beep.h"
#include "sensor_manager.h"
#include "driver/gpio.h"
#include "neopixel.h"

#include "esp_log.h"
#include "globals.h"

#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21
//static tNeopixelContext neopixel;

#include "math.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "interface_bmp390l.h"
#include "interface_bno055.h"
#include "interface_h3lis331dl.h"

#include "lora_task.h"
#include "datalogging.h"
#include "spi_manager.h"

//#include "flightState_manager.h"

#define FUNCTION_DURATION

// Define all other global variables
uint32_t timestamp;
float latitude;
float longitude;
float barometric_agl;
uint32_t gps_altitude;
float barometric_velocity;
float average_barometric_velocity;
double acceleration;
uint8_t pyro_arm = 0;
uint8_t flight_state;
uint8_t flight_event;
double batt_voltage;

// Sensor data variables (moved from sensor_manager.c)
imu_raw_3d_t acc, gyr, mag;         // BNO055 IMU data
imu_float_3d_t high_g_acc;          // H3LIS331DL high-G accelerometer data
baro_double_t baro;                 // BMP390 barometer data

extern uint32_t addr;
#include <stdlib.h>

void app_main(void) {
    printf("Initializing sensors...");
    
    // Initialize I2C bus (which also initializes the sensor interface mutexes)
    i2c_init();
    printf("BNO055 IMU initialized");
    lis331_flight_init();
    printf("H3LIS331DL accelerometer initialized");
    bmp_flight_init();
    printf("BMP390 barometer initialized");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("All sensors initialized");
    printf("Starting sensor reading loop");
    
    while (1) {
        bno055_get_local(&acc, &gyr, &mag, false);
        h3lis331dl_get_local(&high_g_acc, false);
        bmp390_get_local(&baro);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms = 100Hz update rate
    }

    // printf("Hello, World!\n");

    // psu_init_default();
    // spi_manager_init(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
    // lora_task_init();
    // uint8_t res;
    // res = w25qxx_init();
    // if (res) printf("Failed to initalize flash\n");
    
    // while (1) {
    //     printf("Trying to use lora\n");
    //     lora_task();
    //     // vTaskDelay(100 / portTICK_PERIOD_MS);

    //     printf("Trying to use flash\n");
    //     addr = rand() % 500;
    //     save_addr();
    //     // vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
}