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

#include "flight_config.h"
#include "lora_task.h"

#include "flightState_manager.h"


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

// Change mutex to binary semaphore
SemaphoreHandle_t i2c_semaphore;


// Add a timer task to calculate and print frequencies

void init_sensors(void) {
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    bmp_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    bno_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void bno_task(void* pvParameters) {
    while(1) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(2)) == pdTRUE) {
            bno_local(&acc, &gyr, &mag, true);
            xSemaphoreGive(i2c_semaphore);
        }
    }
}

void lis_task(void* pvParameters) {
    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
            lis331_local(&high_g_acc, true);
            xSemaphoreGive(i2c_semaphore);
        }
    }
}

void bmp_task(void* pvParameters) {
    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            bmp_calib(&barometric_agl);
            baro_task();
            xSemaphoreGive(i2c_semaphore);
        }
    }
}

void app_main(void) {
    // Create binary semaphore instead of mutex
    // i2c_semaphore = xSemaphoreCreateBinary();
    // xSemaphoreGive(i2c_semaphore); // Make it available
    
    // // Configure watchdog timer
    // esp_task_wdt_config_t wdt_config = {
    //     .timeout_ms = 5000,
    //     .idle_core_mask = (1 << 0),
    //     .trigger_panic = false
    // };
    // esp_task_wdt_init(&wdt_config);

    // // Initialize sensors with mutex protection
    // xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    // init_sensors();
    // xSemaphoreGive(i2c_semaphore);

    // // Create tasks with optimized priorities
    // xTaskCreatePinnedToCore(
    //     lis_task,          // Fastest sensor gets highest priority
    //     "lis_task",
    //     8192,  // Increased stack size
    //     NULL,
    //     1,
    //     NULL,
    //     1
    // );

    // vTaskDelay(pdMS_TO_TICKS(3));

    // xTaskCreatePinnedToCore(
    //     bno_task,          // Second fastest
    //     "bno_task",
    //     8192,
    //     NULL,
    //     1,
    //     NULL,
    //     1
    // );

    // vTaskDelay(pdMS_TO_TICKS(3));

    // xTaskCreatePinnedToCore(
    //     bmp_task,          // Slowest sensor
    //     "bmp_task",
    //     8192,
    //     NULL,
    //     1,
    //     NULL,
    //     1
    // );

    // vTaskDelay(pdMS_TO_TICKS(2));

    // xTaskCreatePinnedToCore(
    //     flight_state_manager,
    //     "FSM_task",
    //     8192,
    //     NULL,
    //     5,  // Lowest priority of core 1 tasks
    //     NULL,
    //     1
    // );
}

