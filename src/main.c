#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "interface_bmp390l.h"
#include "driver_SAM_M10Q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

// === Task Handles ===
TaskHandle_t flight_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;
TaskHandle_t baro_task_handle = NULL;

// === Task Definitions ===

void flight_task(void *pvParameters) {
    while (1) {
        // Flight state machine logic here
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void gps_task(void *pvParameters) {
    while (1) {
        // GPS polling
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lora_tx(void *pvParameters) {
    while (1) {
        // LoRa telemetry
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void baro_task(void *pvParameters) {
    while (1) {
        // Poll barometer
        // Calculate velocity and acceleration
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// === Main App Entry ===

void app_main(void) {
    // Initialize drivers here

    xTaskCreate(flight_task, "flight_task", 4096, NULL, 5, &flight_task_handle);
    xTaskCreate(gps_task, "gps_task", 2048, NULL, 3, &gps_task_handle);
    xTaskCreate(lora_tx, "lora_tx", 2048, NULL, 3, &lora_task_handle);
    xTaskCreate(baro_task, "baro_task", 3072, NULL, 4, &baro_task_handle);
}