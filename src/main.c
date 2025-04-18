#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

double ground_pressure = 0;
float barometric_agl = 0;

float latitude;
float longitude;
uint32_t gps_altitude;

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
        parse_NMEA(&latitude, &longitude, &gps_altitude);
        printf("Latitude: %.6f, Longitude: %.6f, Altitude: %ld\n", latitude, longitude, gps_altitude);
        vTaskDelay(pdMS_TO_TICKS(50));
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
        barometric_agl = bmp390_barometricAGL();
        // printf("Barometric AGL: %.2f feet\n", barometric_agl);
        // Calculate velocity and acceleration


        vTaskDelay(pdMS_TO_TICKS(50));
        
    }
}

// === Main App Entry ===

void app_main(void) {
    // Initialize drivers here

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    printf("\n\n\nStarting application...\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    fflush(stdout);

    esp_err_t ret1 = i2c_manager_deinit(I2C_NUM_0);
    if (ret1 != ESP_OK) {
        printf("Failed to deinit I2C\n");
        return;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    esp_err_t ret = i2c_manager_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ, I2C_MASTER_PORT);
    if (ret != ESP_OK) {
        printf("Failed to initialize I2C\n");
        return;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gps_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    bmp390_sensorinit();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    update_ground_pressure();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreate(flight_task, "flight_task", 4096, NULL, 5, &flight_task_handle);
    xTaskCreate(gps_task, "gps_task", 2048, NULL, 3, &gps_task_handle);
    xTaskCreate(lora_tx, "lora_tx", 2048, NULL, 3, &lora_task_handle);
    xTaskCreate(baro_task, "baro_task", 3072, NULL, 4, &baro_task_handle);

}