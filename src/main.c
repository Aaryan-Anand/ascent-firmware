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

#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_bno055.h"

#include "driver/gpio.h"
#include "neopixel.h"

#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21

tNeopixelContext neopixel;

#include "math.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "globals.h"
#include "lora_task.h"
#include "gps_task.h"
#include "flightmgr_task.h"
#include "baro_task.h"
#include "imu_task.h"

// === Task Handles ===
TaskHandle_t flight_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;
TaskHandle_t baro_task_handle = NULL;
TaskHandle_t imu_task_handle = NULL;


// === Main App Entry ===

void app_main(void) {

    vTaskDelay(500 / portTICK_PERIOD_MS);

    tNeopixelContext neopixel = neopixel_Init(PIXEL_COUNT, NEOPIXEL_PIN);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);

    printf("\n\n\nStarting application...\n");

    fflush(stdout);

    esp_err_t ret1 = i2c_manager_deinit(I2C_NUM_0);
    if (ret1 != ESP_OK) {
        printf("Failed to deinit I2C\n");
        return;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    esp_err_t ret = i2c_manager_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ, I2C_MASTER_PORT);
    if (ret != ESP_OK) {
        printf("Failed to initialize I2C\n");
        return;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    gps_task_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    baro_task_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    buzzer_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    lora_task_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    imu_task_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    pyro_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
    note(NOTE_G, 8, 300);

    globals_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);


    pyro_activate(PYRO_CHANNEL_1, 150);
    pyro_activate(PYRO_CHANNEL_2, 150);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Create tasks without pinning to specific cores
    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 5, &flight_task_handle, 1);
    xTaskCreatePinnedToCore(lora_tx, "lora_tx", 4096, NULL, 3, &lora_task_handle, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 2048, NULL, 3, &gps_task_handle, 0);
    xTaskCreatePinnedToCore(baro_task, "baro_task", 3072, NULL, 4, &baro_task_handle, 0);
    xTaskCreatePinnedToCore(imu_task, "bno_task", 3072, NULL, 4, &imu_task_handle, 0);
}