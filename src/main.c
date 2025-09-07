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
#include "driver/uart.h"
#include "math.h"

#include "neopixel.h"
#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21

#include "interface_sam_m10q.h"
#include "driver/gpio.h"

#include "i2c_manager.h"
#include "spi_manager.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "globals.h"

#include "sensor_manager.h"

int64_t start;
int64_t end;
int64_t dur;

uint32_t timestamp;
int32_t lon;
int32_t lat;
int32_t height;
int32_t hMSL;
uint8_t fixType;
uint8_t numSV;

void app_main(void) {
    fflush(stdout);
    i2c_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    GPS_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    while(1) {
        start = 0;
        end = 0;
        start = esp_timer_get_time();
        GPS_read(&timestamp, &lon, &lat, &height, &hMSL, &fixType, &numSV);
        end = esp_timer_get_time();
        dur = end - start;
        printf("UTCtstamp: %lu, lon: %ld, lat: %ld, height: %ld, hMSL: %ld, fixType: %u, numSV: %u\n", timestamp, lon, lat, height, hMSL, fixType, numSV);
        printf("GPS_read duration: %lld us\n Sleeping for 5s.\n", dur);

        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}