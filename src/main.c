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

#define MEASURE_PERFORMANCE

void app_main(void) {
    fflush(stdout);
    i2c_init();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    GPS_init();

    vTaskDelay(3000 / portTICK_PERIOD_MS);


    #ifndef MEASURE_PERFORMANCE
    GPS_read(&timestamp, &lon, &lat, &height, &hMSL, &fixType, &numSV);
    #endif

    #ifdef MEASURE_PERFORMANCE
    
    const unsigned MEASUREMENTS = 5000;
    uint64_t start = esp_timer_get_time();

    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        GPS_read(&timestamp, &lon, &lat, &height, &hMSL, &fixType, &numSV);
    }

    uint64_t end = esp_timer_get_time();

    printf("%u iterations took %llu milliseconds (%llu microseconds per invocation)\n",
           MEASUREMENTS, (end - start)/1000, (end - start)/MEASUREMENTS);

    #endif
}

