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

void app_main(void){
    i2c_init();
    bmp_flight_init();
    bno_flight_init();
    lis331_flight_init();


    while(1){
        bno_local(&acc, &gyr, &mag, true);
        printf("Accel: X=%d Y=%d Z=%d \n", acc.x, acc.y, acc.z);
        // lis331_local(&high_g_acc, true);
        // printf("Accel: X=%2f Y=%2f Z=%2f Net=%2f \n", high_g_acc.x, high_g_acc.y, high_g_acc.z, sqrt(high_g_acc.x*high_g_acc.x + high_g_acc.y*high_g_acc.y + high_g_acc.z*high_g_acc.z));
        //bmp_get(&bmp_agl);
        //printf("Baro AGL = %2f \n", bmp_agl);
        vTaskDelay(10);
    }
}