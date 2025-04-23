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
#include "driver_bno055.h"

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

// Add near the top with other includes and definitions
volatile bool print_bno_data = false;

void app_main(void){
    i2c_init();
    bmp_flight_init();
    bno_flight_init();
    lis331_flight_init();

    while(1){
        // Read interrupt status
        bool acc_nm, acc_am, acc_high_g, gyro_drdy, 
             gyr_high_rate, gyr_am, mag_drdy, acc_drdy;
        bno_getinterruptstatus(&acc_nm, &acc_am, &acc_high_g, &gyro_drdy, 
                             &gyr_high_rate, &gyr_am, &mag_drdy, &acc_drdy);

        bno_get(&acc, &gyr, &mag);
        printf("BNO Accel: X=%d Y=%d Z=%d | AM_Int=%d | HG_Int=%d | Print=%d\n", 
               acc.x, acc.y, acc.z, 
               acc_am, // Show if high-G interrupt is triggered
               acc_high_g,
               print_bno_data);
        
        vTaskDelay(10); // Slower update rate for readable output
    }
}