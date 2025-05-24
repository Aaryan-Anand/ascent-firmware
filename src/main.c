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
static tNeopixelContext neopixel;

#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_psu.h"
#include "driver_bno055.h"
#include "driver_w25qxx.h"
#include "driver/gpio.h"

#include "i2c_manager.h"
#include "spi_manager.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "globals.h"

#include "sensor_manager.h"
#include "beep.h"
#include "fail.h"
#include "lora_interface.h"
#include "flash_interface.h"
#include "baro.h"

void validate_esp32(void);
void init_everything(void);
void beep_pyro_cont(void);
void turn_on_cameras(void);
void turn_on_fan(void);
void flash_erase_jingle(void);

void fake_ekf(float *ekf_latitude, float *ekf_longitude, float *ekf_altitude, float *ekf_pitch, float *ekf_yaw, float *ekf_roll) {
    *ekf_latitude = 0.0f;
    *ekf_longitude = 0.0f;
    *ekf_altitude = 0.0f;
    *ekf_pitch = 0.0f;
    *ekf_yaw = 0.0f;
    *ekf_roll = 0.0f;
}

void fake_gps(float *lat, float *lng, uint32_t *alt) {
    *lat = 0.0f;
    *lng = 0.0f;
    *alt = 0.0f;
}

TaskHandle_t primary_task_handle;
#define PRIMARY_LOOP_FQ ((uint32_t)30)
#define PRIMARY_LOOP_MAX_DT ((uint32_t)1e6)/PRIMARY_LOOP_FQ
void primary_task(void *pvParameters) {
    uint32_t cycle = 0;

    while (1) {
        int64_t start_time = esp_timer_get_time();

        int sum = 0;
        sum += pyro_continuity(PYRO_CHANNEL_1);
        sum += pyro_continuity(PYRO_CHANNEL_2)*2;
        uint8_t pyro_arm = sum;

        if (cycle % (uint32_t)(PRIMARY_LOOP_FQ/30) == 0) {
            imu_raw_3d_t acc, gyr, mag;
            imu_float_3d_t high_g_acc;
            bno055_get_local(&acc, &gyr, &mag, false);
            h3lis331dl_get_local(&high_g_acc, false);

            baro_double_t baro;
            bmp390_get_local(&baro);

            float barometric_agl;
            float barometric_velocity;
            float average_barometric_velocity;
            baro_update(&baro, &barometric_agl, &barometric_velocity, &average_barometric_velocity);

            float latitude;
            float longitude;
            uint32_t gps_altitude;
            fake_gps(&latitude, &longitude, &gps_altitude);

            float ekf_latitude;
            float ekf_longitude;
            float ekf_altitude;
            float ekf_pitch;
            float ekf_yaw;
            float ekf_roll;
            fake_ekf(&ekf_latitude, &ekf_longitude, &ekf_altitude, &ekf_pitch, &ekf_yaw, &ekf_roll);

            flash_packet fp = {0, esp_timer_get_time(), pyro_arm, acc, gyr, mag, high_g_acc, baro, barometric_agl, barometric_velocity, average_barometric_velocity, latitude, longitude, gps_altitude, ekf_latitude, ekf_longitude, ekf_altitude, ekf_pitch, ekf_yaw, ekf_roll};
            flash_queue_packet(&fp);
        }

        int64_t end_time = esp_timer_get_time();
        int64_t delta = end_time - start_time;
        long time_ms = delta/1e3;
        uint8_t under = (uint32_t)delta < PRIMARY_LOOP_MAX_DT;
        if (under) {
            ets_delay_us(PRIMARY_LOOP_MAX_DT-delta);
        }
        end_time = esp_timer_get_time();
        delta = end_time - start_time;
        printf("[P] Delta: %" PRId64 "us or %ldms or %f Hz. under? %d (want: 1)\n", delta, time_ms, 1.0f/(time_ms/1000.0f), under);
        // if (under) neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
        // else neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);

        cycle = (cycle + 1) % PRIMARY_LOOP_FQ;
    }
}

TaskHandle_t secondary_task_handle;
#define SECONDARY_LOOP_FQ ((uint32_t)5)
#define SECONDARY_LOOP_MAX_DT ((uint32_t)1e6)/SECONDARY_LOOP_FQ
void secondary_task(void *pvParameters) {
    uint32_t cycle = 0;

    lora_packet_t lora_packet;

    while (1) {
        int64_t start_time = esp_timer_get_time();

        // if (cycle % (uint32_t)(SECONDARY_LOOP_FQ/15) == 0) {
        if (cycle % (uint32_t)(SECONDARY_LOOP_FQ/5) == 0) {
            /*
            uint32_t timestamp; (will be auto set by transmit function)
            float latitude;
            float longitude;
            float barometric_agl;
            uint32_t gps_altitude;
            float barometric_velocity;
            float acceleration;
            uint8_t pyro_arm;
            uint8_t flight_state;
            double batt_voltage;
            */
            lora_packet.latitude = 0;
            lora_packet.longitude = 0;
            lora_packet.barometric_agl = 0;
            lora_packet.gps_altitude = 0;
            lora_packet.barometric_velocity = 0;
            lora_packet.acceleration = 0;

            int sum = 0;
            sum += pyro_continuity(PYRO_CHANNEL_1);
            sum += pyro_continuity(PYRO_CHANNEL_2)*2;
            lora_packet.pyro_arm = sum;

            lora_packet.batt_voltage = psu_read_battery_voltage();

            lora_transmit_packet(&lora_packet);
        }

        // if (cycle % (uint32_t)(SECONDARY_LOOP_FQ/15) != 0) {
        // if (cycle % (uint32_t)(SECONDARY_LOOP_FQ/15) == 0) {
        if (cycle % (uint32_t)(SECONDARY_LOOP_FQ/5) == 0) {
            flash_write_queue(SECONDARY_LOOP_MAX_DT/2);
        }

        int64_t end_time = esp_timer_get_time();
        int64_t delta = end_time - start_time;
        long time_ms = delta/1e3;
        uint8_t under = (uint32_t)delta < SECONDARY_LOOP_MAX_DT;
        if (under) {
            ets_delay_us(SECONDARY_LOOP_MAX_DT-delta);
        }
        end_time = esp_timer_get_time();
        delta = end_time - start_time;
        printf("[S] Delta: %" PRId64 "us or %ldms or %f Hz. under? %d (want: 1)\n", delta, time_ms, 1.0f/(time_ms/1000.0f), under);
        // if (under) neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
        // else neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);

        cycle = (cycle + 1) % SECONDARY_LOOP_FQ;
    }
}

void app_main(void) {
    esp_err_t err;

    validate_esp32();

    err = esp_task_wdt_deinit();
    if (err != ESP_OK) {
        printf("FAILED TO DEINIT TASK WATCH DOG\n");
        return;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    validate_esp32();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    init_everything();

    boot_sound();

    if (should_dump_data()) {
        flash_dump_to_serial();
    } else {
        turn_on_cameras();
        // turn_on_fan();

        beep_pyro_cont();

        flash_prepare_for_flight();
        flash_erase_jingle();

        // TaskHandle_t megolavania_task_handle;
        // xTaskCreatePinnedToCore(megolavania_task, "megolavania_task", 4096, NULL, 1, &megolavania_task_handle, 0);

        vTaskDelay(100 / portTICK_PERIOD_MS);

        xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 1);
        xTaskCreatePinnedToCore(secondary_task, "secondary_task", 8192, NULL, 1, &secondary_task_handle, 0);
    }

    // this main will not exit here even though it looks like it will.
    // the esp will not reset until all tasks are finished
    // but since the primary and secondary tasks are both infinite loops the esp will never restart
}

void validate_esp32(void) {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    printf("\n=== ESP32 System Information ===\n");
    
    uint32_t cpu_freq = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    printf("CPU Clock Speed: %lu MHz\n", cpu_freq);
    
    if (chip_info.cores > 0) {
        printf("Number of Cores: %d\n", chip_info.cores);
    } else {
        printf("Error: Could not detect CPU cores\n");
    }
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    flash_size = flash_size / (1024 * 1024);
    if (flash_size > 0) {
        printf("Flash Size: %lu MB\n", flash_size);
    } else {
        printf("Error: Could not detect flash size\n");
    }
    
    #ifdef CONFIG_SPIRAM
        size_t psram_size = esp_psram_get_size();
        size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        printf("PSRAM Size: %u MB\n", psram_size / (1024 * 1024));
        printf("Free PSRAM: %u MB\n", free_psram / (1024 * 1024));
        
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
        printf("Largest free PSRAM block: %u bytes\n", info.largest_free_block);
    #else
        printf("PSRAM: Not enabled in config\n");
    #endif
    printf("==============================\n\n");
}

void init_everything(void) {
    // NeoPixel
    neopixel = neopixel_Init(PIXEL_COUNT, NEOPIXEL_PIN);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);

    fflush(stdout);

    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    spi_manager_init(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
    vTaskDelay(pdMS_TO_TICKS(50));

    bmp_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    bno_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    GPS_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    buzzer_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    lora_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    pyro_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    psu_init_default();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    flash_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
    note(NOTE_G, 8, 300);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void beep_pyro_cont(void) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            if (pyro_continuity(j+1)) note(NOTE_E, 8, 300);
            else note(NOTE_G, 5, 300);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void turn_on_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,0,1); 
}

void turn_on_fan(void) {
    pyro_activate(PYRO_CHANNEL_4,0,1); 
}

void fail(int n)
{
    while (1) {
        printf("FAIL STATE %d\n", n);
        for (int i = 0; i < n; i++) {
            neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);
            vTaskDelay(100/portTICK_PERIOD_MS);
            neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

void flash_erase_jingle(void) {
    note(NOTE_E, 8, 120);
    note(NOTE_G, 8, 120);
    note(NOTE_C, 7, 200);
    note(NOTE_D, 7, 120);
    note(NOTE_B, 6, 250);
    note(NOTE_E, 7, 400);
}