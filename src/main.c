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
#include "orientation.h"

#include "neopixel.h"
#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21
static tNeopixelContext neopixel;

//#define FUSION_DEBUG

#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_psu.h"
#include "driver_bno055.h"
#include "driver_BMP390L.h"
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
#include "flight.h"
#include "flight_config.h"
#include "sensor_fusion.h"
#include "sensor_filtering.h"
#include "serial_util.h"

#include "sitl.h"

#include "sensor_fusion.h"
// #define GENERAL_DEBUG
// #define ARM_REGARDLESS_OF_TXLOCK

SemaphoreHandle_t spi_bus_mutex = NULL;

void validate_esp32(void);
void init_boot_sequence(void);
void beep_pyro_cont(void);
void turn_on_cameras(void);
void turn_on_fan(void);
void flash_erase_jingle(void);
void fail_if_barometer_bad(void);

void update_loop_rate(void);
void low_power_mode_no_gps(void);
void high_power_mode(void);

// from lora_interface.c
void turn_off_cameras(void);
void turn_off_fan(void);
void fake_tx_lock(void);

uint8_t calc_pyro_arm(void);

void try_to_dump_data();

TaskHandle_t primary_task_handle;
int primary_loop_fq = 50;
TickType_t xFrequency_primary;
void primary_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint32_t cycle = 0;

    int32_t lon;
    int32_t lat;
    int32_t gps_altitude;
    int32_t hMSL;
    uint8_t fixType;
    uint8_t numSV;  

    while (1) {
        uint8_t flight_state = get_flight_state();

        // we are not using a switch case here since we want to be able to declare variables within the different state handlers
        // I know that you can do things like an an empty statement but that is weird

        // when in preflight state we do nothing but send the battery voltage
        if (flight_state == FS_PREFLIGHT) {
            if (cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
                // in pre flight we still want to know that the gps works so just poll it at 1 Hz
                uint32_t UTCtstamp;
                GPS_read(&UTCtstamp, &lon, &lat, &gps_altitude, &hMSL, &fixType, &numSV);

                uint8_t current_flight_state = get_flight_state();
                goober_payload_t telemetry = create_telemetry_payload(0, 0, 0, 0, 0, 0, 0, 0, 0, numSV, current_flight_state);
                lora_queue_packet(&telemetry);

                // we still need to call flight_update to get out of preflight so we just call it with all zeros
                if (flight_update(0, 0, 0, 0)) {
                    if (get_flight_state() == FS_ON_PAD) {
                        update_loop_rate();
                        high_power_mode();
                    }
                }
            }
        } else if (flight_state != FS_LANDED) {
            // for all other states that are not preflight and landed we want to be running at full tilt running all flight tasks
            // the loop will now be running at 50 Hz

            // read the GPS at 10 Hz
            if (cycle % (uint32_t)(primary_loop_fq/10) == 0) {
                uint32_t UTCtstamp;
                GPS_read(&UTCtstamp, &lon, &lat, &gps_altitude, &hMSL, &fixType, &numSV);
            }

            // read all sensors and send data to secondary task
            if (cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
                imu_local_3d_t local_acc, local_gyr, local_mag;
                imu_float_3d_t high_g_acc;
                //dcs_3d_t body_relative_dcs;
                baro_double_t baro;

                // bno055_get_local(&local_acc, &local_gyr, &local_mag, true);
#ifdef IS_SITL
                bno055_get_local(&local_acc, &local_gyr, &local_mag, false);
                local_acc.x = get_current_vertical_accl();
                printf("Serial: %f\n", local_acc.x);
#else
                bno055_get_local(&local_acc, &local_gyr, &local_mag, false);
#endif

                h3lis331dl_get_local(&high_g_acc, true);

                // bmp390_get_local(&baro);
#ifdef IS_SITL
                baro.alt = get_current_baro_alt();
#else
                bmp390_get_local(&baro);
#endif
                
                //printf("orient: %f \t %f \t %f\n", g_orientation.yaw, g_orientation.pitch, g_orientation.roll);
                
                sensor_filter_acc(&local_acc, flight_state);
                sensor_filter_gyr(&local_gyr, flight_state);
                sensor_filter_mag(&local_mag, flight_state);
                sensor_filter_high_g_acc(&high_g_acc, flight_state);
                
                if (g_orientation_mutex && xSemaphoreTake(g_orientation_mutex, pdMS_TO_TICKS(5))) {
                    orientation_update_from_euler_rates(&g_orientation, &local_gyr);
                    orientation_sync_euler_from_quat(&g_orientation);
                    xSemaphoreGive(g_orientation_mutex);
                } else {
                    orientation_update_from_euler_rates(&g_orientation, &local_gyr);
                    orientation_sync_euler_from_quat(&g_orientation);
                }

                float barometric_agl;
                float barometric_velocity;
                float average_barometric_velocity;
                baro_update(&baro, &barometric_agl, &barometric_velocity, &average_barometric_velocity);

                imu_local_3d_t acc;
                accl_update(local_acc, &acc);

                // float phi = 0;
                if (flight_update(barometric_agl, barometric_velocity, average_barometric_velocity, acc.x)) {
                    // if the flight state changes we may need to update the loop rates
                    // the loop rates will only change if we go into landed
                    update_loop_rate();

                    if (get_flight_state() == FS_PREFLIGHT) {
                        low_power_mode_no_gps();
                    }

                    // if we just went into landed power down everything except GPS
                    if (get_flight_state() == FS_LANDED) {
                        low_power_mode_no_gps();
                        turn_off_cameras();
                        turn_off_fan();
                    }
                }

                // always queue up latest telemetry for secondary task
                float yaw = 0, pitch = 0, roll = 0;
                if (g_orientation_mutex && xSemaphoreTake(g_orientation_mutex, pdMS_TO_TICKS(5))) {
                    yaw = g_orientation.yaw;
                    pitch = g_orientation.pitch;
                    roll = g_orientation.roll;
                    xSemaphoreGive(g_orientation_mutex);
                }
                goober_payload_t telemetry = create_telemetry_payload(lat, lon, barometric_agl, average_barometric_velocity, acc.x, yaw, pitch, roll, local_gyr.x, numSV, flight_state);
                lora_queue_packet(&telemetry);

                // if the board is armed, and we are not sitting on the ground before or after flight we record data to the flash
                if (is_tx_lock() && flight_state != FS_ON_PAD && flight_state != FS_LANDED) {
                    flash_packet fp = {
                        .n = 0,
                        .timestamp = esp_timer_get_time(),
                        .pyro_arm = calc_pyro_arm(),
                        .flight_state = flight_state,
                        .acc = local_acc,
                        .gyr = local_gyr,
                        .mag = local_mag,
                        .high_g_acc = high_g_acc,
                        .baro = baro,
                        .barometric_agl = barometric_agl,
                        .barometric_velocity = barometric_velocity,
                        .average_barometric_velocity = average_barometric_velocity,
                        .latitude = lat,
                        .longitude = lon,
                        .gps_altitude = gps_altitude,
                        .bat_voltage = psu_read_battery_voltage(),
                    };
                    flash_queue_packet(&fp);
                }
            }
        } else {
            // this is the FS_LANDED case
            // here we slow down and just transmit GPS coordinates
            // here the loop rate is 2 Hz

            // since the fq is just 2 Hz just poll the gps as fast as the loop goes
            if (cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
                uint32_t UTCtstamp;
                GPS_read(&UTCtstamp, &lon, &lat, &gps_altitude, &hMSL, &fixType, &numSV);

                goober_payload_t telemetry = create_telemetry_payload(lat, lon, 0, 0, 0, 0, 0, 0, 0, numSV, flight_state);
                lora_queue_packet(&telemetry);
            }
        }

        // advance our cycle counter for subsampling tasks
        cycle = (cycle + 1) % primary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_primary);
    }
}

// this is really annoying when enabled
// #define SPI_MUTEX_DEBUG

TaskHandle_t flash_task_handle;
void flash_task(void *pvParameters) {
    while(1) {
        if (spi_bus_mutex != NULL) {
            if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                #ifdef SPI_MUTEX_DEBUG
                    printf("Flash task took SPI mutex\n");
                #endif
                flash_write_queue(25000); // 25000 pulled from old secondary task frequency math
                xSemaphoreGive(spi_bus_mutex);
            }
            #ifdef SPI_MUTEX_DEBUG
            else {
                printf("Flash task failed to take SPI bus mutex!\n");
            }
            #endif
        }
    }
}

TaskHandle_t lora_task_handle;
void lora_task(void *pvParameters) {
    goober_payload_t telemetry;
    
    while(1) {
        if (spi_bus_mutex != NULL) {
            if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                #ifdef SPI_MUTEX_DEBUG
                    printf("LoRa task took SPI mutex\n");
                #endif
                lora_read_latest_queue_packet(&telemetry); // read latest telemetry packet from queue
                slave_lora_task(&telemetry); // TODO: replace w/ new lora logic
                xSemaphoreGive(spi_bus_mutex);
            }
            #ifdef SPI_MUTEX_DEBUG
            else {
                printf("LoRa task failed to take SPI bus mutex!\n");
            }
            #endif
        }
    }
}

#ifdef FUSION_DEBUG
TaskHandle_t fusion_debug_task_handle;
int fusion_debug_loop_fq = 100;
TickType_t xFrequency_fusion_debug;
void fusion_debug_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t flight_state = 1;
    while (1) {
        imu_local_3d_t local_acc, local_gyr, local_mag;
        imu_float_3d_t high_g_acc;
        // Use shared orientation state
        baro_double_t baro;

        bno055_get_local(&local_acc, &local_gyr, &local_mag, true);
        h3lis331dl_get_local(&high_g_acc, true);
        bmp390_get_local(&baro);
        
        if (g_orientation_mutex && xSemaphoreTake(g_orientation_mutex, pdMS_TO_TICKS(5))) {
            orientation_update_from_euler_rates(&g_orientation, &local_gyr);
            orientation_sync_euler_from_quat(&g_orientation);
            xSemaphoreGive(g_orientation_mutex);
        } else {
            orientation_update_from_euler_rates(&g_orientation, &local_gyr);
            orientation_sync_euler_from_quat(&g_orientation);
        }

        sensor_filter_acc(&local_acc, flight_state);   // now applies Hampel-like clamp per axis
        sensor_filter_gyr(&local_gyr, flight_state);
        sensor_filter_mag(&local_mag, flight_state);
        sensor_filter_high_g_acc(&high_g_acc, flight_state);

        //printf("acc: %f \t %f \t %f\t mag: %f \t %f \t %f\t gyr: %f \t %f \t %f\n", local_acc.x, local_acc.y, local_acc.z, local_mag.x, local_mag.y, local_mag.z, local_gyr.x, local_gyr.y, local_gyr.z);
        float yaw = g_orientation.yaw;
        float pitch = g_orientation.pitch;
        float roll = g_orientation.roll;
        printf("orient: %f \t %f \t %f\n", yaw, pitch, roll);
        //printf("x:%f \t y:%f \t z:%f\n", local_acc.x, local_acc.y, local_acc.z);
        //printf("filtered acc: %f \t %f \t %f\t mag: %f \t %f \t %f\t gyr: %f \t %f \t %f\n", local_acc.x, local_acc.y, local_acc.z, local_mag.x, local_mag.y, local_mag.z, local_gyr.x, local_gyr.y, local_gyr.z);
        
        float barometric_agl;
        float barometric_velocity;
        float average_barometric_velocity;
        baro_update(&baro, &barometric_agl, &barometric_velocity, &average_barometric_velocity);

        imu_local_3d_t acc;
        accl_update(local_acc, &acc);
        vTaskDelayUntil(&xLastWakeTime, xFrequency_fusion_debug);
    }
}
#endif

void app_main(void) {
    esp_err_t err;
    //TaskHandle_t megolavania_task_handle;

#ifdef GENERAL_DEBUG
    validate_esp32();
#endif
    err = esp_task_wdt_deinit();
    if (err != ESP_OK) {
        #ifdef GENERAL_DEBUG
        printf("FAILED TO DEINIT TASK WATCH DOG\n");
        #endif
        return;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // validate_esp32();
    // vTaskDelay(100 / portTICK_PERIOD_MS);        THIS IS NOT REQUIRED ANYMORE
    
    buzzer_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ascent_beep();
    globals_init();
    init_boot_sequence();

    // Set origin vectors during boot sequence
#ifndef FUSION_DEBUG

    fail_if_barometer_bad();

    break_beep();
    battery_beep();
    break_beep();
    serial_util_init();

    printf("========================\n");
    flash_print_stats();
    printf("========================\n");
    // try to go into data dumping mode
    // this will prompt the user on SERIAL to enter the word DUMP with in 5 seconds
    // if they do this it will dump all data
    try_to_dump_data();

    // try to go into data dumping mode
    // this will prompt the user on SERIAL to enter the word DUMP with in 5 seconds
    // if they do this it will dump all data
    //try_to_dump_data();
    
    // w25qxx_chip_erase();

    vTaskDelay(1000/portTICK_PERIOD_MS);

    beep_pyro_cont();
#endif
    // xTaskCreatePinnedToCore(meergolavania_task, "megolavania_task", 4096, NULL, 1, &megolavania_task_handle, 0);

    high_power_mode();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    update_loop_rate();

    printf("========================\n");
    flash_print_stats();
    printf("========================\n");


    orientation_init_from_gravity(&g_orientation, false);
    // used to fly ascent in one way coms only or no ground station
    // this is changed in flight_config.h
#ifdef ARM_REGARDLESS_OF_TXLOCK
    fake_tx_lock();
#endif

#ifndef IS_BOOSTER
    pyro_activate(PYRO_CHANNEL_3, 100, 1);
#endif

    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Creating tasks\n");
#ifdef FUSION_DEBUG
    xFrequency_fusion_debug = pdMS_TO_TICKS(1000/fusion_debug_loop_fq);
    xTaskCreatePinnedToCore(fusion_debug_task, "fusion_debug_task", 8192, NULL, 1, &fusion_debug_task_handle, 0);
#else
    xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 1); // create primary task on core 1
    
    spi_bus_mutex = xSemaphoreCreateMutex(); // create the mutex for the spi bus

    xTaskCreatePinnedToCore(flash_task, "flash_task", 8192, NULL, 1, &flash_task_handle, 0);
    xTaskCreatePinnedToCore(lora_task, "lora_task", 8192, NULL, 1, &lora_task_handle, 0);
#endif

    // this main will not exit here even though it looks like it will.
    // the esp will not reset until all tasks are finished
    // but since the primary and secondary tasks are both infinite loops the esp will never restart
}

#ifdef GENERAL_DEBUG    
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
#endif

void init_boot_sequence(void) {
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
    vTaskDelay(pdMS_TO_TICKS(100));
    calibrate_gyr_bias_5s(true);

    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    GPS_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);


    lora_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    pyro_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    psu_init_default();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    flash_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  255) } }, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void beep_pyro_cont(void) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            if (pyro_continuity(j+1)) high_beep();
            else low_beep();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void fail_if_barometer_bad() {
    const int n = 100;
    for (int i = 0; i < n; i++) {
        baro_double_t baro_out;
        bmp390_get_local(&baro_out);
        printf("Baro test (%d/%d): pressure: %f, temp: %f, alt: %f\n", i+1, n, baro_out.pressure, baro_out.temperature, baro_out.alt);
        if (baro_out.pressure <= 0) {
            while (true) {
                error_beep();
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void update_loop_rate(void) {
    uint8_t flight_state = get_flight_state();

    switch (flight_state) {
        case FS_PREFLIGHT:
        case FS_LANDED:
            primary_loop_fq = 1;
            break;

        default:
            primary_loop_fq = 50;
            break;        
    }

    xFrequency_primary = pdMS_TO_TICKS(1000/primary_loop_fq);
}

void low_power_mode_no_gps(void) {
    printf("Entering low power mode with out gps.\n");

    bmp390_pwr_ctrl_t bmp_ctl;
    bmp_ctl.press_en = true;
    bmp_ctl.temp_en = true;
    bmp_ctl.mode = BMP390_MODE_SLEEP;
    bmp390_set_pwr_ctrl(&bmp_ctl);
    vTaskDelay(pdMS_TO_TICKS(1));

    bno_setoprmode(CONFIG);
    bno_setpowermode(SUSPEND);
    vTaskDelay(pdMS_TO_TICKS(1));

    h3lis331dl_set_power_mode(H3LIS331DL_LOW_POWER_0_5HZ);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void high_power_mode(void) {
    printf("Entering high power mode.\n");

    bmp390_pwr_ctrl_t bmp_ctl;
    bmp_ctl.press_en = true;
    bmp_ctl.temp_en = true;
    bmp_ctl.mode = BMP390_MODE_FORCED;
    bmp390_set_pwr_ctrl(&bmp_ctl);
    vTaskDelay(pdMS_TO_TICKS(1));

    bno_setoprmode(CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10));
    bno_setpowermode(NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));
    bno_setoprmode(AMG);
    vTaskDelay(pdMS_TO_TICKS(10));

    h3lis331dl_set_power_mode(H3LIS331DL_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // GPS_high_power_mode(); // THIS DOES NOT FUCKING EXIST
}

uint8_t calc_pyro_arm(void) {
    uint8_t pyro_arm = 0;
    if (pyro_continuity(PYRO_CHANNEL_1)) pyro_arm |= (1);
    if (pyro_continuity(PYRO_CHANNEL_2)) pyro_arm |= (1 << 1);
    if (pyro_continuity(PYRO_CHANNEL_3)) pyro_arm |= (1 << 2);
    if (pyro_continuity(PYRO_CHANNEL_4)) pyro_arm |= (1 << 3);

    return pyro_arm;
}

void try_to_dump_data() {
    printf("You have 5 seconds to enter \"DUMP\" to enter data dumping mode\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    char buf[512];
    int i = 0;
    while (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
        if (strcmp("DUMP", buf) == 0) {
            while (true) {
                printf("Enter the bank to dump (last bank used: %ld):\n", flash_get_last_used_bank());
                while (serial_util_readline_nonblocking(buf, 512, &i, 1000/portTICK_PERIOD_MS)) {
                    int bank = atoi(buf);
                    flash_dump_to_serial(bank);
                    for (int j = 0; j < 3; j++) {
                        flash_erase_jingle();
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                }
            }
        }
    }
}