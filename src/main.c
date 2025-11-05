//MARK: - ESP-IDF
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
#include "driver/gpio.h"
#include "serial_util.h"

#include "neopixel.h"
#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21
static tNeopixelContext neopixel;





//MARK: - ASCENT Drivers
#include "ascent_r2_hardware_definition.h"
//managers
#include "i2c_manager.h"
#include "spi_manager.h"
//drivers
#include "driver_bno055.h"
#include "driver_H3LIS331DL.h"
#include "driver_BMP390L.h"
#include "driver_w25qxx.h"
#include "lora.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_buzzer.h"
//interfaces
#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "beep.h"





//MARK: - SRC Includes
#include "main.h"
#include "globals.h"
//FSM
#include "flight.h"
#include "flight_config.h"
//Memory
#include "flash_interface.h"
#include "nvs_interface.h"
//radio
#include "lora_interface.h"
#include "goober.h"
//Sensor
#include "sensor_manager.h"
#include "sensor_filtering.h"
#include "orientation.h"
#include "sensor_fusion.h"
//SITL
#include "sitl.h"
//BLE
#include "ble.h"







//MARK: - DEBUG Defines
// #define GENERAL_DEBUG
//#define FUSION_DEBUG
//#define DEBUG











//MARK: - Primary Task
TaskHandle_t primary_task_handle;
int primary_loop_fq = 50;
TickType_t xFrequency_primary;
void primary_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint32_t cycle = 0;

    GPS_data_t gps_data;

    int32_t resume = -1;
    bool flash_erase_next_bank_on_landed_finished = false;

    while (1) {
        #ifdef IS_SITL
        sitl_update();
        #endif

        uint8_t flight_state = get_flight_state();
        pyro_update_state();

        // we are not using a switch case here since we want to be able to declare variables within the different state handlers
        // I know that you can do things like an an empty statement but that is weird

        // when in preflight state we do nothing but send the battery voltage
        if (flight_state == FS_PREFLIGHT) {
            primary_preflight(&cycle, &gps_data);
        } 
        
        else if (flight_state != FS_LANDED) {
            primary_flight(&cycle, &gps_data, &flight_state);
        } 
        
        else {
            primary_landed(&cycle, &gps_data, &flight_state, &flash_erase_next_bank_on_landed_finished, &resume);
        }

        // advance our cycle counter for subsampling tasks
        cycle = (cycle + 1) % primary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_primary);
    }
}



//MARK: - Secondary Task
TaskHandle_t secondary_task_handle;
int secondary_loop_fq = 20;
TickType_t xFrequency_secondary;
void secondary_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint32_t cycle = 0;

    while (1) {
        secondary_flight(&cycle);

        cycle = (cycle + 1) % secondary_loop_fq;
        vTaskDelayUntil(&xLastWakeTime, xFrequency_secondary);
    }
}

//MARK: - Fusion Debug
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

//MARK: - Main
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    globals_init();
    init_boot_sequence();

    // Set origin vectors during boot sequence
    #ifndef FUSION_DEBUG
    fail_if_barometer_bad();
    #endif



    printf("========================\n");
    flash_print_stats();
    printf("========================\n");
    // try to go into data dumping mode
    // this will prompt the user on SERIAL to enter the word DUMP with in 5 seconds
    // if they do this it will dump all data

    #ifndef DEBUG
    try_to_dump_data();
    // try to go into data dumping mode
    // this will prompt the user on SERIAL to enter the word DUMP with in 5 seconds
    // if they do this it will dump all data
    //try_to_dump_data();
    
    // w25qxx_chip_erase();

    vTaskDelay(1000/portTICK_PERIOD_MS);

    beep_pyro_cont();

    #endif

    high_power_mode();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    update_loop_rate();

    printf("========================\n");
    flash_print_stats();
    printf("========================\n");


    orientation_init_from_gravity(&g_orientation, false);
    // used to fly ascent in one way coms only or no ground station
    // this is changed in flight_config.h
    #ifdef DEBUG
    activate_txlock();
    #endif
    
    if (atomic_load(&flight_config).arm_at_boot) {
        activate_txlock();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Creating tasks\n");
    #ifdef FUSION_DEBUG
    xFrequency_fusion_debug = pdMS_TO_TICKS(1000/fusion_debug_loop_fq);
    xTaskCreatePinnedToCore(fusion_debug_task, "fusion_debug_task", 8192, NULL, 1, &fusion_debug_task_handle, 0);
    #else
    xTaskCreatePinnedToCore(primary_task, "primary_task", 8192, NULL, 1, &primary_task_handle, 1);
    xTaskCreatePinnedToCore(secondary_task, "secondary_task", 8192, NULL, 1, &secondary_task_handle, 0);
    #endif

    // this main will not exit here even though it looks like it will.
    // the esp will not reset until all tasks are finished
    // but since the primary and secondary tasks are both infinite loops the esp will never restart
}

//MARK: - Validate ESP32
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

//MARK: - Boot Sequence
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

    nvs_interface_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    print_uuid();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    sensor_manager_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    flight_config_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    pyro_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    psu_init_default();
    vTaskDelay(10 / portTICK_PERIOD_MS);


    #ifndef DEBUG
    break_beep();
    battery_beep();
    break_beep();
    serial_util_init();
    #endif


    bmp_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    bno_flight_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    if(GPS_init() != ESP_OK) {
        for (int i = 0; i < 5; i++) {
            error_beep();
            printf("GPS FAILED!!!\n");
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        esp_restart();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);


    lora_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);


    flash_flight_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  255) } }, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ble_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

//MARK: - Pyro Beep
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













//MARK: - Read UUID
void print_uuid(void){
    uint8_t uuid[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
    nvs_retreive_uuid(uuid);
    printf("UUID: ");
    for(uint8_t i = 0; i < 5; i++) {
        printf("%c", (((uint16_t)uuid[i*2] << 4) | uuid[i*2 + 1]));
    }
    printf("%X.%X.%X-%X%X%X", uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
    printf("\n");
}

//MARK: - Update Loop Rate
void update_loop_rate(void) {
    uint8_t flight_state = get_flight_state();

    switch (flight_state) {
        case FS_PREFLIGHT:
        case FS_LANDED:
            primary_loop_fq = 1;
            secondary_loop_fq = 10;
            break;

        default:
            primary_loop_fq = 50;
            secondary_loop_fq = 20;
            break;        
    }

    xFrequency_primary = pdMS_TO_TICKS(1000/primary_loop_fq);
    xFrequency_secondary = pdMS_TO_TICKS(1000/secondary_loop_fq);
}






//MARK: - Power Modes
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

//MARK: - Primary Preflight
void primary_preflight(uint32_t *cycle, GPS_data_t *gps_data){
    if (*cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
        // in pre flight we still want to know that the gps works so just poll it at 1 Hz
        GPS_read(gps_data);

        uint8_t current_flight_state = get_flight_state();
        goober_payload_t telemetry = gooberCreateTelemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, gps_data->numSV, current_flight_state);
        queueLatestTelemetryPayload(&telemetry);

        // we still need to call flight_update to get out of preflight so we just call it with all zeros
        if (flight_update(0, 0, 0, 0)) {
            if (get_flight_state() == FS_ON_PAD) {
                update_loop_rate();
                high_power_mode();
            }
        }
    }
}








//MARK: - Primary Flight
void primary_flight(uint32_t *cycle, GPS_data_t *gps_data, uint8_t *flight_state) {
    // for all other states that are not preflight and landed we want to be running at full tilt running all flight tasks
    // the loop will now be running at 50 Hz

    // read the GPS at 10 Hz
    if (*cycle % (uint32_t)(primary_loop_fq/10) == 0) {
        GPS_read(gps_data);
        // runtime[0] = esp_timer_get_time() - start_time;
        // dt[0] = runtime[0];
    }

    // read all sensors and send data to secondary task
    if (*cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
        imu_local_3d_t local_acc, local_gyr, local_mag;
        imu_float_3d_t high_g_acc;
        //dcs_3d_t body_relative_dcs;
        baro_double_t baro;

        // bno055_get_local(&local_acc, &local_gyr, &local_mag, true);
        #ifdef IS_SITL
        bno055_get_local(&local_acc, &local_gyr, &local_mag, true);
        local_acc.x = get_current_vertical_accl();
        printf("Serial: %f\n", local_acc.x);
        #else
        bno055_get_local(&local_acc, &local_gyr, &local_mag, true);
        #endif

        h3lis331dl_get_local(&high_g_acc, true);

        // bmp390_get_local(&baro);
        #ifdef IS_SITL
        baro.alt = get_current_baro_alt();
        #else
        bmp390_get_local(&baro);
        #endif
        
        //printf("orient: %f \t %f \t %f\n", g_orientation.yaw, g_orientation.pitch, g_orientation.roll);
        
        sensor_filter_acc(&local_acc, *flight_state);
        sensor_filter_gyr(&local_gyr, *flight_state);
        sensor_filter_mag(&local_mag, *flight_state);
        sensor_filter_high_g_acc(&high_g_acc, *flight_state);
        // runtime[2] = esp_timer_get_time() - start_time;
        // dt[2] = runtime[2]-runtime[1];
        if (g_orientation_mutex && xSemaphoreTake(g_orientation_mutex, 0)) {
            orientation_update_from_euler_rates(&g_orientation, &local_gyr);
            orientation_sync_euler_from_quat(&g_orientation);
            xSemaphoreGive(g_orientation_mutex);
        } else {
            orientation_update_from_euler_rates(&g_orientation, &local_gyr);
            orientation_sync_euler_from_quat(&g_orientation);
        }
        // runtime[3] = esp_timer_get_time() - start_time;
        // dt[3] = runtime[3]-runtime[2];
        float barometric_agl;
        float barometric_velocity;
        float average_barometric_velocity;
        baro_update(&baro, &barometric_agl, &barometric_velocity, &average_barometric_velocity);
        // runtime[4] = esp_timer_get_time() - start_time;
        // dt[4] = runtime[4]-runtime[3];
        // float phi = 0;
        if (flight_update(barometric_agl, barometric_velocity, average_barometric_velocity, local_acc.x)) {
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
        // runtime[5] = esp_timer_get_time() - start_time;
        // dt[5] = runtime[5]-runtime[4];
        // always queue up latest telemetry for secondary task
        float yaw = 0, pitch = 0, roll = 0;
        if (g_orientation_mutex && xSemaphoreTake(g_orientation_mutex, 0)) {
            yaw = g_orientation.yaw;
            pitch = g_orientation.pitch;
            roll = g_orientation.roll;
            xSemaphoreGive(g_orientation_mutex);
        }
        goober_payload_t telemetry = gooberCreateTelemetry(gps_data->lat, gps_data->lon, barometric_agl, average_barometric_velocity, local_acc.x, yaw, pitch, roll, local_gyr.x, gps_data->numSV, *flight_state);
        queueLatestTelemetryPayload(&telemetry);

        flash_packet fp = {
            .n = 0,
            .timestamp = esp_timer_get_time(),
            .pyro_arm = calc_pyro_arm(),
            .flight_state = *flight_state,
            .acc = local_acc,
            .gyr = local_gyr,
            .mag = local_mag,
            .high_g_acc = high_g_acc,
            .baro = baro,
            .barometric_agl = barometric_agl,
            .barometric_velocity = barometric_velocity,
            .average_barometric_velocity = average_barometric_velocity,
            .orientation = g_orientation,
            .latitude = gps_data->lat,
            .longitude = gps_data->lon,
            .gps_altitude = gps_data->height,
            .bat_voltage = psu_read_battery_voltage(),
        };
        #ifdef DEBUG
        print_flash_packet(&fp);
        #else
        // if the board is armed, and we are not sitting on the ground before or after flight we record data to the flash
        flash_queue_packet(&fp);
        #endif
    }
}

//MARK: - Primary Landed
void primary_landed(uint32_t *cycle, GPS_data_t *gps_data, uint8_t *flight_state, bool *flash_erase_next_bank_on_landed_finished, int32_t *resume) {
    // this is the FS_LANDED case
    // here we slow down and just transmit GPS coordinates
    // here the loop rate is 2 Hz

    // since the fq is just 2 Hz just poll the gps as fast as the loop goes
    if (*cycle % (uint32_t)(primary_loop_fq/primary_loop_fq) == 0) {
        GPS_read(gps_data);

        goober_payload_t locator_packet = gooberCreateTelemetry(gps_data->lat, gps_data->lon, 0, 0, 0, 0, 0, 0, 0, gps_data->numSV, *flight_state);
        queueLatestTelemetryPayload(&locator_packet);

        if (!*flash_erase_next_bank_on_landed_finished && flash_erase_next_bank_no_advance(1000/primary_loop_fq*1e3/4, resume)) {
            *flash_erase_next_bank_on_landed_finished = true;
            flash_erase_jingle();
        }
    }
}

//MARK: - Secondary Flight
void secondary_flight(uint32_t *cycle) {
    goober_payload_t telemetry_payload;
    peekLatestTelemetryPayload(&telemetry_payload); // get the latest telemetry payload

    int rcv = 0;
    goober_t rcv_packet;
    goober_t rsp_packet;

    goober_t txlock_packet;

    uint8_t flight_state = telemetry_payload.telemetry.flight_state; // pull the latest flight flight state

    if (*cycle % (uint32_t)(secondary_loop_fq/secondary_loop_fq) == 0) {
        if(!is_tx_lock()) {
            rcv = lora_blocking_listen(&rcv_packet, 21);
            if (rcv) {
                rsp_packet = gooberSlaveResponse(rcv_packet, telemetry_payload);
                vTaskDelay(15 / portTICK_PERIOD_MS); // to allow time for the GS to switch to receive mode
                lora_transmit_packet(&rsp_packet);
            }
        } else {
            txlock_packet = gooberCreatePacket(0x41, 0, 0, 0, MSG_TYPE_POST_TELEM, sizeof(goober_post_telemetry_payload_t), &telemetry_payload);
            txlock_packet.DEV_MODE = 0x08;
            lora_transmit_packet(&txlock_packet);
        }
    }

    if (*cycle % (uint32_t)(secondary_loop_fq/secondary_loop_fq) == 0) {
        if (flight_state >FS_ON_PAD && flight_state != FS_LANDED) flash_write_queue(1000/secondary_loop_fq*1e3/4);
    }
}