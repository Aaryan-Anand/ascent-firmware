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

#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_bno055.h"
#include "driver_w25qxx.h"

#include "esp_wifi.h"
#include "esp_cpu.h"
#include "xtensa/core-macros.h"

#include "spi_manager.h"

#include "beep.h"

#include "driver/gpio.h"
#include "neopixel.h"

#include "sensor_manager.h"

#define PIXEL_COUNT  1
#define NEOPIXEL_PIN GPIO_NUM_21
static tNeopixelContext neopixel;

#include "math.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#include "flight_config.h"
#include "lora_task.h"

extern imu_raw_3d_t acc, gyr, mag;
extern imu_float_3d_t high_g_acc;

extern void baro_task(void);  // Add this near the top with other declarations

void imu_task_init()
{
    bno055_init(I2C_MASTER_PORT);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bno_configure_acc(NORMAL, ACC_C_H1000, ACC_C_RANGE_16G);
    bno_setoprmode(CONFIG);
    bno_setoprmode(AMG);
}

void baro_task_init()
{
    bmp390_sensorinit();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    update_ground_pressure();
}

void init_general()
{
    neopixel = neopixel_Init(PIXEL_COUNT, NEOPIXEL_PIN);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);

    fflush(stdout);

	// ret = spi_manager_init(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
    // if (ret != ESP_OK) {
    //     printf("Failed to initialize SPI\n");
    //     return;
    // }
}

void init_everything()
{
    init_general();
    
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    bmp_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    bno_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    gps_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    buzzer_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    lora_task_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    pyro_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    psu_init_default();

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
    note(NOTE_G, 8, 300);

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

enum FlightState
{
    FS_ON_PAD = 0,
    FS_POWERED_FLIGHT,
    FS_COASTING,
    FS_UNDER_DROGUES,
    FS_UNDER_MAINS,
    FS_LANDED,

    FS_FREEFALL,
};

uint32_t timestamp;
float latitude;
float longitude;
float barometric_agl;
uint32_t gps_altitude;
float barometric_velocity;
float average_barometric_velocity;
double acceleration;
uint8_t pyro_arm = 0;
uint8_t flight_state;
double batt_voltage = 99.99;


static void flight_on_pad()
{
    // TODO: also check for accelerometer spike
    printf("Acc X = %d", acc.x);
    if (acc.x > 3000) {
        flight_state = FS_POWERED_FLIGHT;
        return;
    }
}

static void flight_powered_flight()
{
    printf("Acc X = %d", acc.x);
    if (acc.x < 0) {
        flight_state = FS_COASTING;
        return;
    };
}

static bool deploy_drogues()
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(PYRO_CHANNEL_1);
        if (cont) {
            pyro_activate(PYRO_CHANNEL_1, 150*(i+1), 0);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(PYRO_CHANNEL_1);
            if (!cont) return true;
        }
    }

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

static bool deploy_mains()
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(PYRO_CHANNEL_2);
        if (cont) {
            pyro_activate(PYRO_CHANNEL_2, 150*(i+1), 0);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(PYRO_CHANNEL_2);
            if (!cont) return true;
        }
    }

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

static void flight_coasting()
{
    if (barometric_agl > APOGEE_MIN && average_barometric_velocity < 0) {
        if (deploy_drogues()) flight_state = FS_UNDER_DROGUES;
        else flight_state = FS_FREEFALL;
        return;
    }
}

static void flight_under_drogues()
{
    if (barometric_agl < MAINS_ALT) {
        if (deploy_mains()) flight_state = FS_UNDER_MAINS;
    }
}

static void flight_under_mains()
{
    if (fabs(average_barometric_velocity) < 2) {
        flight_state = FS_LANDED;
        return;
    }
}

#define HISTORY_SIZE 3
#define VELOCITY_HISTORY_SIZE 10
#define DT 0.01f

enum FailStates
{
    FAIL_FLASH_INIT=1,
    FAIL_FLASH_CHIP_ERASE=2,
};

void fail_state(int n)
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

void megolavania_task()
{
    while (1) {
        megolavania();
    }
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

// #define MAIN_LOOP_FQ ((uint32_t)1e3)
#define MAIN_LOOP_FQ ((uint32_t)15)
#define MAIN_LOOP_MAX_DT ((uint32_t)1e6)/MAIN_LOOP_FQ

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;
    double x_accel, y_accel, z_accel;
    int64_t timestamp;
    float latitude;
    float longitude;
    float barometric_agl;
    uint32_t gps_altitude;
    float barometric_velocity;
    float average_barometric_velocity;
    double acceleration;
    uint8_t pyro_arm;
    uint8_t flight_state;
} flash_packet;
flash_packet fp;

uint32_t addr = 0;

// void app_main_loop(void* params) {
void app_main(void) {
    esp_err_t err = esp_task_wdt_deinit();
    if (err != ESP_OK) {
        printf("FAILED TO DEINIT TASK WATCH DOG\n");
        return;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("size of flash packet: %u\n", sizeof(flash_packet));

    validate_esp32();

    esp_wifi_stop();
    esp_wifi_deinit();

    init_everything();

    // gpio_set_direction(FLASH_CS, GPIO_MODE_OUTPUT);
    // gpio_set_level(FLASH_CS, 1);
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    // gpio_set_direction(LORA_CS, GPIO_MODE_OUTPUT);
    // gpio_set_level(LORA_CS, 1);
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t res;
    addr = 0;

    // TaskHandle_t megolavania_task_handle;
    // xTaskCreatePinnedToCore(megolavania_task, "megolavania_task", 4096, NULL, 1, &megolavania_task_handle, 1);

//     res = w25qxx_init();
//     if (res) fail_state(FAIL_FLASH_INIT);

//     if (pyro_continuity(PYRO_CHANNEL_1) && pyro_continuity(PYRO_CHANNEL_2)) {
//         printf("DOING CHIP ERASE\n");
//         // res = w25qxx_chip_erase();
//         // if (res) fail_state(FAIL_FLASH_CHIP_ERASE);
//         for (int i = 0; i < 500; i++) {
//             res = w25qxx_sector_erase(i*4096);
//             if (res) fail_state(FAIL_FLASH_CHIP_ERASE);
//         }
//     } else {
//         addr = 0;
//         printf("DUMPING DATA\n");
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//         while (1) {
//             w25qxx_read(addr, &fp, sizeof(flash_packet));
//             addr += sizeof(flash_packet);
//             /*
//     int16_t acc_x, acc_y, acc_z;
//     int16_t mag_x, mag_y, mag_z;
//     int16_t gyr_x, gyr_y, gyr_z;
//     float x_accel, y_accel, z_accel;
//     int64_t timestamp;
//     float latitude;
//     float longitude;
//     float barometric_agl;
//     uint32_t gps_altitude;
//     float barometric_velocity;
//     float average_barometric_velocity;
//     double acceleration;
//     uint8_t pyro_arm = 0;
//     uint8_t flight_state;

//         // printf("Delta: %" PRId64 "us or %ldms or %f\n", delta, time_ms, 1.0f/(time_ms/1000.0f));
//     */
//    printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %"PRId64", %f, %f, %f, %lu, %f, %f, %f, %d, %d\n",
//                 fp.acc_x,
//                 fp.acc_y,
//                 fp.acc_z,
//                 fp.mag_x,
//                 fp.mag_y,
//                 fp.mag_z,
//                 fp.gyr_x,
//                 fp.gyr_y,
//                 fp.gyr_z,
//                 fp.x_accel,
//                 fp.y_accel,
//                 fp.z_accel,
//                 fp.timestamp,
//                 fp.latitude,
//                 fp.longitude,
//                 fp.barometric_agl,
//                 fp.gps_altitude,
//                 fp.barometric_velocity,
//                 fp.average_barometric_velocity,
//                 fp.acceleration,
//                 fp.pyro_arm,
//                 fp.flight_state);
//             bool all = true;
//             char* buf = (char*) &fp;
//             for (int i = 0; i < sizeof(flash_packet); i++) {
//                 if (buf[i] != 0xFF) all=false;
//             }
//             if (all) break;
//         }

//         while (1) {
//             neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  255) } }, 1);
//             vTaskDelay(500/portTICK_PERIOD_MS);
//             neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
//             vTaskDelay(500/portTICK_PERIOD_MS);

//             vTaskDelay(1000/portTICK_PERIOD_MS);
//         }
//     }

    // vTaskDelete(megolavania_task_handle);

    note(NOTE_G, 8, 300);
    note(NOTE_G, 7, 300);
    note(NOTE_G, 8, 300);
    note(NOTE_G, 7, 300);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (pyro_continuity(j+1)) note(NOTE_E, 8, 300);
            else note(NOTE_G, 5, 300);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

#ifdef LED_PYRO
    pyro_activate(PYRO_CHANNEL_1, 150, 0);
    pyro_activate(PYRO_CHANNEL_2, 150, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

    flight_state = FS_ON_PAD;

    uint32_t cycle = 0;
    
    while (1) {
        int64_t start_time = esp_timer_get_time();
        // uint32_t start_ccount = XTHAL_GET_CCOUNT();
        cycle = (cycle + 1) % MAIN_LOOP_FQ;

        // update state variables
        int sum = 0;
        sum += pyro_continuity(PYRO_CHANNEL_1);
        sum += pyro_continuity(PYRO_CHANNEL_2)*2;
        pyro_arm = sum;

        batt_voltage = psu_read_battery_voltage();

        // int64_t s;
        // int64_t d;

        // gps 10 Hz
        // s = esp_timer_get_time();
        if (cycle % (uint32_t)(MAIN_LOOP_FQ/5) == 0) {
            parse_NMEA(&latitude, &longitude, &gps_altitude);
        }
        // d = esp_timer_get_time() - s;
        // printf("GPS: %" PRId64 "us\n", d);

        // lowg imu accl // 1000Hz
        // s = esp_timer_get_time();
        // if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
        //     int16_t acc_x, acc_y, acc_z;
        //     bno_readacc(&acc_x, &acc_y, &acc_z);
        //     acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
        // }
        // d = esp_timer_get_time() - s;
        // printf("ACC: %" PRId64 "us\n", d);

        // lowg imu gyro // 32Hz
        // s = esp_timer_get_time();
        // if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
        //     int16_t gyr_x, gyr_y, gyr_z;
        //     bno_readgyro(&gyr_x, &gyr_y, &gyr_z);
        // }
        // d = esp_timer_get_time() - s;
        // printf("GYR: %" PRId64 "us\n", d);

        // lowg imu mag 10Hz
        // s = esp_timer_get_time();
        // if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
        //     int16_t mag_x, mag_y, mag_z;
        //     bno_readmag(&mag_x, &mag_y, &mag_z);
        // }
        // d = esp_timer_get_time() - s;
        // printf("MAG: %" PRId64 "us\n", d);

        // lora 500 Hz
        // s = esp_timer_get_time();
        // if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
        //     lora_task();
        // }
        // d = esp_timer_get_time() - s;
        // printf("LOR: %" PRId64 "us\n", d);

        // bmp 100 Hz
        // s = esp_timer_get_time();
        // if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
        //     baro_task();
        // }
        // d = esp_timer_get_time() - s;
        // printf("BMP: %" PRId64 "us\n", d);

        /*
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;
    float x_accel, y_accel, z_accel;
    int64_t timestamp;
    float latitude;
    float longitude;
    float barometric_agl;
    uint32_t gps_altitude;
    float barometric_velocity;
    float average_barometric_velocity;
    double acceleration;
    uint8_t pyro_arm = 0;
    uint8_t flight_state;
            h3lis331dl_read_accel(&data.x_accel, &data.y_accel, &data.z_accel);u*/
        if (cycle % (uint32_t)(MAIN_LOOP_FQ/15) == 0) {
            int16_t acc_x, acc_y, acc_z;
            int16_t mag_x, mag_y, mag_z;
            int16_t gyr_x, gyr_y, gyr_z;
            bno_readamg(&acc_x, &acc_y, &acc_z,
                        &mag_x, &mag_y, &mag_z,
                        &gyr_x, &gyr_y, &gyr_z);
            baro_task();
            lis331_local(&high_g_acc, true);
            bno_local(&acc, &gyr, &mag, true);
            // double x_accel, y_accel, z_accel;
            // h3lis331dl_read_accel(&x_accel, &y_accel, &z_accel);
            lora_task();
            acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

            // if (addr < (uint32_t)6e6) {
            // // if (false) {
            //     fp.acc_x = acc_x;
            //     fp.acc_y = acc_y;
            //     fp.acc_z = acc_z;
            //     fp.mag_x = mag_x;
            //     fp.mag_y = mag_y;
            //     fp.mag_z = mag_z;
            //     fp.gyr_x = gyr_x;
            //     fp.gyr_y = gyr_y;
            //     fp.gyr_z = gyr_z;
            //     fp.x_accel = x_accel;
            //     fp.y_accel = y_accel;
            //     fp.z_accel = z_accel;
            //     fp.timestamp = start_time;
            //     fp.latitude = latitude;
            //     fp.longitude = longitude;
            //     fp.barometric_agl = barometric_agl;
            //     fp.gps_altitude = gps_altitude;
            //     fp.barometric_velocity = barometric_velocity;
            //     fp.average_barometric_velocity = average_barometric_velocity;
            //     fp.acceleration = acceleration;
            //     fp.pyro_arm = pyro_arm;
            //     fp.flight_state = flight_state;

            //     w25qxx_write(addr, &fp, sizeof(flash_packet));
            //     addr += sizeof(flash_packet);
            // }
        }


        // hand control over current flight state
        // s = esp_timer_get_time();
        switch (flight_state) {
            case FS_ON_PAD: flight_on_pad(); break;
            case FS_POWERED_FLIGHT: flight_powered_flight(); break;
            case FS_COASTING: flight_coasting(); break;
            case FS_UNDER_DROGUES: flight_under_drogues(); break;
            case FS_UNDER_MAINS: flight_under_mains(); break;
            case FS_FREEFALL: break;
            case FS_LANDED: break;
            default: assert(0); break;
        }
        // d = esp_timer_get_time() - s;
        // printf("FSM: %" PRId64 "us\n\n", d);

        // uint32_t end_ccount = XTHAL_GET_CCOUNT();
        // uint32_t delta_cycles = end_ccount - start_ccount;
        // float delta_us = (float)delta_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

        int64_t end_time = esp_timer_get_time();
        int64_t delta = end_time - start_time;
        // long time_ms = delta/1e3;
        // printf("Delta: %" PRId64 "us or %ldms or %f\n", delta, time_ms, 1.0f/(time_ms/1000.0f));
        uint8_t under = (uint32_t)delta < MAIN_LOOP_MAX_DT;
        if (under) {
            portDISABLE_INTERRUPTS();
            ets_delay_us(MAIN_LOOP_MAX_DT-delta);
            portENABLE_INTERRUPTS();
        }
        end_time = esp_timer_get_time();
        delta = end_time - start_time;
        // printf("Delta: %" PRId64 "us or %ldms or %f. under? %d (want: 1)\n", delta, time_ms, 1.0f/(time_ms/1000.0f), under);
        if (under) neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
        else neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);
    }
}

// void app_main(void) {
//     xTaskCreatePinnedToCore(app_main_loop, "MainLoop", 8192, NULL, 1, NULL, 1);
// }
