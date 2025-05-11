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
#include "wifi_hotspot.h"
#include "wifi_websocket.h"

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

    wifi_AP_init();

    wifi_websocket_init();

    neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
    // note(NOTE_G, 8, 300);

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
    // printf("Acc X = %d", acc.x);
    if (acc.x > 3000) {
        flight_state = FS_POWERED_FLIGHT;
        return;
    }
}

static void flight_powered_flight()
{
    // printf("Acc X = %d", acc.x);
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
        deploy_drogues();
        flight_state = FS_UNDER_DROGUES;
        return;
    }
}

static void flight_under_drogues()
{
    if (barometric_agl < MAINS_ALT) {
        deploy_mains();
        flight_state = FS_UNDER_MAINS;
    }
}

static void flight_under_mains()
{
    if (fabs(average_barometric_velocity) < 5) {
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
#define MAIN_LOOP_FQ ((uint32_t)50)
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

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t res;
    addr = 0;

    // note(NOTE_G, 8, 300);
    // note(NOTE_G, 7, 300);
    // note(NOTE_G, 8, 300);
    // note(NOTE_G, 7, 300);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (pyro_continuity(j+1))
                printf("kewl");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

#ifdef LED_PYRO
    pyro_activate(PYRO_CHANNEL_1, 150, 0);
    pyro_activate(PYRO_CHANNEL_2, 150, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

#ifdef LIVE_VIDEO_PYRO_3
    pyro_activate(PYRO_CHANNEL_3,0,1);
    // note(NOTE_G, 5, 100);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    // note(NOTE_A, 3, 50);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    // note(NOTE_G, 5, 100);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    // note(NOTE_A, 3, 50);
    vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

    flight_state = FS_ON_PAD;

    uint32_t cycle = 0;
    
    while (1) {
        int64_t start_time = esp_timer_get_time();
        cycle = (cycle + 1) % MAIN_LOOP_FQ;

        int sum = 0;
        sum += pyro_continuity(PYRO_CHANNEL_1);
        sum += pyro_continuity(PYRO_CHANNEL_2)*2;
        pyro_arm = sum;

        batt_voltage = psu_read_battery_voltage();

        if (cycle % (uint32_t)(MAIN_LOOP_FQ/5) == 0) {
            parse_NMEA(&latitude, &longitude, &gps_altitude);
        }

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
            lora_task();
            acceleration = acc.x;
        }

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

        int64_t end_time = esp_timer_get_time();
        int64_t delta = end_time - start_time;
        uint8_t under = (uint32_t)delta < MAIN_LOOP_MAX_DT;
        if (under) {
            portDISABLE_INTERRUPTS();
            ets_delay_us(MAIN_LOOP_MAX_DT-delta);
            portENABLE_INTERRUPTS();
        }
        end_time = esp_timer_get_time();
        delta = end_time - start_time;
        if (under) neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 255,  0) } }, 1);
        else neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(255, 0,  0) } }, 1);
    }
}
