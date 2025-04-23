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

#include "flightState_manager.h"

// Add these global variables for frequency monitoring
volatile uint32_t sensor_task_counter = 0;
volatile uint32_t fsm_task_counter = 0;

// Add counters for each sensor task
volatile uint32_t bno_task_counter = 0;
volatile uint32_t lis_task_counter = 0;
volatile uint32_t bmp_task_counter = 0;

// Define all other global variables
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
uint8_t flight_event;
double batt_voltage;

// Change mutex to binary semaphore
SemaphoreHandle_t i2c_semaphore;

// Add a counter for round-robin scheduling
static volatile uint8_t i2c_turn = 0;

// Add a timer task to calculate and print frequencies
void monitor_task(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Sample every 100ms
    
    uint32_t last_bno_count = 0;
    uint32_t last_lis_count = 0;
    uint32_t last_bmp_count = 0;
    uint32_t last_fsm_count = 0;

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Calculate frequencies over 100ms period
        uint32_t bno_freq = (bno_task_counter - last_bno_count) * 10;
        uint32_t lis_freq = (lis_task_counter - last_lis_count) * 10;
        uint32_t bmp_freq = (bmp_task_counter - last_bmp_count) * 10;
        uint32_t fsm_freq = (fsm_task_counter - last_fsm_count) * 10;
        
        // Store current counts for next iteration
        last_bno_count = bno_task_counter;
        last_lis_count = lis_task_counter;
        last_bmp_count = bmp_task_counter;
        last_fsm_count = fsm_task_counter;

        printf("Frequencies - BNO: %lu Hz, LIS: %lu Hz, BMP: %lu Hz, FSM: %lu Hz\n", 
               bno_freq, lis_freq, bmp_freq, fsm_freq);
    }
}

void init_sensors(void) {
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    bmp_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    bno_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    lis331_flight_init();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void bno_task(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 4; // 250Hz
    uint32_t missed_deadlines = 0;
    uint32_t semaphore_timeouts = 0;
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start = esp_timer_get_time();
        
        // Wait for our turn (1)
        while(i2c_turn != 1) {
            vTaskDelay(1);
        }
        
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(2)) == pdTRUE) {
            bno_local(&acc, &gyr, &mag, true);
            xSemaphoreGive(i2c_semaphore);
            bno_task_counter++;
            
            uint64_t end = esp_timer_get_time();
            if ((end - start) > 4000) {
                missed_deadlines++;
            }
            
            // Pass to next sensor
            i2c_turn = 2;
            
            if (bno_task_counter % 250 == 0) {
                printf("BNO: time=%llu us, missed=%lu, timeouts=%lu\n", 
                       end - start, missed_deadlines, semaphore_timeouts);
            }
        }
    }
}

void lis_task(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 2; // 500Hz
    uint32_t missed_deadlines = 0;
    uint32_t semaphore_timeouts = 0;
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start = esp_timer_get_time();
        
        // Wait for our turn (0)
        while(i2c_turn != 0) {
            vTaskDelay(1);
        }
        
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
            lis331_local(&high_g_acc, true);
            xSemaphoreGive(i2c_semaphore);
            lis_task_counter++;
            
            uint64_t end = esp_timer_get_time();
            if ((end - start) > 2000) {
                missed_deadlines++;
            }
            
            // Pass to next sensor
            i2c_turn = 1;
            
            if (lis_task_counter % 500 == 0) {
                printf("LIS: time=%llu us, missed=%lu, timeouts=%lu\n", 
                       end - start, missed_deadlines, semaphore_timeouts);
            }
        }
    }
}

void bmp_task(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10; // 100Hz
    uint32_t missed_deadlines = 0;
    uint32_t semaphore_timeouts = 0;
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start = esp_timer_get_time();
        
        // Wait for our turn (2)
        while(i2c_turn != 2) {
            vTaskDelay(1);
        }
        
        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            bmp_calib(&barometric_agl);
            baro_task();
            xSemaphoreGive(i2c_semaphore);
            bmp_task_counter++;
            
            uint64_t end = esp_timer_get_time();
            if ((end - start) > 10000) {
                missed_deadlines++;
            }
            
            // Pass back to first sensor
            i2c_turn = 0;
            
            if (bmp_task_counter % 100 == 0) {
                printf("BMP: time=%llu us, missed=%lu, timeouts=%lu\n", 
                       end - start, missed_deadlines, semaphore_timeouts);
            }
        }
    }
}

void app_main(void) {
    // Create binary semaphore instead of mutex
    i2c_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_semaphore); // Make it available
    
    // Configure watchdog timer
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = false
    };
    esp_task_wdt_init(&wdt_config);

    // Initialize sensors with mutex protection
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    init_sensors();
    xSemaphoreGive(i2c_semaphore);

    // Create tasks with optimized priorities
    xTaskCreatePinnedToCore(
        lis_task,          // Fastest sensor gets highest priority
        "lis_task",
        8192,  // Increased stack size
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        1
    );

    vTaskDelay(pdMS_TO_TICKS(1));

    xTaskCreatePinnedToCore(
        bno_task,          // Second fastest
        "bno_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 2,
        NULL,
        1
    );

    vTaskDelay(pdMS_TO_TICKS(1));

    xTaskCreatePinnedToCore(
        bmp_task,          // Slowest sensor
        "bmp_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 3,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        flight_state_manager,
        "FSM_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 4,  // Lowest priority of core 1 tasks
        NULL,
        1
    );

    // Monitor task on core 0
    xTaskCreatePinnedToCore(
        monitor_task,
        "monitor_task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}

