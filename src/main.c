#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "interface_bmp390l.h"
#include "interface_sam_m10q.h"
#include "lora.h"
#include "driver_buzzer.h"
#include "driver_pyro.h"
#include "driver_psu.h"
#include "driver_bno055.h"

#include "math.h"

#include "ascent_r2_hardware_definition.h"  // Hardware definitions

#define LORA_FREQ 915e6

double ground_pressure = 0;

uint32_t timestamp;
float latitude;
float longitude;
float barometric_agl;
uint32_t gps_altitude;
float barometric_velocity;
double acceleration;
uint8_t pyro_arm = 0;
uint8_t flight_state;

typedef struct {
    uint32_t timestamp;
    float latitude;
    float longitude;
    float barometric_agl;
    uint32_t gps_altitude;
    float barometric_velocity;
    float acceleration;
    uint8_t pyro_arm;
    uint8_t flight_state;
} lora_packet_t;

#define APOGEE_MIN 300
#define POWERED_ALT 30

// === Task Handles ===
TaskHandle_t flight_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;
TaskHandle_t baro_task_handle = NULL;
TaskHandle_t bno_task_handle = NULL;

// === Task Definitions ===

void flight_task(void *pvParameters) {
    flight_state = 0;

    while (1){
        // beep out continuity
        flight_state = 0;

        if (barometric_agl > POWERED_ALT) {
            flight_state = 1;
            break;
        }

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
    
    while (1){
        static float velocity_samples[10] = {0};
        static int sample_index = 0;
        float sum = 0;
        float average_velocity = 0;

        // Update velocity samples
        velocity_samples[sample_index] = barometric_velocity;
        sample_index = (sample_index + 1) % 10;

        // Calculate the sum of all samples
        for (int i = 0; i < 10; i++) {
            sum += velocity_samples[i];
        }

        // Calculate the average velocity
        average_velocity = sum / 10.0;

        if (barometric_agl > APOGEE_MIN && average_velocity < 0) {
            flight_state = 2;
            break;
        }

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    pyro_init();

    bool cont = pyro_continuity(PYRO_CHANNEL_1);
    
    if (cont == 1) {
            pyro_arm = 1;
    }

    flight_state = 3;

    int i;

    for (i = 0; i < 2; i++) {
        bool cont = pyro_continuity(PYRO_CHANNEL_1);
        if (cont == 0) {
            flight_state = 99;
            break;
        }
        if (i == 0) {
            // pyro_activate(PYRO_CHANNEL_1, 150);
            flight_state = 4;
        } else {
            // pyro_activate(PYRO_CHANNEL_1, 300);
            flight_state = 98;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        flight_state = 5;
    }
    
    while (1){ 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        flight_state = 5;
    }
}

void gps_task(void *pvParameters) {

    portMUX_TYPE *my_spinlock = malloc(sizeof(portMUX_TYPE));

    portMUX_INITIALIZE(my_spinlock);

    while (1) {
        // GPS polling
        // taskENTER_CRITICAL(my_spinlock);
        parse_NMEA(&latitude, &longitude, &gps_altitude);
        // taskEXIT_CRITICAL(my_spinlock);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void lora_tx(void *pvParameters) {
    // Static allocation for packet_data
    uint8_t packet_data[sizeof(lora_packet_t)];

    while (1) {
        
        lora_packet_t packet;
        packet.latitude = latitude;
        packet.longitude = longitude;
        packet.barometric_agl = barometric_agl;
        packet.gps_altitude = gps_altitude;
        packet.barometric_velocity = barometric_velocity;
        packet.acceleration = acceleration;
        packet.pyro_arm = pyro_arm;
        packet.flight_state = flight_state;

        packet.timestamp = esp_timer_get_time() / 1e3;

        memcpy(packet_data, &packet, sizeof(lora_packet_t));
        lora_send_packet(packet_data, sizeof(lora_packet_t));

        printf("Sent packet at %ld ms: Latitude: %.6f, Longitude: %.6f, GPSAltitude: %ld, Baro Altitude: %f, Baro Velocity: %f, Acceleration: %f, Pyro Arm: %d, Flight State: %d\n", packet.timestamp, packet.latitude, packet.longitude, packet.gps_altitude, packet.barometric_agl, packet.barometric_velocity, packet.acceleration, packet.pyro_arm, packet.flight_state);

        int lost = lora_packet_lost();
		if (lost != 0) {
			printf("%d packets lost", lost);
		}

        vTaskDelay(pdMS_TO_TICKS(10));

        // note(NOTE_E,6,100);
    }
}

#define HISTORY_SIZE 3
#define DT 0.03f  // 30 ms in seconds

void baro_task(void *pvParameters) {
    float agl_history[HISTORY_SIZE] = {0};  // Store the last 5 AGL readings
    double pressure_hPa;
    double temperature;

    while (1) {
        // Shift history
        for (int i = HISTORY_SIZE - 1; i > 0; i--) {
            agl_history[i] = agl_history[i - 1];
        }

        // Update with latest AGL
        agl_history[0] = bmp390_barometricAGL();
        barometric_agl = agl_history[0];
        bmp390_read_sensor_data(&pressure_hPa, &temperature);

        // Compute first-order backward finite difference
        if (agl_history[1] != 0) {
            barometric_velocity = barometric_velocity*0.2 + ((agl_history[0] - agl_history[1]) / DT)*0.8;
        }

        
        // printf("AGL: %f, Pressure: %f, Velocity: %f\n", agl_history[0], pressure_hPa, barometric_velocity);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// === Lora Function (needs to be moved to an interface at some point) ===

void lora_initialize() {
 	lora_init();
	
	lora_set_frequency(915e6); // 915MHz
	lora_enable_crc();

	int cr = 1;
	int bw = 9;
	int sf = 7;

	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

	lora_set_tx_power(17);
}

// === BNO Task ===

void bno_task(void *pvParameters){
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;

    bno_setoprmode(CONFIG);
    bno_setoprmode(AMG);

    while (1)
    {
        bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

        acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// === Main App Entry ===

void app_main(void) {

    vTaskDelay(500 / portTICK_PERIOD_MS);

    printf("\n\n\nStarting application...\n");

    vTaskDelay(100 / portTICK_PERIOD_MS);

    fflush(stdout);

    esp_err_t ret1 = i2c_manager_deinit(I2C_NUM_0);
    if (ret1 != ESP_OK) {
        printf("Failed to deinit I2C\n");
        return;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_err_t ret = i2c_manager_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ, I2C_MASTER_PORT);
    if (ret != ESP_OK) {
        printf("Failed to initialize I2C\n");
        return;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    gps_init();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    bmp390_sensorinit();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    update_ground_pressure();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    buzzer_init();

    lora_initialize();

    bno055_init(I2C_MASTER_PORT);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Create tasks without pinning to specific cores
    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 5, &flight_task_handle, 1);
    xTaskCreatePinnedToCore(lora_tx, "lora_tx", 4096, NULL, 3, &lora_task_handle, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 2048, NULL, 3, &gps_task_handle, 0);
    xTaskCreatePinnedToCore(baro_task, "baro_task", 3072, NULL, 4, &baro_task_handle, 0);
    xTaskCreatePinnedToCore(bno_task, "bno_task", 3072, NULL, 4, &bno_task_handle, 0);
}