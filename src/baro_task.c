#include "baro_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "stdint.h"
#include "globals.h"
#include "driver_BMP390L.h"
#include "interface_bmp390l.h"

#define HISTORY_SIZE 3
#define VELOCITY_HISTORY_SIZE 10
#define DT 0.03f  // 30 ms in seconds

void baro_task_init()
{
    bmp390_sensorinit();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    update_ground_pressure();
}

void baro_task(void *pvParameters) {
    float agl_history[HISTORY_SIZE] = {0};  // Store the last 5 AGL readings
    double pressure_hPa;
    double temperature;

    float velocity_samples[VELOCITY_HISTORY_SIZE] = {0};
    int sample_index = 0;

    while (1) {
        // Shift history
        for (int i = HISTORY_SIZE - 1; i > 0; i--) {
            agl_history[i] = agl_history[i - 1];
        }

        // Update with latest AGL
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            agl_history[0] = bmp390_barometricAGL();
            barometric_agl = agl_history[0];
            bmp390_read_sensor_data(&pressure_hPa, &temperature);

            xSemaphoreGive(i2c_mutex);
        }

        // Compute first-order backward finite difference
        if (agl_history[1] != 0) {
            barometric_velocity = barometric_velocity*0.2 + ((agl_history[0] - agl_history[1]) / DT)*0.8;
        }

        // Update velocity samples
        velocity_samples[sample_index] = barometric_velocity;
        sample_index = (sample_index + 1) % VELOCITY_HISTORY_SIZE;

        float sum = 0;
        // Calculate the sum of all samples
        for (int i = 0; i < 10; i++) {
            sum += velocity_samples[i];
        }

        // Calculate the average velocity
        average_barometric_velocity = sum / 10.0;

        // printf("AGL: %f, Pressure: %f, Velocity: %f\n", agl_history[0], pressure_hPa, barometric_velocity);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}