#include "imu_task.h"

#include "driver_bno055.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "stdint.h"
#include "i2c_manager.h"
#include "ascent_r2_hardware_definition.h"
#include "globals.h"
#include "math.h"

void imu_task_init()
{
    bno055_init(I2C_MASTER_PORT);
}

void imu_task(void *pvParameters) {
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;

    bno_setoprmode(CONFIG);
    bno_setoprmode(AMG);

    while (1) {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

            xSemaphoreGive(i2c_mutex);
        }

        acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}