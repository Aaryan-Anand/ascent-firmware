#include "gps_task.h"

// #include "freertos/semphr.h"
#include "stdint.h"
#include "globals.h"
#include "interface_sam_m10q.h"

void gps_task_init()
{
    gps_init();
}

void gps_task(void *pvParameters)
{
    while (1) {
        // GPS polling
        float lat, lng;
        uint32_t gps_alt;
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            parse_NMEA(&lat, &lng, &gps_alt);
            xSemaphoreGive(i2c_mutex);
        }
        latitude = lat;
        longitude = lng;
        gps_altitude = gps_alt;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}