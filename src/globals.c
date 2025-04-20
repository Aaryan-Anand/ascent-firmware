#include "stdint.h"
#include "stdatomic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

_Atomic uint32_t timestamp;
_Atomic float latitude;
_Atomic float longitude;
_Atomic float barometric_agl;
_Atomic uint32_t gps_altitude;
_Atomic float barometric_velocity;
_Atomic float average_barometric_velocity;
_Atomic double acceleration;
_Atomic uint8_t pyro_arm;
_Atomic uint8_t flight_state;

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t spi_mutex;

void globals_init()
{
    pyro_arm = 0;

    i2c_mutex = xSemaphoreCreateMutex();
    spi_mutex = xSemaphoreCreateMutex();
}