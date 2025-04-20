#ifndef GLOBALS_H
#define GLOBALS_H

#include "stdint.h"
#include "stdatomic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern _Atomic uint32_t timestamp;
extern _Atomic float latitude;
extern _Atomic float longitude;
extern _Atomic float barometric_agl;
extern _Atomic uint32_t gps_altitude;
extern _Atomic float barometric_velocity;
extern _Atomic float average_barometric_velocity;
extern _Atomic double acceleration;
extern _Atomic uint8_t pyro_arm;
extern _Atomic uint8_t flight_state;

extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t spi_mutex;

void globals_init();

#endif