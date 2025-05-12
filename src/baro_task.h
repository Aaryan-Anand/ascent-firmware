#ifndef BARO_TASK
#define BARO_TASK

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "interface_bmp390l.h"
#include "driver_pyro.h"
#include "globals.h"
#include "flight_config.h"  // For POWERED_ALT, APOGEE_MIN, and MAINS_ALT
#include <assert.h>
#include "flightState_manager.h"

void baro_task();

#endif