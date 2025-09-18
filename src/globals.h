#ifndef GLOBALS_H
#define GLOBALS_H

#include "sensor_manager.h"
#include "stdint.h"

#include "orientation.h"
#include "freertos/semphr.h"

// Shared orientation across tasks
extern orientation_t g_orientation;
extern SemaphoreHandle_t g_orientation_mutex;

// Initialize global objects (e.g., mutexes)
void globals_init(void);



#endif