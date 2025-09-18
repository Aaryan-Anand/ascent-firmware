#include "globals.h"

orientation_t g_orientation;
SemaphoreHandle_t g_orientation_mutex = NULL;

void globals_init(void) {
    if (g_orientation_mutex == NULL) {
        g_orientation_mutex = xSemaphoreCreateMutex();
    }
}
