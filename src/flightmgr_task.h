#ifndef FLIGHT_MGR_TASK_H
#define FLIGHT_MGR_TASK_H

#include "flight_config.h"

enum FlightState
{
    FS_ON_PAD = 0,
    FS_POWERED_FLIGHT,
    FS_COASTING,
    FS_UNDER_DROGUES,
    FS_UNDER_MAINS,
    FS_LANDED,

    FS_FREEFALL,
};

void flight_task(void *pvParameters);

#endif