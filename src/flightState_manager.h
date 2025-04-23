#ifndef FLIGHT_STATE_MANAGER_H
#define FLIGHT_STATE_MANAGER_H

#include <stdbool.h>

// Flight States
enum FlightState
{
    FS_ON_PAD = 0,
    FS_POWERED_FLIGHT,
    FS_COAST,
    FS_UNDER_DROGUES,
    FS_UNDER_MAINS,
    FS_LANDED,
    FS_FREEFALL,
};

enum FlightEvent
{
    FE_LIFTOFF,
    FE_MOTOR_BURNOUT,
    FE_APOGEE,
    FE_MAIN_EVENT,
    FE_GROUND_HIT,
};

// Public Functions
void flight_state_manager(void* pvParameters);
void baro_task(void);
const char* flight_state_to_string(enum FlightState state);

#endif // FLIGHT_STATE_MANAGER_H 