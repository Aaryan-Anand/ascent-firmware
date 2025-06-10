#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stdbool.h>

enum FlightState
{
    FS_ON_PAD = 0,
    FE_LIFTOFF,
    FS_BOOSTER,
    FE_BURNOUT_BOOSTER,
    FS_COAST_BOOSTER,
    FE_STAGE_SEPARATION,
    FS_SUSTAINER,
    FE_BURNOUT_SUSTAINER,
    FS_COAST_SUSTAINER,
    FE_APOGEE,
    FE_DROGUES_DEPLOYED,
    FS_UNDER_DROGUES,
    FE_PANIC,
    FE_MAIN_DEPLOYED,
    FS_UNDER_MAINS,
    FE_GROUND_HIT,
    FS_LANDED,
    FE_SLEEP,
    FS_PREFLIGHT,
    FE_WAKE,
    //TODO: ADD FTS SAFE. FLIGHT TERMINATION SYSTEM IS REQUIRED. (jk lol)
};

// returns if the state changed or not
bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float raw_vertical_acl
);

uint8_t get_flight_state(void);

const char* get_flight_state_name(void);

#endif