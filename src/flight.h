#ifndef FLIGHT_H
#define FLIGHT_H

enum FlightState
{
    FS_ON_PAD = 0,
    FS_BOOSTER,
    FS_COAST_BOOSTER,
    FS_SUSTAINER,
    FS_COAST_SUSTAINER,
    FS_UNDER_DROGUES,
    FS_UNDER_MAINS,
    FS_LANDED,
};

void flight_update(
    float ekf_agl,
    float ekf_vertical_vel,
    float ekf_vertical_acl,

    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float raw_vertical_acl
);

uint8_t get_flight_state(void);

#endif