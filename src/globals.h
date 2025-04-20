#ifndef GLOBALS_H
#define GLOBALS_H

#include "stdint.h"

extern uint32_t timestamp;
extern float latitude;
extern float longitude;
extern float barometric_agl;
extern uint32_t gps_altitude;
extern float barometric_velocity;
extern float average_barometric_velocity;
extern double acceleration;
extern uint8_t pyro_arm;
extern uint8_t flight_state;

#endif