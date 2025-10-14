#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "stdbool.h"

// #define IS_SITL

// Note all values are in m or m/s

#define IS_BOOSTER

#define ENGINE_GS (9.81*3)
// #define ENGINE_GS (9.81*0.1)
#define APPO_GS (9.81/2)

#define MAINS_ALT (1000/3.28)

#define PANIC_VEL (-240.0f/3.28)

// if this is defined then the board will erase and move to the next flash bank
// on any power up with atleast one continuous pyro, and will fake a txlock,
// signal this can be used to allow ascent to fly with out a ground station or
// two way coms to a ground station
// #define ARM_REGARDLESS_OF_TXLOCK

#endif