#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "stdbool.h"

#define IS_SITL

// Note all values are in m or m/s

#define IS_TWO_STAGE (true)

#define ENGINE_GS 3000
#define APPO_GS 500

#define APOGEE_MIN 500

#define MAINS_ALT 1000

#define PANIC_VEL (-180.0f/3.28)

#endif