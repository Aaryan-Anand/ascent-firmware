#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "stdbool.h"

// Note all values are in m or m/s

#define IS_TWO_STAGE (true)

#define ENGINE_GS (9.81*3)
#define APPO_GS (9.81/2)

#define APOGEE_MIN 500

#define MAINS_ALT 1000

#define PANIC_VEL (-240.0f/3.28)

#define DROGUE_DESCENT (-150.0f/3.28)

#define MAIN_DESCENT (-30.0f/3.28)

#endif