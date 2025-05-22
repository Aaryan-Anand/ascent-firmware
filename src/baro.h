#ifndef BARO_H
#define BARO_H

#include "interface_bmp390l.h"

void baro_update(const baro_double_t * const baro, float *agl, float *vel, float *avg_vel);

#endif
