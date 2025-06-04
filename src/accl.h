#ifndef ACCL_H
#define ACCL_H

#include "interface_bno055.h"

void accl_update(imu_raw_3d_t acc, imu_raw_3d_t* out);

#endif