// sensor_filtering.h
#pragma once
#ifndef SENSOR_FILTERING_H
#define SENSOR_FILTERING_H

#include <stdint.h>
#include <stdbool.h>

#include "interface_bno055.h"
#include "interface_H3LIS331DL.h"

#ifdef __cplusplus
extern "C" {
#endif


void sensor_filter_acc(imu_local_3d_t* local_acc, uint8_t flight_state);
void sensor_filter_gyr(imu_local_3d_t* local_gyr, uint8_t flight_state);
void sensor_filter_mag(imu_local_3d_t* local_mag, uint8_t flight_state);
void sensor_filter_high_g_acc(imu_float_3d_t* high_g_acc, uint8_t flight_state);

void sensor_filter_reset(void);

#ifdef __cplusplus
}
#endif
#endif // SENSOR_FILTERING_H
