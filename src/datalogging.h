#ifndef DATALOGGING_H
#define DATALOGGING_H

#include "stdint.h"

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;
    double x_accel, y_accel, z_accel;
    int64_t timestamp;
    float latitude;
    float longitude;
    float barometric_agl;
    uint32_t gps_altitude;
    float barometric_velocity;
    float average_barometric_velocity;
    double acceleration;
    uint8_t pyro_arm;
    uint8_t flight_state;
} flash_packet;

extern uint32_t addr;

void save_addr();

void recall_addr();

void begin_datalogging();

void write_flash_packet();

#endif