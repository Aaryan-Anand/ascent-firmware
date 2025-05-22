#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "stdint.h"

typedef struct {
    int64_t timestamp;

    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;

    double x_accel, y_accel, z_accel;

    float latitude;
    float longitude;
    uint32_t gps_altitude;

    float barometric_agl;

    uint8_t pyro_arm;
    uint8_t flight_state;

    float ekf_latitude;
    float ekf_longitude;
    float ekf_altitude;
    float ekf_pitch;
    float ekf_yaw;
    float ekf_roll;
} flash_packet;

void flash_flight_init(void);

void flash_prepare_for_flight(void);

void flash_dump_to_serial(void);

void flash_write_packet(flash_packet *packet);

#endif