#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "stdint.h"
#include "interface_bmp390l.h"
#include "interface_bno055.h"
#include "interface_h3lis331dl.h"

typedef struct {
    uint32_t n;
    int64_t timestamp;
    uint8_t pyro_arm;
    uint8_t flight_state;

    imu_local_3d_t acc, gyr, mag;
    imu_float_3d_t high_g_acc;
    baro_double_t baro;

    float barometric_agl;
    float barometric_velocity;
    float average_barometric_velocity;

    float latitude;
    float longitude;
    uint32_t gps_altitude;
} flash_packet;

uint32_t flash_get_addr();

void flash_flight_init(void);

bool flash_prepare_for_flight(void);

void flash_dump_to_serial(void);

void flash_write_packet(flash_packet *packet);

void flash_queue_packet(flash_packet *packet);

void flash_write_queue(int64_t max_time);

#endif