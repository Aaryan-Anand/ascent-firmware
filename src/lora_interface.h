#ifndef LORA_TASK_H
#define LORA_TASK_H

#include "stdint.h"
#include "lora.h"

#define LORA_FREQ 915e6

// lora stuff
typedef struct {
    uint32_t timestamp;
    float latitude;
    float longitude;
    float barometric_agl;
    uint32_t gps_altitude;
    float barometric_velocity;
    float acceleration;
    uint8_t pyro_arm;
    uint8_t flight_state;
    double batt_voltage;
} lora_packet_t;

void lora_flight_init();

void lora_transmit_packet(lora_packet_t *packet);

#endif