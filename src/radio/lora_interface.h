#ifndef LORA_INTERFACE_H
#define LORA_INTERFACE_H

#include "stdbool.h"
#include "stdint.h"
#include "lora.h"
#include "goober.h"
#define LORA_FREQ 920e6

// #define LORA_DEBUG
typedef struct {
    long frequency;
    int bandwidth;
    int coding_rate;
    int spreading_factor;
    int tx_power;
    uint8_t TDD;
} lora_config_t;

void lora_config_init(void);

void lora_config_set(lora_config_t *cfg);

void queueLatestTelemetryPayload(goober_payload_t *payload);

void peekLatestTelemetryPayload(goober_payload_t *payload);

esp_err_t lora_flight_init();

void lora_transmit_packet(goober_t *packet);

int lora_blocking_listen(goober_t *received_packet, uint8_t timeout);

uint8_t calc_pyro_arm(void);
            
#endif