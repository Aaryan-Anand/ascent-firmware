#ifndef LORA_INTERFACE_H
#define LORA_INTERFACE_H

#include "stdbool.h"
#include "stdint.h"
#include "esp_err.h"
#include "lora.h"
#include "lora_types.h"

// #define LORA_DEBUG
typedef struct goober goober_t;                 // forward declaration
typedef union  goober_payload goober_payload_t; // forward declaration

void lora_config_init(void);

void lora_config_set(lora_config_t *cfg);

void queueLatestTelemetryPayload(goober_payload_t *payload);

void peekLatestTelemetryPayload(goober_payload_t *payload);

esp_err_t lora_flight_init();

void lora_transmit_packet(goober_t *packet);

int lora_blocking_listen(goober_t *received_packet, uint8_t timeout);

uint8_t calc_pyro_arm(void);
            
#endif