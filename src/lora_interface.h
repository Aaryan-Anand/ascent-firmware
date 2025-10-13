#ifndef LORA_INTERFACE_H
#define LORA_INTERFACE_H

#include "stdbool.h"
#include "stdint.h"
#include "lora.h"
#include "goober.h"

#define LORA_FREQ 920e6

#define LORA_DEBUG

void queueLatestTelemetryPayload(goober_payload_t *payload);

void peekLatestTelemetryPayload(goober_payload_t *payload);

esp_err_t lora_flight_init();

void lora_transmit_packet(goober_t *packet);

int lora_blocking_listen(goober_t *received_packet, uint8_t timeout);
            
#endif