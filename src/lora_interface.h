#ifndef LORA_INTERFACE_H
#define LORA_INTERFACE_H

#include "stdbool.h"
#include "stdint.h"
#include "lora.h"
#include "goober.h"

#define LORA_FREQ 928e6

#define LORA_DEBUG

void lora_flight_init();

goober_t lora_create_packet(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload);

void lora_transmit_packet(goober_t *packet);

void slave_lora_task(goober_payload_t *telemetry);

void lora_queue_packet(goober_payload_t *payload);

void lora_read_latest_queue_packet(goober_payload_t *payload);


            
#endif