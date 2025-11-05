#ifndef LORA_TYPES_H
#define LORA_TYPES_H

typedef struct lora_config {
	long frequency;
	int bandwidth;
	int coding_rate;
	int spreading_factor;
	int tx_power;
	uint8_t TDD;
} lora_config_t;

#endif

